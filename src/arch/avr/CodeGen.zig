const std = @import("std");
const builtin = @import("builtin");
const mem = std.mem;
const codegen = @import("../../codegen.zig");
const Air = @import("../../Air.zig");
const Liveness = @import("../../Liveness.zig");
const link = @import("../../link.zig");
const Module = @import("../../Module.zig");
const InternPool = @import("../../InternPool.zig");
const Allocator = mem.Allocator;
const log = std.log.scoped(.codegen);
const build_options = @import("build_options");

const CodeGenError = codegen.CodeGenError;
const Result = codegen.Result;
const DebugInfoOutput = codegen.DebugInfoOutput;

const errUnionPayloadOffset = codegen.errUnionPayloadOffset;
const errUnionErrorOffset = codegen.errUnionErrorOffset;

const InnerError = CodeGenError || error{OutOfRegisters};
const ErrorMsg = Module.ErrorMsg;

const Self = @This();
const Mir = @import("Mir.zig");
const Emit = @import("Emit.zig");
const Registers = @import("Registers.zig");
const Register = Registers.Register;
const IO = Registers.IORegisters;
const RegisterManager = Registers.RegisterManager;

mir: std.ArrayList(Mir.Insn),
air: Air,
liveness: Liveness,
register_manager: RegisterManager,
alloc: Allocator,
zcu: Module,
tracking: std.ArrayListUnmanaged(?ProtoData),
args: []ProtoData,
arg_index: u64 = 0,
frame_size: u64 = 0,

fn getReturnLoc(
    self: *const Self,
    fn_type: InternPool.Key.FuncType,
) !ProtoData {
    const rt: std.builtin.Type = Type.fromInterned(fn_type.return_type);
    switch (rt) {
        .NoReturn, .Void => return .{.none},
        .Int => |int| {
            var i = @intFromEnum(Register.r26);
            const odd = int.bits & 1 == 0;
            const size = int.bits + if (odd) 1 else 0;
            i -= size;
            if (i < 8) self.fail("TODO implement stack return types.");
            return .{ .register = @enumFromInt(i) };
        },
        else => self.fail("TODO return type not implemented."),
    }
}

/// Discover which register will be used to call this function
fn calcArgs(
    self: *const Self,
    fn_type: InternPool.Key.FuncType,
) ![]ProtoData {
    // TODO Use Register Manager
    const data = self.alloc.alloc(ProtoData, fn_type.param_types.len);
    var i = @intFromEnum(Register.r26);
    // TODO ATtiny only have r16-r31
    for (fn_type.param_types.get(&self.zcu.intern_pool), data) |arg_type, *arg| {
        if (i < 8) self.fail("Arguments flowing into memory not implemented(register < 8)");
        const ptype = Type.fromInterned(arg_type);
        const t: std.builtin.Type = ptype.zigTypeTag(self.zcu);
        switch (t) {
            .Undefined,
            => arg.* = .{.undef},
            .Int,
            => |int| {
                const odd = int.bits & 1 == 0;
                const size = int.bits + if (odd) 1 else 0;
                i -= size;
                arg.* = .{ .register = @enumFromInt(i) };
            },
            .Pointer,
            => |ptr| {
                if (ptr.is_volatile) self.fail("TODO implement IO space operations");
            },
            .Float,
            .ComptimeFloat,
            => self.fail("Float arguments not implemented"),
            else => self.fail("arg type Not implemented"),
        }
    }
    return data;
}

pub fn generate(
    bin_file: *link.File,
    src_loc: Module.SrcLoc,
    func_index: InternPool.Index,
    air: Air,
    liveness: Liveness,
    code: *std.ArrayList(u8),
    debug_output: DebugInfoOutput,
) CodeGenError!Result {
    _ = src_loc; // autofix
    _ = code; // autofix
    _ = debug_output; // autofix
    const comp = bin_file.comp;
    const zcu = comp.module.?;
    const func = zcu.funcInfo(func_index);
    const fn_owner_decl = zcu.declPtr(func.owner_decl);
    const fn_type = fn_owner_decl.typeOf(zcu);
    var self = Self{
        .mir = std.ArrayList(Mir.Insn).init(comp.gpa),
        .air = air,
        .liveness = liveness,
        .register_manager = .{},
        .alloc = zcu.gpa,
        .zcu = zcu,
        .tracking = std.ArrayListUnmanaged(Mir.Insn).initCapacity(comp.gpa, air.instructions.len),
        .args = undefined,
    };
    self.tracking.items.len = self.tracking.capacity;
    @memset(self.tracking.items, null);
    defer {
        self.mir.deinit();
        self.tracking.deinit();
    }
    const CallConvInfo: struct {
        pub const stack_byte_alignment = 1;
        stack_size: u16 = undefined,
        return_value: ProtoData = .none,
        args: []ProtoData = undefined,
        //
    } = .{};
    const fn_info = zcu.typeToFunc(fn_type).?;
    CallConvInfo.args = self.calcArgs(fn_info) catch return Result{ .fail = "failed" };
    self.args = CallConvInfo.args;
    CallConvInfo.return_value = self.getReturnLoc(fn_info) catch return Result{ .fail = "failed" };
    if (fn_info.cc != .Unspecified) self.fail("Calling Conventions not supported.") catch return Result{ .fail = "failed" };
    {
        // Prologue
        // Y Register is callee saved
        // TODO if they are never used maybe this isn't needed
        try self.mir.append(.{ .tag = .PUSH, .data = .{ .R = Register.Y[0] } });
        try self.mir.append(.{ .tag = .PUSH, .data = .{ .R = Register.Y[1] } });
        try self.mir.append(.{ .tag = .IN, .data = .{ .R_io = .{ .rd = Register.Y[0], .io = IO.SP_L } } });
        try self.mir.append(.{ .tag = .IN, .data = .{ .R_io = .{ .rd = Register.Y[0], .io = IO.SP_H } } });

        self.genBody(air) catch return Result{ .fail = "failed" };
        // Epilogue
        try self.mir.append(.{ .tag = .PUSH, .data = .{ .R = Register.Y[1] } });
        try self.mir.append(.{ .tag = .PUSH, .data = .{ .R = Register.Y[0] } });
        try self.mir.append(.{ .tag = .RET, .data = .none });
    }
    var emit = Emit{};
    emit.emit(self.mir, bin_file);
    unreachable;
}

fn fail(self: *Self, reason: []const u8) !void {
    _ = self; // autofix
    log.err("Avr CodegenFailed: {s}", .{reason});
    return CodeGenError.CodegenFail;
}

pub const ProtoData = union(enum) {
    none,
    undef,
    register: struct { len: u4, start: Register },
    immediate: u22,
    memory: u16,
    IOAddr: u6,
};

fn translateGenResult(self: *Self, result: codegen.GenResult) !ProtoData {
    switch (result) {
        .mcv => |value| {
            return switch (value) {
                .none => .{.none},
                .undef => .{.undef},
                .immediate => |i| .{ .immediate = i },
                .load_tlv => unreachable,
                .load_direct => unreachable,
                .load_got => unreachable,
                .memory => |i| .{ .memory = i },
                .load_symbol => unreachable,
            };
        },
        .fail => |f| self.fail(f),
    }
}

const Value = @import("../../Value.zig");
const Type = @import("../../type.zig").Type;
fn resolveInsn(self: *Self, ref: Air.Inst.Ref, t: Type) !void {
    _ = ref; // autofix
    _ = t; // autofix
    codegen.genTypedValue(self.file, self.srcloc, null, null);
}

fn fromValue(self: *const Self, val: Value) !ProtoData {
    const genResult = codegen.genTypedValue(self.file, self.src_loc, val, self.zcu.funcOwnerDeclIndex(self.func_index));
    return try self.translateGenResult(genResult);
}

fn deRef(self: *const Self, ref: Air.Inst.Ref) !ProtoData {
    // The value this insn is operation on is from an .arg or other insn
    if (ref.toIndex()) |aidx| {
        return self.tracking.items[aidx].?;
    } else { // The value is a const/comptime
        return try self.fromValue(Value.fromInterned(ref.toInterned().?));
    }
}

const reg_range: RegisterManager.RegisterBitSet = blk: {
    var set = RegisterManager.RegisterBitSet.initEmpty();
    set.setRangeValue(.{ .start = 0, .end = Registers.Register.caller_saved.len });
    break :blk set;
};

fn allocReg(self: *Self, air: ?Air.Inst.Index) !struct { Register, RegisterManager.RegisterLock } {
    const reg = try self.register_manager.allocReg(air, reg_range);
    const lock = self.register_manager.lockRegAssumeUnused(reg);
    return .{ .reg = reg, .lock = lock };
}

fn airAdd(self: *Self, index: Air.Inst.Index) !void {
    // Add uses bin_op
    const insn: Air.Inst = self.air.instructions.get(@intFromEnum(index));
    const bin_op = insn.data.bin_op;
    // bin_op
    const lhs: ProtoData = try self.deRef(bin_op.lhs);
    // const lhs_t = bin_op.lhs.toType();

    const rhs: ProtoData = try self.deRef(bin_op.rhs);
    // const rhs_t = bin_op.rhs.toType();
    if (lhs == .register) {
        // TODO Lock lhs
        switch (rhs) {
            .immediate => |i| {
                // Get a register
                const r_reg, const r_lock = try self.allocReg(index);
                // TODO
                defer self.register_manager.unlockReg(r_lock);
                // LDI
                try self.mir.append(.{ .tag = .LDI, .data = .{ .ur_K = .{ .rd = r_reg, .k = i } } });
                // Add
                try self.mir.append(.{ .tag = .ADD, .data = .{ .rr = .{ .rr = r_reg, .rd = lhs.register.start } } });
                // Free Register
            },
            .register => {
                self.mir.append(.{ .tag = .ADD, .data = .{ .rr = .{ .rd = lhs.register.start, .rr = rhs.register.start } } });
            },
            else => {
                self.fail("TODO AddInsn rhs not register or immediate");
            },
        }
    } else {
        self.fail("TODO Addinsn lhs not register");
    }
}

fn airArg(self: *Self, index: Air.Inst.Index) !void {
    var arg_index = self.arg_index;
    while (self.args[arg_index] == .none) arg_index += 1;
    // Advance self.arg_index to the next possible position
    self.arg_index = arg_index + 1;
    const arg = self.args[arg_index];
    // Increase frame_size by the amount of registers used
    self.frame_size += arg.register.len;
    // Track the .arg for other insns using it as a value
    self.tracking.insert(@intFromEnum(index), self.args[arg_index]);
    // This doesn't result in a Mir
}

fn genBody(self: *Self) !void {
    const air_tags: []Air.Inst.Tag = self.air.instructions.items(.tag);
    for (self.air.getMainBody()) |insn| {
        // Go through each Air Insn and convert it to 1 or more Mir insn
        if (self.liveness.isUnused(insn)) continue;
        switch (air_tags[@intFromEnum(insn)]) {
            .dbg_stmt => try self.mir.append(.{ .tag = .dbg_stmt, .data = .none }),
            .dbg_inline_block => try self.mir.append(.{ .tag = .dbg_inline_block, .data = .none }),
            .dbg_var_ptr => try self.mir.append(.{ .tag = .dbg_var_ptr, .data = .none }),
            .dbg_var_val => try self.mir.append(.{ .tag = .dbg_var_val, .data = .none }),
            .arg,
            => self.airArg(insn),
            .add,
            => self.airAdd(insn),
            .sub,
            .mul,
            .shr,
            .shl,
            .bit_and,
            .bit_or,
            .xor,
            => self.fail("only add is implemented."),
            .add_with_overflow,
            .sub_with_overflow,
            .mul_with_overflow,
            .shl_with_overflow,
            => self.fail("overflow math not implemented"),
            .inferred_alloc,
            .inferred_alloc_comptime,
            .reduce,
            .reduce_optimized,
            .splat,
            .shuffle,
            .select,
            => self.fail("Vectors not implemented."),
            .sqrt,
            .sin,
            .cos,
            .tan,
            .exp,
            .exp2,
            .log,
            .log2,
            .log10,
            .abs,
            .floor,
            .ceil,
            .round,
            .max,
            .min,
            => self.fail("math functions not implemented."),
            .div_float,
            .div_float_optimized,
            .trunc_float,
            .int_from_float,
            .int_from_float_optimized,
            .float_from_int,
            .fptrunc,
            .fpext,
            => self.fail("floating point math not implemented."),
            .add_safe,
            .sub_safe,
            .mul_safe,
            => self.fail("safe math not implemented."),
            .add_optimized,
            .sub_optimized,
            .mul_optimized,
            .div_trunc_optimized,
            .div_floor_optimized,
            .div_exact_optimized,
            .rem_optimized,
            .mod_optimized,
            => self.fail("optimized math not implemented."),
            .add_wrap,
            .sub_wrap,
            .mul_wrap,
            => self.fail("wrap math not implemented."),
            .add_sat,
            .sub_sat,
            .mul_sat,
            .shl_sat,
            => self.fail("saturating math not implemented."),
            .div_trunc,
            .div_floor,
            .div_exact,
            .rem,
            .mod,
            => self.fail("Division not implemented."),
            .ptr_add,
            .ptr_sub,
            => unreachable,
            .shr_exact,
            .shl_exact,
            => unreachable,
            .alloc,
            => self.fail("Stack allocation not implemented."),
            .ret_ptr,
            .assembly,
            .not,
            .bitcast,
            .block,
            .loop,
            .br,
            .trap,
            .breakpoint,
            .ret_addr,
            .frame_addr,
            .call,
            .call_always_tail,
            .call_never_tail,
            .call_never_inline,
            .clz,
            .ctz,
            .popcount,
            => unreachable,
            .byte_swap,
            .bit_reverse,
            .neg,
            .neg_optimized,
            .cmp_lt,
            .cmp_lt_optimized,
            .cmp_lte,
            .cmp_lte_optimized,
            .cmp_eq,
            .cmp_eq_optimized,
            .cmp_gte,
            .cmp_gte_optimized,
            .cmp_gt,
            .cmp_gt_optimized,
            .cmp_neq,
            .cmp_neq_optimized,
            .cmp_vector,
            .cmp_vector_optimized,
            .cond_br,
            .switch_br,
            .@"try",
            .try_ptr,
            .is_null,
            .is_non_null,
            .is_null_ptr,
            .is_non_null_ptr,
            .is_err,
            .is_non_err,
            .is_err_ptr,
            .is_non_err_ptr,
            .bool_and,
            .bool_or,
            .load,
            .int_from_ptr,
            .int_from_bool,
            .ret,
            .ret_safe,
            .ret_load,
            .store,
            .store_safe,
            .unreach,
            .intcast,
            .trunc,
            .optional_payload,
            .optional_payload_ptr,
            .optional_payload_ptr_set,
            .wrap_optional,
            .unwrap_errunion_payload,
            .unwrap_errunion_err,
            .unwrap_errunion_payload_ptr,
            .unwrap_errunion_err_ptr,
            .errunion_payload_ptr_set,
            .wrap_errunion_payload,
            .wrap_errunion_err,
            .struct_field_ptr,
            .struct_field_ptr_index_0,
            .struct_field_ptr_index_1,
            .struct_field_ptr_index_2,
            .struct_field_ptr_index_3,
            .struct_field_val,
            .set_union_tag,
            .get_union_tag,
            .slice,
            .slice_len,
            .slice_ptr,
            .ptr_slice_len_ptr,
            .ptr_slice_ptr_ptr,
            .array_elem_val,
            .slice_elem_val,
            .slice_elem_ptr,
            .ptr_elem_val,
            .ptr_elem_ptr,
            .array_to_slice,
            => unreachable,
            .memset,
            .memset_safe,
            .memcpy,
            .cmpxchg_weak,
            .cmpxchg_strong,
            .fence,
            .atomic_load,
            .atomic_store_unordered,
            .atomic_store_monotonic,
            .atomic_store_release,
            .atomic_store_seq_cst,
            .atomic_rmw,
            => unreachable,
            .is_named_enum_value,
            .tag_name,
            .error_name,
            .error_set_has_value,
            .aggregate_init,
            .union_init,
            .prefetch,
            .mul_add,
            .field_parent_ptr,
            .wasm_memory_size,
            .wasm_memory_grow,
            => unreachable,
            .cmp_lt_errors_len,
            .err_return_trace,
            .set_err_return_trace,
            .addrspace_cast,
            .save_err_return_trace_index,
            .vector_store_elem,
            => unreachable,
            .c_va_arg,
            .c_va_copy,
            .c_va_end,
            .c_va_start,
            => unreachable,
            .work_item_id,
            .work_group_size,
            .work_group_id,
            => self.fail("This is not a gpu."),
        }
    }
}
