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
zcu: *Module,
tracking: []?ProtoData,
args: []ProtoData,
arg_index: u64 = 0,
frame_size: u64 = 0,
bin: *link.File,
src_loc: Module.SrcLoc,
func_index: InternPool.Index,

fn getReturnLoc(
    self: *Self,
    fn_type: InternPool.Key.FuncType,
) !ProtoData {
    const rt = Type.fromInterned(fn_type.return_type);
    const t = rt.zigTypeTag(self.zcu);
    switch (t) {
        .NoReturn, .Void => return ProtoData{ .none = {} },
        .Int => {
            const bits = rt.intInfo(self.zcu).bits;
            var i = @intFromEnum(Register.r26);
            const odd = bits & 1 == 0;
            const size = bits + @as(@TypeOf(bits), if (odd) 1 else 0);
            i -= @intCast(size);
            if (i < 8) try self.fail("TODO implement stack return types.", .{});
            return ProtoData{ .register = .{ .start = @enumFromInt(i), .len = @intCast(size >> 3) } };
        },
        else => try self.fail("TODO return type not implemented.", .{}),
    }
    unreachable;
}

/// Discover which register will be used to call this function
fn calcArgs(
    self: *Self,
    fn_type: InternPool.Key.FuncType,
) ![]ProtoData {
    // TODO Use Register Manager
    const data: []ProtoData = try self.alloc.alloc(ProtoData, fn_type.param_types.len);
    var i = @intFromEnum(Register.r26);
    // TODO ATtiny only have r16-r31
    for (fn_type.param_types.get(&self.zcu.intern_pool), data) |arg_type, *arg| {
        if (i < 8) try self.fail("Arguments flowing into memory not implemented(register < 8)", .{});
        const ptype = Type.fromInterned(arg_type);
        const t = ptype.zigTypeTag(self.zcu);
        switch (t) {
            .Undefined,
            => arg.* = ProtoData{ .undef = {} },
            .Int,
            => {
                const bits = ptype.intInfo(self.zcu).bits;
                const odd = bits & 1 == 0;
                const size = bits + @as(@TypeOf(bits), if (odd) 1 else 0);
                // TODO Check if size larger than i
                i -= @intCast(size);
                arg.* = ProtoData{ .register = .{ .start = @enumFromInt(i), .len = @intCast(size >> 3) } };
            },
            .Pointer,
            => {
                // if (ptr.is_volatile)
                try self.fail("TODO implement IO space operations and ptrs", .{});
            },
            .Float,
            .ComptimeFloat,
            => try self.fail("Float arguments not implemented", .{}),
            else => try self.fail("arg type Not implemented", .{}),
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
        .tracking = comp.gpa.alloc(?ProtoData, air.instructions.len) catch unreachable,
        .args = undefined,
        .bin = bin_file,
        .src_loc = src_loc,
        .func_index = func_index,
    };
    @memset(self.tracking, null);
    defer {
        self.mir.deinit();
        self.alloc.free(self.tracking);
    }
    const fn_info = zcu.typeToFunc(fn_type).?;
    const CallConvInfo: struct {
        pub const stack_byte_alignment = 1;
        stack_size: u16 = undefined,
        return_value: ProtoData = .none,
        args: []ProtoData = undefined,
        //
    } = .{
        .return_value = self.getReturnLoc(fn_info) catch unreachable,
        .args = self.calcArgs(fn_info) catch unreachable,
    };
    self.args = CallConvInfo.args;
    if (fn_info.cc != .Unspecified and fn_info.cc != .C) self.fail("Calling Conventions not supported. {s}", .{@tagName(fn_info.cc)}) catch unreachable;
    {
        // Prologue
        // Y Register is callee saved
        // TODO if they are never used maybe this isn't needed
        errdefer unreachable;
        try self.mir.append(.{ .tag = .PUSH, .data = .{ .R = .{ .rd = Register.Y[0] } } });
        try self.mir.append(.{ .tag = .PUSH, .data = .{ .R = .{ .rd = Register.Y[1] } } });
        try self.mir.append(.{ .tag = .IN, .data = .{ .R_io = .{ .rd = Register.Y[0], .io = IO.SP_L } } });
        try self.mir.append(.{ .tag = .IN, .data = .{ .R_io = .{ .rd = Register.Y[0], .io = IO.SP_H } } });

        self.genBody() catch unreachable;
        // Epilogue
        try self.mir.append(.{ .tag = .PUSH, .data = .{ .R = .{ .rd = Register.Y[1] } } });
        try self.mir.append(.{ .tag = .PUSH, .data = .{ .R = .{ .rd = Register.Y[0] } } });
        try self.mir.append(.{ .tag = .RET, .data = .{ .none = .{} } });
    }
    var emit = Emit{};
    emit.emit(self.mir.items, bin_file);
    unreachable;
}

fn fail(self: *Self, comptime reason: []const u8, args: anytype) !void {
    _ = self; // autofix
    log.err("Avr CodegenFailed: " ++ reason, args);
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
                .none => .{ .none = {} },
                .undef => .{ .undef = {} },
                // TODO handle i to large
                .immediate => |i| .{ .immediate = @intCast(i) },
                // TODO Handle address to large
                .memory => |i| .{ .memory = @intCast(i) },
                .load_tlv,
                .load_direct,
                .load_got,
                .load_symbol,
                => |_| {
                    try self.fail("Failed translate MCValue of type {any}", .{value});
                    unreachable;
                },
            };
        },
        .fail => |f| try self.fail("Failed translate MCValue |{s}", .{f.msg}),
    }
    unreachable;
}

const Value = @import("../../Value.zig");
const Type = @import("../../type.zig").Type;
fn resolveInsn(self: *Self, ref: Air.Inst.Ref, t: Type) !void {
    _ = ref; // autofix
    _ = t; // autofix
    codegen.genTypedValue(self.file, self.srcloc, null, null);
}

fn fromValue(self: *Self, val: Value) !ProtoData {
    const genResult = try codegen.genTypedValue(self.bin, self.src_loc, val, self.zcu.funcOwnerDeclIndex(self.func_index));
    return try self.translateGenResult(genResult);
}

fn deRef(self: *Self, ref: Air.Inst.Ref) !ProtoData {
    // The value this insn is operation on is from an .arg or other insn
    if (ref.toIndex()) |aidx| {
        return self.tracking[@intFromEnum(aidx)].?;
    } else { // The value is a const/comptime
        return try self.fromValue(Value.fromInterned(ref.toInterned().?));
    }
}

fn allocReg(self: *Self, air: ?Air.Inst.Index) !struct { Register, RegisterManager.RegisterLock } {
    const reg = try self.register_manager.allocReg(air, Registers.RegRanges.caller_saved);
    const lock = self.register_manager.lockRegAssumeUnused(reg);
    return .{ reg, lock };
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
                // TODO Handle i to large
                try self.mir.append(.{ .tag = .LDI, .data = .{ .r_u8 = .{ .rd = r_reg, .imm = @intCast(i) } } });
                // Add
                try self.mir.append(.{ .tag = .ADD, .data = .{ .RR = .{ .rr = r_reg, .rd = lhs.register.start } } });
                // Free Register
            },
            .register => {
                try self.mir.append(.{ .tag = .ADD, .data = .{ .RR = .{ .rd = lhs.register.start, .rr = rhs.register.start } } });
            },
            else => {
                try self.fail("TODO AddInsn rhs not register or immediate", .{});
            },
        }
    } else {
        try self.fail("TODO Addinsn lhs not register", .{});
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
    self.tracking[@intFromEnum(index)] = self.args[arg_index];
    // This doesn't result in a Mir
}
/// Required by RegisterManager
pub fn spillInstruction(self: *Self, reg: Register, inst: Air.Inst.Index) !void {
    _ = self; // autofix
    _ = reg; // autofix
    _ = inst; // autofix
    unreachable;
    // const tracking = self.inst_tracking.getPtr(inst) orelse return;
    // for (tracking.getRegs()) |tracked_reg| {
    //     if (tracked_reg.id() == reg.id()) break;
    // } else unreachable; // spilled reg not tracked with spilled instruciton
    // try tracking.spill(self, inst);
    // try tracking.trackSpill(self, inst);
}

fn genBody(self: *Self) !void {
    const air_tags: []Air.Inst.Tag = self.air.instructions.items(.tag);
    for (self.air.getMainBody()) |insn| {
        // Go through each Air Insn and convert it to 1 or more Mir insn
        if (self.liveness.isUnused(insn)) continue;
        switch (air_tags[@intFromEnum(insn)]) {
            .dbg_stmt => try self.mir.append(.{ .tag = .dbg_stmt, .data = .{ .none = .{} } }),
            .dbg_inline_block => try self.mir.append(.{ .tag = .dbg_inline_block, .data = .{ .none = .{} } }),
            .dbg_var_ptr => try self.mir.append(.{ .tag = .dbg_var_ptr, .data = .{ .none = .{} } }),
            .dbg_var_val => try self.mir.append(.{ .tag = .dbg_var_val, .data = .{ .none = .{} } }),
            .arg,
            => try self.airArg(insn),
            .add,
            => try self.airAdd(insn),
            .sub,
            .mul,
            .shr,
            .shl,
            .bit_and,
            .bit_or,
            .xor,
            => |ins| try self.fail("Insn not implemented {s}", .{@tagName(ins)}),
            .add_with_overflow,
            .sub_with_overflow,
            .mul_with_overflow,
            .shl_with_overflow,
            => |ins| try self.fail("overflow not implemented {s}", .{@tagName(ins)}),
            .inferred_alloc,
            .inferred_alloc_comptime,
            .reduce,
            .reduce_optimized,
            .splat,
            .shuffle,
            .select,
            => |ins| try self.fail("Vectors not implemented {s}", .{@tagName(ins)}),
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
            => |ins| try self.fail("fast math not implemented {s}", .{@tagName(ins)}),
            .div_float,
            .div_float_optimized,
            .trunc_float,
            .int_from_float,
            .int_from_float_optimized,
            .float_from_int,
            .fptrunc,
            .fpext,
            => |ins| try self.fail("Floats not implemented {s}", .{@tagName(ins)}),
            .add_safe,
            .sub_safe,
            .mul_safe,
            => |ins| try self.fail("safe math not implemented {s}", .{@tagName(ins)}),
            .add_optimized,
            .sub_optimized,
            .mul_optimized,
            .div_trunc_optimized,
            .div_floor_optimized,
            .div_exact_optimized,
            .rem_optimized,
            .mod_optimized,
            => |ins| try self.fail("optimized math not implemented {s}", .{@tagName(ins)}),
            .add_wrap,
            .sub_wrap,
            .mul_wrap,
            => |ins| try self.fail("wrap math not implemented {s}", .{@tagName(ins)}),
            .add_sat,
            .sub_sat,
            .mul_sat,
            .shl_sat,
            => |ins| try self.fail("Saturating math not implemented {s}", .{@tagName(ins)}),
            .div_trunc,
            .div_floor,
            .div_exact,
            .rem,
            .mod,
            => |ins| try self.fail("Division not implemented {s}", .{@tagName(ins)}),
            .ptr_add,
            .ptr_sub,
            => |ins| try self.fail("ptr math not implemented {s}", .{@tagName(ins)}),
            .shr_exact,
            .shl_exact,
            => |ins| try self.fail("exact shifting not implemented {s}", .{@tagName(ins)}),
            .alloc,
            => try self.fail("Stack allocation not implemented.", .{}),
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
            => |ins| try self.fail("Insn not implemented {s}", .{@tagName(ins)}),
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
            => |ins| try self.fail("Insn not implemented {s}", .{@tagName(ins)}),
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
            => |ins| try self.fail("Insn not implemented {s}", .{@tagName(ins)}),
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
            => |ins| try self.fail("Insn not implemented {s}", .{@tagName(ins)}),
            .cmp_lt_errors_len,
            .err_return_trace,
            .set_err_return_trace,
            .addrspace_cast,
            .save_err_return_trace_index,
            .vector_store_elem,
            => |ins| try self.fail("Insn not implemented {s}", .{@tagName(ins)}),
            .c_va_arg,
            .c_va_copy,
            .c_va_end,
            .c_va_start,
            => |ins| try self.fail("Insn not implemented {s}", .{@tagName(ins)}),
            .work_item_id,
            .work_group_size,
            .work_group_id,
            => try self.fail("This is not a gpu.", .{}),
        }
    }
}
