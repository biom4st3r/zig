pub const RegisterManager = @import("../../register_manager.zig").RegisterManager(
    @import("CodeGen.zig"),
    Register,
    Register.caller_saved ++ Register.callee_saved,
);
const std = @import("std");

pub const IORegisters = struct {
    pub const SP_L: u6 = 0x3d;
    pub const SP_H: u6 = 0x3e;
};
pub const Register = enum {
    pub fn isCalleeSaved(r: Register) bool {
        return std.mem.indexOfScalar(Register, callee_saved, r) != null;
    }
    // zig fmt: off
    pub const caller_saved: []const Register = &.{ 
        .r18, .r19, .r20, .r21, 
        .r22, .r23, .r24, .r25, 
        .r26, .r27, .r30, .r31
    };
    pub const callee_saved: []const Register = &.{
        .r2,  .r3,  .r4,  .r5,
        .r6,  .r7,  .r8,  .r9,
        .r10, .r11, .r12, .r13,
        .r14, .r15, .r16, .r17,
        .r28, .r29,
    };
    pub const temp: []const Register = &.{.r0};
    pub const Zero: []const Register  = &.{.r1};
    pub const arguments: []const Register = &.{
        .r8,  .r10, .r12, .r14, 
        .r16, .r18, .r20, .r22, .r24,
    };
    pub const X: []const Register = &.{.r26, .r27};
    pub const Y: []const Register = &.{.r28, .r29};
    pub const SP = Y;
    pub const Z: []const Register = &.{.r30, .r31};
    pub const MulResult: []const Register  = &.{.r0,.r1};

    r0,  r1,  r2,  r3,  r4,  r5,  r6,  r7,
    r8,  r9,  r10, r11, r12, r13, r14, r15,
    r16, r17, r18, r19, r20, r21, r22, r23,
    r24, r25, r26, r27, r28, r29, r30, r31,

    // Prepended to 16-bit addresses
    //   to form 24-bit addresses
    // rampd, rampx, rampy, rampz, eind,

    // Pointer Registers
    // x, // r26:r27
    // y, // r28:R29
    // z, // r30:r31
    // zig fmt: on
};
