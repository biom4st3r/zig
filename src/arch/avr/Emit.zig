const link = @import("../../link.zig");

const Mir = @import("Mir.zig");
const Self = @This();
const std = @import("std");

code: *std.ArrayList(u8),
mir: []Mir.Insn,
// mir_extra: std.AutoHashMap(u64, u32),

const log = std.log.scoped(.Emit);
fn encode_payload(self: *Self, insn: Mir.Insn.Tag, data: anytype) void {
    const section = self.code.addManyAsArray(2) catch unreachable;
    @memcpy(section, std.mem.asBytes(&@byteSwap(data.encode(@intFromEnum(insn)))));
    log.err("Writing: {s}", .{@tagName(insn)});
}

pub fn emit(self: *Self) void {
    for (self.mir) |insn| switch (insn.data) {
        .none => |pl| encode_payload(self, insn.tag, pl),
        .RR => |pl| encode_payload(self, insn.tag, pl),
        .R => |pl| encode_payload(self, insn.tag, pl),
        .r4_u6 => |pl| encode_payload(self, insn.tag, pl),
        .r_u8 => |pl| encode_payload(self, insn.tag, pl),
        .s => |pl| encode_payload(self, insn.tag, pl),
        .R_s => |pl| encode_payload(self, insn.tag, pl),
        .s_k => |pl| encode_payload(self, insn.tag, pl),
        .offset => |pl| encode_payload(self, insn.tag, pl),
        .call => |pl| {
            _ = pl; // autofix
            // encode_payload(self, insn.tag, pl);
            unreachable; // TODO this is needs a u32, but not insns are u16
        },
        .io_s => |pl| encode_payload(self, insn.tag, pl),
        .des => |pl| encode_payload(self, insn.tag, pl),
        .R_Wr => |pl| encode_payload(self, insn.tag, pl),
        .mul => |pl| encode_payload(self, insn.tag, pl),
        .R_io => |pl| encode_payload(self, insn.tag, pl),
        .R_WrQ => |pl| encode_payload(self, insn.tag, pl),
        .R_u16 => |pl| encode_payload(self, insn.tag, pl),
        .r_u7 => |pl| encode_payload(self, insn.tag, pl),
        .movw => |pl| encode_payload(self, insn.tag, pl),
        .rcall => |pl| encode_payload(self, insn.tag, pl),
    };
}
