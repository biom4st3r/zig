const std = @import("std");
const Register = @import("Registers.zig").Register;
fn assert(b: bool) void {
    if (!b) unreachable;
}
pub const Insn = struct {
    tag: Tag,
    data: Data,

    pub const Data = union {
        const Reg = Register; // u5 0-31
        const UpperReg = Register; // u4 16-31
        const SmallReg = Register; // u2 24,26,28,30
        const MulReg = Register; // 16 - 23
        const MovwReg = Register; // 0,2,4,6,8...30
        const SignedHalfImm = i7;
        const Bit = u3;
        const LowerIOAddress = u5;
        const IOAddress = u6;
        const WideRegister = enum { X, Y, Z };
        const WrInc = enum { @"+", o, @"-" };

        none: struct {
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                _ = self; // autofix
                return _insn;
            }
        },
        // xxxx xxrd dddd rrrr
        RR: struct {
            rd: Reg,
            rr: Reg,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                var insn = _insn & 0b1111_1100_0000_0000;
                const rr: u16 = @intCast(@intFromEnum(self.rr));
                const rd: u16 = @intCast(@intFromEnum(self.rd));
                insn |= (0b10000 & rr) << 5;
                insn |= (0b01111 & rr) << 0;
                insn |= rd << 4;
                return insn;
            }
        },
        // xxxx xxxd dddd xxxx
        R: struct {
            rd: Reg,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                var insn = _insn & 0b1111_1110_0000_1111;
                const rd: u16 = @intCast(@intFromEnum(self.rd));
                insn |= rd << 4;
                return insn;
            }
        },
        // ADIW
        // xxxx xxxx IIdd IIII
        r4_u6: struct {
            rd: SmallReg,
            imm: u6,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                var insn = _insn & 0b1111_1111_0000_0000;
                const rd: u16 = switch (self.rd) {
                    .r24 => 0,
                    .r26 => 1,
                    .r28 => 2,
                    .r30 => 3,
                    else => unreachable, // Register must be Register.small_register
                };
                const imm: u16 = @intCast(self.imm);
                insn |= (imm & 0b11_0000) << 2;
                insn |= rd << 4;
                insn |= (imm & 0b00_1111) << 0;
                return insn;
            }
        },
        // xxxx IIII dddd IIII
        r_u8: struct {
            rd: UpperReg,
            imm: u8,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                assert(@intFromEnum(self.rd) >= 16);
                var insn = _insn & 0b1111_0000_0000_0000;
                const rd: u16 = @intCast(@intFromEnum(self.rd) - 16);
                const imm: u16 = @intCast(self.imm);
                insn |= rd << 4;
                insn |= (imm & 0b1111_0000) << 8;
                insn |= (imm & 0b0000_1111) << 0;
                return insn;
            }
        },
        // BCLR
        // xxxx xxxx xsss xxxx
        s: struct {
            s: Bit,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                var insn = _insn & 0b1111_1111_1000_1111;
                const bit: u16 = @intCast(self.s);
                insn |= bit << 4;
                return insn;
            }
        },
        // BLD
        // xxxx xxxd dddd xsss
        R_s: struct {
            rd: Reg,
            s: Bit,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                var insn = _insn & 0b1111_1110_0000_1000;
                const rd: u16 = @intCast(@intFromEnum(self.reg));
                const bit: u16 = @intCast(self.s);
                insn |= bit << 0;
                insn |= rd << 4;
                return insn;
            }
        },
        // BRBC Branch if sreg bit is cleared
        // xxxx xxoo oooo osss
        s_k: struct {
            s: Bit,
            offset: i7,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                var insn = _insn & 0b1111_1100_0000_0000;
                const offset: u16 = @bitCast(@intFromEnum(self.offset));
                const bit: u16 = @intCast(self.s);
                insn |= bit << 0;
                insn |= offset << 3;
                return insn;
            }
        },
        // BRCC
        // xxxx xxoo oooo oxxx
        offset: struct {
            offset: i7,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                var insn = _insn & 0b1111_1100_0000_0111;
                const offset: u16 = @bitCast(@intFromEnum(self.offset));
                insn |= offset << 3;
                return insn;
            }
        },
        // xxxx xxxk kkkk xxxk kkkk kkkk kkkk kkkk
        // Call and Jmp
        call: struct {
            addr: u22,
            pub fn encode(self: *const @This(), _insn: u32) u32 {
                var insn = _insn & 0b1111_1110_0000_1110_0000_0000_0000_0000;
                const offset: u32 = @intCast(@intFromEnum(self.offset));
                insn |= (offset >> 22 - 5) << (19);
                insn |= (offset & 0b1_1111_1111_1111_1111);
                return insn;
            }
        },
        // CBI
        // xxxx xxxx iiii isss
        io_s: struct {
            lower_io: u5,
            s: Bit,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                var insn = _insn & 0b1111_1111_0000_0000;
                const lower_io: u16 = @intCast(@intFromEnum(self.lower_io));
                const s: u16 = @intCast(@intFromEnum(self.s));
                insn |= lower_io << 3;
                insn |= s << 0;
                return insn;
            }
        },
        // DES
        // xxxx xxxx kkkk xxxx
        des: struct {
            k: u4,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                var insn = _insn & 0b1111_1111_0000_1111;
                const k: u16 = @intCast(@intFromEnum(self.k));
                insn |= k << 4;
                return insn;
            }
        },
        // ELPM
        // LD on the X register
        // xxxx xxxd dddd xxxx
        R_Wr: struct {
            rd: Reg,
            wr: WideRegister,
            inc: WrInc,
        },
        // xxxx xxxx xddd xrrr
        mul: struct {
            rd: MulReg,
            rr: MulReg,
        },
        // IN
        // xxxx xIId dddd IIII
        R_io: struct {
            rd: Reg,
            io: u6,
        },
        // LD/ST
        // Q is displacement from addr
        // 10q0 qq0d dddd 0qqq
        R_WrQ: struct {
            rd: Reg,
            wr: WideRegister,
            q: u6,
        },
        // LDS
        // xxxx xxxd dddd xxxx kkkk kkkk kkkk kkkk
        R_u16: struct {
            rd: Reg,
            addr: u16,
        },
        // LDS(AVRrc)
        // xxxx xkkk dddd kkkk
        // ADDR[7:0] = 8,8,10,9,3,2,1,0
        // have fun encoding that
        r_u7: struct {
            rd: UpperReg,
            addr: u7,
        },
        // xxxx xxxx dddd rrrr
        movw: struct {
            rd: MovwReg,
            rr: MovwReg,
        },
        // xxxx kkkk kkkk kkkk
        // rcall and rjmp
        rcall: struct {
            offset: u12,
        },
    };
    pub const Tag = enum {
        placeholder,
        // TODO Maybe add seperate instrutions for ST_X ST_Y ST_Z... etc
        dbg_stmt,
        dbg_inline_block,
        dbg_var_ptr,
        dbg_var_val,
        ADD,
        ADC,
        ADIW,
        SUB,
        SUBI,
        SBC,
        SBCI,
        SBIW,
        AND,
        ANDI,
        OR,
        ORI,
        EOR,
        COM,
        NEG,
        SBR,
        CBR,
        INC,
        DEC,
        TST,
        CLR,
        SER,
        MUL,
        MULS,
        MULSU,
        FMUL,
        FMULS,
        FMULSU,
        DES,
        RJMP,
        IJMP,
        EIJMP,
        JMP,
        RCALL,
        ICALL,
        EICALL,
        CALL,
        RET,
        RETI,
        CPSE,
        CP,
        CPC,
        CPI,
        SBRC,
        SBRS,
        SBIC,
        SBIS,
        BRBS,
        BRBC,
        BREQ,
        BRNE,
        BRCS,
        BRCC,
        BRSH,
        BRLO,
        BRMI,
        BRPL,
        BRGE,
        BRLT,
        BRHS,
        BRHC,
        BRTS,
        BRTC,
        BRVS,
        BRVC,
        BRIE,
        BRID,
        MOV,
        MOVW,
        LDI,
        LDS,
        LD,
        LDD,
        STS,
        ST,
        STD,
        LPM,
        ELPM,
        SPM,
        IN,
        OUT,
        PUSH,
        POP,
        XCH,
        LAS,
        LAC,
        LAT,
        LSL,
        LSR,
        ROL,
        ROR,
        ASR,
        SWAP,
        SBI,
        CBI,
        BST,
        BLD,
        BSET,
        BCLR,
        SEC,
        CLC,
        SEN,
        CLN,
        SEZ,
        CLZ,
        SEI,
        CLI,
        SES,
        CLS,
        SEV,
        CLV,
        SET,
        CLT,
        SEH,
        CLH,
        BREAK,
        NOP,
        SLEEP,
        WDR,
    };
};
