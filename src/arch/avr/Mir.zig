const std = @import("std");
const Register = @import("Registers.zig").Register;
fn assert(b: bool) void {
    if (!b) unreachable;
}
pub const Insn = struct {
    tag: Tag,
    data: Data,

    pub const Data = union(enum) {
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
                const rd: u16 = @intCast(@intFromEnum(self.rd));
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
                const offset: u16 = @intCast(@as(u7, @bitCast(self.offset)));
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
                const offset: u16 = @intCast(@as(u7, @bitCast(self.offset)));
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
                const offset: u32 = @intCast(self.addr);
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
                const lower_io: u16 = @intCast(self.lower_io);
                const s: u16 = @intCast(self.s);
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
                const k: u16 = @intCast(self.k);
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
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                _ = self; // autofix
                var insn = _insn & 0b1111_1111_0000_1111;
                _ = &insn;
                unreachable;
            }
        },
        // xxxx xxxx xddd xrrr
        mul: struct {
            rd: MulReg,
            rr: MulReg,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                _ = self; // autofix
                var insn = _insn & 0b1111_1111_0000_1111;
                _ = &insn;
                unreachable;
            }
        },
        // IN
        // xxxx xIId dddd IIII
        R_io: struct {
            rd: Reg,
            io: u6,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                var insn = _insn & 0b1111_1000_0000_0000;
                const rd: u16 = @intCast(@intFromEnum(self.rd));
                const io: u16 = @intCast(self.io);
                insn |= rd << 4;
                insn |= (io & 0b11_0000) << 3;
                insn |= (io & 0b00_1111) << 0;
                return insn;
            }
        },
        // LD/ST
        // Q is displacement from addr
        // 10q0 qq0d dddd 0qqq
        R_WrQ: struct {
            rd: Reg,
            wr: WideRegister,
            q: u6,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                _ = self; // autofix
                var insn = _insn & 0b1111_1111_0000_1111;
                _ = &insn;
                unreachable;
            }
        },
        // LDS
        // xxxx xxxd dddd xxxx kkkk kkkk kkkk kkkk
        R_u16: struct {
            rd: Reg,
            addr: u16,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                _ = self; // autofix
                var insn = _insn & 0b1111_1111_0000_1111;
                _ = &insn;
                unreachable;
            }
        },
        // LDS(AVRrc)
        // xxxx xkkk dddd kkkk
        // ADDR[7:0] = 8,8,10,9,3,2,1,0
        // have fun encoding that
        r_u7: struct {
            rd: UpperReg,
            addr: u7,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                _ = self; // autofix
                var insn = _insn & 0b1111_1111_0000_1111;
                _ = &insn;
                unreachable;
            }
        },
        // xxxx xxxx dddd rrrr
        movw: struct {
            rd: MovwReg,
            rr: MovwReg,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                _ = self; // autofix
                var insn = _insn & 0b1111_1111_0000_1111;
                _ = &insn;
                unreachable;
            }
        },
        // xxxx kkkk kkkk kkkk
        // rcall and rjmp
        rcall: struct {
            offset: u12,
            pub fn encode(self: *const @This(), _insn: u16) u16 {
                _ = self; // autofix
                var insn = _insn & 0b1111_1111_0000_1111;
                _ = &insn;
                unreachable;
            }
        },
    };
    pub const Tag = enum(u16) {
        // zig fmt: off
        placeholder      = 0b1111_1111_1111_1111,
        dbg_stmt         = 0b1111_1111_1111_1110,
        dbg_inline_block = 0b1111_1111_1111_1101,
        dbg_var_ptr      = 0b1111_1111_1111_1100,
        dbg_var_val      = 0b1111_1111_1111_1011,

        ADC              = 0b0001_1100_0000_0000,
        ADD              = 0b0000_1100_0000_0000,
        ADIW             = 0b1001_0110_0000_0000,
        AND              = 0b0010_0000_0000_0000,
        ANDI             = 0b0111_0000_0000_0000,
        ASR              = 0b1001_0100_0000_0101,
        BCLR             = 0b1001_0100_1000_1000,
        BLD              = 0b1111_1000_0000_0000,
        BRBC             = 0b1111_0100_0000_0000,
        BRBS             = 0b1111_0000_0000_0000,
        // BRCC          = 0b1111_0100_0000_0000, // BRBC 0, k
        // BRCS          = 0b1111_0000_0000_0000, // BRBS 0, k
        BREAK            = 0b1001_0101_1001_1000,
        BREQ             = 0b1111_0000_0000_0001,
        BRGE             = 0b1111_0100_0000_0100,
        BRHC             = 0b1111_0100_0000_0101,
        BRHS             = 0b1111_0000_0000_0101,
        BRID             = 0b1111_0100_0000_0111,
        BRIE             = 0b1111_0000_0000_0111,
        // BRLO          = 0b1111_0000_0000_0000, // BRBS 0, k
        BRLT             = 0b1111_0000_0000_0100,
        BRMI             = 0b1111_0000_0000_0010,
        BRNE             = 0b1111_0100_0000_0001,
        BRPL             = 0b1111_0100_0000_0010,
        // BRSH          = 0b1111_0100_0000_0000, // BRBC 0, k
        BRTC             = 0b1111_0100_0000_0110,
        BRTS             = 0b1111_0000_0000_0110,
        BRVC             = 0b1111_0100_0000_0011,
        BRVS             = 0b1111_0000_0000_0011,
        BSET             = 0b1001_0100_0000_1000,
        BST              = 0b1111_1010_0000_0000,
        CALL             = 0b1001_0100_0000_1110,
        CBI              = 0b1001_1000_0000_0000,
        // CBR           =           0b0ee_0NDI_with_0_complemente0,
        // CLC              = 0b1001_0100_1000_1000, // BCLR 0
        // CLH              = 0b1001_0100_1101_1000, // BCLR 5
        CLI              = 0b1001_0100_1111_1000,
        CLN              = 0b1001_0100_1010_1000,
        // CLR              = 0b0010_0100_0000_0000, // EOR rd,rd
        CLS              = 0b1001_0100_1100_1000,
        CLT              = 0b1001_0100_1110_1000,
        CLV              = 0b1001_0100_1011_1000,
        CLZ              = 0b1001_0100_1001_1000,
        COM              = 0b1001_0100_0000_0000,
        CP               = 0b0001_0100_0000_0000,
        CPC              = 0b0000_0100_0000_0000,
        CPI              = 0b0011_0000_0000_0000,
        CPSE             = 0b0001_0000_0000_0000,
        DEC              = 0b1001_0100_0000_1010,
        DES              = 0b1001_0100_0000_1011,
        EICALL           = 0b1001_0101_0001_1001,
        EIJMP            = 0b1001_0100_0001_1001,

        ELPM_R0          = 0b1001_0101_1101_1000,
        ELPM_RD_Z        = 0b1001_0000_0000_0110,
        ELPM_RD_Z_inc    = 0b1001_0000_0000_0111,

        EOR              = 0b0010_0100_0000_0000,
        FMUL             = 0b0000_0011_0000_1000,
        FMULS            = 0b0000_0011_1000_0000,
        FMULSU           = 0b0000_0011_1000_1000,
        ICALL            = 0b1001_0101_0000_1001,
        IJMP             = 0b1001_0100_0000_1001,
        IN               = 0b1011_0000_0000_0000,
        INC              = 0b1001_0100_0000_0011,
        JMP              = 0b1001_0100_0000_1100,
        LAC              = 0b1001_0010_0000_0110,
        LAS              = 0b1001_0010_0000_0101,
        LAT              = 0b1001_0010_0000_0111,

        LD_X             = 0b1001_0000_0000_1100,
        LD_X_inc         = 0b1001_0000_0000_1101,
        LD_X_dec         = 0b1001_0000_0000_1110,

        // LD_Y             = 0b1000_0000_0000_1000,
        LD_Y_inc         = 0b1001_0000_0000_1001,
        LD_Y_dec         = 0b1001_0000_0000_1010,
        LD_Y_q           = 0b1000_0000_0000_1000,

        // LD_Z             = 0b1000_0000_0000_0000,
        LD_Z_inc         = 0b1001_0000_0000_0001,
        LD_Z_dec         = 0b1001_0000_0000_0010,
        LD_Z_q           = 0b1000_0000_0000_0000,

        LDI              = 0b1110_0000_0000_0000,
        LDS              = 0b1001_0000_0000_0000,
        LDS_rc           = 0b1010_0000_0000_0000,

        LPM_R0           = 0b1001_0101_1100_1000,
        LPM_RD           = 0b1001_0000_0000_0100,
        LPM_RD_Z_inc     = 0b1001_0000_0000_0101,

        // LSL              = 0b0000_1100_0000_0000, // ADD rd,rd
        LSR              = 0b1001_0100_0000_0110,
        MOV              = 0b0010_1100_0000_0000,
        MOVW             = 0b0000_0001_0000_0000,
        MUL              = 0b1001_1100_0000_0000,
        MULS             = 0b0000_0010_0000_0000,
        MULSU            = 0b0000_0011_0000_0000,
        NEG              = 0b1001_0100_0000_0001,
        NOP              = 0b0000_0000_0000_0000,
        OR               = 0b0010_1000_0000_0000,
        ORI              = 0b0110_0000_0000_0000,
        OUT              = 0b1011_1000_0000_0000,
        POP              = 0b1001_0000_0000_1111,
        PUSH             = 0b1001_0010_0000_1111,
        RCALL            = 0b1101_0000_0000_0000,
        RET              = 0b1001_0101_0000_1000,
        RETI             = 0b1001_0101_0001_1000,
        RJMP             = 0b1100_0000_0000_0000,
        // ROL              = 0b0001_1100_0000_0000, // ADC rd,rd
        ROR              = 0b1001_0100_0000_0111,
        SBC              = 0b0000_1000_0000_0000,
        SBCI             = 0b0100_0000_0000_0000,
        SBI              = 0b1001_1010_0000_0000,
        SBIC             = 0b1001_1001_0000_0000,
        SBIS             = 0b1001_1011_0000_0000,
        SBIW             = 0b1001_0111_0000_0000,
        // SBR              = 0b0110_0000_0000_0000, // literally ORI 
        SBRC             = 0b1111_1100_0000_0000,
        SBRS             = 0b1111_1110_0000_0000,
        // SEC              = 0b1001_0100_0000_1000, // BSET 0
        SEH              = 0b1001_0100_0101_1000,
        SEI              = 0b1001_0100_0111_1000,
        SEN              = 0b1001_0100_0010_1000,
        SER              = 0b1110_1111_0000_1111,
        SES              = 0b1001_0100_0100_1000,
        SET              = 0b1001_0100_0110_1000,
        SEV              = 0b1001_0100_0011_1000,
        SEZ              = 0b1001_0100_0001_1000,
        SLEEP            = 0b1001_0101_1000_1000,
        SPM              = 0b1001_0101_1110_1000,

        // SPM_xm_xt        = 0b1001_0101_1110_1000, // SPM
        SPM_xm_xt_Z_inc  = 0b1001_0101_1111_1000,

        ST_X             = 0b1001_0010_0000_1100,
        ST_X_inc         = 0b1001_0010_0000_1101,
        ST_X_dec         = 0b1001_0010_0000_1110,

        // ST_Y             = 0b1000_0010_0000_1000,
        ST_Y_inc         = 0b1001_0010_0000_1001,
        ST_Y_dec         = 0b1001_0010_0000_1010,
        ST_Y_q           = 0b1000_0010_0000_1000,

        // ST_Z             = 0b1000_0010_0000_0000,
        ST_Z_inc         = 0b1001_0010_0000_0001,
        ST_Z_dec         = 0b1001_0010_0000_0010,
        ST_Z_q           = 0b1000_0010_0000_0000,

        STS              = 0b1001_0010_0000_0000,
        STS_rc           = 0b1010_1000_0000_0000,
        SUB              = 0b0001_1000_0000_0000,
        SUBI             = 0b0101_0000_0000_0000,
        SWAP             = 0b1001_0100_0000_0010,
        // TST              = 0b0010_0000_0000_0000, // AND Rd, Rd
        WDR              = 0b1001_0101_1010_1000,
        XCH              = 0b1001_0010_0000_010,
        // zig fmt: on
    };
};
