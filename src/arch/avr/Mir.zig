const Register = @import("Registers.zig").Register;
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

        none: void,
        // xxxx xxrd dddd rrrr
        RR: struct { rd: Reg, rr: Reg },
        // xxxx xxxd dddd xxxx
        R: Reg,
        // ADIW
        // xxxx xxxx IIdd IIII
        r4_u6: struct { rd: SmallReg, imm: u6 },
        // xxxx IIII dddd IIII
        r_u8: struct { rd: UpperReg, imm: u8 },
        // BCLR
        // xxxx xxxx xsss xxxx
        s: Bit,
        // BLD
        // xxxx xxxd dddd xsss
        R_s: struct { rd: Reg, s: Bit },
        // BRBC Branch if sreg bit is cleared
        // xxxx xxoo oooo osss
        s_k: struct { s: Bit, offset: i7 },
        // BRCC
        // xxxx xxoo oooo oxxx
        offset: i7,
        // xxxx xxxk kkkk xxxk kkkk kkkk kkkk kkkk
        // Call and Jmp
        call: u22,
        // CBI
        // xxxx xxxx iiii isss
        io_s: struct { lower_io: u5, s: Bit },
        // DES
        // xxxx xxxx kkkk xxxx
        des: u4,
        // ELPM
        // LD on the X register
        // xxxx xxxd dddd xxxx
        R_Wr: struct { rd: Reg, wr: WideRegister, inc: WrInc },
        // xxxx xxxx xddd xrrr
        mul: struct { rd: MulReg, rr: MulReg },
        // IN
        // xxxx xIId dddd IIII
        R_io: struct { rd: Reg, io: u6 },
        // LD/ST
        // Q is displacement from addr
        // 10q0 qq0d dddd 0qqq
        R_WrQ: struct { rd: Reg, wr: WideRegister, q: u6 },
        // LDS
        // xxxx xxxd dddd xxxx kkkk kkkk kkkk kkkk
        R_u16: struct { rd: Reg, addr: u16 },
        // LDS(AVRrc)
        // xxxx xkkk dddd kkkk
        // ADDR[7:0] = 8,8,10,9,3,2,1,0
        // have fun encoding that
        r_u7: struct { rd: UpperReg, addr: u7 },
        // xxxx xxxx dddd rrrr
        movw: struct { rd: MovwReg, rr: MovwReg },
        // xxxx kkkk kkkk kkkk
        // rcall and rjmp
        rcall: struct { offset: u12 },
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
