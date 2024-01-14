const Instruction = @This();

const testing = @import("std").testing;

opcode: Opcode,
mode: AddressMode,
cycles: u4,

pub const Opcode = enum {
    ADC,
    AND,
    ASL,
    BCC,
    BCS,
    BEQ,
    BIT,
    BMI,
    BNE,
    BPL,
    BRK,
    BVC,
    BVS,
    CLC,
    CLD,
    CLI,
    CLV,
    CMP,
    CPX,
    CPY,
    DEC,
    DEX,
    DEY,
    EOR,
    INC,
    INX,
    INY,
    JMP,
    JSR,
    LDA,
    LDX,
    LDY,
    LSR,
    NOP,
    ORA,
    PHA,
    PHP,
    PLA,
    PLP,
    ROL,
    ROR,
    RTI,
    RTS,
    SBC,
    SEC,
    SED,
    SEI,
    STA,
    STX,
    STY,
    TAX,
    TAY,
    TSX,
    TXA,
    TXS,
    TYA,
};

pub const AddressMode = enum {
    Implied,
    Accumulator,
    Immediate,
    Absolute,
    ZeroPage,
    AbsoluteX,
    AbsoluteY,
    ZeroPageX,
    ZeroPageY,
    Indirect,
    IndexedIndirect,
    IndirectIndexed,
    Relative,
};

pub fn decode(byte: u8) error{InvalidInstruction}!Instruction {
    return switch (byte) {
        0x00 => .{ .opcode = .BRK, .mode = .Implied, .cycles = 7 },
        0x01 => .{ .opcode = .ORA, .mode = .IndexedIndirect, .cycles = 6 },
        0x05 => .{ .opcode = .ORA, .mode = .ZeroPage, .cycles = 3 },
        0x06 => .{ .opcode = .ASL, .mode = .ZeroPage, .cycles = 5 },
        0x08 => .{ .opcode = .PHP, .mode = .Implied, .cycles = 3 },
        0x09 => .{ .opcode = .ORA, .mode = .Immediate, .cycles = 2 },
        0x0A => .{ .opcode = .ASL, .mode = .Accumulator, .cycles = 2 },
        0x0D => .{ .opcode = .ORA, .mode = .Absolute, .cycles = 4 },
        0x0E => .{ .opcode = .ASL, .mode = .Absolute, .cycles = 6 },

        0x10 => .{ .opcode = .BPL, .mode = .Relative, .cycles = 2 },
        0x11 => .{ .opcode = .ORA, .mode = .IndirectIndexed, .cycles = 5 },
        0x15 => .{ .opcode = .ORA, .mode = .ZeroPageX, .cycles = 4 },
        0x16 => .{ .opcode = .ASL, .mode = .ZeroPageX, .cycles = 6 },
        0x18 => .{ .opcode = .CLC, .mode = .Implied, .cycles = 2 },
        0x19 => .{ .opcode = .ORA, .mode = .AbsoluteY, .cycles = 4 },
        0x1D => .{ .opcode = .ORA, .mode = .AbsoluteX, .cycles = 4 },
        0x1E => .{ .opcode = .ASL, .mode = .AbsoluteX, .cycles = 7 },

        0x20 => .{ .opcode = .JSR, .mode = .Absolute, .cycles = 6 },
        0x21 => .{ .opcode = .AND, .mode = .IndexedIndirect, .cycles = 6 },
        0x24 => .{ .opcode = .BIT, .mode = .ZeroPage, .cycles = 3 },
        0x25 => .{ .opcode = .AND, .mode = .ZeroPage, .cycles = 3 },
        0x26 => .{ .opcode = .ROL, .mode = .ZeroPage, .cycles = 5 },
        0x28 => .{ .opcode = .PLP, .mode = .Implied, .cycles = 4 },
        0x29 => .{ .opcode = .AND, .mode = .Immediate, .cycles = 2 },
        0x2A => .{ .opcode = .ROL, .mode = .Accumulator, .cycles = 2 },
        0x2C => .{ .opcode = .BIT, .mode = .Absolute, .cycles = 4 },
        0x2D => .{ .opcode = .AND, .mode = .Absolute, .cycles = 4 },
        0x2E => .{ .opcode = .ROL, .mode = .Absolute, .cycles = 6 },

        0x30 => .{ .opcode = .BMI, .mode = .Relative, .cycles = 2 },
        0x31 => .{ .opcode = .AND, .mode = .IndirectIndexed, .cycles = 5 },
        0x35 => .{ .opcode = .AND, .mode = .ZeroPageX, .cycles = 4 },
        0x36 => .{ .opcode = .ROL, .mode = .ZeroPageX, .cycles = 6 },
        0x38 => .{ .opcode = .SEC, .mode = .Implied, .cycles = 2 },
        0x39 => .{ .opcode = .AND, .mode = .AbsoluteY, .cycles = 4 },
        0x3D => .{ .opcode = .AND, .mode = .AbsoluteX, .cycles = 4 },
        0x3E => .{ .opcode = .ROL, .mode = .AbsoluteX, .cycles = 7 },

        0x40 => .{ .opcode = .RTI, .mode = .Implied, .cycles = 6 },
        0x41 => .{ .opcode = .EOR, .mode = .IndexedIndirect, .cycles = 6 },
        0x45 => .{ .opcode = .EOR, .mode = .ZeroPage, .cycles = 3 },
        0x46 => .{ .opcode = .LSR, .mode = .ZeroPage, .cycles = 5 },
        0x48 => .{ .opcode = .PHA, .mode = .Implied, .cycles = 3 },
        0x49 => .{ .opcode = .EOR, .mode = .Immediate, .cycles = 2 },
        0x4A => .{ .opcode = .LSR, .mode = .Accumulator, .cycles = 2 },
        0x4C => .{ .opcode = .JMP, .mode = .Absolute, .cycles = 3 },
        0x4D => .{ .opcode = .EOR, .mode = .Absolute, .cycles = 4 },
        0x4E => .{ .opcode = .LSR, .mode = .Absolute, .cycles = 6 },

        0x50 => .{ .opcode = .BVC, .mode = .Relative, .cycles = 2 },
        0x51 => .{ .opcode = .EOR, .mode = .IndirectIndexed, .cycles = 5 },
        0x55 => .{ .opcode = .EOR, .mode = .ZeroPageX, .cycles = 4 },
        0x56 => .{ .opcode = .LSR, .mode = .ZeroPageX, .cycles = 6 },
        0x58 => .{ .opcode = .CLI, .mode = .Implied, .cycles = 3 },
        0x59 => .{ .opcode = .EOR, .mode = .AbsoluteY, .cycles = 4 },
        0x5D => .{ .opcode = .EOR, .mode = .AbsoluteX, .cycles = 4 },
        0x5E => .{ .opcode = .LSR, .mode = .AbsoluteX, .cycles = 7 },

        0x60 => .{ .opcode = .RTS, .mode = .Implied, .cycles = 6 },
        0x61 => .{ .opcode = .ADC, .mode = .IndexedIndirect, .cycles = 6 },
        0x65 => .{ .opcode = .ADC, .mode = .ZeroPage, .cycles = 3 },
        0x66 => .{ .opcode = .ROR, .mode = .ZeroPage, .cycles = 5 },
        0x68 => .{ .opcode = .PLA, .mode = .Implied, .cycles = 4 },
        0x69 => .{ .opcode = .ADC, .mode = .Immediate, .cycles = 2 },
        0x6A => .{ .opcode = .ROR, .mode = .Accumulator, .cycles = 2 },
        0x6C => .{ .opcode = .JMP, .mode = .Indirect, .cycles = 5 },
        0x6D => .{ .opcode = .ADC, .mode = .Absolute, .cycles = 4 },
        0x6E => .{ .opcode = .ROR, .mode = .Absolute, .cycles = 6 },

        0x70 => .{ .opcode = .BVS, .mode = .Relative, .cycles = 2 },
        0x71 => .{ .opcode = .ADC, .mode = .IndirectIndexed, .cycles = 5 },
        0x75 => .{ .opcode = .ADC, .mode = .ZeroPageX, .cycles = 4 },
        0x76 => .{ .opcode = .ROR, .mode = .ZeroPageX, .cycles = 6 },
        0x78 => .{ .opcode = .SEI, .mode = .Implied, .cycles = 2 },
        0x79 => .{ .opcode = .ADC, .mode = .AbsoluteY, .cycles = 4 },
        0x7D => .{ .opcode = .ADC, .mode = .AbsoluteX, .cycles = 4 },
        0x7E => .{ .opcode = .ROR, .mode = .Absolute, .cycles = 6 },

        0x81 => .{ .opcode = .STA, .mode = .IndexedIndirect, .cycles = 6 },
        0x84 => .{ .opcode = .STY, .mode = .ZeroPage, .cycles = 3 },
        0x85 => .{ .opcode = .STA, .mode = .ZeroPage, .cycles = 3 },
        0x86 => .{ .opcode = .STX, .mode = .ZeroPage, .cycles = 3 },
        0x88 => .{ .opcode = .DEY, .mode = .Implied, .cycles = 2 },
        0x8A => .{ .opcode = .TXA, .mode = .Implied, .cycles = 2 },
        0x8C => .{ .opcode = .STY, .mode = .Absolute, .cycles = 4 },
        0x8D => .{ .opcode = .STA, .mode = .Absolute, .cycles = 4 },
        0x8E => .{ .opcode = .STX, .mode = .Absolute, .cycles = 4 },

        0x90 => .{ .opcode = .BCC, .mode = .Relative, .cycles = 2 },
        0x91 => .{ .opcode = .STA, .mode = .IndirectIndexed, .cycles = 6 },
        0x94 => .{ .opcode = .STY, .mode = .ZeroPageX, .cycles = 4 },
        0x95 => .{ .opcode = .STA, .mode = .ZeroPageX, .cycles = 4 },
        0x96 => .{ .opcode = .STX, .mode = .ZeroPageY, .cycles = 4 },
        0x98 => .{ .opcode = .TYA, .mode = .Implied, .cycles = 2 },
        0x99 => .{ .opcode = .STA, .mode = .AbsoluteY, .cycles = 5 },
        0x9A => .{ .opcode = .TXS, .mode = .Implied, .cycles = 2 },
        0x9D => .{ .opcode = .STA, .mode = .AbsoluteX, .cycles = 5 },

        0xA0 => .{ .opcode = .LDY, .mode = .Immediate, .cycles = 2 },
        0xA1 => .{ .opcode = .LDA, .mode = .IndexedIndirect, .cycles = 6 },
        0xA2 => .{ .opcode = .LDX, .mode = .Immediate, .cycles = 2 },
        0xA4 => .{ .opcode = .LDY, .mode = .ZeroPage, .cycles = 3 },
        0xA5 => .{ .opcode = .LDA, .mode = .ZeroPage, .cycles = 3 },
        0xA6 => .{ .opcode = .LDX, .mode = .ZeroPage, .cycles = 3 },
        0xA8 => .{ .opcode = .TAY, .mode = .Implied, .cycles = 2 },
        0xA9 => .{ .opcode = .LDA, .mode = .Immediate, .cycles = 2 },
        0xAA => .{ .opcode = .TAX, .mode = .Implied, .cycles = 2 },
        0xAC => .{ .opcode = .LDY, .mode = .Absolute, .cycles = 4 },
        0xAD => .{ .opcode = .LDA, .mode = .Absolute, .cycles = 4 },
        0xAE => .{ .opcode = .LDX, .mode = .Absolute, .cycles = 4 },

        0xB0 => .{ .opcode = .BCS, .mode = .Relative, .cycles = 2 },
        0xB1 => .{ .opcode = .LDA, .mode = .IndirectIndexed, .cycles = 5 },
        0xB4 => .{ .opcode = .LDY, .mode = .ZeroPageX, .cycles = 4 },
        0xB5 => .{ .opcode = .LDA, .mode = .ZeroPageX, .cycles = 4 },
        0xB6 => .{ .opcode = .LDX, .mode = .ZeroPageY, .cycles = 4 },
        0xB8 => .{ .opcode = .CLV, .mode = .Implied, .cycles = 2 },
        0xB9 => .{ .opcode = .LDA, .mode = .AbsoluteY, .cycles = 4 },
        0xBA => .{ .opcode = .TSX, .mode = .Implied, .cycles = 2 },
        0xBC => .{ .opcode = .LDY, .mode = .AbsoluteX, .cycles = 4 },
        0xBD => .{ .opcode = .LDA, .mode = .AbsoluteX, .cycles = 4 },
        0xBE => .{ .opcode = .LDX, .mode = .AbsoluteY, .cycles = 4 },

        0xC0 => .{ .opcode = .CPY, .mode = .Immediate, .cycles = 2 },
        0xC1 => .{ .opcode = .CMP, .mode = .IndexedIndirect, .cycles = 6 },
        0xC4 => .{ .opcode = .CPY, .mode = .ZeroPage, .cycles = 3 },
        0xC5 => .{ .opcode = .CMP, .mode = .ZeroPage, .cycles = 3 },
        0xC6 => .{ .opcode = .DEC, .mode = .ZeroPage, .cycles = 5 },
        0xC8 => .{ .opcode = .INY, .mode = .Implied, .cycles = 2 },
        0xC9 => .{ .opcode = .CMP, .mode = .Immediate, .cycles = 2 },
        0xCA => .{ .opcode = .DEX, .mode = .Implied, .cycles = 2 },
        0xCC => .{ .opcode = .CPY, .mode = .Absolute, .cycles = 4 },
        0xCD => .{ .opcode = .CMP, .mode = .Absolute, .cycles = 4 },
        0xCE => .{ .opcode = .DEC, .mode = .Absolute, .cycles = 6 },

        0xD0 => .{ .opcode = .BNE, .mode = .Relative, .cycles = 2 },
        0xD1 => .{ .opcode = .CMP, .mode = .IndirectIndexed, .cycles = 5 },
        0xD5 => .{ .opcode = .CMP, .mode = .ZeroPageX, .cycles = 4 },
        0xD6 => .{ .opcode = .DEC, .mode = .ZeroPageX, .cycles = 6 },
        0xD8 => .{ .opcode = .CLD, .mode = .Implied, .cycles = 2 },
        0xD9 => .{ .opcode = .CMP, .mode = .AbsoluteY, .cycles = 4 },
        0xDD => .{ .opcode = .CMP, .mode = .AbsoluteX, .cycles = 4 },
        0xDE => .{ .opcode = .DEC, .mode = .AbsoluteX, .cycles = 7 },

        0xE0 => .{ .opcode = .CPX, .mode = .Immediate, .cycles = 2 },
        0xE1 => .{ .opcode = .SBC, .mode = .IndexedIndirect, .cycles = 6 },
        0xE4 => .{ .opcode = .CPX, .mode = .ZeroPage, .cycles = 3 },
        0xE5 => .{ .opcode = .SBC, .mode = .ZeroPage, .cycles = 3 },
        0xE6 => .{ .opcode = .INC, .mode = .ZeroPage, .cycles = 5 },
        0xE8 => .{ .opcode = .INX, .mode = .Implied, .cycles = 2 },
        0xE9 => .{ .opcode = .SBC, .mode = .Immediate, .cycles = 2 },
        0xEA => .{ .opcode = .NOP, .mode = .Implied, .cycles = 2 },
        0xEC => .{ .opcode = .CPX, .mode = .Absolute, .cycles = 4 },
        0xED => .{ .opcode = .SBC, .mode = .Absolute, .cycles = 4 },
        0xEE => .{ .opcode = .INC, .mode = .Absolute, .cycles = 6 },

        0xF0 => .{ .opcode = .BEQ, .mode = .Relative, .cycles = 2 },
        0xF1 => .{ .opcode = .SBC, .mode = .IndirectIndexed, .cycles = 5 },
        0xF5 => .{ .opcode = .SBC, .mode = .ZeroPageX, .cycles = 4 },
        0xF6 => .{ .opcode = .INC, .mode = .ZeroPageX, .cycles = 6 },
        0xF8 => .{ .opcode = .SED, .mode = .Implied, .cycles = 2 },
        0xF9 => .{ .opcode = .SBC, .mode = .AbsoluteY, .cycles = 4 },
        0xFD => .{ .opcode = .SBC, .mode = .AbsoluteX, .cycles = 4 },
        0xFE => .{ .opcode = .INC, .mode = .AbsoluteX, .cycles = 7 },

        else => error.InvalidInstruction,
    };
}

test "It can read an LDA Instruction" {
    const ins = try Instruction.decode(0xA9);
    const expected_instruction = Instruction{ .opcode = .LDA, .mode = .Immediate, .cycles = 2 };

    try testing.expectEqual(expected_instruction, ins);
}
