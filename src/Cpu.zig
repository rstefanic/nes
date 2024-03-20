const Cpu = @This();

const std = @import("std");
const Console = @import("Console.zig");
const Instruction = @import("Instruction.zig");
const testing = std.testing;

// A reference to the console so that we can read/write to memory
console: *Console,

// Track the number of cycles
cycles: u64 = 0,

// Registers
a: u8 = 0,
x: u8 = 0,
y: u8 = 0,
pc: u16 = 0,
sp: u8 = 0xFF,

// Status Register
status: packed struct(u8) {
    carry: bool = false,
    zero_result: bool = false,
    interrupt_disable: bool = false,
    decimal_mode: bool = false,
    break_interrupt: bool = false,
    _: u1 = 1,
    overflow: bool = false,
    negative_result: bool = false,
} = .{},

pub fn reset(self: *Cpu) !void {
    if (self.console.cartridge) |_| {
        const lo_byte = try self.read(0xFFFC);
        const hi_byte = try self.read(0xFFFD);
        self.pc = makeWord(hi_byte, lo_byte);
    }

    // Reset does not write the current PC and status onto the stack like the
    // other interrupts; however, it still goes through the cycles of pushing
    // the return address and status flags onto the stack (even though it
    // discards them) and copies the the value of the reset vector into the PC.
    // So a RESET takes 7 cycles and the stack has 3 fake push operations that
    // decrements the stack pointer 3 times, and interrupt disable is set.
    self.cycles = 7;
    self.sp = 0xFD;
    self.status = .{};
    self.status.interrupt_disable = true;
}

inline fn makeWord(hi_byte: u8, lo_byte: u8) u16 {
    return (@as(u16, hi_byte) << 8) + lo_byte;
}

const stack_begin: u16 = 0x0100;
const stack_end: u16 = 0x01FF;

const StackError = error{
    StackOverflow,
    StackUnderflow,
};

const CpuMemoryAccessError = error{
    InvalidMemoryAddress,
    InvalidPpuReadAddress,
    InvalidPpuWriteAddress,
    MissingCartridge,
};

fn read(self: *Cpu, address: u16) !u8 {
    if (address >= 0x0000 and address <= 0x1FFF) {
        return self.console.memory[address & 0x07FF];
    } else if (address >= 0x2000 and address <= 0x3FFF) {

        // Read a PPU register
        if (self.console.ppu) |ppu| {
            return switch (address & 0x0007) {
                2 => ppu.readPpustatus(),
                4 => ppu.readOamdata(),
                7 => ppu.readPpudata(),
                else => CpuMemoryAccessError.InvalidPpuReadAddress,
            };
        }
    } else if (address >= 0x8000 and address <= 0xFFFF) {
        if (self.console.cartridge) |cartridge| {
            return cartridge.read(address);
        } else {
            return CpuMemoryAccessError.MissingCartridge;
        }
    }

    return CpuMemoryAccessError.InvalidMemoryAddress;
}

fn write(self: *Cpu, address: u16, value: u8) !void {
    if (address >= 0x0000 and address <= 0x1FFF) {
        self.console.memory[address & 0x07FF] = value;
        return;
    } else if (address >= 0x2000 and address <= 0x3FFF) {

        // Write to a PPU register
        if (self.console.ppu) |ppu| {
            switch (address & 0x0007) {
                0 => ppu.writePpuctrl(value),
                1 => ppu.writePpumask(value),
                3 => ppu.writeOamaddr(value),
                4 => ppu.writeOamdata(value),
                5 => ppu.writePpuscroll(value),
                6 => ppu.writePpuaddr(value),
                7 => try ppu.writePpudata(value),
                else => return CpuMemoryAccessError.InvalidPpuWriteAddress,
            }
        }
        return;
    } else if (address >= 0x8000 and address <= 0xFFFF) {
        if (self.console.cartridge) |cartridge| {
            cartridge.write(address, value);
            return;
        } else {
            return CpuMemoryAccessError.MissingCartridge;
        }
    }

    return CpuMemoryAccessError.InvalidMemoryAddress;
}

fn stackPush(self: *Cpu, value: u8) !void {
    const address: u16 = stack_begin + self.sp;
    if (address > stack_end) {
        return StackError.StackOverflow;
    }

    try self.write(address, value);
    self.sp -= 1;
}

fn stackPop(self: *Cpu) !u8 {
    self.sp += 1;
    const address: u16 = stack_begin + self.sp;
    if (address <= stack_begin) {
        return StackError.StackUnderflow;
    }

    const value = try self.read(address);
    return value;
}

pub fn step(self: *Cpu) !void {
    const byte = try self.fetch();
    const ins = Instruction.decode(byte);
    try self.execute(ins);
}

inline fn fetch(self: *Cpu) !u8 {
    const address = self.pc;
    self.pc += 1;
    return try self.read(address);
}

inline fn addressPagesDiffer(address_a: u16, address_b: u16) bool {
    return (address_a & 0xFF00) != (address_b & 0xFF00);
}

fn execute(self: *Cpu, ins: Instruction) !void {
    var address: ?u16 = null;
    var additional_cycles: u2 = 0; // u2 since the maximum additional_cycles are 2

    switch (ins.mode) {
        .Implied, .Accumulator => address = null,
        .Immediate => {
            address = self.pc;
            // Take the next byte since it'll
            // be consumed by the instruction.
            _ = try self.fetch();
        },
        .Relative => {
            const offset: i8 = @bitCast(try self.fetch());
            const effective_address = if (offset < 0)
                self.pc - @as(u16, @intCast(offset * -1))
            else
                self.pc + @as(u16, @intCast(offset));

            if (addressPagesDiffer(self.pc, effective_address)) {
                additional_cycles += 1;
            }

            address = effective_address;
        },
        .Absolute => {
            const lo = try self.fetch();
            const hi = try self.fetch();
            address = makeWord(hi, lo);
        },
        .AbsoluteX => absolute_x: {
            const lo = try self.fetch();
            const hi = try self.fetch();
            const effective_address = makeWord(hi, lo);
            const abs_addr = effective_address +% self.x;
            address = abs_addr;

            // As an optimization, the 6502 will load the Absolute X value from
            // memory as soon as the first part of the addition is complete as
            // an optimization that can save a cycle. If the page boundry is
            // crossed, then the value is fetched again and an extra cycle is
            // required; however, for write operations involving this
            // addressing mode (i.e. the operations listed below), the address
            // calcluation is *always* completed since this optimization could
            // result in storing data at the wrong address. Therefore, the
            // following instructions do not incur an extra page boundry
            // penalty since it's already been accounted for.
            switch (ins.opcode) {
                .ASL, .DEC, .INC, .LSR, .ROL, .ROR, .STA => break :absolute_x,
                else => {
                    if (addressPagesDiffer(effective_address, abs_addr)) {
                        additional_cycles += 1;
                    }
                },
            }
        },
        .AbsoluteY => absolute_y: {
            const lo = try self.fetch();
            const hi = try self.fetch();
            const effective_address = makeWord(hi, lo);
            const abs_addr = effective_address +% self.y;
            address = abs_addr;

            // STA in this address mode already accounts for the cycle penalty.
            // This penalty is encoded in the instruction's cycle count.
            if (ins.opcode == .STA) {
                break :absolute_y;
            }

            if (addressPagesDiffer(effective_address, abs_addr)) {
                additional_cycles += 1;
            }
        },
        .ZeroPage => {
            const lo = try self.fetch();
            address = makeWord(0x00, lo);
        },
        .ZeroPageX => {
            const byte = try self.fetch();
            // The address here needs to wrap around
            // and always remain on the Zero Page.
            address = byte +% self.x;
        },
        .ZeroPageY => {
            const byte = try self.fetch();
            // The address here needs to wrap around
            // and always remain on the Zero Page.
            address = byte +% self.y;
        },
        .Indirect => {
            const lo = try self.fetch();
            const hi = try self.fetch();
            const indirect_addr_lo = makeWord(hi, lo);

            // There's a bug in the 6502 where the CPU won't cross a page
            // boundry when reading the indirect address; so we'll just add 1
            // to the low byte to ensure we don't cross a page boundry.
            const indirect_addr_hi = makeWord(hi, lo +% 1);
            const effective_lo = try self.read(indirect_addr_lo);
            const effective_hi = try self.read(indirect_addr_hi);
            address = makeWord(effective_hi, effective_lo);
        },
        .IndexedIndirect => {
            // The operand + X is a lookup address on the zero-page. This address contains the
            // 16 bit effective address. This lookup address should never overflow onto the
            // next page, so we'll mod the result by 0xFF to stay on the zero-page.
            const byte = try self.fetch() +% self.x;
            const lo = try self.read(makeWord(0x00, byte));
            const hi = try self.read(makeWord(0x00, byte +% 1));
            address = makeWord(hi, lo);
        },
        .IndirectIndexed => indirect_indexed: {
            // The operand is a zero-page address to a pointer. Y is added to
            // the pointer to mimic indexing giving us the effective address.
            const byte = try self.fetch();
            const lo = try self.read(makeWord(0x00, byte));
            const hi = try self.read(makeWord(0x00, byte +% 1));
            const effective_address = makeWord(hi, lo);
            const y_indexed_address = effective_address +% self.y;
            address = y_indexed_address;

            // STA in this address mode already accounts for the cycle penalty.
            // This penalty is encoded in the instruction's cycle count.
            if (ins.opcode == .STA) {
                break :indirect_indexed;
            }

            if (addressPagesDiffer(effective_address, y_indexed_address)) {
                additional_cycles += 1;
            }
        },
    }

    switch (ins.opcode) {
        .SEC => self.sec(),
        .SED => self.sed(),
        .CLC => self.clc(),
        .CLV => self.clv(),
        .SEI => self.sei(),
        .CLD => self.cld(),
        .CLI => self.cli(),
        .LDA => try self.lda(address.?),
        .LDX => try self.ldx(address.?),
        .LDY => try self.ldy(address.?),
        .STA => try self.sta(address.?),
        .STX => try self.stx(address.?),
        .STY => try self.sty(address.?),
        .TAX => self.tax(),
        .TAY => self.tay(),
        .TXA => self.txa(),
        .TYA => self.tya(),
        .TXS => try self.txs(),
        .TSX => try self.tsx(),
        .ADC => try self.adc(address.?),
        .SBC => try self.sbc(address.?),
        .AND => try self.aand(address.?),
        .ORA => try self.ora(address.?),
        .EOR => try self.eor(address.?),
        .INX => self.inx(),
        .INY => self.iny(),
        .DEX => self.dex(),
        .DEY => self.dey(),
        .INC => try self.inc(address.?),
        .DEC => try self.dec(address.?),
        .CMP => try self.cmp(address.?),
        .CPX => try self.cpx(address.?),
        .CPY => try self.cpy(address.?),
        .BIT => try self.bit(address.?),
        .LSR => try self.lsr(address),
        .ASL => try self.asl(address),
        .ROL => try self.rol(address),
        .ROR => try self.ror(address),
        .JMP => self.jmp(address.?),
        .BMI => self.bmi(address.?, &additional_cycles), // Branching instructions add a cycle if branching occurs.
        .BPL => self.bpl(address.?, &additional_cycles), // If no branching occurs, then the additional_cycles are
        .BVS => self.bvs(address.?, &additional_cycles), // set to zero since there are no page boundary penalties.
        .BVC => self.bvc(address.?, &additional_cycles),
        .BCS => self.bcs(address.?, &additional_cycles),
        .BCC => self.bcc(address.?, &additional_cycles),
        .BEQ => self.beq(address.?, &additional_cycles),
        .BNE => self.bne(address.?, &additional_cycles),
        .JSR => try self.jsr(address.?),
        .RTS => try self.rts(),
        .PHA => try self.pha(),
        .PHP => try self.php(),
        .PLA => try self.pla(),
        .PLP => try self.plp(),
        .RTI => try self.rti(),
        .BRK => try self.brk(),
        .NOP => self.nop(),

        // Illegal Opcodes
        .ALR, .ANC, .ANE, .ARR, .DCP, .ISC, .JAM, .LAS, .LAX, .LXA, .RLA, .RRA, .SAX, .SBX, .SHA, .SHX, .SHY, .SLO, .SRE, .TAS, .USBC => self.nop(),
    }

    self.cycles += ins.cycles + additional_cycles;
}

inline fn handleZeroFlagStatus(self: *Cpu, byte: u8) void {
    self.status.zero_result = byte == 0x00;
}

inline fn handleNegativeFlagStatus(self: *Cpu, byte: u8) void {
    self.status.negative_result = (byte & (1 << 7)) == (1 << 7);
}

fn sec(self: *Cpu) void {
    self.status.carry = true;
}

fn sed(self: *Cpu) void {
    self.status.decimal_mode = true;
}

fn clc(self: *Cpu) void {
    self.status.carry = false;
}

fn clv(self: *Cpu) void {
    self.status.overflow = false;
}

fn sei(self: *Cpu) void {
    self.status.interrupt_disable = true;
}

fn cld(self: *Cpu) void {
    self.status.decimal_mode = false;
}

fn cli(self: *Cpu) void {
    self.status.interrupt_disable = false;
}

fn lda(self: *Cpu, address: u16) !void {
    const value = try self.read(address);
    self.a = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn ldx(self: *Cpu, address: u16) !void {
    const value = try self.read(address);
    self.x = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn ldy(self: *Cpu, address: u16) !void {
    const value = try self.read(address);
    self.y = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn sta(self: *Cpu, address: u16) !void {
    try self.write(address, self.a);
}

fn stx(self: *Cpu, address: u16) !void {
    try self.write(address, self.x);
}

fn sty(self: *Cpu, address: u16) !void {
    try self.write(address, self.y);
}

fn tax(self: *Cpu) void {
    const value = self.a;
    self.x = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn tay(self: *Cpu) void {
    const value = self.a;
    self.y = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn txa(self: *Cpu) void {
    const value = self.x;
    self.a = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn tya(self: *Cpu) void {
    const value = self.y;
    self.a = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn txs(self: *Cpu) !void {
    const value = self.x;
    self.sp = value;
}

fn tsx(self: *Cpu) !void {
    const value = self.sp;
    self.x = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn adc(self: *Cpu, address: u16) !void {
    // ADC is kind of a pain. It adds three values together: (1) the accumulator,
    // (2) the byte specified by the operand, and (3) the carry flag. The result
    // of the operation is stored in the accumulator.

    const a = self.a;
    const value = try self.read(address);

    const carry: u1 = if (self.status.carry) 1 else 0;
    var sum: u16 = @as(u16, a) + @as(u16, value) + carry;
    const result: u8 = @truncate(sum);

    self.a = result;
    self.status.carry = sum > 0xFF;
    self.status.overflow = (a ^ sum) & (value ^ sum) & 0x80 == 0x80;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn sbc(self: *Cpu, address: u16) !void {
    // SBC is sort of the same as ADC, but with subtraction. Technically, we
    // could just take the one's complement of the operand and call ADC to get
    // the same result as we get here.

    const a = self.a;
    var value = try self.read(address);

    // For this instruction, the carry flag is interpreted as a "not borrowed"
    // flag which is just the carry flag inverted. Typically when writing 6502,
    // the setup before calling SBC typically looks like this:
    //
    //              ;SBC: A = A - M - ~C
    // SEC          ;set carry in preparation
    // LDA #15      ;load 15 into the accumulator
    // SBC #8       ;subtract 8 -> now 7 in accumulator
    //
    // We get the "not borrowed" flag by flipping the carry flag.
    // If it's clear, it means a borrow did happen.
    // If it's set, it means a borrow did not happen.
    const carry: u1 = if (self.status.carry) 1 else 0;
    const diff = @subWithOverflow(a, value);
    const result = @subWithOverflow(diff[0], ~carry);

    self.a = result[0];
    self.status.carry = !(result[1] == 1 or diff[1] == 1);
    self.status.overflow = (a ^ value) & (a ^ result[0]) & 0x80 == 0x80;
    self.handleZeroFlagStatus(result[0]);
    self.handleNegativeFlagStatus(result[0]);
}

fn aand(self: *Cpu, address: u16) !void {
    const a = self.a;
    const value = try self.read(address);
    const result = a & value;

    self.a = result;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn ora(self: *Cpu, address: u16) !void {
    const a = self.a;
    const value = try self.read(address);
    const result = a | value;

    self.a = result;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn eor(self: *Cpu, address: u16) !void {
    const a = self.a;
    const value = try self.read(address);
    const result = a ^ value;

    self.a = result;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn inx(self: *Cpu) void {
    const result = self.x +% 1;
    self.x = result;

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn iny(self: *Cpu) void {
    const result = self.y +% 1;
    self.y = result;

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn dex(self: *Cpu) void {
    const result = self.x -% 1;
    self.x = result;

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn dey(self: *Cpu) void {
    const result = self.y -% 1;
    self.y = result;

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn inc(self: *Cpu, address: u16) !void {
    const value = try self.read(address);
    const result = value +% 1;
    try self.write(address, result);

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn dec(self: *Cpu, address: u16) !void {
    const value = try self.read(address);
    const result = value -% 1;
    try self.write(address, result);

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn cmp(self: *Cpu, address: u16) !void {
    const value = try self.read(address);
    const result = self.a -% value;

    self.status.carry = self.a >= value;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn cpx(self: *Cpu, address: u16) !void {
    const value = try self.read(address);
    const result = self.x -% value;

    self.status.carry = self.x >= value;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn cpy(self: *Cpu, address: u16) !void {
    const value = try self.read(address);
    const result = self.y -% value;

    self.status.carry = self.y >= value;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn bit(self: *Cpu, address: u16) !void {
    const a = self.a;
    const value = try self.read(address);
    const result = a & value;

    // For BIT, the zero flag is handled as normal but the overflow and negative
    // flags are set/cleared based on the value from memory and not the result.
    self.handleZeroFlagStatus(result);
    self.status.overflow = (value & (1 << 6)) == (1 << 6);
    self.status.negative_result = (value & (1 << 7)) == (1 << 7);
}

/// LSR allows the operand to either be a value in memory or the accumulator.
/// Passing NULL as `address` will perform LSR on the accumulator.
fn lsr(self: *Cpu, address: ?u16) !void {
    const value = if (address) |addr| try self.read(addr) else self.a;
    const shifted_bit = value & 1;
    const result = value >> 1;

    self.status.carry = shifted_bit == 1;
    self.handleZeroFlagStatus(result);
    self.status.negative_result = false; // Since the 7th bit will always be 0 from SHL

    if (address) |addr| {
        try self.write(addr, result);
    } else {
        self.a = result;
    }
}

/// ASL allows the operand to either be a value in memory or the accumulator.
/// Passing NULL as `address` will perform ASL on the accumulator.
fn asl(self: *Cpu, address: ?u16) !void {
    const value = if (address) |addr| try self.read(addr) else self.a;
    const shifted_bit: u1 = if ((value & 0x80) == 0x80) 1 else 0;
    const result = value << 1;

    self.status.carry = shifted_bit == 1;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);

    if (address) |addr| {
        try self.write(addr, result);
    } else {
        self.a = result;
    }
}

fn rol(self: *Cpu, address: ?u16) !void {
    const value = if (address) |addr| try self.read(addr) else self.a;
    const shifted_bit: u1 = if ((value & 0x80) == 0x80) 1 else 0;
    const carry: u1 = if (self.status.carry) 1 else 0;
    const result = (value << 1) | carry;

    self.status.carry = shifted_bit == 1;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);

    if (address) |addr| {
        try self.write(addr, result);
    } else {
        self.a = result;
    }
}

fn ror(self: *Cpu, address: ?u16) !void {
    const value = if (address) |addr| try self.read(addr) else self.a;
    const shifted_bit = value & 1;
    const carry: u8 = if (self.status.carry) 0x80 else 0x00;
    const result = (value >> 1) | carry;

    self.status.carry = shifted_bit == 1;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);

    if (address) |addr| {
        try self.write(addr, result);
    } else {
        self.a = result;
    }
}

fn jmp(self: *Cpu, address: u16) void {
    self.pc = address;
}

fn bmi(self: *Cpu, address: u16, additional_cycles: *u2) void {
    if (self.status.negative_result) {
        self.pc = address;
        additional_cycles.* += 1;
    } else {
        additional_cycles.* = 0;
    }
}

fn bpl(self: *Cpu, address: u16, additional_cycles: *u2) void {
    if (!self.status.negative_result) {
        self.pc = address;
        additional_cycles.* += 1;
    } else {
        additional_cycles.* = 0;
    }
}

fn bvs(self: *Cpu, address: u16, additional_cycles: *u2) void {
    if (self.status.overflow) {
        self.pc = address;
        additional_cycles.* += 1;
    } else {
        additional_cycles.* = 0;
    }
}

fn bvc(self: *Cpu, address: u16, additional_cycles: *u2) void {
    if (!self.status.overflow) {
        self.pc = address;
        additional_cycles.* += 1;
    } else {
        additional_cycles.* = 0;
    }
}

fn bcs(self: *Cpu, address: u16, additional_cycles: *u2) void {
    if (self.status.carry) {
        self.pc = address;
        additional_cycles.* += 1;
    } else {
        additional_cycles.* = 0;
    }
}

fn bcc(self: *Cpu, address: u16, additional_cycles: *u2) void {
    if (!self.status.carry) {
        self.pc = address;
        additional_cycles.* += 1;
    } else {
        additional_cycles.* = 0;
    }
}

fn beq(self: *Cpu, address: u16, additional_cycles: *u2) void {
    if (self.status.zero_result) {
        self.pc = address;
        additional_cycles.* += 1;
    } else {
        additional_cycles.* = 0;
    }
}

fn bne(self: *Cpu, address: u16, additional_cycles: *u2) void {
    if (!self.status.zero_result) {
        self.pc = address;
        additional_cycles.* += 1;
    } else {
        additional_cycles.* = 0;
    }
}

fn jsr(self: *Cpu, address: u16) !void {
    // Push the address of the JSR instruction + 2 (i.e. the third byte of the
    // JSR instruction). RTS pulls the address from the stack and adds one to it.
    const return_address = self.pc - 1;
    try self.stackPush(@truncate(return_address >> 8));
    try self.stackPush(@truncate(return_address & 0x00FF));

    self.pc = address;
}

fn rts(self: *Cpu) !void {
    const lo = try self.stackPop();
    const hi = try self.stackPop();
    const stack_addr = makeWord(hi, lo);

    self.pc = stack_addr + 1;
}

fn pha(self: *Cpu) !void {
    try self.stackPush(self.a);
}

fn php(self: *Cpu) !void {
    // NOTE: The break flag is always set whenever the transfer was caused
    // by software (e.g. PHP or BRK). See here for more information:
    // https://www.masswerk.at/6502/6502_instruction_set.html#break-flag
    const status = @as(u8, @bitCast(self.status)) | 0b00010000;
    try self.stackPush(status);
}

fn pla(self: *Cpu) !void {
    const result = try self.stackPop();
    self.a = result;

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn plp(self: *Cpu) !void {
    // NOTE: The break flag is masked and cleared whenever transferred from the stack
    // to the status register from a PLP/RTI call. See here for more information:
    // https://www.masswerk.at/6502/6502_instruction_set.html#break-flag
    //
    // Bit 5 of the status register is always 1, so we want to ensure it's set here.
    // Something that wasn't pushed as a status register could be popped here.
    const status: u8 = (try self.stackPop() & 0b11101111) | 0b00100000;
    self.status = @bitCast(status);
}

fn rti(self: *Cpu) !void {
    try self.plp();

    const lo = try self.stackPop();
    const hi = try self.stackPop();
    const stack_addr = makeWord(hi, lo);
    self.pc = stack_addr;
}

fn brk(self: *Cpu) !void {
    // NOTE: The break flag is always set whenever the transfer was caused
    // by software (e.g. PHP or BRK). See here for more information:
    // https://www.masswerk.at/6502/6502_instruction_set.html#break-flag
    const status = @as(u8, @bitCast(self.status)) | 0b00010000;
    try self.stackPush(status);

    // Push the current pc in little endian format
    try self.stackPush(@truncate(self.pc >> 8));
    try self.stackPush(@truncate(self.pc & 0x00FF));

    // Read from the IRQ/BRK vector
    const lo_byte = try self.read(0xFFFE);
    const hi_byte = try self.read(0xFFFF);
    self.pc = makeWord(hi_byte, lo_byte);
}

fn nop(self: *Cpu) void {
    _ = self;
}

test "SEC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };

    try cpu.write(0x0000, 0x38);
    try cpu.step();

    try testing.expectEqual(cpu.status.carry, true);
}

test "SED" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };

    try cpu.write(0x0000, 0xF8);
    try cpu.step();

    try testing.expectEqual(cpu.status.decimal_mode, true);
}

test "CLC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.carry = true;

    try cpu.write(0x0000, 0x18);
    try cpu.step();

    try testing.expectEqual(cpu.status.carry, false);
}

test "CLV" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.overflow = true;

    try cpu.write(0x0000, 0xB8);
    try cpu.step();

    try testing.expectEqual(cpu.status.overflow, false);
}

test "SEI" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };

    try cpu.write(0x0000, 0x78);
    try cpu.step();

    try testing.expectEqual(cpu.status.interrupt_disable, true);
}

test "CLD" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.decimal_mode = true;

    try cpu.write(0x0000, 0xD8);
    try cpu.step();

    try testing.expectEqual(cpu.status.decimal_mode, false);
}

test "CLI" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.interrupt_disable = true;

    try cpu.write(0x0000, 0x58);
    try cpu.step();

    try testing.expectEqual(cpu.status.interrupt_disable, false);
}

test "LDA Immediate" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try cpu.write(0x0000, 0xA9); // LDA Instruction
    try cpu.write(0x0001, expected_byte); // Operand
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
    // The zero flag should be clear when setting the A register to 0xFF
    try testing.expectEqual(false, cpu.status.zero_result);
}

test "LDA ZeroPage" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try cpu.write(0x0000, 0xA5); // LDA ZeroPage Instruction
    try cpu.write(0x0001, 0x06); // Operand
    try cpu.write(0x0006, expected_byte);
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "LDA ZeroPage,X" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try cpu.write(0x0000, 0xB5); // LDA ZeroPage,X Instruction
    try cpu.write(0x0001, 0x06); // Operand
    try cpu.write(0x0007, expected_byte);
    cpu.x = 1; // X offset
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "LDA Absolute" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try cpu.write(0x0000, 0xAD); // LDA Absolute Instruction
    try cpu.write(0x0001, 0xFF); // Operand low byte
    try cpu.write(0x0002, 0x01); // Operand high byte
    try cpu.write(0x01FF, expected_byte);
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
    try testing.expectEqual(@as(u64, 4), cpu.cycles);
}

test "LDA Absolute,X" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try cpu.write(0x0000, 0xBD); // LDA Absolute,X Instruction
    try cpu.write(0x0001, 0xFE); // Operand low byte
    try cpu.write(0x0002, 0x01); // Operand high byte
    try cpu.write(0x01FF, 0xFF);
    cpu.x = 1; // X index to be added to get the effective address
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
    try testing.expectEqual(@as(u64, 4), cpu.cycles);
}

test "LDA Absolute,X add additional cycle for crossing page boundaries" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.x = 1; // X index to be added to get the effective address

    try cpu.write(0x0000, 0xBD); // LDA Absolute,X Instruction
    try cpu.write(0x0001, 0xFF); // Operand low byte
    try cpu.write(0x0002, 0x00); // Operand high byte
    try cpu.write(0x0100, 0xFF);
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
    try testing.expectEqual(@as(u64, 5), cpu.cycles);
}

test "LDA Absolute,Y" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try cpu.write(0x0000, 0xB9); // LDA Absolute,Y Instruction
    try cpu.write(0x0001, 0xFD); // Operand low byte
    try cpu.write(0x0002, 0x01); // Operand high byte
    try cpu.write(0x01FF, 0xFF);
    cpu.y = 2; // Y index to be added to get the effective address
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "LDA IndexedIndirect" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try cpu.write(0x0000, 0xA1); // LDA IndirectIndexed Instruction
    try cpu.write(0x0001, 0x70); // Operand

    // Contents at 0x0075 which contains the effective address
    try cpu.write(0x0075, 0xFF); // Low Byte
    try cpu.write(0x0076, 0x01); // High Byte
    try cpu.write(0x01FF, expected_byte);
    cpu.x = 0x05; // X value to be added to the effective address
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "LDA IndirectIndexed" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try cpu.write(0x0000, 0xB1); // LDA IndirectIndexed Instruction
    try cpu.write(0x0001, 0x70); // Operand

    // Contents at 0x0070 which contains the effective address
    try cpu.write(0x0070, 0xEF); // Low Byte
    try cpu.write(0x0071, 0x01); // High Byte
    try cpu.write(0x01FF, expected_byte);
    cpu.y = 0x10; // Y value to be added to the effective address
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "LDX ZeroPage,Y" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try cpu.write(0x0000, 0xB6); // LDX ZeroPage,Y Instruction
    try cpu.write(0x0001, 0x06); // Operand
    try cpu.write(0x0007, expected_byte);
    cpu.y = 1; // Y offset
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.x);
}

test "LDY" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try cpu.write(0x0000, 0xAC); // LDY Absolute Instruction
    try cpu.write(0x0001, 0xFF); // Operand low byte
    try cpu.write(0x0002, 0x01); // Operand high byte
    try cpu.write(0x01FF, expected_byte);
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.y);
}

test "STA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.a = expected_byte;

    try cpu.write(0x0000, 0x8D); // STA Absolute Instruction
    try cpu.write(0x0001, 0x00);
    try cpu.write(0x0002, 0x10);
    try cpu.step();

    try testing.expectEqual(expected_byte, try cpu.read(0x1000));
}

test "STX" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.x = expected_byte;

    try cpu.write(0x0000, 0x8E); // STX Absolute Instruction
    try cpu.write(0x0001, 0x00);
    try cpu.write(0x0002, 0x10);
    try cpu.step();

    try testing.expectEqual(expected_byte, try cpu.read(0x1000));
}

test "STY" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.y = expected_byte;

    try cpu.write(0x0000, 0x8C); // STY Absolute Instruction
    try cpu.write(0x0001, 0x00);
    try cpu.write(0x0002, 0x10);
    try cpu.step();

    try testing.expectEqual(expected_byte, try cpu.read(0x1000));
}

test "TAX" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.a = expected_byte;

    try cpu.write(0x0000, 0xAA); // TAX Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.x);
}

test "TAY" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.a = expected_byte;

    try cpu.write(0x0000, 0xA8); // TAY Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.y);
}

test "TXA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.x = expected_byte;

    try cpu.write(0x0000, 0x8A); // TXA Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "TYA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.y = expected_byte;

    try cpu.write(0x0000, 0x98); // TYA Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "TXS" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.x = 0xFF;

    try cpu.write(0x0000, 0x9A); // TXS Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.sp);
}

test "TSX" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.sp = 0x02;

    try cpu.write(0x0000, 0xBA); // TSX Instruction
    try cpu.step();

    try testing.expectEqual(cpu.sp, cpu.x);
}

test "ADC Immediate" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_sum: u8 = 0xFF;
    cpu.a = 0xFE;

    try cpu.write(0x0000, 0x69); // ADC Immediate
    try cpu.write(0x0001, 0x01);
    try cpu.step();

    try testing.expectEqual(expected_sum, cpu.a);
}

test "ADC Add two signed integers" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_sum: u8 = 0x73; // +115 in hex
    cpu.a = 0xFB; // -5 in hex

    try cpu.write(0x0000, 0x6D); // ADC Absolute instruction
    try cpu.write(0x0001, 0x00);
    try cpu.write(0x0002, 0x01); // At address $0100
    try cpu.write(0x0100, 0x78); // +120 in hex
    try cpu.step();

    try testing.expectEqual(expected_sum, cpu.a);
}

test "ADC signed addition with overflow" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_sum: u8 = 0x7B; // +123 in hex
    cpu.a = 0xFB; // -5 in hex

    try cpu.write(0x0000, 0x6D); // ADC Absolute instruction
    try cpu.write(0x0001, 0x00);
    try cpu.write(0x0002, 0x01); // At address $0100
    try cpu.write(0x0100, 0x80); // -128 in hex
    try cpu.step();

    try testing.expectEqual(expected_sum, cpu.a);
    try testing.expectEqual(true, cpu.status.overflow);
}

test "SBC signed subtraction" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.carry = true;
    const expected_sum: u8 = 0xFD; // -3 in hex
    cpu.a = 0xFE; // -2 in hex

    try cpu.write(0x0000, 0xED); // SBC Absolute instruction
    try cpu.write(0x0001, 0x00);
    try cpu.write(0x0002, 0x01); // At address $0100
    try cpu.write(0x0100, 0x01); // +1 in hex
    try cpu.step();

    try testing.expectEqual(expected_sum, cpu.a);
}

test "SBC signed subtraction with overflow" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.carry = true;
    const expected_sum: u8 = 0x80; // -128 in hex
    cpu.a = 0x7F; // 127 in hex

    try cpu.write(0x0000, 0xED); // SBC Absolute instruction
    try cpu.write(0x0001, 0x00);
    try cpu.write(0x0002, 0x01); // At address $0100
    try cpu.write(0x0100, 0xFF); // -1 in hex
    try cpu.step();

    try testing.expectEqual(expected_sum, cpu.a);
    try testing.expectEqual(true, cpu.status.overflow);
}

test "AND" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_result: u8 = 0x02;
    cpu.a = 0xFF;

    try cpu.write(0x0000, 0x29); // AND Immediate instruction
    try cpu.write(0x0001, 0x02);
    try cpu.step();

    try testing.expectEqual(expected_result, cpu.a);
    try testing.expectEqual(false, cpu.status.zero_result);
    try testing.expectEqual(false, cpu.status.negative_result);
}

test "ORA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_result: u8 = 0x01;
    cpu.a = 0x01;

    try cpu.write(0x0000, 0x09); // ORA Immediate instruction
    try cpu.write(0x0001, 0x00);
    try cpu.step();

    try testing.expectEqual(expected_result, cpu.a);
    try testing.expectEqual(false, cpu.status.zero_result);
    try testing.expectEqual(false, cpu.status.negative_result);
}

test "EOR" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_result: u8 = 0x00;
    cpu.a = 0x01;

    try cpu.write(0x0000, 0x49); // EOR Immediate instruction
    try cpu.write(0x0001, 0x01);
    try cpu.step();

    try testing.expectEqual(expected_result, cpu.a);
    try testing.expectEqual(true, cpu.status.zero_result);
    try testing.expectEqual(false, cpu.status.negative_result);
}

test "INX allows for overflow" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.x = 0xFF;

    try cpu.write(0x0000, 0xE8); // INX instruction
    try cpu.step();

    try testing.expect(cpu.status.zero_result);
}

test "INY" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_value: u8 = 0x01;
    cpu.y = 0x00;

    try cpu.write(0x0000, 0xC8); // INY instruction
    try cpu.step();

    try testing.expectEqual(expected_value, cpu.y);
}

test "DEX allows for overflow" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_value: u8 = 0xFF;
    cpu.x = 0x00;

    try cpu.write(0x0000, 0xCA); // DEX instruction
    try cpu.step();

    try testing.expectEqual(expected_value, cpu.x);
}

test "DEY allows for overflow" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_value: u8 = 0x00;
    cpu.y = 0x01;

    try cpu.write(0x0000, 0x88); // DEY instruction
    try cpu.step();

    try testing.expectEqual(expected_value, cpu.y);
    try testing.expect(cpu.status.zero_result);
}

test "INC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_value: u8 = 0x00;

    try cpu.write(0x0000, 0xE6); // INC ZeroPage instruction
    try cpu.write(0x0001, 0xFF);
    try cpu.write(0x00FF, 0xFF); // Value at ZeroPage $FF
    try cpu.step();

    try testing.expectEqual(expected_value, try cpu.read(0x00FF));
    try testing.expect(cpu.status.zero_result);
}

test "DEC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_value: u8 = 0xFF;

    try cpu.write(0x0000, 0xCE); // DEC Absolute instruction
    try cpu.write(0x0001, 0xFF);
    try cpu.write(0x0002, 0x00);
    try cpu.write(0x00FF, 0x00); // Value at ZeroPage $FF
    try cpu.step();

    try testing.expectEqual(expected_value, try cpu.read(0x00FF));
}

test "CMP" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.a = 0xFF;

    try cpu.write(0x0000, 0xC9); // CMP Immediate instruction
    try cpu.write(0x0001, 0xFF);
    try cpu.step();

    try testing.expect(cpu.status.carry);
    try testing.expect(cpu.status.zero_result);
}

test "CPX" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.x = 0x00;

    try cpu.write(0x0000, 0xE0); // CPX Immediate instruction
    try cpu.write(0x0001, 0xFF);
    try cpu.step();

    try testing.expect(!cpu.status.carry);
}

test "CPY" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.y = 0x10;

    try cpu.write(0x0000, 0xCC); // CPY Absolute instruction
    try cpu.write(0x0001, 0x00);
    try cpu.write(0x0002, 0x01);
    try cpu.write(0x0100, 0x0F);
    try cpu.step();

    try testing.expect(cpu.status.carry);
}

test "BIT" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.a = 0x02;

    try cpu.write(0x0000, 0x2C); // BIT Absolute Instruction
    try cpu.write(0x0001, 0x01);
    try cpu.write(0x0002, 0x0C);
    try cpu.write(0x0C01, 0x8F);
    try cpu.step();

    try testing.expect(cpu.status.negative_result);
    try testing.expect(!cpu.status.zero_result);
    try testing.expect(!cpu.status.overflow);
}

test "LSR" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0x7F;
    cpu.a = 0xFF;

    try cpu.write(0x0000, 0x4A); // LSR Accumulator Instruction
    try cpu.step();

    try testing.expect(!cpu.status.zero_result);
    try testing.expect(!cpu.status.negative_result);
    try testing.expect(cpu.status.carry);
    try testing.expectEqual(expected_byte, cpu.a);
}

test "LSR Zero Flag is set if the result is zero" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0x00;
    cpu.a = 0x01;

    try cpu.write(0x0000, 0x4A); // LSR Accumulator Instruction
    try cpu.step();

    try testing.expect(cpu.status.zero_result);
    try testing.expect(!cpu.status.negative_result);
    try testing.expect(cpu.status.carry);
    try testing.expectEqual(expected_byte, cpu.a);
}

test "ASL" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0x00;
    cpu.a = 0x80;

    try cpu.write(0x0000, 0x0A); // ASL Accumulator Instruction
    try cpu.step();

    try testing.expect(cpu.status.zero_result);
    try testing.expect(!cpu.status.negative_result);
    try testing.expect(cpu.status.carry);
    try testing.expectEqual(expected_byte, cpu.a);
}

test "ROL" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0x80;
    cpu.a = 0xC0;

    try cpu.write(0x0000, 0x2A); // ROL Accumulator Instruction
    try cpu.step();

    try testing.expect(!cpu.status.zero_result);
    try testing.expect(cpu.status.carry);
    try testing.expectEqual(expected_byte, cpu.a);
}

test "ROR" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0x81;
    cpu.status.carry = true;

    try cpu.write(0x0000, 0x66); // ROR Zeropage Instruction
    try cpu.write(0x0001, 0xFF);
    try cpu.write(0x00FF, 0x02);
    try cpu.step();

    try testing.expect(!cpu.status.zero_result);
    try testing.expect(!cpu.status.carry);
    try testing.expectEqual(expected_byte, try cpu.read(0x00FF));
}

test "JMP Absolute" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_pc: u16 = 0x4030;

    try cpu.write(0x0000, 0x4C); // JMP Absolute
    try cpu.write(0x0001, 0x30);
    try cpu.write(0x0002, 0x40);
    try cpu.step();

    try testing.expectEqual(expected_pc, cpu.pc);
}

test "JMP Indirect" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_pc: u16 = 0x2010;

    try cpu.write(0x0000, 0x6C); // JMP Indirect
    try cpu.write(0x0001, 0xFE); // Indirect low byte
    try cpu.write(0x0002, 0x01); // Indirect high byte
    try cpu.write(0x01FE, 0x10);
    try cpu.write(0x01FF, 0x20);
    try cpu.step();

    try testing.expectEqual(expected_pc, cpu.pc);
}

test "BMI" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0081;
    cpu.status.negative_result = true;

    try cpu.write(0x0000, 0x30); // BMI Instruction
    try cpu.write(0x0001, 0x7F);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BMI branching adds an additional cycle" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0080;
    cpu.pc = 0x000E;
    cpu.status.negative_result = true;

    try cpu.write(0x000E, 0x30); // BMI Instruction
    try cpu.write(0x000F, 0x70);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
    try testing.expectEqual(@as(u64, 3), cpu.cycles);
}

test "BMI branching with a page boundary adds two additional cycles" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0100;
    cpu.pc = 0x00EE;
    cpu.status.negative_result = true;

    try cpu.write(0x00EE, 0x30); // BMI Instruction
    try cpu.write(0x00EF, 0x10);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
    try testing.expectEqual(@as(u64, 4), cpu.cycles);
}

test "BPL" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.negative_result = true;

    try cpu.write(0x0000, 0x30); // BPL Instruction
    try cpu.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BVS" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.overflow = true;

    try cpu.write(0x0000, 0x70); // BVS Instruction
    try cpu.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BVC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.overflow = false;

    try cpu.write(0x0000, 0x50); // BVC Instruction
    try cpu.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BCS" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.carry = true;

    try cpu.write(0x0000, 0xB0); // BCS Instruction
    try cpu.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BCC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.carry = false;

    try cpu.write(0x0000, 0x90); // BCC Instruction
    try cpu.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BEQ" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.zero_result = true;

    try cpu.write(0x0000, 0xF0); // BEQ Instruction
    try cpu.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BNE" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.zero_result = false;

    try cpu.write(0x0000, 0xD0); // BNE Instruction
    try cpu.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "JSR" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x3000;
    const expected_stack_address: u16 = 0x1002;
    cpu.pc = 0x1000;

    try cpu.write(0x1000, 0x20); // JSR Instruction
    try cpu.write(0x1001, 0x00);
    try cpu.write(0x1002, 0x30);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);

    // Verify the address was pushed to the stack correctly
    const lo = try cpu.stackPop();
    const hi = try cpu.stackPop();
    const stack_addr = makeWord(hi, lo);
    try testing.expectEqual(expected_stack_address, stack_addr);
}

test "RTS" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x1001;
    try cpu.stackPush(0x10);
    try cpu.stackPush(0x00);

    try cpu.write(0x0000, 0x60); // RTS Instruction
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "PHA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.a = 0xFF;

    try cpu.write(0x0000, 0x48); // PHA Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, try cpu.stackPop());
}

test "PHP" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0b00110001;
    cpu.status.carry = true;

    try cpu.write(0x0000, 0x08); // PHP Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, try cpu.stackPop());
}

test "PLA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    try cpu.stackPush(expected_byte);

    try cpu.write(0x0000, 0x68); // PLA Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "PLP" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0x23; // We always expect bit 5 to be set, even though we're popping the value 0x03.
    try cpu.stackPush(0x03);

    try cpu.write(0x0000, 0x28); // PLP Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, @as(u8, @bitCast(cpu.status)));
    try testing.expect(cpu.status.carry);
    try testing.expect(cpu.status.zero_result);
    try testing.expect(!cpu.status.interrupt_disable);
    try testing.expect(!cpu.status.decimal_mode);
    try testing.expect(!cpu.status.break_interrupt);
    try testing.expect(!cpu.status.overflow);
    try testing.expect(!cpu.status.negative_result);
}

test "RTI" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_stack_reg: u8 = 0x23; // Bit 5 will always be set on the Status Register, even though we're popping 0x03.
    const expected_pc_address: u16 = 0x1000;
    try cpu.stackPush(0x10); // First push the address in little endian
    try cpu.stackPush(0x00);
    try cpu.stackPush(0x03); // Next push the status register

    try cpu.write(0x0000, 0x40); // RTI Instruction
    try cpu.step();

    try testing.expectEqual(expected_stack_reg, @as(u8, @bitCast(cpu.status)));
    try testing.expectEqual(expected_pc_address, cpu.pc);
}

test "NOP" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0001;

    try cpu.write(0x0000, 0xEA); // NOP Instruction
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "Accessing a RAM with mirrors between $0000 and $07FF" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };

    try cpu.write(0x1000, 0xFF);
    try testing.expectEqual(@as(u8, 0xFF), try cpu.read(0x0000));
}
