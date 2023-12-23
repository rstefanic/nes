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
    _: u1 = 0,
    overflow: bool = false,
    negative_result: bool = false,
} = .{},

pub fn reset(self: *Cpu) !void {
    if (self.console.cartridge) |_| {
        const lo_byte = try self.console.read(0xFFFC);
        const hi_byte = try self.console.read(0xFFFD);
        self.pc = makeWord(hi_byte, lo_byte);
    }
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

fn stackPush(self: *Cpu, value: u8) !void {
    const address: u16 = stack_begin + self.sp;
    if (address > stack_end) {
        return StackError.StackOverflow;
    }

    try self.console.write(address, value);
    self.sp -= 1;
}

fn stackPop(self: *Cpu) !u8 {
    self.sp += 1;
    const address: u16 = stack_begin + self.sp;
    if (address <= stack_begin) {
        return StackError.StackUnderflow;
    }

    const value = try self.console.read(address);
    return value;
}

pub fn step(self: *Cpu) !void {
    const byte = try self.fetch();
    const ins = try Instruction.decode(byte);
    try self.execute(ins);
}

inline fn fetch(self: *Cpu) !u8 {
    const address = self.pc;
    self.pc += 1;
    return try self.console.read(address);
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
        .AbsoluteX => {
            const lo = try self.fetch();
            const hi = try self.fetch();
            const effective_address = makeWord(hi, lo);
            if (addressPagesDiffer(effective_address, effective_address + self.x)) {
                additional_cycles += 1;
            }

            address = effective_address + self.x;
        },
        .AbsoluteY => {
            const lo = try self.fetch();
            const hi = try self.fetch();
            const effective_address = makeWord(hi, lo);
            if (addressPagesDiffer(effective_address, effective_address + self.y)) {
                additional_cycles += 1;
            }

            address = effective_address + self.y;
        },
        .ZeroPage => {
            const lo = try self.fetch();
            address = makeWord(0x00, lo);
        },
        .ZeroPageX => {
            const byte = try self.fetch();
            // The address here needs to wrap around
            // and always remain on the Zero Page.
            address = (byte + self.x) % 255;
        },
        .ZeroPageY => {
            const byte = try self.fetch();
            // The address here needs to wrap around
            // and always remain on the Zero Page.
            address = (byte + self.y) % 255;
        },
        .Indirect => {
            const lo = try self.fetch();
            const hi = try self.fetch();
            const indirect_addr = makeWord(hi, lo);
            const effective_lo = try self.console.read(indirect_addr);
            const effective_hi = try self.console.read(indirect_addr + 1);
            address = makeWord(effective_hi, effective_lo);
        },
        .IndexedIndirect => {
            // The operand + X is a lookup address on the zero-page. This address contains the
            // 16 bit effective address. This lookup address should never overflow onto the
            // next page, so we'll mod the result by 0xFF to stay on the zero-page.
            const byte = (try self.fetch() + self.x) % 0xFF;
            const lo = try self.console.read(makeWord(0x00, byte));
            const hi = try self.console.read(makeWord(0x00, byte + 1));
            address = makeWord(hi, lo);
        },
        .IndirectIndexed => {
            // The operand is a zero-page address to a pointer. Y is added to
            // the pointer to mimic indexing giving us the effective address.
            const byte = try self.fetch();
            const lo = try self.console.read(makeWord(0x00, byte));
            const hi = try self.console.read(makeWord(0x00, byte + 1));
            const effective_address = makeWord(hi, lo);
            if (addressPagesDiffer(effective_address, effective_address + self.y)) {
                additional_cycles += 1;
            }

            address = effective_address + self.y;
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
    const value = try self.console.read(address);
    self.a = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn ldx(self: *Cpu, address: u16) !void {
    const value = try self.console.read(address);
    self.x = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn ldy(self: *Cpu, address: u16) !void {
    const value = try self.console.read(address);
    self.y = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

fn sta(self: *Cpu, address: u16) !void {
    try self.console.write(address, self.a);
}

fn stx(self: *Cpu, address: u16) !void {
    try self.console.write(address, self.x);
}

fn sty(self: *Cpu, address: u16) !void {
    try self.console.write(address, self.y);
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
    try self.stackPush(self.x);
}

fn tsx(self: *Cpu) !void {
    const value = try self.stackPop();
    self.x = value;
}

fn adc(self: *Cpu, address: u16) !void {
    // ADC is kind of a pain. It adds three values together: (1) the accumulator,
    // (2) the byte specified by the operand, and (3) the carry flag. The result
    // of the operation is stored in the accumulator.

    const a = self.a;
    const value = try self.console.read(address);

    const carry: u1 = if (self.status.carry) 1 else 0;
    var sum: u16 = @as(u16, a) + @as(u16, value) + carry;
    const result: u8 = @truncate(sum);

    self.a = result;
    self.status.carry = sum > 0xFF;
    self.status.overflow = (a ^ sum) & (value ^ sum) & 0x80 > 0;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn sbc(self: *Cpu, address: u16) !void {
    // SBC is sort of the same as ADC, but with subtraction. Technically, we
    // could just take the one's complement of the operand and call ADC to get
    // the same result as we get here.

    const a = self.a;
    var value = try self.console.read(address);

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
    var diff: u16 = @as(u16, a) -% @as(u16, value) - ~carry;
    const result: u8 = @truncate(diff);

    self.a = result;
    self.status.carry = diff > 0xFF;
    self.status.overflow = (a ^ diff) & (value ^ diff) & 0x80 <= 0;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn aand(self: *Cpu, address: u16) !void {
    const a = self.a;
    const value = try self.console.read(address);
    const result = a & value;

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn ora(self: *Cpu, address: u16) !void {
    const a = self.a;
    const value = try self.console.read(address);
    const result = a | value;

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn eor(self: *Cpu, address: u16) !void {
    const a = self.a;
    const value = try self.console.read(address);
    const result = a ^ value;

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
    const value = try self.console.read(address);
    const result = value +% 1;
    try self.console.write(address, result);

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn dec(self: *Cpu, address: u16) !void {
    const value = try self.console.read(address);
    const result = value -% 1;
    try self.console.write(address, result);

    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn cmp(self: *Cpu, address: u16) !void {
    const value = try self.console.read(address);
    const result = self.a -% value;

    self.status.carry = self.a >= value;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn cpx(self: *Cpu, address: u16) !void {
    const value = try self.console.read(address);
    const result = self.x -% value;

    self.status.carry = self.x >= value;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn cpy(self: *Cpu, address: u16) !void {
    const value = try self.console.read(address);
    const result = self.y -% value;

    self.status.carry = self.y >= value;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);
}

fn bit(self: *Cpu, address: u16) !void {
    const a = self.a;
    const value = try self.console.read(address);
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
    const value = if (address) |addr| try self.console.read(addr) else self.a;
    const shifted_bit = value & 1;
    const result = value >> 1;

    self.status.carry = shifted_bit == 1;
    self.handleZeroFlagStatus(result);
    self.status.negative_result = false; // Since the 7th bit will always be 0 from SHL

    if (address) |addr| {
        try self.console.write(addr, result);
    } else {
        self.a = result;
    }
}

/// ASL allows the operand to either be a value in memory or the accumulator.
/// Passing NULL as `address` will perform ASL on the accumulator.
fn asl(self: *Cpu, address: ?u16) !void {
    const value = if (address) |addr| try self.console.read(addr) else self.a;
    const shifted_bit: u1 = if ((value & 0x80) == 0x80) 1 else 0;
    const result = value << 1;

    self.status.carry = shifted_bit == 1;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);

    if (address) |addr| {
        try self.console.write(addr, result);
    } else {
        self.a = result;
    }
}

fn rol(self: *Cpu, address: ?u16) !void {
    const value = if (address) |addr| try self.console.read(addr) else self.a;
    const shifted_bit: u1 = if ((value & 0x80) == 0x80) 1 else 0;
    const carry: u1 = if (self.status.carry) 1 else 0;
    const result = (value << 1) | carry;

    self.status.carry = shifted_bit == 1;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);

    if (address) |addr| {
        try self.console.write(addr, result);
    } else {
        self.a = result;
    }
}

fn ror(self: *Cpu, address: ?u16) !void {
    const value = if (address) |addr| try self.console.read(addr) else self.a;
    const shifted_bit = value & 1;
    const carry: u8 = if (self.status.carry) 0x80 else 0x00;
    const result = (value >> 1) | carry;

    self.status.carry = shifted_bit == 1;
    self.handleZeroFlagStatus(result);
    self.handleNegativeFlagStatus(result);

    if (address) |addr| {
        try self.console.write(addr, result);
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
    // Push the current address onto the stack
    try self.stackPush(@truncate(self.pc >> 8));
    try self.stackPush(@truncate(self.pc & 0x00FF));

    self.pc = address;
}

fn rts(self: *Cpu) !void {
    const lo = try self.stackPop();
    const hi = try self.stackPop();
    const stack_addr = makeWord(hi, lo);

    self.pc = stack_addr;
}

fn pha(self: *Cpu) !void {
    try self.stackPush(self.a);
}

fn php(self: *Cpu) !void {
    try self.stackPush(@bitCast(self.status));
}

fn pla(self: *Cpu) !void {
    self.a = try self.stackPop();
}

fn plp(self: *Cpu) !void {
    self.status = @bitCast(try self.stackPop());
}

fn rti(self: *Cpu) !void {
    try self.plp();
    try self.rts();
}

fn brk(self: *Cpu) !void {
    try self.stackPush(@bitCast(self.status));

    // Push the current pc in little endian format
    try self.stackPush(@truncate(self.pc >> 8));
    try self.stackPush(@truncate(self.pc & 0x00FF));

    // Read from the IRQ/BRK vector
    const lo_byte = try self.console.read(0xFFFE);
    const hi_byte = try self.console.read(0xFFFF);
    self.pc = makeWord(hi_byte, lo_byte);
}

fn nop(self: *Cpu) void {
    _ = self;
}

test "SEC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };

    try console.write(0x0000, 0x38);
    try cpu.step();

    try testing.expectEqual(cpu.status.carry, true);
}

test "SED" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };

    try console.write(0x0000, 0xF8);
    try cpu.step();

    try testing.expectEqual(cpu.status.decimal_mode, true);
}

test "CLC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.carry = true;

    try console.write(0x0000, 0x18);
    try cpu.step();

    try testing.expectEqual(cpu.status.carry, false);
}

test "CLV" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.overflow = true;

    try console.write(0x0000, 0xB8);
    try cpu.step();

    try testing.expectEqual(cpu.status.overflow, false);
}

test "SEI" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };

    try console.write(0x0000, 0x78);
    try cpu.step();

    try testing.expectEqual(cpu.status.interrupt_disable, true);
}

test "CLD" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.decimal_mode = true;

    try console.write(0x0000, 0xD8);
    try cpu.step();

    try testing.expectEqual(cpu.status.decimal_mode, false);
}

test "CLI" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.interrupt_disable = true;

    try console.write(0x0000, 0x58);
    try cpu.step();

    try testing.expectEqual(cpu.status.interrupt_disable, false);
}

test "LDA Immediate" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try console.write(0x0000, 0xA9); // LDA Instruction
    try console.write(0x0001, expected_byte); // Operand
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
    // The zero flag should be clear when setting the A register to 0xFF
    try testing.expectEqual(false, cpu.status.zero_result);
}

test "LDA ZeroPage" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try console.write(0x0000, 0xA5); // LDA ZeroPage Instruction
    try console.write(0x0001, 0x06); // Operand
    try console.write(0x0006, expected_byte);
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "LDA ZeroPage,X" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try console.write(0x0000, 0xB5); // LDA ZeroPage,X Instruction
    try console.write(0x0001, 0x06); // Operand
    try console.write(0x0007, expected_byte);
    cpu.x = 1; // X offset
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "LDA Absolute" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try console.write(0x0000, 0xAD); // LDA Absolute Instruction
    try console.write(0x0001, 0xFF); // Operand low byte
    try console.write(0x0002, 0x01); // Operand high byte
    try console.write(0x01FF, expected_byte);
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
    try testing.expectEqual(@as(u64, 4), cpu.cycles);
}

test "LDA Absolute,X" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try console.write(0x0000, 0xBD); // LDA Absolute,X Instruction
    try console.write(0x0001, 0xFE); // Operand low byte
    try console.write(0x0002, 0x01); // Operand high byte
    try console.write(0x01FF, 0xFF);
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

    try console.write(0x0000, 0xBD); // LDA Absolute,X Instruction
    try console.write(0x0001, 0xFF); // Operand low byte
    try console.write(0x0002, 0x00); // Operand high byte
    try console.write(0x0100, 0xFF);
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
    try testing.expectEqual(@as(u64, 5), cpu.cycles);
}

test "LDA Absolute,Y" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try console.write(0x0000, 0xB9); // LDA Absolute,Y Instruction
    try console.write(0x0001, 0xFD); // Operand low byte
    try console.write(0x0002, 0x01); // Operand high byte
    try console.write(0x01FF, 0xFF);
    cpu.y = 2; // Y index to be added to get the effective address
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "LDA IndexedIndirect" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try console.write(0x0000, 0xA1); // LDA IndirectIndexed Instruction
    try console.write(0x0001, 0x70); // Operand

    // Contents at 0x0075 which contains the effective address
    try console.write(0x0075, 0xFF); // Low Byte
    try console.write(0x0076, 0x01); // High Byte
    try console.write(0x01FF, expected_byte);
    cpu.x = 0x05; // X value to be added to the effective address
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "LDA IndirectIndexed" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try console.write(0x0000, 0xB1); // LDA IndirectIndexed Instruction
    try console.write(0x0001, 0x70); // Operand

    // Contents at 0x0070 which contains the effective address
    try console.write(0x0070, 0xEF); // Low Byte
    try console.write(0x0071, 0x01); // High Byte
    try console.write(0x01FF, expected_byte);
    cpu.y = 0x10; // Y value to be added to the effective address
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "LDX ZeroPage,Y" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try console.write(0x0000, 0xB6); // LDX ZeroPage,Y Instruction
    try console.write(0x0001, 0x06); // Operand
    try console.write(0x0007, expected_byte);
    cpu.y = 1; // Y offset
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.x);
}

test "LDY" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;

    try console.write(0x0000, 0xAC); // LDY Absolute Instruction
    try console.write(0x0001, 0xFF); // Operand low byte
    try console.write(0x0002, 0x01); // Operand high byte
    try console.write(0x01FF, expected_byte);
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.y);
}

test "STA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.a = expected_byte;

    try console.write(0x0000, 0x8D); // STA Absolute Instruction
    try console.write(0x0001, 0x00);
    try console.write(0x0002, 0x10);
    try cpu.step();

    try testing.expectEqual(expected_byte, try console.read(0x1000));
}

test "STX" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.x = expected_byte;

    try console.write(0x0000, 0x8E); // STX Absolute Instruction
    try console.write(0x0001, 0x00);
    try console.write(0x0002, 0x10);
    try cpu.step();

    try testing.expectEqual(expected_byte, try console.read(0x1000));
}

test "STY" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.y = expected_byte;

    try console.write(0x0000, 0x8C); // STY Absolute Instruction
    try console.write(0x0001, 0x00);
    try console.write(0x0002, 0x10);
    try cpu.step();

    try testing.expectEqual(expected_byte, try console.read(0x1000));
}

test "TAX" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.a = expected_byte;

    try console.write(0x0000, 0xAA); // TAX Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.x);
}

test "TAY" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.a = expected_byte;

    try console.write(0x0000, 0xA8); // TAY Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.y);
}

test "TXA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.x = expected_byte;

    try console.write(0x0000, 0x8A); // TXA Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "TYA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.y = expected_byte;

    try console.write(0x0000, 0x98); // TYA Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "TXS" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.x = 0xFF;

    try console.write(0x0000, 0x9A); // TXS Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, try cpu.stackPop());
}

test "TSX" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    try cpu.stackPush(expected_byte);

    try console.write(0x0000, 0xBA); // TSX Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.x);
}

test "ADC Immediate" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_sum: u8 = 0xFF;
    cpu.a = 0xFE;

    try console.write(0x0000, 0x69); // ADC Immediate
    try console.write(0x0001, 0x01);
    try cpu.step();

    try testing.expectEqual(expected_sum, cpu.a);
}

test "ADC Add two signed integers" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_sum: u8 = 0x73; // +115 in hex
    cpu.a = 0xFB; // -5 in hex

    try console.write(0x0000, 0x6D); // ADC Absolute instruction
    try console.write(0x0001, 0x00);
    try console.write(0x0002, 0x01); // At address $0100
    try console.write(0x0100, 0x78); // +120 in hex
    try cpu.step();

    try testing.expectEqual(expected_sum, cpu.a);
}

test "ADC signed addition with overflow" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_sum: u8 = 0x7B; // +123 in hex
    cpu.a = 0xFB; // -5 in hex

    try console.write(0x0000, 0x6D); // ADC Absolute instruction
    try console.write(0x0001, 0x00);
    try console.write(0x0002, 0x01); // At address $0100
    try console.write(0x0100, 0x80); // -128 in hex
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

    try console.write(0x0000, 0xED); // SBC Absolute instruction
    try console.write(0x0001, 0x00);
    try console.write(0x0002, 0x01); // At address $0100
    try console.write(0x0100, 0x01); // +1 in hex
    try cpu.step();

    try testing.expectEqual(expected_sum, cpu.a);
}

test "SBC signed subtraction with overflow" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.status.carry = true;
    const expected_sum: u8 = 0x80; // -128 in hex
    cpu.a = 0x7F; // 127 in hex

    try console.write(0x0000, 0xED); // SBC Absolute instruction
    try console.write(0x0001, 0x00);
    try console.write(0x0002, 0x01); // At address $0100
    try console.write(0x0100, 0xFF); // -1 in hex
    try cpu.step();

    try testing.expectEqual(expected_sum, cpu.a);
    try testing.expectEqual(true, cpu.status.overflow);
}

test "AND" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.a = 0x00;

    try console.write(0x0000, 0x29); // AND Immediate instruction
    try console.write(0x0001, 0x00);
    try cpu.step();

    try testing.expectEqual(true, cpu.status.zero_result);
    try testing.expectEqual(false, cpu.status.negative_result);
}

test "ORA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.a = 0x01;

    try console.write(0x0000, 0x09); // ORA Immediate instruction
    try console.write(0x0001, 0x00);
    try cpu.step();

    try testing.expectEqual(false, cpu.status.zero_result);
    try testing.expectEqual(false, cpu.status.negative_result);
}

test "EOR" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.a = 0x01;

    try console.write(0x0000, 0x49); // EOR Immediate instruction
    try console.write(0x0001, 0x01);
    try cpu.step();

    try testing.expectEqual(true, cpu.status.zero_result);
    try testing.expectEqual(false, cpu.status.negative_result);
}

test "INX allows for overflow" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.x = 0xFF;

    try console.write(0x0000, 0xE8); // INX instruction
    try cpu.step();

    try testing.expect(cpu.status.zero_result);
}

test "INY" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_value: u8 = 0x01;
    cpu.y = 0x00;

    try console.write(0x0000, 0xC8); // INY instruction
    try cpu.step();

    try testing.expectEqual(expected_value, cpu.y);
}

test "DEX allows for overflow" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_value: u8 = 0xFF;
    cpu.x = 0x00;

    try console.write(0x0000, 0xCA); // DEX instruction
    try cpu.step();

    try testing.expectEqual(expected_value, cpu.x);
}

test "DEY allows for overflow" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_value: u8 = 0x00;
    cpu.y = 0x01;

    try console.write(0x0000, 0x88); // DEY instruction
    try cpu.step();

    try testing.expectEqual(expected_value, cpu.y);
    try testing.expect(cpu.status.zero_result);
}

test "INC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_value: u8 = 0x00;

    try console.write(0x0000, 0xE6); // INC ZeroPage instruction
    try console.write(0x0001, 0xFF);
    try console.write(0x00FF, 0xFF); // Value at ZeroPage $FF
    try cpu.step();

    try testing.expectEqual(expected_value, try console.read(0x00FF));
    try testing.expect(cpu.status.zero_result);
}

test "DEC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_value: u8 = 0xFF;

    try console.write(0x0000, 0xCE); // DEC Absolute instruction
    try console.write(0x0001, 0xFF);
    try console.write(0x0002, 0x00);
    try console.write(0x00FF, 0x00); // Value at ZeroPage $FF
    try cpu.step();

    try testing.expectEqual(expected_value, try console.read(0x00FF));
}

test "CMP" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.a = 0xFF;

    try console.write(0x0000, 0xC9); // CMP Immediate instruction
    try console.write(0x0001, 0xFF);
    try cpu.step();

    try testing.expect(cpu.status.carry);
    try testing.expect(cpu.status.zero_result);
}

test "CPX" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.x = 0x00;

    try console.write(0x0000, 0xE0); // CPX Immediate instruction
    try console.write(0x0001, 0xFF);
    try cpu.step();

    try testing.expect(!cpu.status.carry);
}

test "CPY" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.y = 0x10;

    try console.write(0x0000, 0xCC); // CPY Absolute instruction
    try console.write(0x0001, 0x00);
    try console.write(0x0002, 0x01);
    try console.write(0x0100, 0x0F);
    try cpu.step();

    try testing.expect(cpu.status.carry);
}

test "BIT" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    cpu.a = 0x02;

    try console.write(0x0000, 0x2C); // BIT Absolute Instruction
    try console.write(0x0001, 0x01);
    try console.write(0x0002, 0x0C);
    try console.write(0x0C01, 0x8F);
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

    try console.write(0x0000, 0x4A); // LSR Accumulator Instruction
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

    try console.write(0x0000, 0x4A); // LSR Accumulator Instruction
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

    try console.write(0x0000, 0x0A); // ASL Accumulator Instruction
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

    try console.write(0x0000, 0x2A); // ROL Accumulator Instruction
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

    try console.write(0x0000, 0x66); // ROR Zeropage Instruction
    try console.write(0x0001, 0xFF);
    try console.write(0x00FF, 0x02);
    try cpu.step();

    try testing.expect(!cpu.status.zero_result);
    try testing.expect(!cpu.status.carry);
    try testing.expectEqual(expected_byte, try console.read(0x00FF));
}

test "JMP Absolute" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_pc: u16 = 0x4030;

    try console.write(0x0000, 0x4C); // JMP Absolute
    try console.write(0x0001, 0x30);
    try console.write(0x0002, 0x40);
    try cpu.step();

    try testing.expectEqual(expected_pc, cpu.pc);
}

test "JMP Indirect" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_pc: u16 = 0x2010;

    try console.write(0x0000, 0x6C); // JMP Indirect
    try console.write(0x0001, 0xFE); // Indirect low byte
    try console.write(0x0002, 0x01); // Indirect high byte
    try console.write(0x01FE, 0x10);
    try console.write(0x01FF, 0x20);
    try cpu.step();

    try testing.expectEqual(expected_pc, cpu.pc);
}

test "BMI" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0081;
    cpu.status.negative_result = true;

    try console.write(0x0000, 0x30); // BMI Instruction
    try console.write(0x0001, 0x7F);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BMI branching adds an additional cycle" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0080;
    cpu.pc = 0x000E;
    cpu.status.negative_result = true;

    try console.write(0x000E, 0x30); // BMI Instruction
    try console.write(0x000F, 0x70);
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

    try console.write(0x00EE, 0x30); // BMI Instruction
    try console.write(0x00EF, 0x10);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
    try testing.expectEqual(@as(u64, 4), cpu.cycles);
}

test "BPL" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.negative_result = true;

    try console.write(0x0000, 0x30); // BPL Instruction
    try console.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BVS" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.overflow = true;

    try console.write(0x0000, 0x70); // BVS Instruction
    try console.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BVC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.overflow = false;

    try console.write(0x0000, 0x50); // BVC Instruction
    try console.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BCS" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.carry = true;

    try console.write(0x0000, 0xB0); // BCS Instruction
    try console.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BCC" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.carry = false;

    try console.write(0x0000, 0x90); // BCC Instruction
    try console.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BEQ" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.zero_result = true;

    try console.write(0x0000, 0xF0); // BEQ Instruction
    try console.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "BNE" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0005;
    cpu.status.zero_result = false;

    try console.write(0x0000, 0xD0); // BNE Instruction
    try console.write(0x0001, 0x03);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "JSR" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x3000;
    cpu.pc = 0x1000;

    try console.write(0x1000, 0x20); // JSR Instruction
    try console.write(0x1001, 0x00);
    try console.write(0x1002, 0x30);
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);

    // Verify the address pushed to the stack
    const lo = try cpu.stackPop();
    const hi = try cpu.stackPop();
    const stack_addr = makeWord(hi, lo);
    try testing.expectEqual(@as(u16, 0x1003), stack_addr);
}

test "RTS" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x1000;
    try cpu.stackPush(0x10);
    try cpu.stackPush(0x00);

    try console.write(0x0000, 0x60); // RTS Instruction
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}

test "PHA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    cpu.a = 0xFF;

    try console.write(0x0000, 0x48); // PHA Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, try cpu.stackPop());
}

test "PHP" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0x01;
    cpu.status.carry = true;

    try console.write(0x0000, 0x08); // PHP Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, try cpu.stackPop());
}

test "PLA" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0xFF;
    try cpu.stackPush(expected_byte);

    try console.write(0x0000, 0x68); // PLA Instruction
    try cpu.step();

    try testing.expectEqual(expected_byte, cpu.a);
}

test "PLP" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_byte: u8 = 0x03;
    try cpu.stackPush(expected_byte);

    try console.write(0x0000, 0x28); // PLP Instruction
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
    const expected_stack_reg: u8 = 0x03;
    const expected_address: u16 = 0x1000;
    try cpu.stackPush(0x10); // First push the address in little endian
    try cpu.stackPush(0x00);
    try cpu.stackPush(0x03); // Next push the status register

    try console.write(0x0000, 0x40); // RTI Instruction
    try cpu.step();

    try testing.expectEqual(expected_stack_reg, @as(u8, @bitCast(cpu.status)));
    try testing.expectEqual(expected_address, cpu.pc);
}

test "NOP" {
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    const expected_address: u16 = 0x0001;

    try console.write(0x0000, 0xEA); // NOP Instruction
    try cpu.step();

    try testing.expectEqual(expected_address, cpu.pc);
}
