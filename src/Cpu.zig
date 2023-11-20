const Cpu = @This();

const std = @import("std");
const Bus = @import("Bus.zig");
const Instruction = @import("Instruction.zig");
const testing = std.testing;

// A reference to the Bus so that we can read/write memory
bus: *Bus,

// Track the number of cycles
cycles: u64 = 0,

// Registers
a: u8 = 0,
x: u8 = 0,
y: u8 = 0,
pc: u16 = 0,
sp: u16 = 0,

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

pub fn init(bus: *Bus) Cpu {
    var cpu = Cpu{
        .bus = bus,
    };

    bus.connectCpu(&cpu);

    // Read from the reset vector to init the PC
    const lo_byte = try bus.read(0xFFFC);
    const hi_byte = try bus.read(0xFFFD);
    cpu.pc = makeWord(hi_byte, lo_byte);

    return cpu;
}

inline fn makeWord(hi_byte: u8, lo_byte: u8) u16 {
    return (@as(u16, hi_byte) << 1) + lo_byte;
}

pub fn step(self: *Cpu) !void {
    const byte = try self.fetch();
    const ins = try Instruction.decode(byte);
    try self.execute(ins);
}

inline fn fetch(self: *Cpu) !u8 {
    const address = self.pc;
    self.pc += 1;
    return try self.bus.read(address);
}

fn execute(self: *Cpu, ins: Instruction) !void {
    var operand: ?u8 = null;
    switch (ins.mode) {
        .Implied => operand = null,
        .Immediate => {
            operand = try self.fetch();
        },
        .Accumulator => {
            operand = self.a;
        },
        .Relative => {
            // The following byte to be read is signed. Once the sign is known,
            // the byte is converted to be unsigned so it can be used to
            // add/subtract from the PC register to get the offset.
            var is_negative = false;
            var signed_byte: i8 = @bitCast(try self.fetch());
            if (signed_byte < 0) {
                is_negative = true;
                signed_byte *= -1;
            }

            // TODO: I worry about an overflow/underflow that cound occur here.
            const unsigned_byte: u8 = @intCast(signed_byte);
            var offset = self.pc;
            if (is_negative) offset -= unsigned_byte else offset += unsigned_byte;

            operand = try self.bus.read(offset);
        },
        .Absolute => {
            const lo = try self.fetch();
            const hi = try self.fetch();
            const address = makeWord(hi, lo);
            operand = try self.bus.read(address);
        },
        .ZeroPage => {
            const lo = try self.fetch();
            const address = makeWord(0x00, lo);
            operand = try self.bus.read(address);
        },
        .ZeroPageX => {
            const byte = try self.fetch();
            // The address here needs to wrap around
            // and always remain on the Zero Page.
            const address = (byte + self.x) % 255;
            operand = try self.bus.read(address);
        },
        .ZeroPageY => {
            const byte = try self.fetch();
            // The address here needs to wrap around
            // and always remain on the Zero Page.
            const address = (byte + self.y) % 255;
            operand = try self.bus.read(address);
        },
        .AbsoluteX => {
            const lo = try self.fetch();
            const hi = try self.fetch();
            const address = makeWord(hi, lo);
            operand = try self.bus.read(address);
            operand.? += self.x;
        },
        .AbsoluteY => {
            const lo = try self.fetch();
            const hi = try self.fetch();
            const address = makeWord(hi, lo);
            operand = try self.bus.read(address);
            operand.? += self.y;
        },
        .Indirect => {
            const lo = try self.fetch();
            const hi = try self.fetch();
            const indirect_addr = makeWord(hi, lo);
            const address = try self.bus.read(indirect_addr);
            operand = try self.bus.read(address);
        },
        .IndexedIndirect => {
            const byte = try self.fetch();
            const address = makeWord(0x00, byte + self.x);
            operand = try self.bus.read(address);
        },
        .IndirectIndexed => {
            const byte = try self.fetch();
            const address = makeWord(0x00, byte);
            operand = try self.bus.read(address);
            operand.? += self.y;
        },
    }

    switch (ins.opcode) {
        .SEC => self.sec(),
        .CLC => self.clc(),
        .CLV => self.clv(),
        .SEI => self.sei(),
        .CLI => self.cli(),
        .LDA => self.lda(operand.?),
        else => return error.OpcodeExecutionNotYetImplemented,
    }

    self.cycles += ins.cycles;
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

fn clc(self: *Cpu) void {
    self.status.carry = false;
}

fn clv(self: *Cpu) void {
    self.status.overflow = false;
}

fn sei(self: *Cpu) void {
    self.status.interrupt_disable = true;
}

fn cli(self: *Cpu) void {
    self.status.interrupt_disable = false;
}

fn lda(self: *Cpu, value: u8) void {
    self.a = value;

    self.handleZeroFlagStatus(value);
    self.handleNegativeFlagStatus(value);
}

test "SEC" {
    var bus = Bus{};
    var cpu = Cpu.init(&bus);

    try bus.write(0x0000, 0x38);
    try cpu.step();

    try testing.expectEqual(cpu.status.carry, true);
}

test "CLC" {
    var bus = Bus{};
    var cpu = Cpu.init(&bus);
    cpu.status.carry = true;

    try bus.write(0x0000, 0x18);
    try cpu.step();

    try testing.expectEqual(cpu.status.carry, false);
}

test "CLV" {
    var bus = Bus{};
    var cpu = Cpu.init(&bus);
    cpu.status.overflow = true;

    try bus.write(0x0000, 0xB8);
    try cpu.step();

    try testing.expectEqual(cpu.status.overflow, false);
}

test "SEI" {
    var bus = Bus{};
    var cpu = Cpu.init(&bus);

    try bus.write(0x0000, 0x78);
    try cpu.step();

    try testing.expectEqual(cpu.status.interrupt_disable, true);
}

test "CLI" {
    var bus = Bus{};
    var cpu = Cpu.init(&bus);
    cpu.status.interrupt_disable = true;

    try bus.write(0x0000, 0x58);
    try cpu.step();

    try testing.expectEqual(cpu.status.interrupt_disable, false);
}
