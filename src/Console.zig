const Console = @This();

const std = @import("std");
const testing = std.testing;

const Cartridge = @import("Cartridge.zig");
const Cpu = @import("Cpu.zig");
const Ppu = @import("Ppu.zig");

cpu: ?*Cpu = null,
cartridge: ?*Cartridge = null,
ppu: ?*Ppu = null,
memory: [0x800]u8 = std.mem.zeroes([0x800]u8),

const ConsoleError = error{
    InvalidMemoryAddress,
    MissingCartridge,
};

pub fn step(self: *Console) !void {
    try self.cpu.?.step();
}

pub fn connectCpu(self: *Console, cpu: *Cpu) void {
    self.cpu = cpu;
}

pub fn connectCartridge(self: *Console, cartridge: *Cartridge) void {
    self.cartridge = cartridge;
}

pub fn connectPpu(self: *Console, ppu: *Ppu) void {
    self.ppu = ppu;
}

pub fn read(self: *Console, address: u16) !u8 {
    if (address >= 0x0000 and address <= 0x1FFF) {
        return self.memory[address & 0x07FF];
    } else if (address >= 0x8000 and address <= 0xFFFF) {
        if (self.cartridge) |cartridge| {
            return cartridge.read(address);
        } else {
            return ConsoleError.MissingCartridge;
        }
    }

    return ConsoleError.InvalidMemoryAddress;
}

pub fn write(self: *Console, address: u16, value: u8) !void {
    if (address >= 0x0000 and address <= 0x1FFF) {
        self.memory[address & 0x07FF] = value;
        return;
    } else if (address >= 0x8000 and address <= 0xFFFF) {
        if (self.cartridge) |cartridge| {
            cartridge.write(address, value);
        } else {
            return ConsoleError.MissingCartridge;
        }
    }

    return ConsoleError.InvalidMemoryAddress;
}

test "Accessing a RAM with mirrors between $0000 and $07FF" {
    var console = Console{};

    try console.write(0x1000, 0xFF);
    try testing.expectEqual(@as(u8, 0xFF), try console.read(0x0000));
}
