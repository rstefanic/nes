const Console = @This();

const std = @import("std");
const Cartridge = @import("Cartridge.zig");
const Cpu = @import("Cpu.zig");

cpu: ?*Cpu = null,
cartridge: ?*Cartridge = null,
memory: [0x10000]u8 = [_]u8{0xEA} ** 0x10000,

const ConsoleError = error{InvalidMemoryAddress};

pub fn connectCpu(self: *Console, cpu: *Cpu) void {
    self.cpu = cpu;
}

pub fn connectCartridge(self: *Console, cartridge: *Cartridge) void {
    self.cartridge = cartridge;
}

pub fn read(self: *Console, address: u16) !u8 {
    if (address >= 0x0000 and address <= 0xFFFF) {
        return self.memory[address];
    }

    return ConsoleError.InvalidMemoryAddress;
}

pub fn write(self: *Console, address: u16, value: u8) !void {
    if (address >= 0x0000 and address <= 0xFFFF) {
        self.memory[address] = value;
        return;
    }

    return ConsoleError.InvalidMemoryAddress;
}
