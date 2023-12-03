const Bus = @This();

const std = @import("std");
const Cpu = @import("Cpu.zig");

cpu: ?*Cpu = null,
memory: [0x10000]u8 = [_]u8{0xEA} ** 0x10000,

const BusError = error{InvalidMemoryAddress};

pub fn connectCpu(self: *Bus, cpu: *Cpu) void {
    self.cpu = cpu;
}

pub fn read(self: *Bus, address: u16) !u8 {
    if (address >= 0x0000 and address <= 0xFFFF) {
        return self.memory[address];
    }

    return BusError.InvalidMemoryAddress;
}

pub fn write(self: *Bus, address: u16, value: u8) !void {
    if (address >= 0x0000 and address <= 0xFFFF) {
        self.memory[address] = value;
        return;
    }

    return BusError.InvalidMemoryAddress;
}
