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
