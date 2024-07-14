const Console = @This();

const std = @import("std");
const testing = std.testing;

const Cartridge = @import("Cartridge.zig");
const Cpu = @import("Cpu.zig");
const Ppu = @import("Ppu.zig");
const Controller = @import("Controller.zig");

cpu: ?*Cpu = null,
cartridge: ?*Cartridge = null,
ppu: ?*Ppu = null,
controller1: ?*Controller = null,
memory: [0x800]u8 = std.mem.zeroes([0x800]u8),

pub fn step(self: *Console) !void {
    const start_cycle_count = self.cpu.?.cycles;
    try self.cpu.?.step();
    const end_cycle_count = self.cpu.?.cycles;

    // The PPU runs 3 cycles for every CPU cycle
    var cycles = end_cycle_count - start_cycle_count;
    while (cycles > 0) : (cycles -= 1) {
        try self.ppu.?.step();
        try self.ppu.?.step();
        try self.ppu.?.step();
    }
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

pub fn connectController1(self: *Console, controller: *Controller) void {
    self.controller1 = controller;
}
