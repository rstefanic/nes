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
controller2: ?*Controller = null,
memory: [0x800]u8 = std.mem.zeroes([0x800]u8),
cycles: u64 = 0,

pub fn step(self: *Console) !void {
    if (@mod(self.cycles, 3) == 0) {
        try self.cpu.?.step();
    }

    try self.ppu.?.step();

    self.cycles += 1;
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

pub fn connectController2(self: *Console, controller: *Controller) void {
    self.controller2 = controller;
}
