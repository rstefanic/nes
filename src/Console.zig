const Console = @This();

const std = @import("std");
const testing = std.testing;

const Apu = @import("Apu.zig");
const Cartridge = @import("Cartridge.zig");
const Cpu = @import("Cpu.zig");
const Ppu = @import("Ppu.zig");
const Controller = @import("Controller.zig");

apu: ?*Apu = null,
cpu: ?*Cpu = null,
cartridge: ?*Cartridge = null,
ppu: ?*Ppu = null,
controller1: ?*Controller = null,
controller2: ?*Controller = null,
memory: [0x800]u8 = std.mem.zeroes([0x800]u8),
cycles: u64 = 0,

dma: struct {
    page: u8 = 0x00,
    addr: u8 = 0x00,
    in_progress: bool = false,
    in_sync: bool = false,
} = .{},

pub fn step(self: *Console) !void {
    if (self.dma.in_progress) dma: {
        if (!self.dma.in_sync) {
            if (@mod(self.cycles, 2) == 1) {
                self.dma.in_sync = true;
            }

            break :dma;
        }

        // The NES alternates between reading from the CPU and writing that
        // data to the PPU. For simplicity, we'll just write the data to PPU
        // as soon as we read from the CPU instead of being cycle accurate.
        if (@mod(self.cycles, 2) == 1) {
            // There are 256 possible addresses. Dividing by 4 will give us an
            // index into the OAM for the correct sprite.
            var sprite = self.ppu.?.oam[self.dma.addr >> 2];
            const addr = (@as(u16, self.dma.page) << 8) | self.dma.addr;
            const value = try self.cpu.?.read(addr);

            // There are 4 bytes that make up a sprite, so the part of the
            // sprite that we'll write to changes based on the address.
            switch (@mod(self.dma.addr, 4)) {
                0 => sprite.y = value,
                1 => sprite.index = value,
                2 => sprite.attributes = @bitCast(value),
                else => sprite.x = value,
            }

            self.ppu.?.oam[self.dma.addr >> 2] = sprite;
            self.dma.addr +%= 1;

            // When dma.addr is 0, it means we've written 256 bytes and the DMA
            // transfer is over.
            if (self.dma.addr == 0x00) {
                self.dma.in_progress = false;
                self.dma.in_sync = false;
                self.dma.page = 0;
            }
        }
    } else {
        if (@mod(self.cycles, 3) == 0) {
            try self.cpu.?.step();
        }
    }

    try self.ppu.?.step();

    self.cycles += 1;
}

pub fn connectApu(self: *Console, apu: *Apu) void {
    self.apu = apu;
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
