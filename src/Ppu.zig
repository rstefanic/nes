const Ppu = @This();

const std = @import("std");

const Console = @import("Console.zig");
const Palette = @import("Palette.zig");

console: *Console,

ppuctrl: packed struct(u8) {
    nametable_addr: u2 = 0,
    i: bool = false,
    s: bool = false,
    b: bool = false,
    h: bool = false,
    p: bool = false,
    v: bool = false,
} = .{},

ppumask: packed struct(u8) {
    greyscale: bool = false,
    show_leftmost_background: bool = false,
    show_leftmost_sprites: bool = false,
    show_background: bool = false,
    show_sptires: bool = false,
    r: bool = false,
    g: bool = false,
    b: bool = false,
} = .{},

ppustatus: packed struct(u8) {
    _unused: u5 = 0,
    sprite_overflow: bool = false,
    sprite_zero_hit: bool = false,
    vertical_blank: bool = false,
} = .{},

oamaddr: u8 = 0,
oamdata: u8 = 0,
ppuscroll: u8 = 0,
ppuaddr: u8 = 0,
ppudata: u8 = 0,
oamdma: u8 = 0,

palette: Palette = Palette.init(),
buffer: [256 * 240]u8 = [_]u8{0x2C} ** (256 * 240),

pub fn read(self: *Ppu, address: u16) u8 {
    return switch (address & 0x0007) {
        0 => @bitCast(self.ppuctrl),
        1 => @bitCast(self.ppumask),
        2 => @bitCast(self.ppustatus),
        3 => self.oamaddr,
        4 => self.oamdata,
        5 => self.ppuscroll,
        6 => self.ppuaddr,
        else => self.ppudata,
    };
}

pub fn write(self: *Ppu, address: u16, value: u8) void {
    switch (address & 0x0007) {
        0 => self.ppuctrl = @bitCast(value),
        1 => self.ppumask = @bitCast(value),
        2 => self.ppustatus = @bitCast(value),
        3 => self.oamaddr = value,
        4 => self.oamdata = value,
        5 => self.ppuscroll = value,
        6 => self.ppuaddr = value,
        else => self.ppudata = value,
    }
}
