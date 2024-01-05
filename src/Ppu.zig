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

palette: Palette = Palette.default(),
buffer: [256 * 240]u8 = [_]u8{0x2C} ** (256 * 240),


const PpuMemoryAccessError = error{
    InvalidMemoryAddress,
    MissingCartridge,
};

fn read(self: *Ppu, address: u16) !u8 {
    // Address space for accessing the left/right pattern table
    if (address >= 0x0000 and address <= 0x1FFF) {
        if (self.console.cartridge) |cartridge| {
            return cartridge.chr_rom_bank[address];
        } else {
            return PpuMemoryAccessError.MissingCartridge;
        }
    }

    return PpuMemoryAccessError.InvalidMemoryAddress;
}
