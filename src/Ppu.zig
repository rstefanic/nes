const Ppu = @This();

const std = @import("std");

const Console = @import("Console.zig");
const Palette = @import("Palette.zig");
const Tile = [64]u2;

const scanlines_per_frame = 262;
const dots_per_scanline = 341;

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

scanlines: i16 = 0,
dots: i16 = 0,

palette: Palette = Palette.default(),
buffer: [256 * 240]u8 = [_]u8{0x2C} ** (256 * 240),

// Pattern Tables define the raw image data
left_pattern_table: [256]Tile = undefined,
right_pattern_table: [256]Tile = undefined,

pub fn reset(self: *Ppu) !void {
    // A CPU reset takes 7 cycles. Since the PPU runs 3 cycles
    // per CPU cycle, we'll set the dots here to 21 to mimic that.
    self.dots = 21;
}

const PpuMemoryAccessError = error{
    InvalidMemoryAddress,
    MissingCartridge,
};

pub fn step(self: *Ppu) !void {
    self.dots += 1;
    if (self.dots >= dots_per_scanline) {
        self.dots = 0;
        self.scanlines += 1;

        if (self.scanlines >= scanlines_per_frame) {
            self.scanlines = -1;
        }
    }
}

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

/// This is just for debugging purposes. In more complicated mappers,
/// the pattern tables may change. For mapper 0, this will be fine
/// since they only have a left and right pattern table and they
/// never change.
pub fn setupPatternTables(self: *Ppu) !void {
    try self.buildPatternTable(&self.left_pattern_table, 0x0000);
    try self.buildPatternTable(&self.right_pattern_table, 0x1000);
}

fn buildPatternTable(self: *Ppu, table: *[256]Tile, chr_rom_offset_init: u16) !void {
    var tile_idx: u16 = 0;
    while (tile_idx < table.len) : (tile_idx += 1) {
        const chr_rom_offset = chr_rom_offset_init + tile_idx * 16;

        var tile_pixel_idx: u16 = 0;
        while (tile_pixel_idx < 8) : (tile_pixel_idx += 1) {
            const lo = try self.read(chr_rom_offset + tile_pixel_idx);
            const hi = try self.read(chr_rom_offset + tile_pixel_idx + 8);
            const tile_pixel_offset = tile_pixel_idx * 8;

            table[tile_idx][0 + tile_pixel_offset] = @truncate((lo >> 7 & 1) + (hi >> 7 & 1));
            table[tile_idx][1 + tile_pixel_offset] = @truncate((lo >> 6 & 1) + (hi >> 6 & 1));
            table[tile_idx][2 + tile_pixel_offset] = @truncate((lo >> 5 & 1) + (hi >> 5 & 1));
            table[tile_idx][3 + tile_pixel_offset] = @truncate((lo >> 4 & 1) + (hi >> 4 & 1));
            table[tile_idx][4 + tile_pixel_offset] = @truncate((lo >> 3 & 1) + (hi >> 3 & 1));
            table[tile_idx][5 + tile_pixel_offset] = @truncate((lo >> 2 & 1) + (hi >> 2 & 1));
            table[tile_idx][6 + tile_pixel_offset] = @truncate((lo >> 1 & 1) + (hi >> 1 & 1));
            table[tile_idx][7 + tile_pixel_offset] = @truncate((lo & 1) + (hi & 1));
        }
    }
}
