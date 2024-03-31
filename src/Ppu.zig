const Ppu = @This();

const std = @import("std");

const Console = @import("Console.zig");
const Palette = @import("Palette.zig");
const Tile = [64]u2;

const scanlines_per_frame = 262;
const dots_per_scanline = 341;
const visible_dots_per_scanline = 256;

console: *Console,

ppuctrl: packed struct(u8) {
    nametable_addr: u2 = 0,
    i: bool = false, // VRAM address Increment -- false adds 1, true adds 32
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
ppuaddr: u16 = 0,
ppudata: u8 = 0,
oamdma: u8 = 0,

// Internal registers
v: u8 = 0,
t: u8 = 0,
x: u8 = 0,
w: bool = false, // write latch -- is set if we're in the middle of an ppuaddr write
ppudata_buffer: u8 = 0,

scanlines: i16 = -1,
dots: i16 = 0,

framedata: struct {
    ptrn_tbl_idx: u16 = 0, // Index to locate a tile on the pattern table
    bg_ptrn_lsb: u8 = 0, // Current background pattern tile's lsbits and msbits
    bg_ptrn_msb: u8 = 0,
    bg_ptrn_lsb_buf: u8 = 0, // Buffer used to store the next background tile's bits
    bg_ptrn_msb_buf: u8 = 0,
} = .{},

palette: Palette = Palette.default(),
buffer: [256 * 240]u8 = [_]u8{0x2C} ** (256 * 240),

palette_ram: [32]u8 = [_]u8{0x00} ** 32,
nametables: [0x800]u8 = std.mem.zeroes([0x800]u8),

// Pattern Tables define the raw image data
left_pattern_table: [256]Tile = undefined,
right_pattern_table: [256]Tile = undefined,

pub fn reset(self: *Ppu) !void {
    // A CPU reset takes 7 cycles. Since the PPU runs 3 cycles
    // per CPU cycle, we'll set the dots here to 21 to mimic that.
    self.dots = 21;
}

pub inline fn readPpustatus(self: *Ppu) u8 {
    const ppustatus: u8 = @bitCast(self.ppustatus);

    self.w = false; // clears the address latch bit
    self.ppustatus.vertical_blank = false; // clears the vertical blank flag

    return ppustatus;
}

pub inline fn readOamdata(self: *Ppu) u8 {
    return self.oamdata;
}

pub inline fn readPpudata(self: *Ppu) !u8 {
    var data = self.ppudata_buffer;
    self.ppudata_buffer = try self.read(self.ppuaddr);

    // Palette data is returned immediately instead of being
    // primed on the internal ppudata buffer.
    if (self.ppuaddr >= 0x3F00 and self.ppuaddr <= 0x3FFF) {
        data = self.ppudata_buffer;
    }

    self.ppuaddr += if (self.ppuctrl.i) 32 else 1;
    return data;
}

pub inline fn writePpuctrl(self: *Ppu, value: u8) void {
    self.ppuctrl = @bitCast(value);
}

pub inline fn writePpumask(self: *Ppu, value: u8) void {
    self.ppumask = @bitCast(value);
}

pub inline fn writeOamaddr(self: *Ppu, value: u8) void {
    self.oamaddr = value;
}

pub inline fn writeOamdata(self: *Ppu, value: u8) void {
    self.oamdata = value;
}

pub inline fn writePpuscroll(self: *Ppu, value: u8) void {
    self.ppuscroll = value;
}

pub inline fn writePpuaddr(self: *Ppu, value: u8) void {
    if (self.w) { // set the lo byte if we're in the middle of a write
        self.ppuaddr += value;
    } else { // set the hi byte
        self.ppuaddr = @as(u16, value) << 8;
    }

    self.w = !self.w;
}

pub inline fn writePpudata(self: *Ppu, value: u8) !void {
    try self.write(self.ppuaddr, value);
    self.ppuaddr += if (self.ppuctrl.i) 32 else 1;
}

const PpuMemoryAccessError = error{
    InvalidMemoryAddress,
    MissingCartridge,
};

pub fn step(self: *Ppu) !void {
    { // Handle current dot/scanline
        if (self.scanlines > -1 and self.scanlines < 240) {
            if (self.dots < visible_dots_per_scanline) {
                switch (@mod(self.dots - 1, 8)) {
                    0 => { // Fetch the Nametable entry
                        // Move the BG Pattern table buffer data into the current bg pattern table
                        self.framedata.bg_ptrn_lsb = self.framedata.bg_ptrn_lsb_buf;
                        self.framedata.bg_ptrn_msb = self.framedata.bg_ptrn_msb_buf;

                        // The screen is 32 x 30 tiles. Determine which tile we need
                        // based on the current dot and scanline.
                        const x = @divTrunc(self.dots, 8);
                        const y = @divTrunc(self.scanlines, 8);
                        const idx: usize = @intCast(x + (y * 32));

                        self.framedata.ptrn_tbl_idx = self.nametables[idx];
                    },
                    2 => { // Fetch the Attribute entry
                    },
                    4 => {
                        const tile_addr = self.framedata.ptrn_tbl_idx << 4; // Multiply the index by 16 since each pattern table entry is 16 bytes
                        const tile_row_offset: u8 = @intCast(@mod(self.scanlines, 8));
                        self.framedata.bg_ptrn_lsb_buf = try self.read(tile_addr + tile_row_offset);
                    },
                    6 => {
                        const tile_addr = self.framedata.ptrn_tbl_idx << 4;
                        const hi_byte_offset = 8;
                        const tile_row_offset: u8 = @intCast(@mod(self.scanlines, 8));
                        self.framedata.bg_ptrn_msb_buf = try self.read(tile_addr + tile_row_offset + hi_byte_offset);
                    },
                    else => {}, // Cycles 1, 3, 5, and 7
                }

                { // Draw the current pixel
                    const palette = try self.getPaletteById(0); // FIXME: Grab me from the AT
                    const pixel_offset: u3 = @intCast(@mod(self.dots, 8));

                    const lo: u2 = @truncate(self.framedata.bg_ptrn_lsb >> (7 - pixel_offset) & 1);
                    const hi: u2 = @truncate(self.framedata.bg_ptrn_msb >> (7 - pixel_offset) & 1);
                    const color: u2 = lo + hi;

                    const x: usize = @intCast(self.dots);
                    const y: usize = @intCast(self.scanlines);
                    self.buffer[x + (y * visible_dots_per_scanline)] = palette[color];
                }
            }
        }

        if (self.scanlines == 241) {
            if (self.dots == 1) {
                self.ppustatus.vertical_blank = true;
            }
        }

        if (self.scanlines == 261) {
            if (self.dots == 1) {
                self.ppustatus.vertical_blank = false;
                self.ppustatus.sprite_zero_hit = false;
                self.ppustatus.sprite_overflow = false;
            }
        }
    }

    { // Increment the dots and scanlines
        self.dots += 1;
        if (self.dots >= dots_per_scanline) {
            self.dots = 0;
            self.scanlines += 1;

            if (self.scanlines >= scanlines_per_frame) {
                self.scanlines = -1;
            }
        }
    }
}

pub fn getPaletteById(self: *Ppu, palette_id: u8) ![4]u8 {
    const offset: u16 = palette_id * 4;

    return .{
        try self.read(0x3F00 + offset),
        try self.read(0x3F01 + offset),
        try self.read(0x3F02 + offset),
        try self.read(0x3F03 + offset),
    };
}

fn read(self: *Ppu, address: u16) !u8 {
    // Address space for accessing the left/right pattern table
    if (address >= 0x0000 and address <= 0x1FFF) {
        if (self.console.cartridge) |cartridge| {
            return cartridge.chr_rom_bank[address];
        } else {
            return PpuMemoryAccessError.MissingCartridge;
        }
    } else if (address >= 0x2000 and address <= 0x3000) {
        if (self.console.cartridge) |cartridge| {
            const is_horizontal_arrangement = cartridge.header.flag6.mirroring;

            // horizontal arrangement, vertically mirrored
            if (is_horizontal_arrangement) {
                if (address >= 0x2000 and address <= 0x27FF) {
                    return self.nametables[address & 0x07FF];
                }

                return self.nametables[address & 0x0FFF];
            }

            // vertical arrangement, horizontally mirrored
            if (address >= 0x2000 and address <= 0x27FF) {
                return self.nametables[address & 0x03FF];
            }

            return self.nametables[address & 0x07FF];
        } else {
            return PpuMemoryAccessError.MissingCartridge;
        }
    } else if (address >= 0x3F00 and address <= 0x3FFF) {
        // Addresses $3F20-$3FFF are mirrors of the first 32 bytes for our palette RAM. So take
        // the lower 5 bits of the address to index into the palette RAM and mimic the mirroring.
        return self.palette_ram[address & 0x001F];
    }

    return PpuMemoryAccessError.InvalidMemoryAddress;
}

fn write(self: *Ppu, address: u16, value: u8) !void {
    if (address >= 0x0000 and address <= 0x1FFF) {
        if (self.console.cartridge) |cartridge| {
            cartridge.chr_rom_bank[address] = value;
            return;
        } else {
            return PpuMemoryAccessError.MissingCartridge;
        }
    } else if (address >= 0x2000 and address <= 0x2FFF) {
        if (self.console.cartridge) |cartridge| {
            const is_horizontal_arrangement = cartridge.header.flag6.mirroring;

            // horizontal arrangement, vertically mirrored
            if (is_horizontal_arrangement) {
                if (address >= 0x2000 and address <= 0x27FF) {
                    self.nametables[address & 0x07FF] = value;
                    return;
                }

                self.nametables[address & 0x0FFF] = value;
                return;
            }

            // vertical arrangement, horizontally mirrored
            if (address >= 0x2000 and address <= 0x27FF) {
                self.nametables[address & 0x03FF] = value;
                return;
            }

            self.nametables[address & 0x07FF] = value;
            return;
        } else {
            return PpuMemoryAccessError.MissingCartridge;
        }
    } else if (address >= 0x3F00 and address <= 0x3FFF) {
        self.palette_ram[address & 0x001F] = value;
        return;
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
