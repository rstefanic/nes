const Ppu = @This();

const std = @import("std");

const Console = @import("Console.zig");
const Palette = @import("Palette.zig");
const Tile = [64]u2;

const scanlines_per_frame = 260;
const dots_per_scanline = 341;
const visible_dots_per_scanline = 256;

/// yyy NN YYYYY XXXXX
/// ||| || ||||| +++++-- coarse X scroll
/// ||| || +++++-------- coarse Y scroll
/// ||| ++-------------- nametable select
/// +++----------------- fine Y scroll
const PpuAddress = packed struct(u16) {
    coarse_x: u5 = 0,
    coarse_y: u5 = 0,
    nametable: u2 = 0,
    fine_y: u3 = 0,
    _unused: u1 = 0,
};

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
ppudata: u8 = 0,
oamdma: u8 = 0,

// Internal registers
ppuaddr: PpuAddress = .{}, // Referenced as 'v' in the NesDev Wiki
temp_ppuaddr: PpuAddress = .{}, // Referenced as 't' in the NesDev Wiki
x: u8 = 0,
w: bool = false, // write latch -- is set if we're in the middle of an ppuaddr/ppuscroll write
ppudata_buffer: u8 = 0,

scanlines: i16 = -1,
dots: i16 = 0,

framedata: struct {
    nametable_entry: u8 = 0, // Index to locate a tile on the pattern table
    attribute_entry: u8 = 0, // Color data for the pattern taken from the Attribute table
    bg_ptrn_lsb: u8 = 0, // Current background pattern tile's lsbits and msbits
    bg_ptrn_msb: u8 = 0,
    attribute_byte_buf: u8 = 0,
    bg_ptrn_lsb_buf: u8 = 0, // Buffer used to store the next background tile's bits
    bg_ptrn_msb_buf: u8 = 0,
} = .{},

palette: Palette = Palette.default(),

// The display consists of 32x30 tiles, where each tile is 8x8 pixels.
// Tiles can be further grouped into 2x2 groups called quadrants.
// Quadrants can be further grouped into 2x2 groups called blocks.
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
    var ppuaddr: u16 = @bitCast(self.ppuaddr);
    self.ppudata_buffer = try self.read(ppuaddr);

    // Palette data is returned immediately instead of being
    // primed on the internal ppudata buffer.
    if (ppuaddr >= 0x3F00 and ppuaddr <= 0x3FFF) {
        data = self.ppudata_buffer;
    }

    ppuaddr += if (self.ppuctrl.i) 32 else 1;
    self.ppuaddr = @bitCast(ppuaddr);
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
    if (self.w) { // Second write is to Y
        self.temp_ppuaddr.fine_y = @truncate(value & 0x7); // 3 lsb
        self.temp_ppuaddr.coarse_y = @truncate(value & 0xF8); // 5 msb
    } else { // First write is to X
        self.x = @truncate(value & 0x7); // 3 lsb
        self.temp_ppuaddr.coarse_x = @truncate(value & 0xF8); // 5 msb
    }

    self.w = !self.w;
}

pub inline fn writePpuaddr(self: *Ppu, value: u8) void {
    var temp_ppuaddr: u16 = @bitCast(self.temp_ppuaddr);

    if (self.w) { // set the lo byte if we're in the middle of a write
        temp_ppuaddr += value;
        self.temp_ppuaddr = @bitCast(temp_ppuaddr);
        self.ppuaddr = self.temp_ppuaddr;
    } else { // set the hi byte
        temp_ppuaddr = @as(u16, value) << 8;
        self.temp_ppuaddr = @bitCast(temp_ppuaddr);
    }

    self.w = !self.w;
}

pub inline fn writePpudata(self: *Ppu, value: u8) !void {
    var ppuaddr: u16 = @bitCast(self.ppuaddr);
    try self.write(ppuaddr, value);
    ppuaddr += if (self.ppuctrl.i) 32 else 1;

    self.ppuaddr = @bitCast(ppuaddr);
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
                    0 => {
                        // Shift the buffered framedata before the next tile is fetched
                        self.framedata.bg_ptrn_lsb = self.framedata.bg_ptrn_lsb_buf;
                        self.framedata.bg_ptrn_msb = self.framedata.bg_ptrn_msb_buf;
                        self.framedata.attribute_entry = self.framedata.attribute_byte_buf;

                        const nametable_offset: u16 = 0x2000;
                        const addr: u16 = @bitCast(self.ppuaddr);
                        const tile = try self.read(nametable_offset + (addr & 0x0FFF));

                        self.framedata.nametable_entry = tile;
                    },
                    2 => {
                        const attribute_table_offset: u16 = 0x23C0; // 0010_0011_1100_0000
                        const nametable: u16 = @as(u16, self.ppuaddr.nametable) << 10; // 0000_nn00_0000_0000
                        const coarse_y: u16 = (self.ppuaddr.coarse_y >> 2) << 3; // 0000_0000_00yy_y000
                        const coarse_x: u16 = self.ppuaddr.coarse_x >> 2; // 0000_0000_0000_0xxx
                        const addr = attribute_table_offset | nametable | coarse_y | coarse_x; // 0010_nn11_11yy_yxxx
                        const attribute = try self.read(addr);

                        self.framedata.attribute_byte_buf = attribute;
                    },
                    4 => {
                        var tile_addr: u16 = self.framedata.nametable_entry;
                        tile_addr <<= 4; // Multiply by 16 since each pattern table entry is 16 bytes
                        tile_addr |= if (self.ppuctrl.b) 0x1000 else 0x0000;
                        const tile_row_offset: u8 = @intCast(@mod(self.scanlines, 8));
                        self.framedata.bg_ptrn_lsb_buf = try self.read(tile_addr + tile_row_offset);
                    },
                    6 => {
                        var tile_addr: u16 = self.framedata.nametable_entry;
                        tile_addr <<= 4; // Multiply by 16 since each pattern table entry is 16 bytes
                        tile_addr |= if (self.ppuctrl.b) 0x1000 else 0x0000;
                        const hi_byte_offset = 8;
                        const tile_row_offset: u8 = @intCast(@mod(self.scanlines, 8));
                        self.framedata.bg_ptrn_msb_buf = try self.read(tile_addr + tile_row_offset + hi_byte_offset);
                    },
                    else => {}, // Cycles 1, 3, 5, and 7
                }

                {
                    // Draw the current pixel

                    const quad: u3 = quadrant: {
                        const x = self.dots >> 4; // Dividing by 16 gives us the current quadrant
                        const y = self.scanlines >> 4;
                        break :quadrant @intCast(@mod(x, 2) + (@mod(y, 2) * 2));
                    };

                    // Use the quadrant to tell us what part of the attribute byte we need to read to select the color palette
                    const color_pal_id: u3 = @truncate(self.framedata.attribute_entry >> (quad * 2));

                    const palette = try self.getPaletteById(color_pal_id);

                    const pixel: u3 = @intCast(@mod(self.dots, 8)); // The pixel within the tile that we're drawing
                    const lo: u2 = @truncate(self.framedata.bg_ptrn_lsb >> (7 - pixel) & 1);
                    const hi: u2 = @truncate(self.framedata.bg_ptrn_msb >> (7 - pixel) & 1);
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
                if (self.ppuctrl.v) {
                    try self.console.cpu.?.nmi();
                }
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
    } else if (address >= 0x2000 and address <= 0x3EFF) {
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

    std.debug.print("Invalid PPU read: 0x{x}\n", .{address});
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
    } else if (address >= 0x2000 and address <= 0x3EFF) {
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

    std.debug.print("Invalid PPU write: 0x{x}\n", .{address});
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
