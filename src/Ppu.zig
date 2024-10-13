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

const Sprite = packed struct(u32) {
    y: u8 = 0,
    index: u8 = 0,
    attributes: packed struct(u8) {
        palette: u2 = 0,
        _unused: u3 = 0,
        priority: bool = false,
        flip_horizontal: bool = false,
        flip_vertical: bool = false,
    } = .{},
    x: u8 = 0,
};

console: *Console,

ppuctrl: packed struct(u8) {
    nametable_addr: u2 = 0,
    i: bool = false, // VRAM address Increment -- false adds 1, true adds 32
    s: bool = false, // sprite pattern table address for 8x8 sprites
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
    show_sprites: bool = false,
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
    bg_ptrn_lsb: u8 = 0, // Current background pattern tile's lsbits and msbits
    bg_ptrn_msb: u8 = 0,

    buffer: struct {
        attribute: u2 = 0,
        bg_ptrn_lsb: u8 = 0,
        bg_ptrn_msb: u8 = 0,
    } = .{},

    shift_registers: struct {
        bg_ptrn_lsb: u16 = 0,
        bg_ptrn_msb: u16 = 0,
        attrib_lsb: u16 = 0,
        attrib_msb: u16 = 0,
    } = .{},
} = .{},

palette: Palette = Palette.default(),

// Double buffer used to prevent screen tearing. Writing occurs with the
// `writer` pointer that contains an incomplete frame while `reader` is
// a complete frame that's ready to be read.
buffer: struct {
    // The buffers are composed of 256x240 bytes where each byte is a pixel
    one: [256 * 240]u8 = [_]u8{0x2C} ** (256 * 240),
    two: [256 * 240]u8 = [_]u8{0x2C} ** (256 * 240),

    // Pointers are swapped once the writer buffer has a complete frame
    reader: *[256 * 240]u8 = undefined,
    writer: *[256 * 240]u8 = undefined,
} = .{},

palette_ram: [32]u8 = [_]u8{0x00} ** 32,
nametables: [0x800]u8 = std.mem.zeroes([0x800]u8),
oam: [64]Sprite = [_]Sprite{.{}} ** 64,

scanline_sprites: struct {
    // indexes into the OAM
    sprites: [8]u8 = [_]u8{0} ** 8,
    count: u8 = 0,
} = .{},

pub fn reset(self: *Ppu) !void {
    // A CPU reset takes 7 cycles. Since the PPU runs 3 cycles
    // per CPU cycle, we'll set the dots here to 21 to mimic that.
    self.dots = 21;

    // Start with buffer one as the writer
    self.buffer.writer = &self.buffer.one;
    self.buffer.reader = &self.buffer.two;
}

pub inline fn readPpustatus(self: *Ppu) u8 {
    const ppustatus: u8 = @bitCast(self.ppustatus);

    self.w = false; // clears the address latch bit
    self.ppustatus.vertical_blank = false; // clears the vertical blank flag

    return ppustatus;
}

pub inline fn readOamdata(self: *Ppu) u8 {
    // OAM is 256 bytes of 64 sprites where each sprite is 4 bytes
    const idx = self.oamaddr >> 2; // divide by 4 to find the correct Sprite in the OAM list

    return switch (@mod(self.oamaddr, 4)) {
        0 => self.oam[idx].y,
        1 => self.oam[idx].index,
        2 => @bitCast(self.oam[idx].attributes),
        else => self.oam[idx].x,
    };
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
    self.temp_ppuaddr.nametable = self.ppuctrl.nametable_addr;
}

pub inline fn writePpumask(self: *Ppu, value: u8) void {
    self.ppumask = @bitCast(value);
}

pub inline fn writeOamaddr(self: *Ppu, value: u8) void {
    self.oamaddr = value;
}

pub inline fn writeOamdata(self: *Ppu, value: u8) void {
    // OAM is 256 bytes of 64 sprites where each sprite is 4 bytes
    const idx = self.oamaddr >> 2; // divide by 4 to find the correct Sprite in the OAM list

    switch (@mod(self.oamaddr, 4)) {
        0 => self.oam[idx].y = value,
        1 => self.oam[idx].index = value,
        2 => self.oam[idx].attributes = @bitCast(value),
        else => self.oam[idx].x = value,
    }

    self.oamaddr = self.oamaddr +% 1;
}

pub inline fn writePpuscroll(self: *Ppu, value: u8) void {
    if (self.w) { // Second write is to Y
        self.temp_ppuaddr.fine_y = @truncate(value & 0x7); // 3 lsb
        self.temp_ppuaddr.coarse_y = @truncate(value >> 3); // 5 msb
    } else { // First write is to X
        self.x = @truncate(value & 0x7); // 3 lsb
        self.temp_ppuaddr.coarse_x = @truncate(value >> 3); // 5 msb
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
    // Pre-render line
    if (self.scanlines == -1) {
        if (self.dots == 1) {
            self.ppustatus.vertical_blank = false;
            self.ppustatus.sprite_zero_hit = false;
            self.ppustatus.sprite_overflow = false;
        }

        // Reset the vertical scroll registers for the next frame
        if (self.dots >= 280 and self.dots <= 304) {
            if (self.ppumask.show_background or self.ppumask.show_sprites) {
                const mask: u2 = 0b10;
                self.ppuaddr.nametable = (self.ppuaddr.nametable & ~mask) | (self.temp_ppuaddr.nametable & mask);
                self.ppuaddr.coarse_y = self.temp_ppuaddr.coarse_y;
                self.ppuaddr.fine_y = self.temp_ppuaddr.fine_y;
            }
        }
    }

    // Visible Scanlines
    if (self.scanlines >= 0 and self.scanlines < 240) {
        if ((self.dots > 0 and self.dots <= 257) or (self.dots >= 321 and self.dots <= 336)) {
            // Update Shifters
            if (self.ppumask.show_background) {
                self.framedata.shift_registers.bg_ptrn_lsb <<= 1;
                self.framedata.shift_registers.bg_ptrn_msb <<= 1;
                self.framedata.shift_registers.attrib_lsb <<= 1;
                self.framedata.shift_registers.attrib_msb <<= 1;
            }

            switch (@mod(self.dots - 1, 8)) {
                0 => {
                    // Shift the buffered framedata before determining the next tile
                    self.framedata.shift_registers.bg_ptrn_lsb |= self.framedata.buffer.bg_ptrn_lsb;
                    self.framedata.shift_registers.bg_ptrn_msb |= self.framedata.buffer.bg_ptrn_msb;

                    // Set the lower 8 bits of the attribute shift registers is necessary;
                    // by defaut, they're unset due to the shifts that happen each cycle
                    if ((self.framedata.buffer.attribute & 0x01) > 0) {
                        self.framedata.shift_registers.attrib_lsb |= 0xFF;
                    }
                    if ((self.framedata.buffer.attribute & 0x02) > 0) {
                        self.framedata.shift_registers.attrib_msb |= 0xFF;
                    }

                    const nametable_offset: u16 = 0x2000;
                    const addr: u16 = @bitCast(self.ppuaddr);
                    const tile = try self.read(nametable_offset + (addr & 0x0FFF));

                    self.framedata.nametable_entry = tile;
                },
                2 => {
                    const attribute_table_offset: u16 = 0x23C0; // 0010_0011_1100_0000
                    const nametable: u16 = @as(u16, self.ppuaddr.nametable) << 10; // 0000_nn00_0000_0000
                    const coarse_y: u16 = (@as(u16, self.ppuaddr.coarse_y) >> 2) << 3; // 0000_0000_00yy_y000
                    const coarse_x: u16 = @as(u16, self.ppuaddr.coarse_x) >> 2; // 0000_0000_0000_0xxx
                    const addr = attribute_table_offset | nametable | coarse_y | coarse_x; // 0010_nn11_11yy_yxxx
                    const attribute = try self.read(addr);

                    // Determine which part of the attribute byte is needed for this tile based on the quadrant
                    const quad: u3 = quadrant: {
                        const x = self.ppuaddr.coarse_x >> 1; // Dividing by 2 gives us the current x & y quadrant
                        const y = self.ppuaddr.coarse_y >> 1;
                        break :quadrant @intCast(@mod(x, 2) + (@mod(y, 2) * 2));
                    };

                    self.framedata.buffer.attribute = @truncate(attribute >> (quad * 2));
                },
                4 => {
                    var tile_addr: u16 = self.framedata.nametable_entry;
                    tile_addr <<= 4; // Multiply by 16 since each pattern table entry is 16 bytes
                    tile_addr |= if (self.ppuctrl.b) 0x1000 else 0x0000;
                    self.framedata.buffer.bg_ptrn_lsb = try self.read(tile_addr + self.ppuaddr.fine_y);
                },
                6 => {
                    var tile_addr: u16 = self.framedata.nametable_entry;
                    tile_addr <<= 4; // Multiply by 16 since each pattern table entry is 16 bytes
                    tile_addr |= if (self.ppuctrl.b) 0x1000 else 0x0000;
                    const hi_byte_offset = 8;
                    self.framedata.buffer.bg_ptrn_msb = try self.read(tile_addr + self.ppuaddr.fine_y + hi_byte_offset);
                },
                // Scroll register X increment
                7 => increment_x: {
                    if (!(self.ppumask.show_background or self.ppumask.show_sprites)) {
                        break :increment_x;
                    }

                    if (self.ppuaddr.coarse_x == 31) {
                        self.ppuaddr.coarse_x = 0;
                        self.ppuaddr.nametable ^= 0b01; // Filp the horizontal nametable (nametable X bit)
                        break :increment_x;
                    }

                    self.ppuaddr.coarse_x += 1;
                },
                else => {}, // Cycles 1, 3, and 5
            }

            // Scroll register Y increment at the end of the visible scanline
            if (self.dots == 256) increment_y: {
                if (!(self.ppumask.show_background or self.ppumask.show_sprites)) {
                    break :increment_y;
                }

                if (self.ppuaddr.fine_y < 7) {
                    self.ppuaddr.fine_y += 1;
                    break :increment_y;
                }

                self.ppuaddr.fine_y = 0;

                const y_limit: u5 = 29;
                if (self.ppuaddr.coarse_y == y_limit) {
                    self.ppuaddr.coarse_y = 0;
                    self.ppuaddr.nametable ^= 0b10; // Filp the vertical nametable (nametable Y bit)
                } else {
                    self.ppuaddr.coarse_y += 1;
                }
            }

            // Reset the horizontal scroll registers for the next scanline
            if (self.dots == 257) {
                if (self.ppumask.show_background or self.ppumask.show_sprites) {
                    const mask: u2 = 0b01;
                    self.ppuaddr.nametable = (self.ppuaddr.nametable & ~mask) | (self.temp_ppuaddr.nametable & mask);
                    self.ppuaddr.coarse_x = self.temp_ppuaddr.coarse_x;
                }
            }

            // Unused Nametable fetches
            if (self.dots == 337 or self.dots == 339) {
                const nametable_offset: u16 = 0x2000;
                const addr: u16 = @bitCast(self.ppuaddr);
                const tile = try self.read(nametable_offset + (addr & 0x0FFF));

                self.framedata.nametable_entry = tile;
            }
        }
    }

    // Sprite evaluation for next scanline
    if (self.scanlines < 240) {
        if (self.dots == 340) {
            var count: u8 = 0;
            var idx: u8 = 0;
            while (idx < 64) : (idx += 1) sprite_eval: {
                const sprite = self.oam[idx];

                // Add one looking for sprites on the next scanline
                // Add another one since sprite locations are one pixel below its 'y' coordinate.
                const diff: i16 = self.scanlines - (sprite.y +% 1) + 1;
                const sprite_height: usize = if (self.ppuctrl.h) 16 else 8;

                if (diff >= 0 and diff < sprite_height) {
                    if (count < 8) {
                        self.scanline_sprites.sprites[count] = idx;
                        count += 1;
                    } else {
                        // Set the sprite overflow flag and exit if we reach this
                        self.ppustatus.sprite_overflow = true;
                        break :sprite_eval;
                    }
                }
            }

            self.scanline_sprites.count = count;
        }
    }

    // Vertical blanking period
    if (self.scanlines == 241) {
        if (self.dots == 1) {
            self.ppustatus.vertical_blank = true;

            if (self.buffer.reader == &self.buffer.one) {
                self.buffer.reader = &self.buffer.two;
                self.buffer.writer = &self.buffer.one;
            } else {
                self.buffer.reader = &self.buffer.one;
                self.buffer.writer = &self.buffer.two;
            }

            if (self.ppuctrl.v) {
                try self.console.cpu.?.nmi();
            }
        }
    }

    // Dot 0 is an "idle" cycle and the data fetching for pixel data doesn't
    // start until dot 1. Thus the visible dots are 1 through 256 (inclusive)
    // since there are 256 horizontal pixels per frame.
    const is_visible_dot = (self.dots > 0 and self.dots <= visible_dots_per_scanline);
    const is_visible_scanline = (self.scanlines > -1) and (self.scanlines < 240);

    if (is_visible_dot and is_visible_scanline) {
        var pixel: u8 = (try self.getPaletteById(0))[0]; // use the backdrop pixel as a default
        var bg_transparent = false;

        if (self.ppumask.show_background) background: {
            const shift_offset: u16 = @as(u16, 0x8000) >> @as(u4, @truncate(self.x));

            const tile_lo: u2 = if ((self.framedata.shift_registers.bg_ptrn_lsb & shift_offset) > 0) 1 else 0;
            const tile_hi: u2 = if ((self.framedata.shift_registers.bg_ptrn_msb & shift_offset) > 0) 2 else 0;
            const bg_pixel: u2 = tile_hi | tile_lo;
            bg_transparent = bg_pixel == 0x00;

            const attrib_lo: u2 = if ((self.framedata.shift_registers.attrib_lsb & shift_offset) > 0) 1 else 0;
            const attrib_hi: u2 = if ((self.framedata.shift_registers.attrib_msb & shift_offset) > 0) 2 else 0;
            const palette_id: u2 = attrib_hi | attrib_lo;
            const palette = try self.getPaletteById(palette_id);

            if (!self.ppumask.show_leftmost_background and self.dots <= 8) {
                break :background;
            }

            if (!bg_transparent) {
                pixel = palette[bg_pixel];
            }
        }

        if (self.ppumask.show_sprites) {
            // We've determined which sprites to draw by going through the OAM
            // list already. Sprites in OAM are listed by priority, iterating
            // through them in reverse here will automatically overwrite the
            // lower priority sprites with the higher priority ones.
            var i: usize = self.scanline_sprites.count;
            std.debug.assert(i <= 8);
            while (i > 0) : (i -= 1) {
                const idx = self.scanline_sprites.sprites[i - 1];
                const sprite = self.oam[idx];
                const x_diff = self.dots - sprite.x - 1; // subtract one to compensate for dot 1 being the first visible dot
                const y_diff = self.scanlines - (sprite.y +% 1); // sprite locations are one pixel below its 'y' coordinate

                if (x_diff >= 0 and x_diff < 8) sprite: {
                    const palette = try self.getPaletteById(@as(u8, sprite.attributes.palette) + 4); // +4 to access the FG palettes

                    var tile_addr: u16 = sprite.index;
                    tile_addr <<= 4; // Multiply by 16 since each pattern table entry is 16 bytes
                    tile_addr |= if (self.ppuctrl.s) 0x1000 else 0x0000;

                    if (sprite.attributes.flip_vertical) {
                        const sprite_height: u16 = if (self.ppuctrl.h) 16 else 8;
                        tile_addr |= (sprite_height - 1) - @as(u16, @intCast(y_diff));
                    } else {
                        tile_addr |= @as(u16, @intCast(y_diff));
                    }

                    var tile_lo = try self.read(tile_addr);
                    var tile_hi = try self.read(tile_addr + 8);

                    if (sprite.attributes.flip_horizontal) {
                        tile_lo = @bitReverse(tile_lo);
                        tile_hi = @bitReverse(tile_hi);
                    }

                    const pixel_offset = @as(u8, 0x80) >> @as(u3, @intCast(x_diff));
                    const pixel_lo: u2 = if ((tile_lo & pixel_offset) > 0) 1 else 0;
                    const pixel_hi: u2 = if ((tile_hi & pixel_offset) > 0) 2 else 0;
                    const fg_pixel: u2 = pixel_hi | pixel_lo;
                    const fg_transparent = fg_pixel == 0x00;

                    if (fg_transparent) {
                        break :sprite;
                    }

                    if (idx == 0) { // detect if sprite hit zero has occurred
                        if (self.ppumask.show_background and !bg_transparent) {
                            // Account for the leftmost sprite and background flags from the PPUMASK. If both the leftmost
                            // sprites and background are visible, then it's a hit. Otherwise, if one is hidden, then the
                            // sprite hit zero can only occurs if we're right of the 8 leftmost pixels.
                            if (self.ppumask.show_leftmost_sprites and self.ppumask.show_leftmost_background) {
                                self.ppustatus.sprite_zero_hit = true;
                            } else if (self.dots > 8) {
                                self.ppustatus.sprite_zero_hit = true;
                            }
                        }
                    }

                    if (!self.ppumask.show_leftmost_sprites and self.dots <= 8) {
                        break :sprite;
                    }

                    if (!bg_transparent and sprite.attributes.priority) {
                        break :sprite;
                    }

                    pixel = palette[fg_pixel];
                }
            }
        }

        // Subtract 1 from the dots since the visible dots start at 1 and the buffer is zero-based
        const x: usize = @intCast(self.dots - 1);
        const y: usize = @as(usize, @intCast(self.scanlines)) << 8;
        self.buffer.writer.*[x + y] = pixel;
    }

    // Increment the dots and scanlines
    self.dots += 1;
    if (self.dots >= dots_per_scanline) {
        self.dots = 0;
        self.scanlines += 1;

        if (self.scanlines >= scanlines_per_frame) {
            self.scanlines = -1;
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
                return self.nametables[address & 0x07FF];
            }

            // vertical arrangement, horizontally mirrored
            if (address >= 0x2000 and address <= 0x27FF) {
                return self.nametables[address & 0x03FF];
            } else {
                return self.nametables[address & 0x07FF];
            }
        } else {
            return PpuMemoryAccessError.MissingCartridge;
        }
    } else if (address >= 0x3F00 and address <= 0x3FFF) {
        // Addresses $3F20-$3FFF are mirrors of the first 32 bytes for our palette RAM. So take
        // the lower 5 bits of the address to index into the palette RAM and mimic the mirroring.
        var pal_addr = address & 0x001F;

        // Entry 0 of each palette is a mirror the backdrop color
        // https://www.nesdev.org/wiki/PPU_palettes#Palette_RAM
        if ((pal_addr % 4) == 0) {
            pal_addr &= 0x000F;
        }

        return self.palette_ram[pal_addr];
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
                self.nametables[address & 0x07FF] = value;
                return;
            }

            // vertical arrangement, horizontally mirrored
            if (address >= 0x2000 and address <= 0x27FF) {
                self.nametables[address & 0x03FF] = value;
                return;
            } else {
                self.nametables[address & 0x07FF] = value;
                return;
            }
        } else {
            return PpuMemoryAccessError.MissingCartridge;
        }
    } else if (address >= 0x3F00 and address <= 0x3FFF) {
        var pal_addr = address & 0x001F;

        if ((pal_addr % 4) == 0) {
            pal_addr &= 0x000F;
        }

        self.palette_ram[pal_addr] = value;
        return;
    }

    std.debug.print("Invalid PPU write: 0x{x}\n", .{address});
    return PpuMemoryAccessError.InvalidMemoryAddress;
}
