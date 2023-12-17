const Cartridge = @This();

const std = @import("std");
const Allocator = std.mem.Allocator;

const Header = struct {
    signature: [4]u8,
    prg_rom_size: u8,
    chr_rom_size: u8,
    flag6: packed struct(u8) {
        mirroring: bool,
        prg_ram_battery: bool,
        trainer: bool,
        ignore_mirroring: bool,
        lower_mapper_number: u4,
    },
    flag7: packed struct(u8) {
        vs_unitsystem: bool,
        play_choice: bool,
        nes_2_format: u2,
        upper_mapper_number: u4,
    },
    flag8: u8, // These flags are rarely used
    flag9: u8,
    flag10: u8,
    flag11: u8, // Flags 11 - 15 are unused padding
    flag12: u8,
    flag13: u8,
    flag14: u8,
    flag15: u8,

    pub fn init(mem: *[16]u8) !Header {
        if (mem[0] != 'N' or mem[1] != 'E' or mem[2] != 'S' or mem[3] != 0x1a) {
            return error.InvalidFileSignature;
        }

        return .{
            .signature = .{ mem[0], mem[1], mem[2], mem[3] },
            .prg_rom_size = mem[4],
            .chr_rom_size = mem[5],
            .flag6 = @bitCast(mem[6]),
            .flag7 = @bitCast(mem[7]),
            .flag8 = mem[8],
            .flag9 = mem[9],
            .flag10 = mem[10],
            .flag11 = mem[11],
            .flag12 = mem[12],
            .flag13 = mem[13],
            .flag14 = mem[14],
            .flag15 = mem[15],
        };
    }
};

allocator: Allocator,
header: Header,
prg_rom_bank: []u8,
chr_rom_bank: []u8,

const KILOBYTE: u32 = 1024;
const MEGABYTE: u32 = 1024 * KILOBYTE;

pub fn init(allocator: Allocator, filename: []u8) !Cartridge {
    var file = try std.fs.cwd().openFile(filename, .{});
    defer file.close();

    var data = try file.readToEndAlloc(allocator, MEGABYTE); // Shouldn't be larger than a MB, right?
    defer allocator.free(data);

    const header = try Header.init(data[0..@sizeOf(Header)]);
    const prg_rom_bank_size = header.prg_rom_size * (16 * KILOBYTE);
    const chr_rom_bank_size = header.chr_rom_size * (8 * KILOBYTE);

    const prg_rom_bank = try allocator.alloc(u8, prg_rom_bank_size);
    errdefer allocator.free(prg_rom_bank);
    const chr_rom_bank = try allocator.alloc(u8, chr_rom_bank_size);

    const prg_rom_idx = data.ptr + @sizeOf(Header);
    const chr_rom_idx = data.ptr + @sizeOf(Header) + prg_rom_bank_size;
    @memcpy(prg_rom_bank[0..prg_rom_bank_size], prg_rom_idx);
    @memcpy(chr_rom_bank[0..chr_rom_bank_size], chr_rom_idx);

    return .{
        .allocator = allocator,
        .header = header,
        .prg_rom_bank = prg_rom_bank,
        .chr_rom_bank = chr_rom_bank,
    };
}

pub fn deinit(self: *Cartridge) void {
    self.allocator.free(self.prg_rom_bank);
    self.allocator.free(self.chr_rom_bank);
}
