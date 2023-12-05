const Cartridge = @This();

const std = @import("std");

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
        };
    }
};

header: Header,
memory: [0xFFFF]u8,

pub fn init(filename: []u8) !Cartridge {
    var file = try std.fs.cwd().openFile(filename, .{});
    defer file.close();

    var mem: [0xFFFF]u8 = [_]u8{0} ** 0xFFFF;
    const bytes_read = try file.read(&mem);

    if (bytes_read < 16) {
        return error.InvalidRomSize;
    }

    return .{
        .memory = mem,
        .header = try Header.init(mem[0..16]),
    };
}
