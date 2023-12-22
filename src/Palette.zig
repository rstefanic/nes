const Palette = @This();
const NtscPalette = @embedFile("ntscpalette.pal");

const Color = struct { r: u8, g: u8, b: u8 };

colors: [0x40]Color = undefined,

pub fn default() Palette {
    return Palette.init(NtscPalette);
}

pub fn init(color_data: *const [192:0]u8) Palette {
    var palette = Palette{};
    var palette_idx: usize = 0;

    var i: usize = 0;
    while (i < color_data.len) : (i += 3) {
        palette.colors[palette_idx] = Color{
            .r = color_data[i],
            .g = color_data[i + 1],
            .b = color_data[i + 2],
        };

        palette_idx += 1;
    }

    return palette;
}
