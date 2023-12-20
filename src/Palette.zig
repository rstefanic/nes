const Palette = @This();
const NtscPalette = @embedFile("ntscpalette.pal");

const Color = struct { r: u8, g: u8, b: u8 };

colors: [0x40]Color = undefined,

pub fn init() Palette {
    var palette = Palette{};
    var color_idx: usize = 0;

    var i: usize = 0;
    while (i < NtscPalette.len) : (i += 3) {
        palette.colors[color_idx] = Color{
            .r = NtscPalette[i],
            .g = NtscPalette[i + 1],
            .b = NtscPalette[i + 2],
        };

        color_idx += 1;
    }

    return palette;
}
