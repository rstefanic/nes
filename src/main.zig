const std = @import("std");

const Cartridge = @import("Cartridge.zig");
const Console = @import("Console.zig");
const Cpu = @import("Cpu.zig");
const Palette = @import("Palette.zig");
const Ppu = @import("Ppu.zig");
const Controller = @import("Controller.zig");
const NesLog = @import("NesLog.zig");
const raylib = @cImport({
    @cInclude("raylib.h");
});

const WINDOW_WIDTH = 1280;
const WINDOW_HEIGHT = 768;
const TIME_PER_FRAME = 1.0 / 60.0;

const DISPLAY_WIDTH = 256;
const DISPLAY_HEIGHT = 240;

const PALETTE_WIDTH = 128;
const PALETTE_HEIGHT = 32;
const PALETTE_COUNT = 8;

const PatternTableDisplay = struct {
    texture: raylib.Texture2D,
    buffer: [256 * 64]raylib.Color,
};

var mode: enum { Debug, Regular } = .Regular;

const Display = struct {
    // The output is where the PPU frame is rendered. It's made up of a texture
    // which lives in the GPU memory while the buffer contains the raw pixel
    // data. The PPU's palette colors are translated into raylib colors before
    // the data is copied from the buffer to the texture on each frame.
    output: struct {
        texture: raylib.Texture2D,
        buffer: [DISPLAY_WIDTH * DISPLAY_HEIGHT]raylib.Color,
    },

    debug: struct {
        pattern_table: struct {
            left: PatternTableDisplay,
            right: PatternTableDisplay,
        },

        palette: struct {
            textures: [PALETTE_COUNT]raylib.Texture2D,
            buffers: [PALETTE_COUNT][PALETTE_WIDTH * PALETTE_HEIGHT]raylib.Color,
        },
    },
};

pub fn main() !void {
    var buffer: [1000]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buffer);
    const allocator = fba.allocator();

    var console = Console{};
    var cpu = Cpu{ .console = &console };
    var cartridge: ?Cartridge = null;
    var ppu = Ppu{ .console = &console };
    var controller1 = Controller{ .console = &console };
    var controller2 = Controller{ .console = &console };

    // Parse program arguments
    {
        const args = try std.process.argsAlloc(allocator);
        defer std.process.argsFree(allocator, args);

        var i: usize = 1; // start at one to skip the program name
        const max_args_count = 2;
        while (i < args.len and i <= max_args_count) : (i += 1) {
            const arg = args[i];
            const debug_flag = "--d";

            if (std.mem.eql(u8, arg, debug_flag)) {
                mode = .Debug;
                continue;
            }

            // Treat the argument as the ROM filename to try and open it
            var gpa = std.heap.GeneralPurposeAllocator(.{}){};
            cartridge = try Cartridge.init(gpa.allocator(), arg);
            if (cartridge.?.header.mapperNumber() != 0) {
                std.debug.print("ROM must be mapper 0\n", .{});
                cartridge.?.deinit();
                return;
            }
        }
    }

    defer {
        if (cartridge) |_| {
            cartridge.?.deinit();
        }
    }

    console.connectCpu(&cpu);
    console.connectPpu(&ppu);
    console.connectCartridge(&cartridge.?);
    console.connectController1(&controller1);
    console.connectController2(&controller2);
    try ppu.setupPatternTables();
    try cpu.reset();
    try ppu.reset();

    var neslog = NesLog.init();

    // The first log entry is the expected start state of the CPU,
    // which we know is valid. So this log entry is ignored.
    _ = try neslog.next();

    raylib.SetTraceLogLevel(raylib.LOG_ERROR);
    raylib.InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "NES");
    defer raylib.CloseWindow();

    // Output Display Setup
    var display: Display = undefined;

    const image = raylib.GenImageColor(DISPLAY_WIDTH, DISPLAY_HEIGHT, raylib.BLACK);
    display.output.texture = raylib.LoadTextureFromImage(image);
    display.output.buffer = [_]raylib.Color{raylib.BLACK} ** (DISPLAY_WIDTH * DISPLAY_HEIGHT);
    raylib.SetTextureFilter(display.output.texture, raylib.TEXTURE_FILTER_BILINEAR);

    raylib.UnloadImage(image);
    defer raylib.UnloadTexture(display.output.texture);

    // Pattern Table Setup
    const pattern_table_image = raylib.GenImageColor(128, 128, raylib.RED);
    display.debug.pattern_table.left.texture = raylib.LoadTextureFromImage(pattern_table_image);
    display.debug.pattern_table.left.buffer = [_]raylib.Color{raylib.BLUE} ** (256 * 64);
    display.debug.pattern_table.right.texture = raylib.LoadTextureFromImage(pattern_table_image);
    display.debug.pattern_table.right.buffer = [_]raylib.Color{raylib.BLUE} ** (256 * 64);

    raylib.UnloadImage(pattern_table_image);
    defer raylib.UnloadTexture(display.debug.pattern_table.left.texture);
    defer raylib.UnloadTexture(display.debug.pattern_table.right.texture);

    // Palette Setup
    const palette_image = raylib.GenImageColor(PALETTE_WIDTH, PALETTE_HEIGHT, raylib.BLACK);

    // Create the palette textures and buffers
    {
        var i: usize = 0;
        while (i < PALETTE_COUNT) : (i += 1) {
            display.debug.palette.textures[i] = raylib.LoadTextureFromImage(palette_image);
        }

        display.debug.palette.buffers = .{[_]raylib.Color{raylib.BLACK} ** (PALETTE_WIDTH * PALETTE_HEIGHT)} ** PALETTE_COUNT;
    }

    raylib.UnloadImage(palette_image);

    // Unload the allocated textures together at the end
    defer {
        var i: usize = 0;
        while (i < PALETTE_COUNT) : (i += 1) {
            raylib.UnloadTexture(display.debug.palette.textures[i]);
        }
    }

    while (!raylib.WindowShouldClose()) {
        raylib.BeginDrawing();
        defer raylib.EndDrawing();
        raylib.ClearBackground(raylib.BLACK);

        { // User Input
            // Controller 1
            controller1.buttons.a = raylib.IsKeyDown(raylib.KEY_Z);
            controller1.buttons.b = raylib.IsKeyDown(raylib.KEY_X);
            controller1.buttons.select = raylib.IsKeyDown(raylib.KEY_F);
            controller1.buttons.start = raylib.IsKeyDown(raylib.KEY_G);
            controller1.buttons.up = raylib.IsKeyDown(raylib.KEY_UP);
            controller1.buttons.down = raylib.IsKeyDown(raylib.KEY_DOWN);
            controller1.buttons.left = raylib.IsKeyDown(raylib.KEY_LEFT);
            controller1.buttons.right = raylib.IsKeyDown(raylib.KEY_RIGHT);

            // Controller 2
            controller2.buttons.a = raylib.IsKeyDown(raylib.KEY_KP_0);
            controller2.buttons.b = raylib.IsKeyDown(raylib.KEY_KP_DECIMAL);
            controller2.buttons.select = raylib.IsKeyDown(raylib.KEY_KP_ADD);
            controller2.buttons.start = raylib.IsKeyDown(raylib.KEY_KP_ENTER);
            controller2.buttons.up = raylib.IsKeyDown(raylib.KEY_KP_8);
            controller2.buttons.down = raylib.IsKeyDown(raylib.KEY_KP_5);
            controller2.buttons.left = raylib.IsKeyDown(raylib.KEY_KP_4);
            controller2.buttons.right = raylib.IsKeyDown(raylib.KEY_KP_6);
        }

        // Simulate enough of the Console to draw a frame every second
        const start = raylib.GetTime();
        var dt: f64 = 0;
        while (dt < TIME_PER_FRAME) {
            try console.step();
            dt = raylib.GetTime() - start;
        }

        if (raylib.IsFileDropped()) {
            const file = raylib.LoadDroppedFiles();
            defer raylib.UnloadDroppedFiles(file);
            const filename = file.paths.*;

            attempt_palette_load: {
                if (!raylib.IsFileExtension(filename, ".pal")) {
                    std.debug.print("NES: Unrecognized file format \"{s}\". Expected \".pal\".\n", .{raylib.GetFileExtension(filename)});
                    break :attempt_palette_load;
                }

                const file_size = raylib.GetFileLength(filename);
                const palette_size = 192;
                if (file_size != palette_size) {
                    std.debug.print("NES: Incorrect palette size. Expected \".pal\" file size {d} but received file size {d}.\n", .{ palette_size, file_size });
                    break :attempt_palette_load;
                }

                var bytes_read: c_int = 0;
                const palette = raylib.LoadFileData(filename, &bytes_read);
                ppu.palette = Palette.init(@ptrCast(palette));
            }
        }

        drawOutput(&display, &ppu);
        try drawPatternTables(&display, &ppu);
        try drawPalettes(&display, &ppu, allocator);
        try printConsoleInfo(&console, allocator);

        raylib.DrawFPS(5, 735);
    }
}

fn drawOutput(display: *Display, ppu: *Ppu) void {
    var i: usize = 0;
    while (i < ppu.buffer.len) : (i += 1) {
        const pal_code = ppu.buffer[i];
        const color = ppu.palette.colors[pal_code];

        display.output.buffer[i] = raylib.Color{
            .r = color.r,
            .g = color.g,
            .b = color.b,
            .a = 255, // alpha
        };
    }

    raylib.UpdateTexture(display.output.texture, &display.output.buffer);
    raylib.DrawTextureEx(display.output.texture, raylib.Vector2{ .x = 0, .y = 0 }, 0.0, 3.0, raylib.WHITE);
}

fn drawPatternTables(display: *Display, ppu: *Ppu) !void {
    const palette = try ppu.getPaletteById(1);
    const color1 = ppu.palette.colors[palette[0]];
    const color2 = ppu.palette.colors[palette[1]];
    const color3 = ppu.palette.colors[palette[2]];
    const color4 = ppu.palette.colors[palette[3]];

    const colors: [4]raylib.Color = [_]raylib.Color{
        raylib.Color{ .r = color1.r, .g = color1.g, .b = color1.b, .a = 255 },
        raylib.Color{ .r = color2.r, .g = color2.g, .b = color2.b, .a = 255 },
        raylib.Color{ .r = color3.r, .g = color3.g, .b = color3.b, .a = 255 },
        raylib.Color{ .r = color4.r, .g = color4.g, .b = color4.b, .a = 255 },
    };

    const tile_height = 8; // Each tile in the pattern table is made up of 8x8 pixels
    const tile_width = 8;
    const pt_display_width = 128; // This is the width of the pattern table texture
    var x: u32 = 0; // x & y here are used to index into the left/right pattern table buffers
    var y: u32 = 0;
    var i: usize = 0; // Keeps track of which tile we're looking at in the pattern tables

    // Add each tile from the left pattern table to the corresponding texture. Since the left and right pattern
    // tables are the same size and they're built in the same way, we can build the right pattern table at
    // the same time using the same indexes used to access the left pattern table's tiles and buffers.
    while (i < ppu.left_pattern_table.len) : (i += 1) {
        if (i > 0) {
            // The pattern tables are made up of 16x16 tiles (where each tile is 8x8 pixels).
            // For every 16 tiles, we want to start drawing the tiles on the next "row".
            if ((i % 16) == 0) {
                y += 8;
                x = 0;
            } else {
                x += 8;
            }
        }

        var j: u32 = 0;

        // Go through this tile's pixels and add them to the pattern table buffers.
        while (j < tile_height) : (j += 1) { // Tile Row

            var k: u32 = 0;
            const tile_row = j * tile_height;
            while (k < tile_width) : (k += 1) { // Tile Column
                const left_pt_pixel = ppu.left_pattern_table[i][tile_row + k];
                const left_pt_color = colors[left_pt_pixel];

                const right_pt_pixel = ppu.right_pattern_table[i][tile_row + k];
                const right_pt_color = colors[right_pt_pixel];

                const buffer_column = x + k;
                const buffer_row = (y + j) * pt_display_width;
                const idx = buffer_column + buffer_row;

                display.debug.pattern_table.left.buffer[idx] = left_pt_color;
                display.debug.pattern_table.right.buffer[idx] = right_pt_color;
            }
        }
    }

    raylib.UpdateTexture(display.debug.pattern_table.left.texture, &display.debug.pattern_table.left.buffer);
    raylib.UpdateTexture(display.debug.pattern_table.right.texture, &display.debug.pattern_table.right.buffer);
    raylib.DrawTexture(display.debug.pattern_table.left.texture, 850, 5, raylib.WHITE);
    raylib.DrawTexture(display.debug.pattern_table.right.texture, 1050, 5, raylib.WHITE);
}

fn drawPalettes(display: *Display, ppu: *Ppu, allocator: std.mem.Allocator) !void {
    // Update palette textures
    {
        var i: u8 = 0;
        while (i < PALETTE_COUNT) : (i += 1) {
            const palette = try ppu.getPaletteById(i);
            var buf = display.debug.palette.buffers[i];

            var j: usize = 0;
            while (j < 4) : (j += 1) { // There are 4 colors per palette
                const color = ppu.palette.colors[palette[j]];
                const offset = PALETTE_HEIGHT * j;

                // Each of the 4 colors that make up a palette will be displayed as 32x32 rectangles
                var k: usize = 0;
                while (k < 32) : (k += 1) {
                    var l: usize = 0;
                    while (l < 32) : (l += 1) {
                        const idx = (offset + k) + (l * PALETTE_WIDTH);
                        buf[idx] = raylib.Color{
                            .r = color.r,
                            .g = color.g,
                            .b = color.b,
                            .a = 255, // alpha
                        };
                    }
                }
            }

            raylib.UpdateTexture(display.debug.palette.textures[i], &buf);
        }
    }

    // Draw palette textures
    {
        const bg_pal_column = 800;
        const fg_pal_column = 1050;
        const y_start = 500;
        const y_padding = 38;

        var options: DrawTextOptions = .{
            .pos_x = bg_pal_column,
            .pos_y = y_start - y_padding, // palettes start at `y_start`; add a label above it
            .font_size = 24,
        };

        try drawText(allocator, "BG Palettes", .{}, options);
        options.pos_x = fg_pal_column;
        try drawText(allocator, "FG Palettes", .{}, options);

        var x: c_int = bg_pal_column;
        var y: c_int = y_start;

        var i: usize = 0;
        while (i < 8) : (i += 1) {
            raylib.DrawTexture(display.debug.palette.textures[i], x, y, raylib.WHITE);

            // The palette buffer blocks are 32 pixels high.
            // 6 pixels are added for spacing here
            y += y_padding;

            if (@mod(i + 1, 4) == 0) {
                x = fg_pal_column;
                y = y_start; // reset y
            }
        }
    }
}

fn printConsoleInfo(console: *Console, allocator: std.mem.Allocator) !void {
    const cpu = console.cpu.?;

    // Screen Drawing
    const y_spacing = 42;
    var options: DrawTextOptions = .{
        .pos_x = 800,
        .pos_y = 150,
        .font_size = 32,
    };

    // Draw CPU Information
    try drawText(allocator, "cycles: {d}", .{console.cycles}, options);

    options.pos_y += y_spacing;
    try drawText(allocator, "a: 0x{x:0>2}", .{cpu.a}, options);

    options.pos_y += y_spacing;
    try drawText(allocator, "x: 0x{x:0>2}", .{cpu.x}, options);

    options.pos_y += y_spacing;
    try drawText(allocator, "y: 0x{x:0>2}", .{cpu.y}, options);

    options.pos_y += y_spacing;
    try drawText(allocator, "pc: 0x{x:0>4}", .{cpu.pc}, options);

    options.pos_y += y_spacing;
    try drawText(allocator, "sp: 0x{x:0>4}", .{cpu.sp}, options);

    { // Status Registers
        const sr_spacing: c_int = 24;
        options.font_size = 28;
        options.pos_y += y_spacing;
        try drawText(allocator, "s:", .{}, options);

        options.pos_x += sr_spacing * 2;
        options.color = flagColor(cpu.status.negative_result);
        try drawText(allocator, "N", .{}, options);

        options.pos_x += sr_spacing;
        options.color = flagColor(cpu.status.overflow);
        try drawText(allocator, "V", .{}, options);

        options.pos_x += sr_spacing;
        options.color = raylib.GRAY;
        try drawText(allocator, "-", .{}, options);

        options.pos_x += sr_spacing;
        options.color = raylib.GRAY;
        try drawText(allocator, "-", .{}, options);

        options.pos_x += sr_spacing;
        options.color = flagColor(cpu.status.decimal_mode);
        try drawText(allocator, "D", .{}, options);

        options.pos_x += sr_spacing;
        options.color = flagColor(cpu.status.interrupt_disable);
        try drawText(allocator, "I", .{}, options);

        options.pos_x += sr_spacing;
        options.color = flagColor(cpu.status.zero_result);
        try drawText(allocator, "Z", .{}, options);

        options.pos_x += sr_spacing;
        options.color = flagColor(cpu.status.carry);
        try drawText(allocator, "C", .{}, options);

        options.color = raylib.GRAY; // Reset color
    }
}

inline fn flagColor(state: bool) raylib.Color {
    return if (state) raylib.GREEN else raylib.RED;
}

const DrawTextOptions = struct {
    pos_x: c_int = 0,
    pos_y: c_int = 0,
    font_size: c_int = 12,
    color: raylib.Color = raylib.GRAY,
};

fn drawText(allocator: std.mem.Allocator, comptime fmt: []const u8, args: anytype, options: DrawTextOptions) !void {
    const buf = try std.fmt.allocPrintZ(allocator, fmt, args);
    defer allocator.free(buf);
    const c_buf: [*c]const u8 = @ptrCast(buf); // Cast it to a C style pointer so raylib can use it
    raylib.DrawText(c_buf, options.pos_x, options.pos_y, options.font_size, options.color);
}

test {
    std.testing.refAllDecls(@This());
}
