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

const WIDTH = 1280;
const HEIGHT = 768;

pub fn main() !void {
    var buffer: [1000]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buffer);
    const allocator = fba.allocator();

    var console = Console{};
    var cpu = Cpu{ .console = &console };
    var cartridge: ?Cartridge = null;
    var ppu = Ppu{ .console = &console };
    var controller = Controller{ .console = &console };

    const args = try std.process.argsAlloc(allocator);
    if (args.len > 1) { // Treat any arguments as a filepath to a ROM
        var gpa = std.heap.GeneralPurposeAllocator(.{}){};
        const cartridge_filename = args[1];
        cartridge = try Cartridge.init(gpa.allocator(), cartridge_filename);
        console.connectCartridge(&cartridge.?);
    }
    std.process.argsFree(allocator, args);

    defer {
        if (cartridge) |_| {
            cartridge.?.deinit();
        }
    }

    console.connectCpu(&cpu);
    console.connectPpu(&ppu);
    console.connectController(&controller);
    try ppu.setupPatternTables();
    try cpu.reset();
    try ppu.reset();

    var neslog = NesLog.init();

    // The first log entry is the expected start state of the CPU,
    // which we know is valid. So this log entry is ignored.
    _ = try neslog.next();

    raylib.SetTraceLogLevel(raylib.LOG_ERROR);
    raylib.InitWindow(WIDTH, HEIGHT, "NES");
    defer raylib.CloseWindow();

    // Output Display Texture Setup
    const display_w = 256;
    const display_h = 240;
    const output_display = raylib.Rectangle{ .x = 0, .y = 0, .width = display_w, .height = display_h };
    const upscaled_output_display = raylib.Rectangle{ .x = 0, .y = 0, .width = display_w * 3, .height = display_h * 3 };
    const output_img = raylib.GenImageColor(display_w, display_h, raylib.BLACK);
    defer raylib.UnloadImage(output_img);
    var output_texture = raylib.LoadTextureFromImage(output_img);
    defer raylib.UnloadTexture(output_texture);
    raylib.SetTextureFilter(output_texture, raylib.TEXTURE_FILTER_BILINEAR);
    var output_buffer: [display_w * display_h]raylib.Color = [_]raylib.Color{raylib.BLACK} ** (display_w * display_h);

    // Left Pattern Table Texture Setup
    const left_pattern_table_display = raylib.Rectangle{ .x = 0, .y = 0, .width = 128, .height = 128 };
    const left_pattern_table_img = raylib.GenImageColor(128, 128, raylib.RED);
    defer raylib.UnloadImage(left_pattern_table_img);
    var left_pattern_table_texture = raylib.LoadTextureFromImage(left_pattern_table_img);
    defer raylib.UnloadTexture(left_pattern_table_texture);
    var left_pattern_table_buffer: [256 * 64]raylib.Color = [_]raylib.Color{raylib.BLUE} ** (256 * 64);

    // Right Pattern Table Texture Setup
    const right_pattern_table_display = raylib.Rectangle{ .x = 0, .y = 0, .width = 128, .height = 128 };
    const right_pattern_table_img = raylib.GenImageColor(128, 128, raylib.BLACK);
    defer raylib.UnloadImage(right_pattern_table_img);
    var right_pattern_table_tex = raylib.LoadTextureFromImage(right_pattern_table_img);
    defer raylib.UnloadTexture(right_pattern_table_tex);
    var right_pattern_table_buffer: [256 * 64]raylib.Color = [_]raylib.Color{raylib.GREEN} ** (256 * 64);

    while (!raylib.WindowShouldClose()) {
        raylib.BeginDrawing();
        defer raylib.EndDrawing();
        raylib.ClearBackground(raylib.BLACK);

        {
            // User Input
            controller.buttons.a = raylib.IsKeyDown(raylib.KEY_Z);
            controller.buttons.b = raylib.IsKeyDown(raylib.KEY_X);
            controller.buttons.select = raylib.IsKeyDown(raylib.KEY_F);
            controller.buttons.start = raylib.IsKeyDown(raylib.KEY_G);
            controller.buttons.up = raylib.IsKeyDown(raylib.KEY_UP);
            controller.buttons.down = raylib.IsKeyDown(raylib.KEY_DOWN);
            controller.buttons.left = raylib.IsKeyDown(raylib.KEY_LEFT);
            controller.buttons.right = raylib.IsKeyDown(raylib.KEY_RIGHT);
        }

        while (true) {
            try console.step();
            if (console.ppu.?.scanlines == -1) break;
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

        // Build Left/Right Pattern Table Textures
        build_pattern_tables: {
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

            if (cartridge) |c| {
                // If the CHR ROM bank is empty, then there's nothing to draw :(
                if (c.chr_rom_bank.len == 0) {
                    break :build_pattern_tables;
                }

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

                            left_pattern_table_buffer[idx] = left_pt_color;
                            right_pattern_table_buffer[idx] = right_pt_color;
                        }
                    }
                }

                raylib.UpdateTexture(left_pattern_table_texture, &left_pattern_table_buffer);
                raylib.UpdateTexture(right_pattern_table_tex, &right_pattern_table_buffer);
            }
        }

        { // Draw PPU Buffer
            var i: usize = 0;
            while (i < ppu.buffer.len) : (i += 1) {
                const pal_code = ppu.buffer[i];
                const color = ppu.palette.colors[pal_code];

                output_buffer[i] = raylib.Color{
                    .r = color.r,
                    .g = color.g,
                    .b = color.b,
                    .a = 255, // alpha
                };
            }

            raylib.UpdateTexture(output_texture, &output_buffer);
        }

        raylib.DrawTexturePro(output_texture, output_display, upscaled_output_display, raylib.Vector2{ .x = -5, .y = -5 }, 0.0, raylib.WHITE);
        raylib.DrawTextureRec(left_pattern_table_texture, left_pattern_table_display, raylib.Vector2{ .x = 850, .y = 5 }, raylib.WHITE);
        raylib.DrawTextureRec(right_pattern_table_tex, right_pattern_table_display, raylib.Vector2{ .x = 1050, .y = 5 }, raylib.WHITE);

        // Screen Drawing
        const y_spacing = 42;
        var options: DrawTextOptions = .{
            .pos_x = 800,
            .pos_y = 150,
            .font_size = 32,
        };

        // Draw CPU Information
        try drawText(allocator, "cycles: {d}", .{cpu.cycles}, options);

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

        raylib.DrawFPS(5, 735);
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
    var c_buf: [*c]const u8 = @ptrCast(buf); // Cast it to a C style pointer so raylib can use it
    raylib.DrawText(c_buf, options.pos_x, options.pos_y, options.font_size, options.color);
}

test {
    std.testing.refAllDecls(@This());
}
