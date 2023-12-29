const std = @import("std");

const Cartridge = @import("Cartridge.zig");
const Console = @import("Console.zig");
const Cpu = @import("Cpu.zig");
const Palette = @import("Palette.zig");
const Ppu = @import("Ppu.zig");
const NesLog = @import("NesLog.zig");
const raylib = @cImport({
    @cInclude("raylib.h");
});

const WIDTH = 800;
const HEIGHT = 600;

pub fn main() !void {
    var buffer: [1000]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buffer);
    const allocator = fba.allocator();

    var console = Console{};
    var cpu = Cpu{ .console = &console };
    var cartridge: ?Cartridge = null;
    var ppu = Ppu{ .console = &console };

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
    try cpu.reset();

    var neslog = NesLog.init();

    // The first log entry is the expected start state of the CPU,
    // which we know is valid. So this log entry is ignored.
    _ = try neslog.next();

    raylib.SetConfigFlags(raylib.FLAG_VSYNC_HINT);
    raylib.InitWindow(WIDTH, HEIGHT, "NES");
    defer raylib.CloseWindow();

    while (!raylib.WindowShouldClose()) {
        raylib.BeginDrawing();
        defer raylib.EndDrawing();
        raylib.ClearBackground(raylib.BLACK);

        // Events
        if (raylib.IsKeyPressed(raylib.KEY_S) or raylib.IsKeyPressed(raylib.KEY_SPACE)) {
            try console.step();

            if (try neslog.next()) |log| {
                const same = log.compare(&cpu);
                if (!same) {
                    std.debug.print("NESLOG: Incorrect CPU state on line {d}\n", .{neslog.current_line_num});
                    std.debug.print("\tLOG: A:{X} X:{X} Y:{X} P:{X} SP:{X} CYC:{d}\n", .{
                        log.a,
                        log.x,
                        log.y,
                        log.p,
                        log.sp,
                        log.cycles,
                    });
                    std.debug.print("\tCPU: A:{X} X:{X} Y:{X} P:{X} SP:{X} CYC:{d}\n", .{
                        cpu.a,
                        cpu.x,
                        cpu.y,
                        @as(u8, @bitCast(cpu.status)),
                        cpu.sp,
                        cpu.cycles,
                    });
                }
            }
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

        const window_padding = 10;
        const x_start: c_int = 5;
        const y_start: c_int = 5;
        var x: c_int = x_start;
        var y: c_int = y_start;

        // Draw PPU Buffer
        var i: usize = 0;
        while (i < ppu.buffer.len) : (i += 1) {

            // The NES Display buffer is 256 x 240. We need to check if we've
            // drawn 256 pixels across here, and if we have, jump to the next
            // line to start the next row of pixels.
            //
            // Since 256 x 240 is kind of small, we're going to double the the
            // pixel display and draw a 2 x 2 rectangle for every pixel in the
            // PPU buffer.
            if ((i % 256) == 0) {
                y += 2;
                x = x_start;
            } else {
                x += 2;
            }

            const pal_code = ppu.buffer[i];
            const color = ppu.palette.colors[pal_code];

            raylib.DrawRectangle(x, y, 2, 2, raylib.Color{
                .r = color.r,
                .g = color.g,
                .b = color.b,
                .a = 255, // alpha
            });
        }

        // Screen Drawing
        const y_spacing = 42;
        var options: DrawTextOptions = .{
            .pos_x = 530,
            .pos_y = 20,
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

        // Draw instructions
        options.pos_y = 500;
        options.font_size = 22;
        options.pos_x = window_padding;
        try drawText(allocator, "Press \"s\" or \"<space>\" to step through the next instruction.", .{}, options);
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
