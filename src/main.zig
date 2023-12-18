const std = @import("std");

const Cartridge = @import("Cartridge.zig");
const Console = @import("Console.zig");
const Cpu = @import("Cpu.zig");
const Ppu = @import("Ppu.zig");
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
        }

        // Screen Drawing
        const x_spacing = 42;
        const y_spacing = 42;
        const window_padding = 10;
        var options: DrawTextOptions = .{
            .pos_x = window_padding,
            .pos_y = 0,
            .font_size = 32,
        };

        // Draw CPU Information
        options.font_size = 24;
        options.pos_x = window_padding;

        options.pos_y += y_spacing * 2;
        try drawText(allocator, "a: 0x{x:0>2}", .{cpu.a}, options);

        options.pos_x += WIDTH / 2;
        try drawText(allocator, "cycles: {d}", .{cpu.cycles}, options);

        options.pos_x = window_padding;
        options.pos_y += y_spacing;
        try drawText(allocator, "x: 0x{x:0>2}", .{cpu.x}, options);

        options.pos_x += WIDTH / 2;
        try drawText(allocator, "pc: 0x{x:0>4}", .{cpu.pc}, options);

        options.pos_x = window_padding;
        options.pos_y += y_spacing;
        try drawText(allocator, "y: 0x{x:0>2}", .{cpu.y}, options);

        options.pos_x += WIDTH / 2;
        try drawText(allocator, "sp: 0x{x:0>4}", .{cpu.sp}, options);

        // Status Registers
        options.pos_y += y_spacing;
        options.pos_x = window_padding;
        try drawText(allocator, "s:", .{}, options);

        options.pos_x += x_spacing;
        options.color = flagColor(cpu.status.negative_result);
        try drawText(allocator, "N", .{}, options);

        options.pos_x += x_spacing;
        options.color = flagColor(cpu.status.overflow);
        try drawText(allocator, "V", .{}, options);

        options.pos_x += x_spacing;
        options.color = raylib.GRAY;
        try drawText(allocator, "-", .{}, options);

        options.pos_x += x_spacing;
        options.color = raylib.GRAY;
        try drawText(allocator, "-", .{}, options);

        options.pos_x += x_spacing;
        options.color = flagColor(cpu.status.decimal_mode);
        try drawText(allocator, "D", .{}, options);

        options.pos_x += x_spacing;
        options.color = flagColor(cpu.status.interrupt_disable);
        try drawText(allocator, "I", .{}, options);

        options.pos_x += x_spacing;
        options.color = flagColor(cpu.status.zero_result);
        try drawText(allocator, "Z", .{}, options);

        options.pos_x += x_spacing;
        options.color = flagColor(cpu.status.carry);
        try drawText(allocator, "C", .{}, options);

        // Draw instructions
        options.pos_y += y_spacing * 2;
        options.pos_x = window_padding;
        options.color = raylib.GRAY;
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
