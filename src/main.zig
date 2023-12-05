const std = @import("std");

const Cartridge = @import("Cartridge.zig");
const Console = @import("Console.zig");
const Cpu = @import("Cpu.zig");
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
    var cpu = Cpu.init(&console);

    const args = try std.process.argsAlloc(allocator);
    if (args.len > 1) { // Treat any arguments as a filepath to a ROM
        const cartridge_filename = args[1];
        var cartridge = try Cartridge.init(cartridge_filename);
        console.connectCartridge(&cartridge);
    }
    std.process.argsFree(allocator, args);

    raylib.SetConfigFlags(raylib.FLAG_VSYNC_HINT);
    raylib.InitWindow(WIDTH, HEIGHT, "NES");
    defer raylib.CloseWindow();

    while (!raylib.WindowShouldClose()) {
        raylib.BeginDrawing();
        defer raylib.EndDrawing();
        raylib.ClearBackground(raylib.BLACK);

        // Events

        if (raylib.IsKeyPressed(raylib.KEY_S) or raylib.IsKeyPressed(raylib.KEY_SPACE)) {
            try cpu.step();
        }

        // Screen Drawing

        const x_spacing = 42;
        const y_spacing = 42;
        const window_padding = 10;
        const address_start = 80;
        var options: DrawTextOptions = .{
            .pos_x = window_padding,
            .pos_y = 0,
            .font_size = 32,
        };

        // Draw Memory

        const mdd = MemoryDebugDisplay.init(console.memory, cpu.pc);
        for (mdd.memory_display, 0..) |mem, i| {
            const current_address = mdd.starting_address + i;

            if ((current_address & 0xFFF0) == current_address) {
                options.pos_x = window_padding;
                options.pos_y += y_spacing;
                try drawText(allocator, "{x:0>4}", .{current_address}, options);
                options.pos_x = address_start;
            }

            options.pos_x += x_spacing;
            options.color = if (cpu.pc == current_address) raylib.WHITE else raylib.GRAY;
            try drawText(allocator, "{x}", .{mem}, options);
            options.color = raylib.GRAY; // Reset color back to GRAY
        }

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

const MemoryDebugDisplay = struct {
    memory_display: []const u8,
    starting_address: u16,

    pub fn init(memory: [0x10000]u8, pc: u16) MemoryDebugDisplay {
        var start_row: u16 = undefined;
        var end_row: u16 = undefined;

        // We want to show five rows from memory at all times. Therefore, if
        // the PC is within the first two rows or the last three rows, then we
        // manually adjust the window to fix it at the start or end so that we
        // don't overflow; otherwise we just calculate the start and end by
        // taking two rows before the PC and three rows after the PC.
        if (pc < 0x0020) {
            start_row = 0x0000;
            end_row = 0x0050;
        } else if (pc > 0xFFC0) {
            start_row = 0xFFA0;
            end_row = 0xFFF0;
        } else {
            start_row = (pc & 0xFFF0) - 0x0020;
            end_row = (pc & 0xFFF0) + 0x0030;
        }

        return .{
            .memory_display = memory[start_row..end_row],
            .starting_address = start_row,
        };
    }
};
