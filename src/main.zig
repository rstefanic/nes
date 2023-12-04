const std = @import("std");
const Bus = @import("Bus.zig");
const Cpu = @import("Cpu.zig");
const raylib = @cImport({
    @cInclude("raylib.h");
});

pub fn main() !void {
    var bus = Bus{};
    var cpu = Cpu.init(&bus);

    var buffer: [256]u8 = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buffer);
    const allocator = fba.allocator();

    raylib.SetConfigFlags(raylib.FLAG_VSYNC_HINT);
    raylib.InitWindow(800, 600, "NES");
    defer raylib.CloseWindow();

    while (!raylib.WindowShouldClose()) {
        raylib.BeginDrawing();
        defer raylib.EndDrawing();
        raylib.ClearBackground(raylib.BLACK);

        // Events
        if (raylib.IsKeyPressed(raylib.KEY_S) or raylib.IsKeyPressed(raylib.KEY_SPACE)) {
            try cpu.step();
        }

        var start_row: u16 = undefined;
        var end_row: u16 = undefined;
        const pc = cpu.pc;

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

        // Screen Drawing

        const mem = bus.memory[start_row..end_row];
        const mem_font_size = 32;
        const instruction_font_size = 24;
        const x_spacing = 42;
        const y_spacing = 42;
        const x_padding = 10;
        const address_start = 80;
        var x_offset: c_int = x_padding;
        var y_offset: c_int = 0;

        // Draw Memory

        for (mem, 0..) |m, i| {
            const current_address = start_row + i;

            if ((current_address & 0xFFF0) == current_address) {
                x_offset = x_padding;
                y_offset += y_spacing;
                const fmt = try std.fmt.allocPrintZ(allocator, "{x:0>4}", .{current_address});
                defer allocator.free(fmt);
                const c_fmt: [*c]const u8 = @ptrCast(fmt);
                raylib.DrawText(c_fmt, x_offset, y_offset, mem_font_size, raylib.GRAY);
                x_offset = address_start;
            }

            x_offset += x_spacing;
            const fmt = try std.fmt.allocPrintZ(allocator, "{x}", .{m});
            defer allocator.free(fmt);
            const c_fmt: [*c]const u8 = @ptrCast(fmt);
            const color = if (pc == current_address) raylib.WHITE else raylib.GRAY;
            raylib.DrawText(c_fmt, x_offset, y_offset, mem_font_size, color);
        }

        // Draw CPU Information

        y_offset += y_spacing * 2;
        var options: DrawTextOptions = .{ .pos_x = x_padding, .pos_y = y_offset, .font_size = instruction_font_size };
        try drawText(allocator, "a: 0x{x:0>2}", .{cpu.a}, options);

        options.pos_x += 300;
        try drawText(allocator, "cycles: {d}", .{cpu.cycles}, options);

        options.pos_x = x_padding;
        options.pos_y += y_spacing;
        try drawText(allocator, "x: 0x{x:0>2}", .{cpu.x}, options);

        options.pos_x += 300;
        try drawText(allocator, "pc: 0x{x:0>4}", .{cpu.pc}, options);

        options.pos_x = x_padding;
        options.pos_y += y_spacing;
        try drawText(allocator, "y: 0x{x:0>2}", .{cpu.y}, options);

        options.pos_x += 300;
        try drawText(allocator, "sp: 0x{x:0>4}", .{cpu.sp}, options);

        // Status Registers
        options.pos_y += y_spacing;
        options.pos_x = x_padding;
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
        options.pos_x = x_padding;
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
