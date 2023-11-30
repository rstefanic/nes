const std = @import("std");
const Bus = @import("Bus.zig");
const Cpu = @import("Cpu.zig");
const raylib = @cImport({
    @cInclude("raylib.h");
});

pub fn main() !void {
    var bus = Bus{};
    var cpu = Cpu.init(&bus);

    var buffer: [1000]u8 = undefined;
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

        const mem = bus.memory[start_row..end_row];
        const font_size = 38;
        const x_spacing = 38;
        const y_spacing = 38;
        const x_padding = 10;
        const address_start = 90;
        var x_offset: c_int = x_padding;
        var y_offset: c_int = 0;

        for (mem, 0..) |m, i| {
            const current_address = start_row + i;

            if ((current_address & 0xFFF0) == current_address) {
                x_offset = x_padding;
                y_offset += y_spacing;
                const fmt = try std.fmt.allocPrintZ(allocator, "{x:0>4}", .{current_address});
                defer allocator.free(fmt);
                const c_fmt: [*c]const u8 = @ptrCast(fmt);
                raylib.DrawText(c_fmt, x_offset, y_offset, font_size, raylib.GRAY);
                x_offset = address_start;
            }

            x_offset += x_spacing;
            const fmt = try std.fmt.allocPrintZ(allocator, "{x}", .{m});
            defer allocator.free(fmt);
            const c_fmt: [*c]const u8 = @ptrCast(fmt);
            const color = if (pc == current_address) raylib.WHITE else raylib.GRAY;
            raylib.DrawText(c_fmt, x_offset, y_offset, font_size, color);
        }

        y_offset += y_spacing * 3;
        const instruction_font_size = 24;
        raylib.DrawText("Press \"s\" or \"<space>\" to step through the next instruction.", x_padding, y_offset, instruction_font_size, raylib.GRAY);
    }
}
