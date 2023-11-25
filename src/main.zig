const std = @import("std");
const Bus = @import("Bus.zig");
const Cpu = @import("Cpu.zig");
const raylib = @cImport({
    @cInclude("raylib.h");
});

pub fn main() !void {
    var bus = Bus{};
    _ = Cpu.init(&bus);

    raylib.SetConfigFlags(raylib.FLAG_VSYNC_HINT);
    raylib.InitWindow(800, 600, "NES");
    defer raylib.CloseWindow();

    while (!raylib.WindowShouldClose()) {
        raylib.BeginDrawing();
        defer raylib.EndDrawing();

        raylib.ClearBackground(raylib.RAYWHITE);
        raylib.DrawText("Work in progress...", 300, 250, 20, raylib.LIGHTGRAY);
    }
}
