const std = @import("std");
const Bus = @import("Bus.zig");
const Cpu = @import("Cpu.zig");

const sleep = std.time.sleep;
const testing = std.testing;

pub fn main() !void {
    var bus = Bus{};
    var cpu = Cpu.init(&bus);

    try bus.write(0x0000, 0x38);

    while (true) {
        try cpu.step();
        std.debug.print("{d}\n", .{cpu.pc});
        sleep(1_000_000_000);
    }
}
