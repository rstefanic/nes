const NesLog = @This();

const Cartridge = @import("Cartridge.zig");
const Console = @import("Console.zig");
const Cpu = @import("Cpu.zig");

const std = @import("std");
const testing = std.testing;

const NesLogStep = struct {
    a: u8,
    x: u8,
    y: u8,
    p: u8,
    sp: u8,
    cycles: u64,

    pub fn compare(self: NesLogStep, cpu: *Cpu) bool {
        return self.a == cpu.a //
        and self.x == cpu.x //
        and self.y == cpu.y //
        and self.p == @as(u8, @bitCast(cpu.status)) //
        and self.sp == cpu.sp //
        and self.cycles == cpu.cycles;
    }
};

lines: std.mem.SplitIterator(u8, .sequence),
current_line_num: usize = 0,

pub fn init() NesLog {
    const nestest = @embedFile("data/nestest.log");
    return .{
        .lines = std.mem.splitSequence(u8, nestest, "\n"),
    };
}

pub fn next(self: *NesLog) !?NesLogStep {
    if (self.lines.next()) |line| {
        self.current_line_num += 1;

        const cpu_data_idx_start = 48;
        const data = line[cpu_data_idx_start..]; // Example: "A:00 X:00 Y:00 P:24 SP:FD PPU:  0, 21 CYC:7"

        // The char position for each register value is known and they never change.
        // Each parse function extracts the value and calls `parseInt` on them.
        return .{
            .a = try parseAReg(data),
            .x = try parseXReg(data),
            .y = try parseYReg(data),
            .p = try parsePReg(data),
            .sp = try parseSpReg(data),
            .cycles = try parseCycles(data),
        };
    } else {
        return null;
    }
}

inline fn parseAReg(data: []const u8) !u8 {
    return try std.fmt.parseInt(u8, data[2..4], 16);
}

inline fn parseXReg(data: []const u8) !u8 {
    return try std.fmt.parseInt(u8, data[7..9], 16);
}

inline fn parseYReg(data: []const u8) !u8 {
    return try std.fmt.parseInt(u8, data[12..14], 16);
}

inline fn parsePReg(data: []const u8) !u8 {
    return try std.fmt.parseInt(u8, data[17..19], 16);
}

inline fn parseSpReg(data: []const u8) !u8 {
    return try std.fmt.parseInt(u8, data[23..25], 16);
}

inline fn parseCycles(data: []const u8) !u64 {
    return try std.fmt.parseInt(u64, data[42..], 10);
}

test "The CPU output should match the NES Log" {
    var neslog = NesLog.init();
    var console = Console{};
    var cpu = Cpu{ .console = &console };
    var rom = "roms/nestest.nes".*;
    var cartridge: Cartridge = try Cartridge.init(testing.allocator, &rom);
    defer cartridge.deinit();

    console.connectCpu(&cpu);
    console.connectCartridge(&cartridge);
    try cpu.reset();

    // The nestest rom starts at 0xC000, so we'll
    // override whatever was set during reset.
    cpu.pc = 0xC000;

    _ = try neslog.next();
    try console.step();

    while (try neslog.next()) |log| {
        const same = log.compare(&cpu);
        if (!same) { // Report an error and fail the test
            std.debug.print("\nNESLOG: Incorrect CPU state on line {d}\n", .{neslog.current_line_num});
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

            try testing.expect(false);
        }

        try console.step();
    }

    try testing.expect(true); // Hooray! It's all good!
}
