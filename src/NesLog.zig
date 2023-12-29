const NesLog = @This();

const Cpu = @import("Cpu.zig");

const std = @import("std");

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

inline fn parseCycles(data: []const u8) !u8 {
    return try std.fmt.parseInt(u8, data[42..], 10);
}
