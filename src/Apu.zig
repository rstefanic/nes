const Apu = @This();

const Console = @import("Console.zig");

const PulseChannel = struct {
    envelope: packed struct(u8) {
        volume_envelope: u4 = 0,
        constant_volume: bool = false,
        envelope_loop: bool = false,
        duty: u2 = 0,
    } = .{},

    sweep: packed struct(u8) {
        shift: u3 = 0,
        negate: bool = false,
        period: u3 = 0,
        enable: bool = false,
    } = .{},

    timer_low: u8 = 0,

    length_counter: packed struct(u8) {
        timer_high: u3 = 0,
        length_counter_load: u5 = 0,
    } = .{},
};

console: *Console,

pulse_one: PulseChannel = .{},
pulse_two: PulseChannel = .{},

triangle: struct {
    envelope: packed struct(u8) {
        volume_envelope: u4 = 0,
        constant_volume: bool = false,
        envelope_loop: bool = false,
        duty: u2 = 0,
    } = .{},
    timer_low: u8 = 0,
    length_counter: packed struct(u8) {
        timer_high: u3 = 0,
        length_counter_load: u5 = 0,
    } = .{},
} = .{},

noise: struct {
    envelope: packed struct(u8) {
        volume_envelope: u4 = 0,
        constant_volume: bool = false,
        envelope_loop: bool = false,
        _unused: u2 = 0,
    } = .{},
    loop_noise: packed struct(u8) {
        noise_period: u4 = 0,
        _unused: u3 = 0,
        loop: bool = false,
    } = .{},
    length_counter: packed struct(u8) {
        _unused: u3 = 0,
        length_counter_load: u5 = 0,
    } = .{},
} = .{},

dmc: struct {
    control: packed struct(u8) {
        frequency: u4 = 0,
        _unused: u3 = 0,
        irq_enabled: bool = false,
    } = .{},
    load_counter: u7 = 0,
    sample_address: u8 = 0,
    sample_length: u8 = 0,
} = .{},

status: packed struct(u8) {
    enable_pulse_one: bool = false,
    enable_pulse_two: bool = false,
    enable_triangle: bool = false,
    enable_noise: bool = false,
    enable_dmc: bool = false,
    _unused: u3 = 0,
} = .{},
