const Controller = @This();

const Console = @import("Console.zig");

console: *Console,

buttons: packed struct(u8) {
    a: bool = false,
    b: bool = false,
    select: bool = false,
    start: bool = false,
    up: bool = false,
    down: bool = false,
    left: bool = false,
    right: bool = false,
} = .{},

state: u8 = 0, // Shift register containing the state of the controller when polled

pub fn read(self: *Controller) u8 {

    // Read the polled data one bit at a time from the state.
    // The msb is returned and the register is shifted one.

    const button = (self.state & 0x80) > 0;
    self.state <<= 1;
    return if (button) 1 else 0;
}

pub fn write(self: *Controller, value: u8) void {
    if (value == 1) { // Signal to the controller to poll the input
        self.state = @bitCast(self.buttons);
    }
}
