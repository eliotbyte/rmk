# Azoteq IQS5xx Trackpad

The Azoteq IQS5xx-B000 family (IQS550, IQS572, IQS525) are I²C capacitive
trackpad controllers, commonly used in keyboards via Azoteq's TPS43 and TPS65
trackpad modules.

::: note

- `keyboard.toml` configuration is supported on nRF52 and RP2040 only; other chips
  need the [Rust API](#rust-configuration).
- Relative single-finger cursor movement and the IC's built-in
  [gestures](#gestures) are reported. Multi-finger absolute positions,
  pressure, area, and raw channel data are not.
- Scaling is not supported yet; cursor movements will likely feel fast and
  imprecise.
- An `RDY` (ready) pin is strongly recommended. Without it, the driver falls
  back to timed polling and may stall the I²C bus through clock-stretching if
  it polls mid-cycle. See [RDY vs polling](#rdy-vs-polling).
- Each `[[input_device.iqs5xx]]` claims its own I²C peripheral. Sharing a bus
  with another I²C device (e.g. an OLED) isn't supported yet.
- Persisting parameters to the IC's non-volatile memory (which requires
  toggling `NRST`) isn't supported; configuration is rewritten on every boot.

:::

## Hardware

- `SDA` / `SCL` — I²C bus, 7-bit address `0x74`.
- `RDY` — active-high digital output the device drives high during the I²C
  communication window. Connect to a GPIO that supports async edge waits
  (`embedded_hal_async::digital::Wait`).
- `NRST` — active-low reset. Not used by this driver, but required if you
  want to persist parameters to the IC's non-volatile memory (out of scope
  here).

## `toml` configuration

```toml
[[input_device.iqs5xx]]
name = "trackpad0"
id = 0 # optional 0-255 pointing-device id; the generated PointingProcessor
       # is paired to it. Defaults to 0.

i2c.instance = "I2C0"  # RP2040: I2C0 / I2C1.  nRF52: TWISPI0 / TWISPI1.
i2c.sda = "PIN_4"
i2c.scl = "PIN_5"

# Optional: RDY (data-ready) pin. Strongly recommended.
rdy = "PIN_15"

# Axis tweaks applied in PointingProcessor.
# proc_invert_x = true
# proc_invert_y = true
# proc_swap_xy = true
```

### Split

To add the trackpad to the central or a peripheral:

```toml
[[split.central.input_device.iqs5xx]]
name = ...

# resp.
[[split.peripheral.input_device.iqs5xx]]
name = ...
```

For split keyboards the device runs on whichever side it's wired to; the
matching `PointingProcessor` is generated on the central automatically.

## Gestures

The IC recognizes taps, swipes, two-finger scrolling and pinch zoom on its own.
Give a gesture an action under `gestures` to enable it; gestures without an
action stay off. The action runs on the central like a key press, so it can be
a mouse button, a shortcut or a layer key.

```toml
[[input_device.iqs5xx]]
name = "trackpad0"
# ... i2c and rdy as above ...

[input_device.iqs5xx.gestures]
single_tap = "MouseBtn1"
# Held while the finger rests, and the cursor still moves: drag and drop.
press_and_hold = "MouseBtn1"
two_finger_tap = "MouseBtn2"
# Once per zoom step; Ctrl + = / Ctrl + - zoom most apps.
zoom_in = "WM(Equal, LCtrl)"
zoom_out = "WM(Minus, LCtrl)"
# Two-finger scroll and zoom are told apart by how the fingers move (see below).
# How far both fingers together move before deciding, in percent of the
# trackpad's longer side. Default: 4.
# two_finger_decide_percent = 4
# A zoom needs the fingers moving in opposite directions along the line between
# them, within this many degrees. Lower is stricter. Default: 25.
# zoom_angle = 25
# Change of finger distance per zoom step, in percent of the longer side. Default: 6.
# zoom_step_percent = 6
# Two fingers moving together far in one direction fire once per touch, in place
# of scrolling that way; lift the fingers to swipe again. Directions are the
# cursor's. E.g. back / forward:
# two_finger_swipe_right = "MouseBtn4"
# two_finger_swipe_left = "MouseBtn5"
# two_finger_swipe_up = "..."
# two_finger_swipe_down = "..."
# How far the fingers move, in percent of the trackpad's size in that
# direction. Default: 15.
# two_finger_swipe_percent = 15
# How many degrees a swipe may stray from its direction. Default: 30.
# two_finger_swipe_angle = 30
# Two-finger scrolling.
scroll = true
# Trackpad movement per scroll step; larger scrolls slower. Default: 8.
scroll_divisor = 8
# Content follows the fingers.
natural_scroll = true
# One-finger swipes, by cursor direction. The cursor moves during a swipe too.
# swipe_left = "..."
# swipe_right = "..."
# swipe_up = "..."
# swipe_down = "..."
```

On a split keyboard use `[split.central.input_device.iqs5xx.gestures]` or
`[split.peripheral.input_device.iqs5xx.gestures]`.

| Gesture | When it fires |
|---|---|
| `single_tap` | One finger touches and lifts without moving |
| `press_and_hold` | One finger stays still; the action is held until the finger lifts |
| `two_finger_tap` | Two fingers tap together |
| `scroll` | Two fingers move the same way |
| `zoom_in` / `zoom_out` | Two fingers move apart / together along the line between them |
| `swipe_*` | One finger moves quickly in one direction |
| `two_finger_swipe_*` | Two fingers move together far in one direction, once per touch |

The IC's own scroll and zoom aren't used: its zoom only checks that the
distance between the fingers changed, so fingers drifting apart while scrolling
zoom the page. Instead the driver reads both finger positions and, once they
have moved `two_finger_decide_percent`, compares their directions: within 45° of
each other is a scroll, opposite and along the line between the fingers (within
`zoom_angle`) is a zoom. Anything else, such as rotating or moving one finger
only, keeps waiting and ends up a scroll. Moving together within
`two_finger_swipe_angle` of a direction that has a `two_finger_swipe_*` is a
swipe instead of a scroll: it fires once the fingers have moved
`two_finger_swipe_percent`, and turns into a scroll if they veer off. The touch keeps that gesture until it
is no longer exactly two fingers.

Gestures press virtual keys (`KeyboardEventPos::Virtual`): their actions are in
`BehaviorConfig::virtual_keys`, and scrolling reaches the `PointingProcessor` on
the `H`/`V` axes, where `device_scroll` turns it into wheel and pan reports.
They don't depend on the active layer, and Vial can't edit them.

## Rust configuration

Construct the device directly. For a split keyboard, add the device to whichever
side (`central.rs` or `peripheral.rs`) the trackpad is physically wired to.

```rust
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{Config, I2c};
use rmk::input_device::iqs5xx::Iqs5xx;
use rmk::input_device::pointing::{PointingProcessor, PointingProcessorConfig};

// 1. Bring up the I2C bus the trackpad is on.
let mut i2c_cfg = Config::default();
i2c_cfg.frequency = 400_000;
let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, i2c_cfg);

// 2. Configure the RDY pin (recommended). Use `None` if you don't have one.
let rdy = Some(Input::new(p.PIN_15, Pull::None));

// 3. Construct the device. The first argument is an RMK pointing-device id;
//    pick any 0-255, just don't reuse it for another pointing device.
const POINTING_DEV_ID: u8 = 0;
let mut trackpad = Iqs5xx::new(POINTING_DEV_ID, i2c, rdy);
// Optional: enable gestures. Each `Some(i)` presses `KeyboardEventPos::Virtual(i)`,
// whose action is `behavior_config.virtual_keys[i]`, e.g.
// `virtual_keys: &[k!(MouseBtn1), k!(MouseBtn2)]`.
// let mut trackpad = trackpad.with_gestures(Iqs5xxGestures {
//     single_tap: Some(0),
//     two_finger_tap: Some(1),
//     scroll: true,
//     ..Default::default()
// });

// 4. Add a PointingProcessor on the central side to convert motion events
//    into mouse reports. Axis tweaks (invert / swap) live here.
let proc_config = PointingProcessorConfig {
    // invert_x: true,
    // invert_y: true,
    // swap_xy: true,
    ..Default::default()
};
let mut trackpad_proc = PointingProcessor::new(&keymap, proc_config);

run_all!(trackpad, trackpad_proc, /* matrix, ... */);
```

::: note

`PointingProcessor` must run on the **central** side, even if the trackpad is
wired to a peripheral. The peripheral runs the `Iqs5xx` device and forwards
events over the split link; the central converts them to USB/BLE HID reports.

You can switch between Cursor, Scroll, Sniper and Caret modes per layer.
See the [PointingProcessor](./pointing_processor) page for all options.

:::

## RDY vs polling

The IQS5xx alternates between _scanning_ the touch panel and an I²C
_communication window_. When a window is open it drives `RDY` high.

- **With `RDY`**: the driver waits for `RDY` high before issuing I²C reads,
  so transactions complete inside the window with no clock-stretching. The
  driver puts the IC into "event mode" so the device only opens a window when
  it actually has touch data, which keeps idle bus traffic minimal.
- **Without `RDY`** (`rdy = None` / no `rdy` in TOML): the driver issues
  reads on a fixed ~15 ms cadence. If a read lands mid-scan the IC
  clock-stretches SCL until the current cycle ends, freezing any device
  sharing the bus. The driver compensates with a conservative report
  interval and a longer per-transaction timeout, but you may still see
  latency spikes — particularly during long holds.

If your PCB doesn't route `RDY`, hand-soldering a jumper to a spare GPIO is
generally worth it.

## References

- [IQS5xx-B000 Trackpad and Touchpad Datasheet (Azoteq)](https://www.azoteq.com/images/stories/pdf/iqs5xx-b000_trackpad_datasheet.pdf)
