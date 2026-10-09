# Azoteq IQS5xx Trackpad

The Azoteq IQS5xx-B000 family (IQS550, IQS572, IQS525) are I²C capacitive
trackpad controllers, commonly used in keyboards via Azoteq's TPS43 and TPS65
trackpad modules.

::: note

- `keyboard.toml` configuration is supported on nRF52 and RP2040 only; other chips
  need the [Rust API](#rust-configuration).
- Without [gestures](#gestures), only relative single-finger cursor movement
  is reported. With them, the positions of up to five fingers are, and RMK
  recognizes taps, drags and two-finger scrolling from them. Raw channel data is
  not read.
- Use [Sniper mode](./pointing_processor#sniper) to reduce cursor sensitivity.
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
# Optional cursor acceleration in PointingProcessor: motion faster than `from`
# sensor counts per second is scaled up in proportion to its speed, to at most
# `max` percent. Slower motion passes unchanged. Counts depend on the sensor's
# resolution, so retune `from` after changing it.
# acceleration = { from = 1500, max = 250 }
# The same for scroll mode and two-finger scrolling, applied before the scroll divisor.
# scroll_acceleration = { from = 1500, max = 300 }
```

## Gestures

Add a `gestures` table to a trackpad to turn gestures on for it:

```toml
[[input_device.iqs5xx]]
name = "trackpad0"
# ...

[input_device.iqs5xx.gestures]
# Every setting is optional; these are the defaults.
# scroll_divisor = 8        # trackpad movement per scroll step; larger scrolls slower
# natural_scroll = false    # content follows the fingers, as on a phone
# tap_move_percent = 3      # a tap moves at most this far (in % of the trackpad); more is a cursor move
# tap_min_ms = 0            # a tap lasts at least this long; shorter is a graze, e.g. 20
# tap_drag_ms = 180         # a touch starting this soon after a tap drags; a tap clicks
#                           # once this has passed. 0 turns tap drags off, so taps click at once
# drag_lock_ms = 0          # a drag lasts this long after the fingers lift, so a finger put
#                           # back down goes on with it; a quick tap ends it. 300 is a good start
```

What each gesture does is set per layer in `[[keymap.layer]]`, like encoders,
with one table per trackpad that has gestures — the central's first, then each
peripheral's. Each action takes the same syntax as a key in `keys`:

```toml
[[keymap.layer]]
keys = "..."
touch = [{ tap = "MouseBtn1", two_finger_tap = "MouseBtn2", three_finger_tap = "MouseBtn3" }]
```

| Gesture | Recognized when |
|---|---|
| `tap`, `two_finger_tap`, `three_finger_tap` | The fingers touch and lift without moving, within 180 ms for one finger and 300 ms for more |
| `tap`, held | One finger touches within `tap_drag_ms` of a tap lifting, close to it, and moves or rests: the tap becomes a drag, its action pressed until the finger lifts. Tap, then touch and move drags; two quick taps double-click; tap, tap, touch and move double-clicks and drags. A tap waits out `tap_drag_ms` before it clicks. During the drag another finger can take over when the first runs out of room. |

A gesture with no action on the active layer isn't recognized there at all.

A gesture a layer leaves out is transparent and takes its action from the layer
below, so usually only the base layer lists `touch`, and a layer above it lists
just what it changes; `"No"` turns a gesture off there. A layer lists every
trackpad with gestures or none.

Gesture actions can't be edited from Vial yet.

### Split

To add the trackpad to the central or a peripheral:

```toml
[[split.central.input_device.iqs5xx]]
name = ...

# resp.
[[split.peripheral.input_device.iqs5xx]]
name = ...
```

For split keyboards the device runs on whichever side it's wired to, and so does
the `TouchGestureProcessor` of a trackpad with gestures: finger positions never
leave that side. Only cursor motion, scrolling and gestures go to the central,
where the matching `PointingProcessor` is generated automatically.

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

// 4. Add a PointingProcessor on the central side to convert motion events
//    into mouse reports. Axis tweaks (invert / swap) live here.
let proc_config = PointingProcessorConfig {
    // invert_x: true,
    // invert_y: true,
    // swap_xy: true,
    // acceleration: Some(PointerAcceleration { from_counts_per_s: 1500, max_percent: 250 }),
    // scroll_acceleration: Some(PointerAcceleration { from_counts_per_s: 1500, max_percent: 300 }),
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

### Gestures in Rust

Have the trackpad publish finger positions, and add a `TouchGestureProcessor`
on the side the trackpad is wired to. Its `PointingProcessor` runs on the central.
Gesture actions go in the keymap data's touch map, one `TouchAction` per trackpad
and layer, and the `TouchGestureProcessor` gets the same actions for its trackpad,
to tell which gestures a layer binds:

```rust
use rmk::input_device::iqs5xx::Iqs5xxMode;
use rmk::input_device::touch::{TouchGestureConfig, TouchGestureProcessor};
use rmk::types::action::{TouchAction, TouchGesture};

const TRACKPAD_TOUCH: TouchAction = TouchAction::new()
    .with(TouchGesture::Tap, k!(MouseBtn1))
    .with(TouchGesture::TwoFingerTap, k!(MouseBtn2));
static TRACKPAD_TOUCH_LAYERS: [TouchAction; NUM_LAYER] = [TRACKPAD_TOUCH; NUM_LAYER];

let mut keymap_data = KeymapData::new_with_touch(
    keymap,
    [[]; NUM_LAYER], // or the encoder map
    [[TRACKPAD_TOUCH]; NUM_LAYER],
);

let mut trackpad = Iqs5xx::new(POINTING_DEV_ID, i2c, rdy).with_mode(Iqs5xxMode::RawTouch);
let mut trackpad_touch = TouchGestureProcessor::new(
    &TRACKPAD_TOUCH_LAYERS,
    TouchGestureConfig {
        device_id: POINTING_DEV_ID,
        touchpad_id: 0, // its index in the touch map
        ..Default::default()
    },
);
// Two-finger scrolling arrives on the H/V axes; `device_scroll` sets its speed.
let mut trackpad_proc = PointingProcessor::new(&keymap, PointingProcessorConfig {
    device_id: POINTING_DEV_ID,
    ..Default::default()
});

run_all!(trackpad, trackpad_touch, trackpad_proc, /* matrix, ... */);
```

Finger positions travel as `TouchEvent`s, which have no subscriber unless
`keyboard.toml` configures a trackpad with gestures. Without one, reserve a
`TouchEvent` and a `LayerChangeEvent` subscriber per `TouchGestureProcessor`, on
top of the defaults:

```toml
[event.touch]
subs = 1

[event.layer_change]
subs = 2
```

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
