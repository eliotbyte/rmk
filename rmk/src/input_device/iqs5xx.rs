//! Azoteq IQS5xx trackpad controller driver.
//!
//! This IC is commonly used in keyboards via Azoteq's TPS43 and TPS65 modules.
//!
//! # Communication
//!
//! ```text
//!
//! RDY    | low     | high    | low     | high    |
//! mode   | scan    | I2C     | scan    | I2C     |
//!                  |<-- report rate -->|
//! ```
//!
//! The device operates in cycles. It drives the `RDY` pin high to indicate
//! (§8.1) that scanning/processing is complete and an I2C communication window
//! is open. The window lasts until RMK sends an "end session" command (§8.7)
//! or the I2C timeout is hit (§8.6). The device then restarts scanning/processing
//! to prepare for the next window. It attempts to open a window once every
//! "report rate" (more accurately, report *interval*, as it's measured in
//! milliseconds; §4.1), setting the `RR_MISSED` bit if it's not able to keep up.
//!
//! If RMK requests "event mode", the I2C communication window is skipped
//! entirely when there's no event of interest.
//!
//! RMK can force a communication window to open; the device "clock stretches"
//! until it is able to respond.
//!
//! Report intervals vary based on configuration and mode (active and four
//! successively deeper idle states).
//!
//! In the best case, IQS550 reports are at a rate of ~100 Hz / interval of ~10
//! ms. Notably, this is long enough that (unlike with trackball controllers
//! supported by RMK) it's not necessary to aggregate several reports into one
//! USB HID mouse report. In fact the opposite may be preferred: spreading one
//! report across several USB HID reports for smooth scrolling.
//!
//! # `RDY` vs polling
//!
//! This driver supports usage with or without a `RDY` (ready/motion) pin.
//! Routing such a pin (even by hand-soldering a wire onto an existing PCB) is
//! recommended, particularly if the I2C bus is shared with another device. As
//! mentioned above, sending an I2C command outside a communication window will
//! cause it to respond by I2C clock-stretching, essentially freezing the bus
//! for all devices until it is ready. If RMK communicates promptly after `RDY`
//! goes high, this is unlikely to happen. When RMK just guesses when to
//! communicate based on timing, it's far more likely. It's possible to minimize
//! chances of a stall by making the report interval relatively consistent, but
//! this requires compromises:
//!
//! 1. requesting a conservative report rate: experimentally, even though the
//!    IQS550 advertises "typical report rate: 100 Hz (with single touch /
//!    all channels active)", it will miss targets shorter than ~14 ms (~70 Hz)
//!    during longer touch events.
//! 2. perhaps giving extra time beyond the requested cycle time before polling,
//!    lowering the duty cycle, and raising the communication window timeout to
//!    compensate.
//! 3. preventing transition to idle modes or setting their cycle times to match
//!    the active mode.
//! 4. disabling event mode.
//! 5. disabling auto-tuning during operation (re-ATI).
//!
//! # Reported data
//!
//! The IC exposes (§5.2, §6):
//!
//! * single-finger relative cursor movement (§5.2.2)
//! * per-finger absolute position, pressure, and area (§5.2.3-§5.2.5)
//! * detected gestures (§6): single/two-finger tap, press-and-hold, swipes,
//!   scroll, zoom/pinch
//! * raw per-channel count/delta data (§8.10.6)
//!
//! This driver reads the motion block at 0x000C (previous cycle time, gesture
//! events, system info, number of fingers, relative XY) and the absolute
//! position of the first two fingers. Relative XY is published as cursor
//! movement. The IC's one-finger gestures and two-finger tap (§6) are enabled
//! per [`Iqs5xxGestures`] and press virtual keys (`KeyboardEventPos::Virtual`),
//! whose actions live in `BehaviorConfig::virtual_keys`. Two-finger scroll and
//! zoom are recognized here from the finger positions instead of by the IC,
//! whose zoom only looks at the distance between the fingers: scrolling is
//! published on the H/V axes, zoom steps press virtual keys. Raw channel data
//! is not read.
//!
//! # Configuration
//!
//! The Azoteq driver supports a variety of configuration, including parameters of
//! the physical touchpad and configuration of gesture recognition.
//!
//! * Some of these can be configured persistently shortly after resetting the
//!   device via the `NRST` pin. This driver does not support that mechanism.
//! * All can be configured at runtime. Currently this driver hardcodes a few of these.
//!
//! # References
//!
//! * [datasheet](https://www.azoteq.com/images/stories/pdf/iqs5xx-b000_trackpad_datasheet.pdf).
//!   Section markers (§) in comments refer to the datasheet unless otherwise noted.

use embassy_time::{Duration, Instant, Timer};
use embedded_hal::i2c::Operation;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use rmk_macro::input_device;

use crate::event::{Axis, AxisEvent, AxisValType, KeyboardEvent, KeyboardEventPos, PointingEvent, publish_event_async};
use crate::fmt::Debug;

const I2C_ADDR: u8 = 0x74; // default I2C bus address according to §8.2.

const END_SESSION: [u8; 2] = [0xEE, 0xEE]; // §8.7. Address + dummy data byte; a zero-data write doesn't actually trigger end-of-comms.

#[input_device(publish = PointingEvent)]
pub struct Iqs5xx<I, RDY>
where
    I: I2c,
    I::Error: Debug,
    RDY: Wait,
{
    /// The RMK pointing device id of this device (*not* the I2C bus address).
    pointing_device_id: u8,

    i2c: I,

    window_detection: WindowDetection<RDY>,

    initialized: bool,

    gestures: Iqs5xxGestures,

    /// `gestures.two_finger` in pixels of the resolution set at init.
    two_finger_px: TwoFingerPx,

    state: GestureState,
}

/// Gesture state carried from one cycle to the next.
#[derive(Debug, Default)]
struct GestureState {
    /// Whether the press-and-hold virtual key is down.
    holding: bool,
    two_finger: TwoFingerState,
}

/// A finger position, in pixels.
type Point = (i32, i32);

/// Where a two-finger touch stands. Once it is a scroll or a zoom it stays one
/// until it is no longer exactly two fingers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum TwoFingerState {
    #[default]
    Idle,
    /// Two fingers down since `start`, not moved far enough to tell.
    Deciding {
        start: [Point; 2],
    },
    Scrolling {
        last: [Point; 2],
    },
    /// Zooming; `base` is the finger distance at the last zoom step.
    Zooming {
        base: u32,
    },
}

/// How two-finger scroll and zoom are told apart.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TwoFingerConfig {
    /// How far the two fingers together travel before the touch is decided, in
    /// percent of the trackpad's longer side.
    pub decide_percent: u8,
    /// A zoom needs both fingers moving in opposite directions along the line between
    /// them, within this angle; it's `cos(angle)` in permille (25° is 906).
    pub zoom_cos_permille: u16,
    /// How much the distance between the fingers changes per zoom step, in percent of
    /// the trackpad's longer side.
    pub zoom_step_percent: u8,
}

impl Default for TwoFingerConfig {
    fn default() -> Self {
        Self {
            decide_percent: 4,
            zoom_cos_permille: 906,
            zoom_step_percent: 6,
        }
    }
}

/// [`TwoFingerConfig`] with distances in pixels.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct TwoFingerPx {
    decide: u32,
    zoom_step: u32,
    zoom_cos_permille: u32,
}

impl TwoFingerPx {
    fn new(config: &TwoFingerConfig, span: u16) -> Self {
        Self {
            decide: u32::from(percent_of(span, config.decide_percent)),
            zoom_step: u32::from(percent_of(span, config.zoom_step_percent)).max(1),
            zoom_cos_permille: u32::from(config.zoom_cos_permille.min(1000)),
        }
    }
}

/// The IQS5xx's gestures to enable, each with the index of the virtual key
/// (`KeyboardEventPos::Virtual`) it presses. `None` leaves a gesture off.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Iqs5xxGestures {
    /// One-finger tap: the key is tapped when the finger lifts (§6.1).
    pub single_tap: Option<u8>,
    /// One finger held still: the key stays pressed until the finger lifts, and the
    /// cursor moves meanwhile, which makes a drag (§6.2).
    pub press_and_hold: Option<u8>,
    /// One-finger swipes along the sensor axes (§6.3). The cursor moves as well.
    pub swipe_x_neg: Option<u8>,
    pub swipe_x_pos: Option<u8>,
    pub swipe_y_neg: Option<u8>,
    pub swipe_y_pos: Option<u8>,
    /// Two-finger tap (§6.4).
    pub two_finger_tap: Option<u8>,
    /// Two fingers moving together scroll, published on the H/V axes.
    pub scroll: bool,
    /// Two fingers moving apart / together: tapped once per zoom step.
    pub zoom_in: Option<u8>,
    pub zoom_out: Option<u8>,
    /// How two-finger scroll and zoom are recognized.
    pub two_finger: TwoFingerConfig,
}

impl Iqs5xxGestures {
    /// Single Finger Gestures register value, §8.10.21.
    fn single_finger_enable(&self) -> u8 {
        [
            self.single_tap,
            self.press_and_hold,
            self.swipe_x_neg,
            self.swipe_x_pos,
            self.swipe_y_pos,
            self.swipe_y_neg,
        ]
        .iter()
        .enumerate()
        .fold(0, |bits, (bit, key)| if key.is_some() { bits | 1 << bit } else { bits })
    }

    /// Multi-finger Gestures register value, §8.10.22. The IC's scroll and zoom stay
    /// off: the driver recognizes them itself.
    fn multi_finger_enable(&self) -> u8 {
        u8::from(self.two_finger_tap.is_some())
    }

    fn zoom(&self) -> bool {
        self.zoom_in.is_some() || self.zoom_out.is_some()
    }
}

/// `percent` of `span` pixels, saturating at the register's range.
fn percent_of(span: u16, percent: u8) -> u16 {
    u16::try_from(u32::from(span) * u32::from(percent) / 100).unwrap_or(u16::MAX)
}

/// The part of one cycle's data the gestures depend on.
struct Motion {
    gesture_events_0: u8,
    gesture_events_1: u8,
    fingers: u8,
    dx: i16,
    dy: i16,
    /// Absolute positions of fingers 1 and 2; meaningful while that many are down.
    points: [Point; 2],
}

/// What one cycle turns into: virtual key presses `(index, pressed)` in order, and
/// the axes to publish, X/Y for the cursor or H/V for scrolling.
#[derive(Debug, Default, PartialEq, Eq)]
struct CycleOutput {
    keys: heapless::Vec<(u8, bool), 4>,
    axes: Option<[(Axis, i16); 2]>,
}

fn sub(a: Point, b: Point) -> Point {
    (a.0 - b.0, a.1 - b.1)
}

fn dot(a: Point, b: Point) -> i64 {
    i64::from(a.0) * i64::from(b.0) + i64::from(a.1) * i64::from(b.1)
}

fn len(a: Point) -> u32 {
    dot(a, a).unsigned_abs().isqrt() as u32
}

/// Whether the angle between `a` and `b` is within the one whose cosine is
/// `cos_permille` / 1000, or of its opposite when `opposite`. Integer only:
/// `cos(a, b) = dot / (|a| |b|)`, compared squared.
fn within_angle(a: Point, b: Point, cos_permille: u32, opposite: bool) -> bool {
    let d = dot(a, b);
    if d == 0 || (d < 0) != opposite {
        return false;
    }
    let lhs = i128::from(d) * i128::from(d) * 1_000_000;
    let rhs = i128::from(cos_permille * cos_permille) * i128::from(dot(a, a)) * i128::from(dot(b, b));
    lhs >= rhs
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TwoFinger {
    Scroll,
    Zoom,
}

/// Tell a two-finger touch that moved from `start` to `now` apart: a zoom when both
/// fingers move in opposite directions along the line between them, a scroll when
/// they move the same way, or `None` while that is still unclear.
fn classify(start: [Point; 2], now: [Point; 2], px: &TwoFingerPx, zoom: bool) -> Option<TwoFinger> {
    let d1 = sub(now[0], start[0]);
    let d2 = sub(now[1], start[1]);
    let travel = len(d1) + len(d2);
    if travel < px.decide {
        return None;
    }
    let axis = sub(start[1], start[0]);
    let cos = px.zoom_cos_permille;
    if zoom
        && len(d1) >= px.decide / 4
        && len(d2) >= px.decide / 4
        && within_angle(d1, d2, cos, true)
        && (within_angle(d1, axis, cos, false) || within_angle(d1, axis, cos, true))
        && (within_angle(d2, axis, cos, false) || within_angle(d2, axis, cos, true))
    {
        return Some(TwoFinger::Zoom);
    }
    // Within 45° of each other; failing that, scroll once it has moved a lot, so a
    // touch that never settles still does something harmless.
    if within_angle(d1, d2, 707, false) || travel >= 3 * px.decide {
        return Some(TwoFinger::Scroll);
    }
    None
}

/// Decode one cycle against the enabled gestures, updating the state carried
/// between cycles.
fn decode_cycle(gestures: &Iqs5xxGestures, px: &TwoFingerPx, motion: &Motion, state: &mut GestureState) -> CycleOutput {
    let mut out = CycleOutput::default();
    let tap = |key: Option<u8>, out: &mut CycleOutput| {
        if let Some(key) = key {
            let _ = out.keys.push((key, true));
            let _ = out.keys.push((key, false));
        }
    };
    let g0 = motion.gesture_events_0;

    let hold = g0 & 0b10 != 0;
    if let Some(key) = gestures.press_and_hold
        && hold != state.holding
    {
        state.holding = hold;
        let _ = out.keys.push((key, hold));
    }
    if g0 & 0b1 != 0 {
        tap(gestures.single_tap, &mut out);
    }
    for (bit, key) in [
        (2, gestures.swipe_x_neg),
        (3, gestures.swipe_x_pos),
        (4, gestures.swipe_y_pos),
        (5, gestures.swipe_y_neg),
    ] {
        if g0 & 1 << bit != 0 {
            tap(key, &mut out);
        }
    }
    if motion.gesture_events_1 & 0b1 != 0 {
        tap(gestures.two_finger_tap, &mut out);
    }

    if motion.fingers != 2 {
        state.two_finger = TwoFingerState::Idle;
        if motion.fingers == 1 && (motion.dx != 0 || motion.dy != 0) {
            out.axes = Some([(Axis::X, motion.dx), (Axis::Y, motion.dy)]);
        }
        return out;
    }
    if !gestures.scroll && !gestures.zoom() {
        return out;
    }

    let now = motion.points;
    state.two_finger = match state.two_finger {
        TwoFingerState::Idle => TwoFingerState::Deciding { start: now },
        TwoFingerState::Deciding { start } => match classify(start, now, px, gestures.zoom()) {
            Some(TwoFinger::Scroll) => TwoFingerState::Scrolling { last: now },
            Some(TwoFinger::Zoom) => TwoFingerState::Zooming {
                base: len(sub(start[1], start[0])),
            },
            None => TwoFingerState::Deciding { start },
        },
        TwoFingerState::Scrolling { last } => {
            let (d1, d2) = (sub(now[0], last[0]), sub(now[1], last[1]));
            let (h, v) = ((d1.0 + d2.0) / 2, (d1.1 + d2.1) / 2);
            if gestures.scroll && (h != 0 || v != 0) {
                let clamp = |x: i32| x.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16;
                out.axes = Some([(Axis::H, clamp(h)), (Axis::V, clamp(v))]);
            }
            TwoFingerState::Scrolling { last: now }
        }
        zooming @ TwoFingerState::Zooming { .. } => zooming,
    };
    // At most one zoom step per cycle; the rest follow on the next cycles.
    if let TwoFingerState::Zooming { base } = &mut state.two_finger {
        let distance = len(sub(now[1], now[0]));
        if distance >= *base + px.zoom_step {
            *base += px.zoom_step;
            tap(gestures.zoom_in, &mut out);
        } else if distance + px.zoom_step <= *base {
            *base -= px.zoom_step;
            tap(gestures.zoom_out, &mut out);
        }
    }
    out
}

/// Manner of detecting a "communication window" between cycles.
pub enum WindowDetection<RDY> {
    /// Wait for a high state of the given GPIO connected to the `RDY` pin.
    Rdy(RDY),

    /// Open a communication window every `interval` (the device clock-stretches
    /// if we arrive mid-cycle).
    Poll { last_end: Instant, interval_ms: u16 },
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
enum Error<I2cError> {
    I2c { tag: &'static str, inner: I2cError },
    InvalidProductInfo([u8; 4]),
    Reset,
}

/// Start communication with the device without waiting for an I2C window.
///
/// As described in §8.8.2, if the device is in lower-power mode, it may
/// initially NAK. The operations should be idempotent for retry.
///
/// Additionally, the device will likely stall the bus via clock stretching.
async fn i2c_force_tx<'a, I: I2c>(
    i2c: &mut I,
    tag: &'static str,
    operations: &mut [Operation<'a>],
) -> Result<(), Error<I::Error>> {
    // There should be at most 1 NAK, but let's give an extra try.
    const MAX_ATTEMPTS: usize = 3;
    let mut attempt = 0;
    while let Err(e) = i2c.transaction(I2C_ADDR, operations).await {
        attempt += 1;
        if attempt == MAX_ATTEMPTS {
            return Err(Error::I2c { tag, inner: e });
        }

        // Datasheet requires 150 µs delay; give a little slack.
        Timer::after(Duration::from_micros(200)).await;
    }
    Ok(())
}

/// Perform operations that are expected to fall within a communication window and thus not require retry.
async fn i2c_tx<'a, I: I2c>(
    i2c: &mut I,
    tag: &'static str,
    operations: &mut [Operation<'a>],
) -> Result<(), Error<I::Error>> {
    i2c.transaction(I2C_ADDR, operations)
        .await
        .map_err(|inner| Error::I2c { tag, inner })
}

impl<I: I2c, RDY> Iqs5xx<I, RDY>
where
    I: I2c,
    I::Error: Debug,
    RDY: Wait,
{
    pub fn new(rmk_id: u8, i2c: I, rdy: Option<RDY>) -> Self {
        Self {
            i2c,
            window_detection: match rdy {
                None => WindowDetection::Poll {
                    last_end: Instant::now(),
                    interval_ms: 15, // conservative value
                },
                Some(rdy) => WindowDetection::Rdy(rdy),
            },
            initialized: false,
            pointing_device_id: rmk_id,
            gestures: Iqs5xxGestures::default(),
            two_finger_px: TwoFingerPx::default(),
            state: GestureState::default(),
        }
    }

    /// Enable the IC's gestures, each pressing its virtual key.
    pub fn with_gestures(mut self, gestures: Iqs5xxGestures) -> Self {
        self.gestures = gestures;
        self
    }

    /// Initialize the device.
    async fn init(&mut self) -> Result<(), Error<I::Error>> {
        // Force-open a communication window; the device may never become ready otherwise.
        let mut product_info = [0u8; 4]; // §7.9.1.
        i2c_force_tx(
            &mut self.i2c,
            "read_product_info",
            &mut [Operation::Write(&[0, 0]), Operation::Read(&mut product_info)],
        )
        .await?;
        let ic = match product_info {
            [0, 40, 0, 15] => "IQS550",
            [0, 58, 0, 15] => "IQS572",
            [0, 52, 0, 15] => "IQS525",
            _ => {
                return Err(Error::InvalidProductInfo(product_info));
            }
        };

        let mut channels = [0u8; 2]; // §5.1.1: 0x063D Total Rx, 0x063E Total Tx
        i2c_tx(
            &mut self.i2c,
            "read_channels",
            &mut [Operation::Write(&[0x06, 0x3D]), Operation::Read(&mut channels)],
        )
        .await?;
        // §8.10.20: with SWITCH_XY_AXIS=0 (as set in `xy_config` below), Rx drives
        // output X and Tx drives output Y. §5.1.1: max useful resolution per axis
        // is (channels - 1) * 256.
        let x_resolution = u16::from(channels[0].saturating_sub(1)) * 256;
        let y_resolution = u16::from(channels[1].saturating_sub(1)) * 256;

        let (system_config_0, system_config_1, i2c_timeout_ms, active_interval_ms) = match &mut self.window_detection {
            WindowDetection::Rdy(_) => (0b01100100, 0b00001111, 30, 9),
            WindowDetection::Poll { interval_ms, .. } => (0b11100100, 0, 100, *interval_ms),
        };
        // I2C timeout register at 0x058A; §8.6.
        let i2c_timeout = [0x05, 0x8A, i2c_timeout_ms];
        #[rustfmt::skip]
        let config = [
            0x05, 0x8E, // System Config 0 at 0x058E; §8.10.9
            system_config_0,
            system_config_1, // System Config 1 at 0x058F; §8.10.10
        ];
        // Report rate registers at 0x057A..0x0583; §4.1. Active and idle-touch
        // get the same value so a stationary finger keeps the cadence we asked
        // for (the device drops to idle-touch on a held finger; in poll mode a
        // mismatched idle-touch rate would misalign our polling clock). Idle
        // mode cycle time is pinned at 25 ms so the device doesn't stretch out
        // cycles between touches. LP1/LP2 are left at NV defaults.
        #[rustfmt::skip]
        let report_rates = [
            0x05, 0x7A, // address: 0x057A
            (active_interval_ms >> 8) as u8, active_interval_ms as u8, // active mode
            (active_interval_ms >> 8) as u8, active_interval_ms as u8, // idle-touch mode
            0, 25, // idle mode (ms)
        ];
        const ACK_RESET: [u8; 3] = [
            0x04,
            0x31,        // System Control 0 at 0x0431; §8.10.7
            0b1000_0000, // ACK_RESET bit
        ];
        #[rustfmt::skip]
        const XY_CONFIG: [u8; 3] = [
            0x06, 0x69, // XY Config 0 at 0x0669; §8.10.20
            0b0001,     // FLIP_X | !SWITCH_XY_AXIS (Rx→X, Tx→Y; see resolution above)
        ];
        #[rustfmt::skip]
        let gestures = [
            0x06, 0xB7, // Single-/Multi-finger Gestures at 0x06B7/0x06B8; §8.10.21-§8.10.22
            self.gestures.single_finger_enable(),
            self.gestures.multi_finger_enable(),
        ];

        // X/Y Resolution at 0x066E..0x0671 (2 bytes each); §5.4.
        // We set the maximum possible resolution to get the best precision out of the device.
        // This is beneficial for things like being able to scroll slowly and smoothly.
        #[rustfmt::skip]
        let xy_resolution = [
            0x06, 0x6E,
            (x_resolution >> 8) as u8, x_resolution as u8,
            (y_resolution >> 8) as u8, y_resolution as u8,
        ];

        // Each block must be its own transaction. Per the `embedded_hal::i2c`
        // contract, adjacent `Operation::Write`s in one `transaction` emit no
        // STOP or restart between them — their bytes go on the wire as one
        // contiguous I2C write. Since each of our blocks starts with a 2-byte
        // register address, bundling them would only place the *first* block
        // correctly; every later block's address bytes would land as data in
        // whichever register the IQS5xx's auto-incrementing write pointer has
        // reached by then, and the intended target registers wouldn't be
        // touched at all.
        for (tag, write) in [
            ("i2c_timeout", &i2c_timeout[..]),
            ("config", &config[..]),
            ("report_rates", &report_rates[..]),
            ("ack_reset", &ACK_RESET[..]),
            ("xy_config", &XY_CONFIG[..]),
            ("gestures", &gestures[..]),
            ("xy_resolution", &xy_resolution[..]),
        ] {
            i2c_tx(&mut self.i2c, tag, &mut [Operation::Write(write)]).await?;
        }

        self.two_finger_px = TwoFingerPx::new(&self.gestures.two_finger, x_resolution.max(y_resolution));

        i2c_tx(&mut self.i2c, "end_session", &mut [Operation::Write(&END_SESSION[..])]).await?;

        if let WindowDetection::Poll { ref mut last_end, .. } = self.window_detection {
            *last_end = Instant::now();
        }
        self.initialized = true;
        info!(
            "iqs5xx {}: initialized {} (rx={}, tx={} => x_res={}, y_res={})",
            self.pointing_device_id, ic, channels[0], channels[1], x_resolution, y_resolution,
        );
        Ok(())
    }

    async fn read_motion(&mut self) -> Result<Motion, Error<I::Error>> {
        // Motion block at 0x000C..0x0015 per table 8.1: previous cycle time
        // (§4.1.1), gesture events 0/1 (§8.10.1-§8.10.2), system info 0/1
        // (§8.10.3-§8.10.4), number of fingers (§5.2.1), relative XY (§5.2.2),
        // then absolute X/Y, strength and area of fingers 1 and 2 (§5.2.3-§5.2.6),
        // 7 bytes each at 0x0016 and 0x001D.
        let mut data = [0u8; 24];
        let mut operations = [
            // In theory, it's possible to skip the initial address selection
            // write if the last window closed cleanly and RMK has previously
            // set the "default read address" register. However, it's unclear
            // if this register's contents are preserved across resets, so
            // it's probably unwise to use this in combination with the watchdog.
            // Let's be conservative and select the address each cycle.
            Operation::Write(&[0x00, 0x0C]),
            Operation::Read(&mut data),
            Operation::Write(&END_SESSION[..]),
        ];
        match self.window_detection {
            WindowDetection::Rdy(ref mut rdy) => {
                rdy.wait_for_high().await.expect("pin wait failure");
                i2c_tx(&mut self.i2c, "read_motion", &mut operations).await?;
            }
            WindowDetection::Poll {
                ref mut last_end,
                interval_ms,
            } => {
                Timer::at(last_end.saturating_add(Duration::from_millis(u64::from(interval_ms)))).await;
                i2c_force_tx(&mut self.i2c, "read_motion", &mut operations).await?;
                *last_end = Instant::now();
            }
        }
        let prev_cycle_time_ms = data[0];
        let gesture_events_0 = data[1];
        let gesture_events_1 = data[2];
        let system_info_0 = data[3]; // §8.10.3
        let system_info_1 = data[4]; // §8.10.4
        let number_of_fingers = data[5];
        let dx = i16::from_be_bytes(unwrap!(data[6..8].try_into()));
        let dy = i16::from_be_bytes(unwrap!(data[8..10].try_into()));
        let point = |at: usize| {
            let coordinate = |at: usize| i32::from(u16::from_be_bytes([data[at], data[at + 1]]));
            (coordinate(at), coordinate(at + 2))
        };
        let points = [point(10), point(17)];

        // §8.10.3: system_info_0.
        let charging_mode = match system_info_0 & 0b111 {
            0b000 => "active",
            0b001 => "idle-touch",
            0b010 => "idle",
            0b011 => "lp1",
            0b100 => "lp2",
            _ => "invalid",
        };
        if (system_info_0 & 0b0001_0000) != 0 {
            debug!("iqs5xx {} re-ati", self.pointing_device_id);
        }
        if (system_info_0 & 0b0000_1000) != 0 {
            error!("iqs5xx {} ati error", self.pointing_device_id);
        }
        if (system_info_0 & 0b1000_0000) != 0 {
            self.initialized = false;
            return Err(Error::Reset);
        }
        debug!(
            "iqs5xx {} motion data: cycle_ms={} gestures=[{},{}] mode={} system=[{},{}] n_fingers={} dx={} dy={}",
            self.pointing_device_id,
            prev_cycle_time_ms,
            gesture_events_0,
            gesture_events_1,
            charging_mode,
            system_info_0,
            system_info_1,
            number_of_fingers,
            dx,
            dy,
        );
        Ok(Motion {
            gesture_events_0,
            gesture_events_1,
            fingers: number_of_fingers,
            dx,
            dy,
            points,
        })
    }

    async fn publish_virtual_key(key: u8, pressed: bool) {
        publish_event_async(KeyboardEvent {
            pressed,
            pos: KeyboardEventPos::Virtual(key),
        })
        .await;
    }

    async fn read_pointing_event(&mut self) -> PointingEvent {
        loop {
            // Check initialization status on each iteration because the device
            // can reset and require re-initialization.
            // A reset mid-drag must not leave the press-and-hold key down.
            if !self.initialized
                && self.state.holding
                && let Some(key) = self.gestures.press_and_hold
            {
                self.state.holding = false;
                Self::publish_virtual_key(key, false).await;
            }
            if !self.initialized
                && let Err(e) = self.init().await
            {
                error!(
                    "iqs5xx {} initialization failed: {:?}; will retry in 1 second",
                    self.pointing_device_id, e,
                );
                Timer::after_secs(1).await;
                continue;
            }
            match self.read_motion().await {
                Ok(motion) => {
                    let out = decode_cycle(&self.gestures, &self.two_finger_px, &motion, &mut self.state);
                    for (key, pressed) in out.keys {
                        Self::publish_virtual_key(key, pressed).await;
                    }
                    if let Some([(axis_a, a), (axis_b, b)]) = out.axes {
                        let rel = |axis, value| AxisEvent {
                            typ: AxisValType::Rel,
                            axis,
                            value,
                        };
                        return PointingEvent {
                            device_id: self.pointing_device_id,
                            axes: [rel(axis_a, a), rel(axis_b, b), rel(Axis::Z, 0)],
                        };
                    }
                }
                Err(e) => {
                    error!("iqs5xx {} failure: {:?}", self.pointing_device_id, e);
                    Timer::after_millis(5).await;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: Iqs5xxGestures = Iqs5xxGestures {
        single_tap: Some(0),
        press_and_hold: Some(1),
        swipe_x_neg: Some(2),
        swipe_x_pos: Some(3),
        swipe_y_neg: Some(4),
        swipe_y_pos: Some(5),
        two_finger_tap: Some(6),
        scroll: true,
        zoom_in: Some(7),
        zoom_out: Some(8),
        two_finger: TwoFingerConfig {
            decide_percent: 4,
            zoom_cos_permille: 906,
            zoom_step_percent: 6,
        },
    };

    /// A 1000-pixel trackpad: deciding at 40 px, a zoom step every 60 px.
    const PX: TwoFingerPx = TwoFingerPx {
        decide: 40,
        zoom_step: 60,
        zoom_cos_permille: 906,
    };

    fn one_finger(gesture_events_0: u8, dx: i16, dy: i16) -> Motion {
        Motion {
            gesture_events_0,
            gesture_events_1: 0,
            fingers: 1,
            dx,
            dy,
            points: [(0, 0); 2],
        }
    }

    fn two_fingers(a: Point, b: Point) -> Motion {
        Motion {
            gesture_events_0: 0,
            gesture_events_1: 0,
            fingers: 2,
            dx: 0,
            dy: 0,
            points: [a, b],
        }
    }

    fn lifted() -> Motion {
        Motion {
            fingers: 0,
            ..one_finger(0, 0, 0)
        }
    }

    fn run(motions: &[Motion]) -> (Vec<CycleOutput>, GestureState) {
        let mut state = GestureState::default();
        let outs = motions
            .iter()
            .map(|motion| decode_cycle(&ALL, &PX, motion, &mut state))
            .collect();
        (outs, state)
    }

    fn keys(outs: &[CycleOutput]) -> Vec<(u8, bool)> {
        outs.iter().flat_map(|out| out.keys.iter().copied()).collect()
    }

    fn axes(outs: &[CycleOutput]) -> Vec<[(Axis, i16); 2]> {
        outs.iter().filter_map(|out| out.axes).collect()
    }

    #[test]
    fn enable_registers_follow_the_datasheet_bit_order() {
        assert_eq!(ALL.single_finger_enable(), 0b11_1111);
        // Scroll and zoom are the driver's, not the IC's.
        assert_eq!(ALL.multi_finger_enable(), 0b001);
        let swipe_y_neg_only = Iqs5xxGestures {
            swipe_y_neg: Some(0),
            ..Default::default()
        };
        assert_eq!(swipe_y_neg_only.single_finger_enable(), 0b10_0000);
        assert_eq!(Iqs5xxGestures::default().single_finger_enable(), 0);
        assert_eq!(Iqs5xxGestures::default().multi_finger_enable(), 0);
    }

    #[test]
    fn one_finger_moves_the_cursor() {
        let (outs, _) = run(&[one_finger(0, 3, -4)]);
        assert!(keys(&outs).is_empty());
        assert_eq!(axes(&outs), vec![[(Axis::X, 3), (Axis::Y, -4)]]);
    }

    #[test]
    fn taps_press_and_release_their_keys() {
        let (outs, _) = run(&[one_finger(0b1, 0, 0)]);
        assert_eq!(keys(&outs), vec![(0, true), (0, false)]);
        let mut two_finger_tap = lifted();
        two_finger_tap.gesture_events_1 = 0b1;
        let (outs, _) = run(&[two_finger_tap]);
        assert_eq!(keys(&outs), vec![(6, true), (6, false)]);
    }

    #[test]
    fn swipes_map_to_their_bits() {
        for (bit, key) in [(2, 2), (3, 3), (4, 5), (5, 4)] {
            let (outs, _) = run(&[one_finger(1 << bit, 0, 0)]);
            assert_eq!(keys(&outs), vec![(key, true), (key, false)], "bit {bit}");
        }
    }

    #[test]
    fn press_and_hold_stays_down_and_drags() {
        let (outs, state) = run(&[one_finger(0b10, 0, 0), one_finger(0b10, 5, 1)]);
        assert_eq!(keys(&outs), vec![(1, true)]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 5), (Axis::Y, 1)]]);
        assert!(state.holding);
        let (outs, state) = run(&[one_finger(0b10, 0, 0), lifted()]);
        assert_eq!(keys(&outs), vec![(1, true), (1, false)]);
        assert!(!state.holding);
    }

    #[test]
    fn two_fingers_moving_together_scroll() {
        let (outs, state) = run(&[
            two_fingers((100, 500), (300, 500)),
            two_fingers((100, 520), (300, 522)), // decided: same direction
            two_fingers((100, 530), (300, 532)),
        ]);
        assert!(keys(&outs).is_empty());
        assert_eq!(axes(&outs), vec![[(Axis::H, 0), (Axis::V, 10)]]);
        assert!(matches!(state.two_finger, TwoFingerState::Scrolling { .. }));
    }

    #[test]
    fn two_fingers_moving_apart_along_their_line_zoom_in() {
        let (outs, _) = run(&[
            two_fingers((400, 500), (600, 500)), // 200 apart
            two_fingers((380, 501), (620, 499)), // decided: opposite, along the line; 240
            two_fingers((340, 500), (660, 500)), // 320: a step past 200 + 60
            two_fingers((345, 500), (655, 500)), // 310: short of the next at 320
        ]);
        assert!(axes(&outs).is_empty());
        assert_eq!(keys(&outs), vec![(7, true), (7, false)]);
    }

    #[test]
    fn two_fingers_pinching_zoom_out() {
        let (outs, _) = run(&[
            two_fingers((300, 500), (700, 500)), // 400 apart
            two_fingers((330, 500), (670, 500)), // decided; 340: a step below 400 - 60
            two_fingers((340, 500), (660, 500)), // 320: short of the next at 280
        ]);
        assert_eq!(keys(&outs), vec![(8, true), (8, false)]);
    }

    #[test]
    fn opposite_but_across_the_line_is_not_a_zoom() {
        // Rotating: the fingers move apart from each other's path, not along it.
        let (outs, state) = run(&[two_fingers((400, 500), (600, 500)), two_fingers((400, 470), (600, 530))]);
        assert!(keys(&outs).is_empty());
        assert!(!matches!(state.two_finger, TwoFingerState::Zooming { .. }));
    }

    #[test]
    fn one_finger_still_is_not_a_zoom() {
        let (outs, state) = run(&[two_fingers((400, 500), (600, 500)), two_fingers((400, 500), (650, 500))]);
        assert!(keys(&outs).is_empty());
        assert!(!matches!(state.two_finger, TwoFingerState::Zooming { .. }));
    }

    #[test]
    fn small_jitter_decides_nothing() {
        let (outs, state) = run(&[
            two_fingers((400, 500), (600, 500)),
            two_fingers((390, 500), (610, 500)), // opposite, but only 20 px in all
        ]);
        assert!(keys(&outs).is_empty() && axes(&outs).is_empty());
        assert!(matches!(state.two_finger, TwoFingerState::Deciding { .. }));
    }

    #[test]
    fn a_scroll_stays_a_scroll_until_the_fingers_lift() {
        let (outs, _) = run(&[
            two_fingers((400, 500), (600, 500)),
            two_fingers((400, 540), (600, 540)), // scrolling
            two_fingers((350, 540), (650, 540)), // fingers spread: still scrolls
            two_fingers((300, 540), (700, 540)),
            lifted(),
            two_fingers((400, 500), (600, 500)),
            two_fingers((350, 500), (650, 500)), // a new touch zooms: 300, past 260
            two_fingers((345, 500), (655, 500)),
        ]);
        // The spread mid-scroll scrolls by zero and doesn't zoom; only the new touch does.
        assert_eq!(keys(&outs), vec![(7, true), (7, false)]);
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn a_third_finger_ends_the_two_finger_gesture() {
        let mut three = two_fingers((0, 0), (0, 0));
        three.fingers = 3;
        let (_, state) = run(&[
            two_fingers((400, 500), (600, 500)),
            two_fingers((400, 540), (600, 540)),
            three,
        ]);
        assert_eq!(state.two_finger, TwoFingerState::Idle);
    }

    #[test]
    fn distances_are_a_share_of_the_trackpad() {
        assert_eq!(percent_of(2304, 15), 345);
        assert_eq!(percent_of(2304, 0), 0);
        assert_eq!(percent_of(u16::MAX, 255), u16::MAX);
        let px = TwoFingerPx::new(&TwoFingerConfig::default(), 1000);
        assert_eq!(
            px,
            TwoFingerPx {
                decide: 40,
                zoom_step: 60,
                zoom_cos_permille: 906
            }
        );
    }
}
