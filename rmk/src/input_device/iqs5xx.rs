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
//! position of the first three fingers. Relative XY is published as cursor
//! movement. The IC's one-finger gestures and two-finger tap (§6) are enabled
//! per [`Iqs5xxGestures`] and press virtual keys (`KeyboardEventPos::Virtual`),
//! whose actions live in `BehaviorConfig::virtual_keys`. Two-finger scroll and
//! zoom are recognized here from the finger positions instead of by the IC,
//! whose zoom only looks at the distance between the fingers: scrolling is
//! published on the H/V axes, zoom steps press virtual keys. Three-finger taps
//! and swipes, which the IC doesn't have, are recognized here too. Raw channel
//! data is not read.
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

    /// Acceleration of cursor motion, if any.
    cursor_acceleration: Option<Acceleration>,

    /// The trackpad's longer side in pixels of the resolution set at init.
    span: u32,

    /// When the previous cycle was read.
    last_at_ms: u64,

    /// Fractions of a pixel that acceleration carries to the next cycle, for the cursor
    /// and for scrolling.
    cursor_rest: Point,
    scroll_rest: Point,

    state: GestureState,
}

/// Gesture state carried from one cycle to the next.
#[derive(Debug, Default)]
struct GestureState {
    /// The press-and-hold drag in progress, its key down.
    drag: Option<Drag>,
    two_finger: TwoFingerState,
    touch: Touch,
}

/// A press-and-hold drag. The key stays down while any finger touches, so another
/// finger can take over when the first runs out of room; the finger that landed
/// last moves the cursor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Drag {
    /// Which finger slots were down last cycle.
    present: [bool; 3],
    /// The slot moving the cursor, and where it was last cycle.
    lead: Option<(usize, Point)>,
}

impl Drag {
    fn start(motion: &Motion) -> Self {
        Self {
            present: motion.present,
            lead: Self::any_finger(motion),
        }
    }

    fn any_finger(motion: &Motion) -> Option<(usize, Point)> {
        (0..3).find(|&i| motion.present[i]).map(|i| (i, motion.points[i]))
    }

    /// This cycle's cursor motion. A finger landing takes the lead without moving the
    /// cursor; when the lead lifts, a remaining finger takes over from where it is. A
    /// step longer than `jump` is the IC renumbering fingers, not motion.
    fn follow(&mut self, motion: &Motion, jump: u32) -> Point {
        let landed = (0..3).find(|&i| motion.present[i] && !self.present[i]);
        self.present = motion.present;
        if let Some(i) = landed {
            self.lead = Some((i, motion.points[i]));
            return (0, 0);
        }
        match self.lead {
            Some((i, last)) if motion.present[i] => {
                let now = motion.points[i];
                self.lead = Some((i, now));
                let step = sub(now, last);
                if len(step) > jump { (0, 0) } else { step }
            }
            _ => {
                self.lead = Self::any_finger(motion);
                (0, 0)
            }
        }
    }
}

/// One touch: from the first finger landing until no finger is left.
#[derive(Debug, Default)]
struct Touch {
    /// When the first finger landed; `None` while no finger is down.
    started_ms: Option<u64>,
    /// The most fingers down at once so far.
    max_fingers: u8,
    three_finger: ThreeFingerState,
}

/// Where the three-finger part of a touch stands.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ThreeFingerState {
    #[default]
    Idle,
    /// Three fingers down since `start`; `moved` once they went further than a tap allows.
    Tracking { start: [Point; 3], moved: bool },
    /// A three-finger swipe fired; nothing more until the fingers lift.
    Swiped,
}

/// A finger position, in pixels.
type Point = (i32, i32);

/// Where a two-finger touch stands. Once it is a scroll or a zoom it stays one
/// until it is no longer exactly two fingers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum TwoFingerState {
    #[default]
    Idle,
    /// Two fingers down at `start` since `started_ms`, not moved far enough to tell.
    Deciding {
        start: [Point; 2],
        started_ms: u64,
    },
    Scrolling {
        last: [Point; 2],
        axis: ScrollAxis,
    },
    /// Zooming; `base` is the finger distance at the last zoom step.
    Zooming {
        base: u32,
    },
    /// Moving together along `dir`, which has a two-finger swipe, since `started_ms`.
    /// The motion is held back until it turns out to be a flick, lifted quickly after
    /// moving far enough (`key` fires), or a scroll.
    Flicking {
        start: [Point; 2],
        last: [Point; 2],
        started_ms: u64,
        dir: Point,
        key: u8,
    },
}

/// The axes a two-finger scroll moves along, fixed when it starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ScrollAxis {
    Both,
    Horizontal,
    Vertical,
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
    /// How far both fingers move together for a two-finger swipe, in percent of the
    /// trackpad's size in the swipe's direction.
    pub swipe_percent: u8,
    /// A two-finger swipe lifts within this many milliseconds of touching; a longer
    /// move scrolls.
    pub swipe_ms: u16,
    /// A two- or three-finger swipe keeps within this angle of its axis; it's
    /// `cos(angle)` in permille (30° is 866).
    pub swipe_cos_permille: u16,
    /// How far three fingers move together for a three-finger swipe, in percent of the
    /// trackpad's size in the swipe's direction.
    pub three_swipe_percent: u8,
    /// A three-finger tap lifts every finger within this many milliseconds of the
    /// first touching.
    pub three_tap_ms: u16,
}

impl Default for TwoFingerConfig {
    fn default() -> Self {
        Self {
            decide_percent: 4,
            zoom_cos_permille: 906,
            zoom_step_percent: 6,
            swipe_percent: 10,
            swipe_ms: 250,
            swipe_cos_permille: 866,
            three_swipe_percent: 15,
            three_tap_ms: 300,
        }
    }
}

/// Pointer acceleration. Motion faster than `from_percent_per_s` of the trackpad's
/// longer side per second is scaled up in proportion to its speed, up to
/// `max_percent`; slower motion passes unchanged, so fine positioning keeps its
/// precision while fast moves cover more ground.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Acceleration {
    pub from_percent_per_s: u16,
    pub max_percent: u16,
}

/// `step`, taken over `dt_ms` on a trackpad `span` pixels long, under `accel`. `rest`
/// carries the fractions of a pixel between calls.
fn accelerate(step: Point, dt_ms: u64, span: u32, accel: Acceleration, rest: &mut Point) -> Point {
    let from = u64::from(span) * u64::from(accel.from_percent_per_s) / 100;
    let speed = u64::from(len(step)) * 1000 / dt_ms.max(1);
    let gain = if from == 0 || speed <= from {
        100
    } else {
        (speed * 100 / from).min(u64::from(accel.max_percent.max(100))) as i32
    };
    let scale = |d: i32, rest: &mut i32| {
        let total = d * gain + *rest;
        let out = total / 100;
        *rest = total - out * 100;
        out
    };
    (scale(step.0, &mut rest.0), scale(step.1, &mut rest.1))
}

/// [`TwoFingerConfig`] with distances in pixels.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct TwoFingerPx {
    decide: u32,
    zoom_step: u32,
    zoom_cos_permille: u32,
    /// Swipe distance along X and along Y: two fingers side by side have far less room
    /// across the trackpad's short side than a share of its long side.
    swipe: [u32; 2],
    swipe_ms: u64,
    swipe_cos_permille: u32,
    three_swipe: [u32; 2],
    three_tap_ms: u64,
}

impl TwoFingerPx {
    fn new(config: &TwoFingerConfig, x_resolution: u16, y_resolution: u16) -> Self {
        let span = x_resolution.max(y_resolution);
        let swipe = |resolution| u32::from(percent_of(resolution, config.swipe_percent)).max(1);
        let three_swipe = |resolution| u32::from(percent_of(resolution, config.three_swipe_percent)).max(1);
        Self {
            decide: u32::from(percent_of(span, config.decide_percent)),
            zoom_step: u32::from(percent_of(span, config.zoom_step_percent)).max(1),
            zoom_cos_permille: u32::from(config.zoom_cos_permille.min(1000)),
            swipe: [swipe(x_resolution), swipe(y_resolution)],
            swipe_ms: u64::from(config.swipe_ms),
            swipe_cos_permille: u32::from(config.swipe_cos_permille.min(1000)),
            three_swipe: [three_swipe(x_resolution), three_swipe(y_resolution)],
            three_tap_ms: u64::from(config.three_tap_ms),
        }
    }

    /// The swipe distance along `dir`, a unit vector on one axis.
    fn swipe_along(&self, dir: Point) -> u32 {
        self.swipe[usize::from(dir.0 == 0)]
    }

    /// The three-finger swipe distance along `dir`.
    fn three_swipe_along(&self, dir: Point) -> u32 {
        self.three_swipe[usize::from(dir.0 == 0)]
    }
}

/// The IQS5xx's gestures to enable, each with the index of the virtual key
/// (`KeyboardEventPos::Virtual`) it presses. `None` leaves a gesture off.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Iqs5xxGestures {
    /// One-finger tap: the key is tapped when the finger lifts (§6.1).
    pub single_tap: Option<u8>,
    /// One finger held still starts a drag (§6.2): the key stays pressed until no
    /// finger is left, and the finger that landed last moves the cursor.
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
    /// Scroll along both axes at once. Off, a scroll keeps to the axis it started
    /// along until the fingers lift.
    pub scroll_both_axes: bool,
    /// Two fingers moving apart / together: tapped once per zoom step.
    pub zoom_in: Option<u8>,
    pub zoom_out: Option<u8>,
    /// Two fingers moving together far along a sensor axis: tapped once per touch,
    /// in place of scrolling that way.
    pub two_finger_swipe_x_neg: Option<u8>,
    pub two_finger_swipe_x_pos: Option<u8>,
    pub two_finger_swipe_y_neg: Option<u8>,
    pub two_finger_swipe_y_pos: Option<u8>,
    /// Three fingers tapping together. The IC has no three-finger gestures; the driver
    /// recognizes them from the finger positions.
    pub three_finger_tap: Option<u8>,
    /// Three fingers moving together far along a sensor axis: tapped once per touch.
    pub three_finger_swipe_x_neg: Option<u8>,
    pub three_finger_swipe_x_pos: Option<u8>,
    pub three_finger_swipe_y_neg: Option<u8>,
    pub three_finger_swipe_y_pos: Option<u8>,
    /// How two- and three-finger gestures are recognized.
    pub two_finger: TwoFingerConfig,
    /// Acceleration of two-finger scrolling, if any.
    pub scroll_acceleration: Option<Acceleration>,
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

    /// Each two-finger swipe's direction and key.
    fn two_finger_swipes(&self) -> [(Point, Option<u8>); 4] {
        [
            ((-1, 0), self.two_finger_swipe_x_neg),
            ((1, 0), self.two_finger_swipe_x_pos),
            ((0, -1), self.two_finger_swipe_y_neg),
            ((0, 1), self.two_finger_swipe_y_pos),
        ]
    }

    /// Each three-finger swipe's direction and key.
    fn three_finger_swipes(&self) -> [(Point, Option<u8>); 4] {
        [
            ((-1, 0), self.three_finger_swipe_x_neg),
            ((1, 0), self.three_finger_swipe_x_pos),
            ((0, -1), self.three_finger_swipe_y_neg),
            ((0, 1), self.three_finger_swipe_y_pos),
        ]
    }

    /// Whether the driver tracks two-finger touches at all.
    fn two_finger_motion(&self) -> bool {
        self.scroll || self.zoom() || self.two_finger_swipes().iter().any(|(_, key)| key.is_some())
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
    /// Absolute positions of fingers 1 to 3, meaningful where `present`.
    points: [Point; 3],
    /// Which of the three finger slots hold a finger (nonzero touch strength). A
    /// finger keeps its slot while others land or lift (§5.2.6).
    present: [bool; 3],
    /// When the cycle was read.
    at_ms: u64,
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

fn clamp16(x: i32) -> i16 {
    x.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

/// The average motion of three fingers from `from` to `to`.
fn average3(from: [Point; 3], to: [Point; 3]) -> Point {
    let d = [0, 1, 2].map(|i| sub(to[i], from[i]));
    ((d[0].0 + d[1].0 + d[2].0) / 3, (d[0].1 + d[1].1 + d[2].1) / 3)
}

/// The average motion of the two fingers from `from` to `to`.
fn average(from: [Point; 2], to: [Point; 2]) -> Point {
    let (d1, d2) = (sub(to[0], from[0]), sub(to[1], from[1]));
    ((d1.0 + d2.0) / 2, (d1.1 + d2.1) / 2)
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

    // A drag keeps the key down until no finger is left, and nothing else happens
    // meanwhile: no taps, scrolling or other gestures.
    if let Some(key) = gestures.press_and_hold {
        if let Some(drag) = &mut state.drag {
            if motion.fingers == 0 {
                let _ = out.keys.push((key, false));
                state.drag = None;
            } else {
                let (dx, dy) = drag.follow(motion, px.decide * 4);
                if (dx, dy) != (0, 0) {
                    out.axes = Some([(Axis::X, clamp16(dx)), (Axis::Y, clamp16(dy))]);
                }
            }
            state.two_finger = TwoFingerState::Idle;
            state.touch = Touch::default();
            return out;
        }
        if g0 & 0b10 != 0 {
            let _ = out.keys.push((key, true));
            state.drag = Some(Drag::start(motion));
            return out;
        }
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

    // A scroll that starts with `moved` keeps to its main axis, unless both are allowed.
    let scroll_axis = |moved: Point| match moved {
        _ if gestures.scroll_both_axes => ScrollAxis::Both,
        (h, v) if h.abs() > v.abs() => ScrollAxis::Horizontal,
        _ => ScrollAxis::Vertical,
    };
    let scroll = |moved: Point, axis: ScrollAxis, out: &mut CycleOutput| {
        let moved = match axis {
            ScrollAxis::Both => moved,
            ScrollAxis::Horizontal => (moved.0, 0),
            ScrollAxis::Vertical => (0, moved.1),
        };
        if gestures.scroll && moved != (0, 0) {
            out.axes = Some([(Axis::H, clamp16(moved.0)), (Axis::V, clamp16(moved.1))]);
        }
    };

    // Three fingers: a tap or a swipe, and nothing else for the rest of the touch, so
    // fingers landing or lifting one by one don't move the cursor or scroll.
    let had_three = state.touch.max_fingers >= 3;
    if motion.fingers == 0 {
        let touch = &state.touch;
        if let Some(started_ms) = touch.started_ms
            && touch.max_fingers == 3
            && matches!(touch.three_finger, ThreeFingerState::Tracking { moved: false, .. })
            && motion.at_ms.saturating_sub(started_ms) <= px.three_tap_ms
        {
            tap(gestures.three_finger_tap, &mut out);
        }
        state.touch = Touch::default();
    } else {
        state.touch.started_ms.get_or_insert(motion.at_ms);
        state.touch.max_fingers = state.touch.max_fingers.max(motion.fingers);
    }
    if motion.fingers == 3 {
        let now = motion.points;
        state.touch.three_finger = match state.touch.three_finger {
            ThreeFingerState::Idle => ThreeFingerState::Tracking {
                start: now,
                moved: false,
            },
            ThreeFingerState::Tracking { start, moved } => {
                let moved_by = average3(start, now);
                let swipe = gestures.three_finger_swipes().into_iter().find_map(|(dir, key)| {
                    let key = key?;
                    (within_angle(moved_by, dir, px.swipe_cos_permille, false)
                        && dot(moved_by, dir) >= i64::from(px.three_swipe_along(dir)))
                    .then_some(key)
                });
                match swipe {
                    Some(key) => {
                        tap(Some(key), &mut out);
                        ThreeFingerState::Swiped
                    }
                    None => ThreeFingerState::Tracking {
                        start,
                        moved: moved || len(moved_by) >= px.decide,
                    },
                }
            }
            ThreeFingerState::Swiped => ThreeFingerState::Swiped,
        };
    }
    if had_three || state.touch.max_fingers >= 3 {
        state.two_finger = TwoFingerState::Idle;
        return out;
    }

    if motion.fingers != 2 {
        // Lifted soon after touching, far enough along a swipe: a flick.
        if let TwoFingerState::Flicking {
            start,
            last,
            started_ms,
            dir,
            key,
        } = state.two_finger
            && motion.at_ms.saturating_sub(started_ms) <= px.swipe_ms
            && dot(average(start, last), dir) >= i64::from(px.swipe_along(dir))
        {
            tap(Some(key), &mut out);
        }
        state.two_finger = TwoFingerState::Idle;
        if motion.fingers == 1 && (motion.dx != 0 || motion.dy != 0) {
            out.axes = Some([(Axis::X, motion.dx), (Axis::Y, motion.dy)]);
        }
        return out;
    }
    if !gestures.two_finger_motion() {
        return out;
    }

    let now = [motion.points[0], motion.points[1]];
    state.two_finger = match state.two_finger {
        TwoFingerState::Idle => TwoFingerState::Deciding {
            start: now,
            started_ms: motion.at_ms,
        },
        TwoFingerState::Deciding { start, started_ms } => match classify(start, now, px, gestures.zoom()) {
            // Moving together: maybe a swipe if that way has one, a scroll otherwise.
            Some(TwoFinger::Scroll) => {
                let moved = average(start, now);
                let swipe = gestures.two_finger_swipes().into_iter().find_map(|(dir, key)| {
                    let key = key?;
                    within_angle(moved, dir, px.swipe_cos_permille, false).then_some((dir, key))
                });
                match swipe {
                    Some((dir, key)) => TwoFingerState::Flicking {
                        start,
                        last: now,
                        started_ms,
                        dir,
                        key,
                    },
                    None => TwoFingerState::Scrolling {
                        last: now,
                        axis: scroll_axis(moved),
                    },
                }
            }
            Some(TwoFinger::Zoom) => TwoFingerState::Zooming {
                base: len(sub(start[1], start[0])),
            },
            None => TwoFingerState::Deciding { start, started_ms },
        },
        TwoFingerState::Scrolling { last, axis } => {
            scroll(average(last, now), axis, &mut out);
            TwoFingerState::Scrolling { last: now, axis }
        }
        TwoFingerState::Flicking {
            start,
            started_ms,
            dir,
            key,
            ..
        } => {
            let moved = average(start, now);
            let slow = motion.at_ms.saturating_sub(started_ms) > px.swipe_ms;
            if slow || !within_angle(moved, dir, px.swipe_cos_permille, false) {
                // Too slow or off the swipe's line: a scroll, caught up on the held-back motion.
                let axis = scroll_axis(moved);
                scroll(moved, axis, &mut out);
                TwoFingerState::Scrolling { last: now, axis }
            } else {
                TwoFingerState::Flicking {
                    start,
                    last: now,
                    started_ms,
                    dir,
                    key,
                }
            }
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
            cursor_acceleration: None,
            span: 0,
            last_at_ms: 0,
            cursor_rest: (0, 0),
            scroll_rest: (0, 0),
            state: GestureState::default(),
        }
    }

    /// Accelerate cursor motion.
    pub fn with_cursor_acceleration(mut self, acceleration: Acceleration) -> Self {
        self.cursor_acceleration = Some(acceleration);
        self
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

        self.two_finger_px = TwoFingerPx::new(&self.gestures.two_finger, x_resolution, y_resolution);
        self.span = u32::from(x_resolution.max(y_resolution));

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
        // then absolute X/Y, strength and area of fingers 1 to 3 (§5.2.3-§5.2.6),
        // 7 bytes each at 0x0016, 0x001D and 0x0024.
        let mut data = [0u8; 31];
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
        let points = [point(10), point(17), point(24)];
        let present = [10, 17, 24].map(|at| data[at + 4] != 0 || data[at + 5] != 0);

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
            present,
            at_ms: Instant::now().as_millis(),
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
                && self.state.drag.take().is_some()
                && let Some(key) = self.gestures.press_and_hold
            {
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
                    let dt_ms = motion.at_ms.saturating_sub(self.last_at_ms);
                    self.last_at_ms = motion.at_ms;
                    for (key, pressed) in out.keys {
                        Self::publish_virtual_key(key, pressed).await;
                    }
                    if let Some([(axis_a, a), (axis_b, b)]) = out.axes {
                        let span = self.span;
                        let (acceleration, rest) = if axis_a == Axis::X {
                            (self.cursor_acceleration, &mut self.cursor_rest)
                        } else {
                            (self.gestures.scroll_acceleration, &mut self.scroll_rest)
                        };
                        let (a, b) = match acceleration {
                            Some(acceleration) => {
                                let (a, b) = accelerate((i32::from(a), i32::from(b)), dt_ms, span, acceleration, rest);
                                (clamp16(a), clamp16(b))
                            }
                            None => (a, b),
                        };
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
        scroll_both_axes: false,
        zoom_in: Some(7),
        zoom_out: Some(8),
        // Sideways only, so vertical two-finger motion still scrolls.
        two_finger_swipe_x_neg: Some(9),
        two_finger_swipe_x_pos: Some(10),
        two_finger_swipe_y_neg: None,
        two_finger_swipe_y_pos: None,
        three_finger_tap: Some(11),
        three_finger_swipe_x_neg: Some(12),
        three_finger_swipe_x_pos: Some(13),
        three_finger_swipe_y_neg: Some(14),
        three_finger_swipe_y_pos: Some(15),
        two_finger: TwoFingerConfig {
            decide_percent: 4,
            zoom_cos_permille: 906,
            zoom_step_percent: 6,
            swipe_percent: 20,
            swipe_ms: 250,
            swipe_cos_permille: 866,
            three_swipe_percent: 15,
            three_tap_ms: 300,
        },
        scroll_acceleration: None,
    };

    /// A 1000-pixel trackpad: deciding at 40 px, a zoom step every 60 px, a swipe of
    /// 100 px within 30°, lifted within 250 ms.
    const PX: TwoFingerPx = TwoFingerPx {
        decide: 40,
        zoom_step: 60,
        zoom_cos_permille: 906,
        swipe: [100, 100],
        swipe_ms: 250,
        swipe_cos_permille: 866,
        three_swipe: [150, 150],
        three_tap_ms: 300,
    };

    fn one_finger(gesture_events_0: u8, dx: i16, dy: i16) -> Motion {
        Motion {
            gesture_events_0,
            gesture_events_1: 0,
            fingers: 1,
            dx,
            dy,
            points: [(0, 0); 3],
            present: [true, false, false],
            at_ms: 0,
        }
    }

    fn two_fingers(a: Point, b: Point) -> Motion {
        two_fingers_at(0, a, b)
    }

    fn two_fingers_at(at_ms: u64, a: Point, b: Point) -> Motion {
        Motion {
            gesture_events_0: 0,
            gesture_events_1: 0,
            fingers: 2,
            dx: 0,
            dy: 0,
            points: [a, b, (0, 0)],
            present: [true, true, false],
            at_ms,
        }
    }

    fn three_fingers_at(at_ms: u64, a: Point, b: Point, c: Point) -> Motion {
        Motion {
            fingers: 3,
            points: [a, b, c],
            present: [true; 3],
            ..two_fingers_at(at_ms, a, b)
        }
    }

    fn lifted_at(at_ms: u64) -> Motion {
        Motion { at_ms, ..lifted() }
    }

    fn lifted() -> Motion {
        Motion {
            fingers: 0,
            present: [false; 3],
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

    /// A cycle with the given finger slots down; `hold` sets the IC's press-and-hold.
    fn slots(hold: bool, s: [Option<Point>; 3]) -> Motion {
        Motion {
            gesture_events_0: if hold { 0b10 } else { 0 },
            gesture_events_1: 0,
            fingers: s.iter().filter(|p| p.is_some()).count() as u8,
            dx: 0,
            dy: 0,
            points: s.map(|p| p.unwrap_or((0, 0))),
            present: s.map(|p| p.is_some()),
            at_ms: 0,
        }
    }

    #[test]
    fn press_and_hold_stays_down_and_drags() {
        let (outs, state) = run(&[
            slots(true, [Some((100, 100)), None, None]),
            slots(true, [Some((105, 101)), None, None]),
        ]);
        assert_eq!(keys(&outs), vec![(1, true)]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 5), (Axis::Y, 1)]]);
        assert!(state.drag.is_some());
        let (outs, state) = run(&[slots(true, [Some((100, 100)), None, None]), lifted()]);
        assert_eq!(keys(&outs), vec![(1, true), (1, false)]);
        assert!(state.drag.is_none());
    }

    #[test]
    fn a_drag_goes_on_while_any_finger_touches_and_follows_the_last_one() {
        let (outs, _) = run(&[
            slots(true, [Some((100, 100)), None, None]),
            slots(false, [Some((100, 100)), Some((300, 300)), None]), // a second finger lands
            slots(false, [Some((100, 100)), Some((310, 300)), None]), // and leads
            slots(false, [None, Some((315, 300)), None]),             // the first lifts
            slots(false, [Some((120, 120)), Some((315, 300)), None]), // and lands again: leads
            slots(false, [Some((125, 121)), Some((315, 300)), None]),
            slots(false, [Some((126, 121)), None, None]), // the second lifts
            lifted(),
        ]);
        assert_eq!(keys(&outs), vec![(1, true), (1, false)]);
        assert_eq!(
            axes(&outs),
            vec![
                [(Axis::X, 10), (Axis::Y, 0)],
                [(Axis::X, 5), (Axis::Y, 0)],
                [(Axis::X, 5), (Axis::Y, 1)],
                [(Axis::X, 1), (Axis::Y, 0)],
            ]
        );
    }

    #[test]
    fn when_the_lead_lifts_the_other_finger_takes_over_without_a_jump() {
        let (outs, _) = run(&[
            slots(true, [Some((100, 100)), None, None]),
            slots(false, [Some((100, 100)), Some((300, 300)), None]),
            slots(false, [Some((102, 100)), None, None]), // the lead lifts
            slots(false, [Some((107, 100)), None, None]),
        ]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 5), (Axis::Y, 0)]]);
    }

    #[test]
    fn a_drag_ignores_a_finger_jumping_across_the_trackpad() {
        let (outs, _) = run(&[
            slots(true, [Some((100, 100)), None, None]),
            slots(false, [Some((600, 100)), None, None]), // 500 px in one cycle
            slots(false, [Some((604, 100)), None, None]),
        ]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 4), (Axis::Y, 0)]]);
    }

    #[test]
    fn nothing_else_happens_during_a_drag() {
        let mut two_finger_tap = slots(false, [Some((100, 100)), Some((300, 300)), None]);
        two_finger_tap.gesture_events_1 = 0b1;
        let (outs, _) = run(&[
            slots(true, [Some((100, 100)), None, None]),
            slots(false, [Some((100, 100)), Some((300, 300)), None]),
            two_finger_tap,
            slots(false, [Some((100, 160)), Some((300, 360)), None]), // would scroll
        ]);
        assert_eq!(keys(&outs), vec![(1, true)]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 0), (Axis::Y, 60)]]);
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
    fn a_quick_sideways_flick_swipes_when_the_fingers_lift() {
        let (outs, _) = run(&[
            two_fingers_at(0, (700, 500), (900, 500)),
            two_fingers_at(20, (680, 501), (880, 502)), // decided: together, sideways
            two_fingers_at(60, (600, 505), (800, 505)), // 100 px: held back, no scroll
            lifted_at(100),                             // a flick: swipe
            two_fingers_at(200, (400, 500), (600, 500)),
            two_fingers_at(220, (420, 499), (620, 498)),
            two_fingers_at(250, (520, 500), (720, 500)), // the other way
            lifted_at(300),
        ]);
        assert_eq!(keys(&outs), vec![(9, true), (9, false), (10, true), (10, false)]);
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn a_slow_sideways_move_scrolls_with_the_held_back_motion() {
        let (outs, _) = run(&[
            two_fingers_at(0, (400, 500), (600, 500)),
            two_fingers_at(20, (380, 500), (580, 500)),
            two_fingers_at(300, (300, 500), (500, 500)), // past 250 ms: a scroll
            two_fingers_at(320, (290, 500), (490, 500)),
            lifted_at(400),
        ]);
        assert!(keys(&outs).is_empty());
        assert_eq!(
            axes(&outs),
            vec![[(Axis::H, -100), (Axis::V, 0)], [(Axis::H, -10), (Axis::V, 0)]]
        );
    }

    #[test]
    fn a_short_flick_does_nothing() {
        let (outs, _) = run(&[
            two_fingers_at(0, (400, 500), (600, 500)),
            two_fingers_at(20, (380, 500), (580, 500)), // 20 px of the 100 a swipe needs
            lifted_at(60),
        ]);
        assert!(keys(&outs).is_empty() && axes(&outs).is_empty());
    }

    #[test]
    fn fingers_resting_before_lifting_dont_swipe() {
        // No cycles come while the fingers rest; lifting a second later is no flick.
        let (outs, _) = run(&[
            two_fingers_at(0, (700, 500), (900, 500)),
            two_fingers_at(20, (680, 500), (880, 500)),
            two_fingers_at(60, (580, 500), (780, 500)),
            lifted_at(1000),
        ]);
        assert!(keys(&outs).is_empty());
    }

    #[test]
    fn a_flick_that_turns_vertical_scrolls() {
        let (outs, state) = run(&[
            two_fingers_at(0, (400, 500), (600, 500)),
            two_fingers_at(20, (380, 500), (580, 500)), // sideways: maybe a flick
            two_fingers_at(40, (380, 560), (580, 560)), // now mostly down: scroll
            two_fingers_at(60, (380, 570), (580, 570)),
        ]);
        assert!(keys(&outs).is_empty());
        // Mostly down, so it keeps to vertical, the held-back motion included.
        assert_eq!(
            axes(&outs),
            vec![[(Axis::H, 0), (Axis::V, 60)], [(Axis::H, 0), (Axis::V, 10)]]
        );
        assert!(matches!(state.two_finger, TwoFingerState::Scrolling { .. }));
    }

    #[test]
    fn a_vertical_scroll_ignores_sideways_motion_until_the_fingers_lift() {
        let (outs, _) = run(&[
            two_fingers((400, 500), (600, 500)),
            two_fingers((400, 540), (600, 540)), // decided: a vertical scroll
            two_fingers((450, 545), (650, 545)), // mostly sideways now: only the 5 down count
            two_fingers((500, 545), (700, 545)), // only sideways: nothing
        ]);
        assert_eq!(axes(&outs), vec![[(Axis::H, 0), (Axis::V, 5)]]);
    }

    #[test]
    fn scroll_both_axes_scrolls_diagonally() {
        let gestures = Iqs5xxGestures {
            scroll_both_axes: true,
            ..ALL
        };
        let mut state = GestureState::default();
        let outs: Vec<_> = [
            two_fingers((400, 500), (600, 500)),
            two_fingers((400, 540), (600, 540)),
            two_fingers((450, 545), (650, 545)),
        ]
        .iter()
        .map(|motion| decode_cycle(&gestures, &PX, motion, &mut state))
        .collect();
        assert_eq!(axes(&outs), vec![[(Axis::H, 50), (Axis::V, 5)]]);
    }

    #[test]
    fn a_diagonal_two_finger_move_is_not_a_swipe() {
        let (_, state) = run(&[
            two_fingers((400, 500), (600, 500)),
            two_fingers((370, 530), (570, 530)), // 45°: outside the 30° of a swipe
        ]);
        assert!(matches!(state.two_finger, TwoFingerState::Scrolling { .. }));
    }

    const A: Point = (300, 500);
    const B: Point = (450, 480);
    const C: Point = (600, 500);

    #[test]
    fn three_fingers_tapping_click_once_all_lift() {
        let (outs, _) = run(&[
            Motion {
                at_ms: 0,
                ..one_finger(0, 0, 0)
            },
            two_fingers_at(10, A, B),
            three_fingers_at(20, A, B, C),
            three_fingers_at(60, (302, 501), (451, 480), (600, 502)), // a little jitter
            two_fingers_at(120, A, B),
            lifted_at(150),
        ]);
        assert_eq!(keys(&outs), vec![(11, true), (11, false)]);
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn a_slow_or_moving_three_finger_touch_is_no_tap() {
        let (outs, _) = run(&[three_fingers_at(0, A, B, C), lifted_at(500)]);
        assert!(keys(&outs).is_empty());
        let (outs, _) = run(&[
            three_fingers_at(0, A, B, C),
            three_fingers_at(40, (300, 560), (450, 540), (600, 560)), // 60 px: moved
            lifted_at(100),
        ]);
        assert!(keys(&outs).is_empty());
    }

    #[test]
    fn four_fingers_are_no_three_finger_tap() {
        let mut four = three_fingers_at(20, A, B, C);
        four.fingers = 4;
        let (outs, _) = run(&[three_fingers_at(0, A, B, C), four, lifted_at(100)]);
        assert!(keys(&outs).is_empty());
    }

    #[test]
    fn three_fingers_swipe_once_per_touch() {
        let right = |x: i32| [(A.0 + x, A.1), (B.0 + x, B.1), (C.0 + x, C.1)];
        let down = |y: i32| [(A.0, A.1 + y), (B.0, B.1 + y), (C.0, C.1 + y)];
        let [a, b, c] = right(100);
        let [a2, b2, c2] = right(160);
        let [a3, b3, c3] = right(400);
        let [a4, b4, c4] = right(640);
        let [d, e, f] = down(160);
        let (outs, _) = run(&[
            three_fingers_at(0, A, B, C),
            three_fingers_at(20, a, b, c),    // 100 px of the 150 a swipe needs
            three_fingers_at(40, a2, b2, c2), // swipe right
            three_fingers_at(60, a3, b3, c3), // further: nothing more
            three_fingers_at(70, a4, b4, c4),
            lifted_at(80),
            three_fingers_at(200, A, B, C),
            three_fingers_at(220, d, e, f), // swipe down
            lifted_at(240),
        ]);
        assert_eq!(keys(&outs), vec![(13, true), (13, false), (15, true), (15, false)]);
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn a_three_finger_touch_never_scrolls_or_moves_the_cursor() {
        let (outs, _) = run(&[
            three_fingers_at(0, A, B, C),
            two_fingers_at(20, A, B), // one finger lifts; the other two scroll down
            two_fingers_at(40, (A.0, A.1 + 50), (B.0, B.1 + 50)),
            two_fingers_at(60, (A.0, A.1 + 90), (B.0, B.1 + 90)),
            Motion {
                at_ms: 80,
                ..one_finger(0, 5, 5)
            },
            lifted_at(400),
            // A fresh two-finger touch scrolls again.
            two_fingers_at(500, A, B),
            two_fingers_at(520, (A.0, A.1 + 50), (B.0, B.1 + 50)),
            two_fingers_at(540, (A.0, A.1 + 60), (B.0, B.1 + 60)),
        ]);
        assert!(keys(&outs).is_empty());
        assert_eq!(axes(&outs), vec![[(Axis::H, 0), (Axis::V, 10)]]);
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

    /// From 100% of a 1000-pixel trackpad per second, so 1000 px/s, up to 2.5×.
    const ACCEL: Acceleration = Acceleration {
        from_percent_per_s: 100,
        max_percent: 250,
    };

    #[test]
    fn slow_motion_is_not_accelerated() {
        let mut rest = (0, 0);
        // 5 px in 10 ms is 500 px/s, below 1000.
        assert_eq!(accelerate((5, -3), 10, 1000, ACCEL, &mut rest), (5, -3));
        assert_eq!(rest, (0, 0));
    }

    #[test]
    fn fast_motion_gains_in_proportion_to_its_speed_up_to_the_max() {
        let mut rest = (0, 0);
        // 2000 px/s: twice the threshold, twice the motion.
        assert_eq!(accelerate((20, 0), 10, 1000, ACCEL, &mut rest), (40, 0));
        // 10000 px/s would be 10×, capped at 2.5×.
        assert_eq!(accelerate((100, 0), 10, 1000, ACCEL, &mut rest), (250, 0));
    }

    #[test]
    fn acceleration_carries_fractions_over() {
        let mut rest = (0, 0);
        // 1500 px/s: 1.5 × 15 = 22.5, then 22.5 + 0.5 left over = 23.
        assert_eq!(accelerate((15, 0), 10, 1000, ACCEL, &mut rest), (22, 0));
        assert_eq!(accelerate((15, 0), 10, 1000, ACCEL, &mut rest), (23, 0));
    }

    #[test]
    fn distances_are_a_share_of_the_trackpad() {
        assert_eq!(percent_of(2304, 15), 345);
        assert_eq!(percent_of(2304, 0), 0);
        assert_eq!(percent_of(u16::MAX, 255), u16::MAX);
        // Longer along Y: decide and zoom follow Y, a swipe follows its own axis.
        let px = TwoFingerPx::new(&TwoFingerConfig::default(), 600, 1000);
        assert_eq!(
            px,
            TwoFingerPx {
                decide: 40,
                zoom_step: 60,
                zoom_cos_permille: 906,
                swipe: [60, 100],
                swipe_ms: 250,
                swipe_cos_permille: 866,
                three_swipe: [90, 150],
                three_tap_ms: 300,
            }
        );
        assert_eq!(px.swipe_along((-1, 0)), 60);
        assert_eq!(px.swipe_along((0, 1)), 100);
    }
}
