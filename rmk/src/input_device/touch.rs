//! Gesture recognition for multi-touch devices such as trackpads.
//!
//! A touchpad publishes [`TouchEvent`]s, raw scans of where each finger is, which
//! stay on the board it's on. [`TouchGestureProcessor`] runs on that board too and
//! turns them into:
//!
//! * cursor motion from one finger, and two-finger scrolling, as [`PointingEvent`]s
//!   on the X/Y and H/V axes for a [`crate::input_device::pointing::PointingProcessor`];
//! * gestures, which press [`KeyboardEventPos::Touch`] positions whose actions live
//!   in the keymap's touch map, per layer like encoders.
//!
//! Only these cross the split link. A gesture without an action on the active layer
//! is not recognized at all. The processor follows the active layer through
//! [`LayerChangeEvent`]s, and gets its touchpad's actions per layer when built.

use embassy_time::Instant;
use rmk_macro::processor;
use rmk_types::action::{KeyAction, TouchAction, TouchGesture};

use crate::event::{
    Axis, AxisEvent, AxisValType, KeyboardEvent, KeyboardEventPos, LayerChangeEvent, PointingEvent, TOUCH_MAX_CONTACTS,
    TouchEvent, TouchPos, publish_event_async,
};

/// Fingers the recognizer follows: every contact a [`TouchEvent`] carries.
const TOUCH_MAX_FINGERS: usize = TOUCH_MAX_CONTACTS;

/// A finger position or a motion, in touchpad units.
type Point = (i32, i32);

/// How [`TouchGestureProcessor`] recognizes gestures. Distances are a share of the
/// touchpad, so they suit any size.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchGestureConfig {
    /// The pointing device whose frames to process. Cursor motion and scrolling go out
    /// with this id too.
    pub device_id: u8,
    /// The touchpad's index in the touch map.
    pub touchpad_id: u8,
    /// A one-finger tap moves at most this far, in percent of the touchpad's longer
    /// side; more is a cursor move.
    pub tap_move_percent: u8,
    /// A tap lasts at least this many milliseconds; a shorter touch is a graze.
    pub tap_min_ms: u16,
}

impl Default for TouchGestureConfig {
    fn default() -> Self {
        Self {
            device_id: 0,
            touchpad_id: 0,
            tap_move_percent: 3,
            tap_min_ms: 0,
        }
    }
}

/// A finger moving this far, in percent of the touchpad's longer side, makes a touch
/// no tap, and decides what two fingers do.
const DECIDE_PERCENT: u8 = 4;

/// A one-finger tap lifts within this many milliseconds of touching, as in libinput.
const TAP_MS: u64 = 180;

/// A two- or three-finger tap lifts every finger within this many milliseconds of
/// the first touching.
const MULTI_FINGER_TAP_MS: u64 = 300;

/// The recognizer's distances in touchpad units, for one touchpad size.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Px {
    size: (u16, u16),
    decide: u32,
    tap_move: u32,
}

impl Px {
    fn new(config: &TouchGestureConfig, size: (u16, u16)) -> Self {
        let percent_of = |span: u16, percent: u8| u32::from(span) * u32::from(percent) / 100;
        let span = size.0.max(size.1);
        Self {
            size,
            decide: percent_of(span, DECIDE_PERCENT),
            tap_move: percent_of(span, config.tap_move_percent),
        }
    }
}

/// One frame, as the recognizer sees it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Frame {
    /// How many fingers touch.
    count: u8,
    /// Finger positions by slot.
    slots: [Option<Point>; TOUCH_MAX_FINGERS],
    at_ms: u64,
}

impl Frame {
    fn from_event(event: &TouchEvent, at_ms: u64) -> Self {
        Self {
            count: event.contacts.iter().flatten().count() as u8,
            slots: event.contacts.map(|c| c.map(|c| (i32::from(c.x), i32::from(c.y)))),
            at_ms,
        }
    }

    /// The first `N` fingers, if that many slots hold one.
    fn first<const N: usize>(&self) -> Option<[Point; N]> {
        let mut points = [(0, 0); N];
        let mut found = self.slots.iter().flatten();
        for point in &mut points {
            *point = *found.next()?;
        }
        Some(points)
    }
}

/// What a frame or a timeout turns into: gesture key presses `(gesture, pressed)` in
/// order, and the axes to publish, X/Y for the cursor or H/V for scrolling.
#[derive(Debug, Default, PartialEq, Eq)]
struct Output {
    keys: heapless::Vec<(TouchGesture, bool), 2>,
    axes: Option<[(Axis, i16); 2]>,
}

impl Output {
    fn tap(&mut self, gesture: TouchGesture) {
        let _ = self.keys.push((gesture, true));
        let _ = self.keys.push((gesture, false));
    }

    fn cursor(&mut self, (dx, dy): Point) {
        if (dx, dy) != (0, 0) {
            self.axes = Some([(Axis::X, clamp16(dx)), (Axis::Y, clamp16(dy))]);
        }
    }

    fn scroll(&mut self, (dh, dv): Point) {
        if (dh, dv) != (0, 0) {
            self.axes = Some([(Axis::H, clamp16(dh)), (Axis::V, clamp16(dv))]);
        }
    }
}

/// One touch: from the first finger landing until no finger is left.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Touch {
    started_ms: u64,
    /// The most fingers down at once so far.
    max_fingers: u8,
    /// Where each slot's finger landed.
    landed: [Option<Point>; TOUCH_MAX_FINGERS],
    /// A finger went further than `decide`.
    moved: bool,
    /// The furthest a finger went from where it landed.
    travel: u32,
    /// Something other than a tap happened: a scroll.
    acted: bool,
}

/// The axis a two-finger scroll moves along, fixed when it starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ScrollAxis {
    Horizontal,
    Vertical,
}

/// Where a two-finger touch stands. Once it is a scroll it stays one until it is no
/// longer exactly two fingers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum TwoFinger {
    #[default]
    Idle,
    /// Two fingers down at `start`, not moved far enough to tell.
    /// While one of them rests, the other's motion already moves the cursor, from
    /// `last`; `sent` is how much of each finger's motion went out that way.
    Deciding {
        start: [Point; 2],
        last: [Point; 2],
        sent: [Point; 2],
    },
    Scrolling {
        last: [Point; 2],
        axis: ScrollAxis,
    },
    /// Only one of the two fingers moves: it moves the cursor. `last` is where both
    /// were on the last frame.
    Pointing {
        finger: usize,
        last: [Point; 2],
    },
}

/// Gesture recognition, apart from the event plumbing so it can be tested on its own.
#[derive(Debug)]
struct Recognizer {
    config: TouchGestureConfig,
    px: Px,
    touch: Option<Touch>,
    /// Last frame's slots.
    previous: [Option<Point>; TOUCH_MAX_FINGERS],
    two_finger: TwoFinger,
}

impl Recognizer {
    fn new(config: TouchGestureConfig) -> Self {
        Self {
            config,
            px: Px::default(),
            touch: None,
            previous: [None; TOUCH_MAX_FINGERS],
            two_finger: TwoFinger::Idle,
        }
    }

    fn frame(&mut self, frame: &Frame, size: (u16, u16), bound: &impl Fn(TouchGesture) -> bool) -> Output {
        if self.px.size != size {
            self.px = Px::new(&self.config, size);
        }
        let mut out = Output::default();
        let previous = self.previous;
        self.previous = frame.slots;

        if frame.count == 0 {
            self.lift(frame.at_ms, bound, &mut out);
            self.reset();
            return out;
        }

        let touch = self.touch.get_or_insert(Touch {
            started_ms: frame.at_ms,
            ..Touch::default()
        });
        touch.max_fingers = touch.max_fingers.max(frame.count);
        for (landed, now) in touch.landed.iter_mut().zip(frame.slots) {
            match (*landed, now) {
                (None, Some(now)) => *landed = Some(now),
                (Some(start), Some(now)) => {
                    let distance = len(sub(now, start));
                    touch.travel = touch.travel.max(distance);
                    touch.moved |= distance > self.px.decide;
                }
                _ => {}
            }
        }

        // Three fingers: a tap, and nothing else for the rest of the touch, so fingers
        // landing or lifting one by one don't move the cursor or scroll.
        if self.touch.is_some_and(|t| t.max_fingers >= 3) {
            self.two_finger = TwoFinger::Idle;
            return out;
        }

        match frame.count {
            1 => {
                self.two_finger = TwoFinger::Idle;
                // The finger left over from a scroll doesn't move the cursor.
                let after_two_fingers = self.touch.is_some_and(|t| t.max_fingers >= 2 && t.acted);
                // A finger keeps its slot while another lifts, so its motion goes on
                // without a jump.
                if !after_two_fingers
                    && let Some(i) = frame.slots.iter().position(Option::is_some)
                    && let (Some(now), Some(last)) = (frame.slots[i], previous[i])
                {
                    out.cursor(sub(now, last));
                }
            }
            2 => self.two_fingers(frame, &mut out),
            _ => {}
        }
        out
    }

    fn reset(&mut self) {
        self.touch = None;
        self.two_finger = TwoFinger::Idle;
    }

    /// The last finger lifted: a tap if the touch was short and still.
    fn lift(&mut self, at_ms: u64, bound: &impl Fn(TouchGesture) -> bool, out: &mut Output) {
        self.two_finger = TwoFinger::Idle;
        let Some(touch) = self.touch else {
            return;
        };
        if touch.moved || touch.acted {
            return;
        }
        let elapsed = at_ms.saturating_sub(touch.started_ms);
        // Shorter is a graze.
        if elapsed < u64::from(self.config.tap_min_ms) {
            return;
        }
        let (gesture, limit) = match touch.max_fingers {
            1 if touch.travel > self.px.tap_move => return,
            1 => (TouchGesture::Tap, TAP_MS),
            2 => (TouchGesture::TwoFingerTap, MULTI_FINGER_TAP_MS),
            3 => (TouchGesture::ThreeFingerTap, MULTI_FINGER_TAP_MS),
            _ => return,
        };
        if elapsed <= limit && bound(gesture) {
            out.tap(gesture);
        }
    }

    fn two_fingers(&mut self, frame: &Frame, out: &mut Output) {
        let Some(now) = frame.first::<2>() else {
            return;
        };
        let scroll_axis = |(h, v): Point| {
            if h.abs() > v.abs() {
                ScrollAxis::Horizontal
            } else {
                ScrollAxis::Vertical
            }
        };
        let scroll = |moved: Point, axis: ScrollAxis, out: &mut Output| {
            out.scroll(match axis {
                ScrollAxis::Horizontal => (moved.0, 0),
                ScrollAxis::Vertical => (0, moved.1),
            });
        };
        let next = match self.two_finger {
            TwoFinger::Idle => TwoFinger::Deciding {
                start: now,
                last: now,
                sent: [(0, 0); 2],
            },
            TwoFinger::Deciding { start, last, mut sent } => match classify(start, now, &self.px) {
                // Moving together: a scroll.
                Some(TwoFingerKind::Scroll) => TwoFinger::Scrolling {
                    last: now,
                    axis: scroll_axis(average(start, now)),
                },
                // The rest of its motion moves the cursor too, as long as the other
                // finger rested; otherwise it has been held back long and would jump.
                Some(TwoFingerKind::Point(finger)) => {
                    if len(sub(now[1 - finger], start[1 - finger])) < self.px.decide / 4 {
                        out.cursor(sub(sub(now[finger], start[finger]), sent[finger]));
                    }
                    TwoFinger::Pointing { finger, last: now }
                }
                None => {
                    // One finger resting while the other moves is most likely a cursor
                    // move: follow it now, rather than send it all at once when told.
                    let moved = [0, 1].map(|i| len(sub(now[i], start[i])));
                    let mover = usize::from(moved[1] > moved[0]);
                    if moved[1 - mover] < self.px.decide / 4
                        && moved[mover] >= self.px.decide / 8
                        && moved[mover] >= 2 * moved[1 - mover]
                    {
                        let step = sub(now[mover], last[mover]);
                        out.cursor(step);
                        sent[mover] = (sent[mover].0 + step.0, sent[mover].1 + step.1);
                    }
                    TwoFinger::Deciding { start, last: now, sent }
                }
            },
            TwoFinger::Scrolling { last, axis } => {
                scroll(common(last, now), axis, out);
                TwoFinger::Scrolling { last: now, axis }
            }
            TwoFinger::Pointing { finger, last } => {
                out.cursor(sub(now[finger], last[finger]));
                TwoFinger::Pointing { finger, last: now }
            }
        };
        self.two_finger = next;
        // Moving the cursor with one of two fingers is no gesture.
        if !matches!(
            self.two_finger,
            TwoFinger::Idle | TwoFinger::Deciding { .. } | TwoFinger::Pointing { .. }
        ) && let Some(touch) = &mut self.touch
        {
            touch.acted = true;
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TwoFingerKind {
    Scroll,
    /// Only this finger moves.
    Point(usize),
}

/// Tell a two-finger touch that moved from `start` to `now` apart: a scroll when both
/// move the same way at a similar speed, a cursor move when only one moves, or `None`
/// while that is still unclear.
fn classify(start: [Point; 2], now: [Point; 2], px: &Px) -> Option<TwoFingerKind> {
    let d1 = sub(now[0], start[0]);
    let d2 = sub(now[1], start[1]);
    let (l1, l2) = (len(d1), len(d2));
    let travel = l1 + l2;
    if travel < px.decide {
        return None;
    }
    let (slow, fast) = (l1.min(l2), l1.max(l2));
    let faster = usize::from(l2 > l1);
    // Both moving, neither more than three times as fast: a finger resting while the
    // other moves drifts a little, and that's no gesture of two fingers.
    let both_move = slow >= px.decide / 4 && slow * 3 >= fast;
    // Both moving within 45° of each other.
    if both_move && within_angle(d1, d2, 707, false) {
        return Some(TwoFingerKind::Scroll);
    }
    // One finger moving while the other stays put; failing anything else, the faster
    // one moves the cursor once it has moved a lot, which is harmless.
    if (fast >= px.decide && slow < px.decide / 4) || travel >= 3 * px.decide {
        return Some(TwoFingerKind::Point(faster));
    }
    None
}

/// The motion two fingers share from `from` to `to`: on each axis, the smaller of
/// their motions if they go the same way, nothing otherwise. One finger moving alone
/// shares nothing.
fn common(from: [Point; 2], to: [Point; 2]) -> Point {
    let (d1, d2) = (sub(to[0], from[0]), sub(to[1], from[1]));
    let shared = |a: i32, b: i32| {
        if a.signum() == b.signum() {
            a.signum() * a.abs().min(b.abs())
        } else {
            0
        }
    };
    (shared(d1.0, d2.0), shared(d1.1, d2.1))
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

fn clamp16(x: i32) -> i16 {
    x.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

/// The average motion of the two fingers from `from` to `to`.
fn average(from: [Point; 2], to: [Point; 2]) -> Point {
    let (d1, d2) = (sub(to[0], from[0]), sub(to[1], from[1]));
    ((d1.0 + d2.0) / 2, (d1.1 + d2.1) / 2)
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

/// Whether `gesture` has an action on `layer` in `layers`, one touchpad's actions per
/// layer, or on a layer below where it is transparent.
fn bound(layers: &[TouchAction], layer: u8, gesture: TouchGesture) -> bool {
    layers
        .iter()
        .take(usize::from(layer) + 1)
        .rev()
        .map(|actions| actions.get(gesture))
        .find(|&action| action != KeyAction::Transparent)
        .is_some_and(|action| action != KeyAction::No)
}

/// Turns a touchpad's [`TouchEvent`]s into cursor motion, scrolling and gestures, on
/// the board the touchpad is on. See the [module docs](self).
#[processor(subscribe = [TouchEvent, LayerChangeEvent])]
pub struct TouchGestureProcessor<'a> {
    /// The touchpad's gesture actions per layer, as in the keymap's touch map.
    layers: &'a [TouchAction],
    /// The active layer.
    layer: u8,
    recognizer: Recognizer,
}

impl<'a> TouchGestureProcessor<'a> {
    /// `layers` holds the touchpad's gesture actions per layer, as in the keymap's
    /// touch map. Only whether a gesture has an action matters here: the keymap
    /// performs it.
    pub fn new(layers: &'a [TouchAction], config: TouchGestureConfig) -> Self {
        Self {
            layers,
            layer: 0,
            recognizer: Recognizer::new(config),
        }
    }

    async fn on_touch_event(&mut self, event: TouchEvent) {
        if event.device_id != self.recognizer.config.device_id {
            return;
        }
        let frame = Frame::from_event(&event, Instant::now().as_millis());
        let (layers, layer) = (self.layers, self.layer);
        let out = self
            .recognizer
            .frame(&frame, event.max, &|gesture| bound(layers, layer, gesture));
        self.publish(out).await;
    }

    async fn on_layer_change_event(&mut self, event: LayerChangeEvent) {
        self.layer = event.0;
    }

    async fn publish(&self, out: Output) {
        let config = &self.recognizer.config;
        for (gesture, pressed) in out.keys {
            publish_event_async(KeyboardEvent {
                pressed,
                pos: KeyboardEventPos::Touch(TouchPos {
                    id: config.touchpad_id,
                    gesture,
                }),
            })
            .await;
        }
        if let Some([(axis_a, a), (axis_b, b)]) = out.axes {
            let rel = |axis, value| AxisEvent {
                typ: AxisValType::Rel,
                axis,
                value,
            };
            publish_event_async(PointingEvent {
                device_id: config.device_id,
                axes: [rel(axis_a, a), rel(axis_b, b), rel(Axis::Z, 0)],
            })
            .await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 1000-unit touchpad: deciding at 40.
    const SIZE: (u16, u16) = (1000, 1000);

    fn bound(_: TouchGesture) -> bool {
        true
    }

    /// A frame at `at_ms` with these finger slots.
    fn f(at_ms: u64, slots: &[Option<Point>]) -> Frame {
        let mut all = [None; TOUCH_MAX_FINGERS];
        all[..slots.len()].copy_from_slice(slots);
        Frame {
            count: all.iter().flatten().count() as u8,
            slots: all,
            at_ms,
        }
    }

    fn one(at_ms: u64, a: Point) -> Frame {
        f(at_ms, &[Some(a)])
    }

    fn two(at_ms: u64, a: Point, b: Point) -> Frame {
        f(at_ms, &[Some(a), Some(b)])
    }

    fn three(at_ms: u64, a: Point, b: Point, c: Point) -> Frame {
        f(at_ms, &[Some(a), Some(b), Some(c)])
    }

    fn lift(at_ms: u64) -> Frame {
        f(at_ms, &[])
    }

    fn run_with(
        config: TouchGestureConfig,
        bound: impl Fn(TouchGesture) -> bool,
        frames: &[Frame],
    ) -> (Vec<Output>, Recognizer) {
        let mut recognizer = Recognizer::new(config);
        let outs = frames
            .iter()
            .map(|frame| recognizer.frame(frame, SIZE, &bound))
            .collect();
        (outs, recognizer)
    }

    fn run(frames: &[Frame]) -> (Vec<Output>, Recognizer) {
        run_with(TouchGestureConfig::default(), bound, frames)
    }

    fn keys(outs: &[Output]) -> Vec<(TouchGesture, bool)> {
        outs.iter().flat_map(|out| out.keys.iter().copied()).collect()
    }

    fn axes(outs: &[Output]) -> Vec<[(Axis, i16); 2]> {
        outs.iter().filter_map(|out| out.axes).collect()
    }

    fn tapped(gesture: TouchGesture) -> Vec<(TouchGesture, bool)> {
        vec![(gesture, true), (gesture, false)]
    }

    use TouchGesture::*;

    #[test]
    fn a_gesture_is_bound_by_the_first_layer_down_that_isnt_transparent() {
        let tap = KeyAction::Single(rmk_types::action::Action::Key(rmk_types::keycode::KeyCode::Hid(
            rmk_types::keycode::HidKeyCode::MouseBtn1,
        )));
        let layers = [
            TouchAction::new().with(Tap, tap).with(TwoFingerTap, tap),
            TouchAction::transparent().with(TwoFingerTap, KeyAction::No),
            TouchAction::transparent(),
        ];
        assert!(super::bound(&layers, 0, Tap));
        assert!(!super::bound(&layers, 0, ThreeFingerTap));
        // Transparent on layers 1 and 2: layer 0's action.
        assert!(super::bound(&layers, 2, Tap));
        // Taken away on layer 1, so on layer 2 too.
        assert!(super::bound(&layers, 0, TwoFingerTap));
        assert!(!super::bound(&layers, 2, TwoFingerTap));
        // A layer past the touch map is no layer of its own.
        assert!(super::bound(&layers, 9, Tap));
    }

    #[test]
    fn one_finger_moves_the_cursor() {
        let (outs, _) = run(&[one(0, (100, 100)), one(10, (103, 96))]);
        assert!(keys(&outs).is_empty());
        assert_eq!(axes(&outs), vec![[(Axis::X, 3), (Axis::Y, -4)]]);
    }

    #[test]
    fn a_short_still_touch_taps() {
        let (outs, _) = run(&[one(0, (100, 100)), one(50, (102, 101)), lift(100)]);
        assert_eq!(keys(&outs), tapped(Tap));
    }

    #[test]
    fn a_long_or_moving_touch_is_no_tap() {
        let (outs, _) = run(&[one(0, (100, 100)), lift(250)]);
        assert!(keys(&outs).is_empty());
        let (outs, _) = run(&[one(0, (100, 100)), one(20, (200, 100)), lift(50)]);
        assert!(keys(&outs).is_empty());
    }

    #[test]
    fn two_fingers_tapping_tap_once_both_lift() {
        let (outs, _) = run(&[one(0, (300, 500)), two(10, (300, 500), (500, 500)), lift(150)]);
        assert_eq!(keys(&outs), tapped(TwoFingerTap));
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn a_gesture_without_an_action_is_not_recognized() {
        let (outs, _) = run_with(
            TouchGestureConfig::default(),
            |g| g != Tap,
            &[one(0, (100, 100)), lift(100)],
        );
        assert!(keys(&outs).is_empty());
    }

    #[test]
    fn a_short_touch_that_moves_is_a_cursor_move_not_a_tap() {
        // 35 is past the 30 a tap may move, though short of the 40 that decides gestures.
        let (outs, _) = run(&[one(0, (100, 100)), one(20, (135, 100)), lift(60)]);
        assert!(keys(&outs).is_empty());
        assert_eq!(axes(&outs), vec![[(Axis::X, 35), (Axis::Y, 0)]]);
    }

    #[test]
    fn a_touch_too_short_for_a_tap_is_a_graze() {
        let config = TouchGestureConfig {
            tap_min_ms: 20,
            ..TouchGestureConfig::default()
        };
        let (outs, _) = run_with(config, bound, &[one(0, (100, 100)), lift(10)]);
        assert!(keys(&outs).is_empty());
        let (outs, _) = run_with(config, bound, &[one(0, (100, 100)), lift(30)]);
        assert_eq!(keys(&outs), tapped(Tap));
    }

    #[test]
    fn the_finger_left_from_a_scroll_doesnt_move_the_cursor() {
        let (outs, _) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (400, 540), (600, 540)),
            one(20, (400, 545)),
            one(30, (420, 560)),
        ]);
        assert!(axes(&outs).iter().all(|a| a[0].0 == Axis::H));
    }

    #[test]
    fn one_finger_moving_beside_a_resting_one_moves_the_cursor() {
        let (outs, recognizer) = run(&[
            two(0, (300, 500), (600, 500)),
            two(10, (300, 500), (600, 450)), // only the second moves: 50
            two(20, (300, 501), (610, 440)),
            f(30, &[None, Some((620, 430))]), // the resting one lifts: the mover goes on
        ]);
        assert!(keys(&outs).is_empty());
        assert_eq!(
            axes(&outs),
            vec![
                [(Axis::X, 0), (Axis::Y, -50)],
                [(Axis::X, 10), (Axis::Y, -10)],
                [(Axis::X, 10), (Axis::Y, -10)],
            ]
        );
        assert!(matches!(recognizer.touch, Some(t) if !t.acted));
    }

    #[test]
    fn once_scrolling_one_finger_moving_alone_does_nothing() {
        let (outs, _) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (400, 530), (600, 530)), // decided: a scroll
            two(20, (400, 540), (600, 540)), // scrolls 10
            two(30, (400, 600), (600, 540)), // only one moves: no scroll, no cursor
            two(40, (450, 650), (600, 540)),
            one(50, (460, 660)), // the other lifts: still no cursor
            one(60, (480, 680)),
        ]);
        assert_eq!(axes(&outs), vec![[(Axis::H, 0), (Axis::V, 10)]]);
    }

    #[test]
    fn a_second_finger_moving_beside_a_resting_one_moves_the_cursor_without_a_jump() {
        // From a recording on a TPS43 (2048 tall): the first finger rests, the second
        // sets off slowly. The cursor used to get all 108 at once when it was told.
        let config = TouchGestureConfig::default();
        let mut recognizer = Recognizer::new(config);
        let bound = |_| true;
        let frames = [
            (89338, (926, 1299), (1627, 643)),
            (89353, (926, 1297), (1627, 643)),
            (89361, (925, 1295), (1627, 642)),
            (89398, (925, 1295), (1626, 642)),
            (89421, (925, 1294), (1625, 641)),
            (89443, (925, 1294), (1625, 638)),
            (89458, (925, 1294), (1624, 635)),
            (89466, (925, 1294), (1622, 629)),
            (89481, (925, 1294), (1619, 619)),
            (89488, (925, 1294), (1617, 604)),
            (89503, (925, 1294), (1615, 585)),
            (89518, (925, 1294), (1614, 564)),
            (89533, (925, 1295), (1615, 535)),
            (89541, (924, 1296), (1616, 505)),
            (89556, (924, 1297), (1616, 481)),
        ];
        let mut total = (0, 0);
        for (at_ms, a, b) in frames {
            let out = recognizer.frame(
                &Frame {
                    count: 2,
                    slots: [Some(a), Some(b), None, None, None],
                    at_ms,
                },
                (1792, 2048),
                &bound,
            );
            if let Some([(Axis::X, x), (Axis::Y, y)]) = out.axes {
                // The finger itself moves up to 30 a frame here.
                assert!(y.abs() <= 40, "a {y} step at {at_ms}");
                total = (total.0 + i32::from(x), total.1 + i32::from(y));
            }
        }
        // All of the second finger's motion still reaches the cursor.
        assert_eq!(total, (-11, -162));
        assert!(matches!(recognizer.two_finger, TwoFinger::Pointing { finger: 1, .. }));
    }

    #[test]
    fn a_scroll_follows_only_the_motion_both_fingers_share() {
        let (outs, _) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (400, 530), (600, 530)), // decided: a scroll
            two(20, (400, 545), (600, 540)), // one went further: the shared 10 scrolls
            two(30, (400, 560), (600, 540)), // one stopped: nothing
        ]);
        assert_eq!(axes(&outs), vec![[(Axis::H, 0), (Axis::V, 10)]]);
    }

    #[test]
    fn two_fingers_moving_together_scroll() {
        let (outs, recognizer) = run(&[
            two(0, (100, 500), (300, 500)),
            two(10, (100, 520), (300, 522)), // decided: same direction
            two(20, (100, 530), (300, 532)),
        ]);
        assert!(keys(&outs).is_empty());
        assert_eq!(axes(&outs), vec![[(Axis::H, 0), (Axis::V, 10)]]);
        assert!(matches!(recognizer.two_finger, TwoFinger::Scrolling { .. }));
    }

    #[test]
    fn a_finger_moving_away_from_a_drifting_one_moves_the_cursor() {
        // As recorded on a TPS43: one finger rests but drifts slowly away, while the
        // other moves away from it ten times as fast.
        let steps: Vec<_> = (0..=10)
            .map(|i| two(i * 10, (400 - 2 * i as i32, 500), (600 + 20 * i as i32, 500)))
            .collect();
        let (outs, recognizer) = run(&steps);
        assert!(keys(&outs).is_empty());
        assert!(matches!(recognizer.two_finger, TwoFinger::Pointing { finger: 1, .. }));
    }

    #[test]
    fn a_resting_finger_setting_off_across_the_line_keeps_pointing() {
        let (outs, recognizer) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (400, 500), (650, 500)), // the second finger moves the cursor
            two(20, (400, 540), (700, 500)), // and the first sets off across
        ]);
        assert!(keys(&outs).is_empty());
        assert!(matches!(recognizer.two_finger, TwoFinger::Pointing { finger: 1, .. }));
    }

    #[test]
    fn small_jitter_decides_nothing() {
        let (outs, recognizer) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (390, 500), (610, 500)), // opposite, but only 20 in all
        ]);
        assert!(keys(&outs).is_empty() && axes(&outs).is_empty());
        assert!(matches!(recognizer.two_finger, TwoFinger::Deciding { .. }));
    }

    #[test]
    fn a_scroll_stays_a_scroll_until_the_fingers_lift() {
        let (outs, recognizer) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (400, 540), (600, 540)), // scrolling
            two(20, (350, 540), (650, 540)), // fingers spread: still a scroll, by zero
            two(30, (300, 540), (700, 540)),
        ]);
        assert!(keys(&outs).is_empty() && axes(&outs).is_empty());
        assert!(matches!(recognizer.two_finger, TwoFinger::Scrolling { .. }));
    }

    #[test]
    fn a_vertical_scroll_ignores_sideways_motion_until_the_fingers_lift() {
        let (outs, _) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (400, 540), (600, 540)), // decided: a vertical scroll
            two(20, (450, 545), (650, 545)), // mostly sideways now: only the 5 down count
            two(30, (500, 545), (700, 545)), // only sideways: nothing
        ]);
        assert_eq!(axes(&outs), vec![[(Axis::H, 0), (Axis::V, 5)]]);
    }

    const A: Point = (300, 500);
    const B: Point = (450, 480);
    const C: Point = (600, 500);

    #[test]
    fn three_fingers_tapping_tap_once_all_lift() {
        let (outs, _) = run(&[
            one(0, A),
            two(10, A, B),
            three(20, A, B, C),
            three(60, (302, 501), (451, 480), (600, 502)), // a little jitter
            two(120, A, B),
            lift(150),
        ]);
        assert_eq!(keys(&outs), tapped(ThreeFingerTap));
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn a_slow_or_moving_three_finger_touch_is_no_tap() {
        let (outs, _) = run(&[three(0, A, B, C), lift(500)]);
        assert!(keys(&outs).is_empty());
        let (outs, _) = run(&[
            three(0, A, B, C),
            three(40, (300, 560), (450, 540), (600, 560)), // 60: moved
            lift(100),
        ]);
        assert!(keys(&outs).is_empty());
    }

    #[test]
    fn four_fingers_are_no_three_finger_tap() {
        let four = Frame {
            count: 4,
            ..three(20, A, B, C)
        };
        let (outs, _) = run(&[three(0, A, B, C), four, lift(100)]);
        assert!(keys(&outs).is_empty());
    }

    #[test]
    fn a_three_finger_touch_never_scrolls_or_moves_the_cursor() {
        let (outs, _) = run(&[
            three(0, A, B, C),
            two(20, A, B), // one finger lifts; the other two scroll down
            two(40, (A.0, A.1 + 50), (B.0, B.1 + 50)),
            two(60, (A.0, A.1 + 90), (B.0, B.1 + 90)),
            one(80, (A.0 + 5, A.1 + 95)),
            one(90, (A.0 + 10, A.1 + 100)),
            lift(400),
            // A fresh two-finger touch scrolls again.
            two(500, A, B),
            two(520, (A.0, A.1 + 50), (B.0, B.1 + 50)),
            two(540, (A.0, A.1 + 60), (B.0, B.1 + 60)),
        ]);
        assert!(keys(&outs).is_empty());
        assert_eq!(axes(&outs), vec![[(Axis::H, 0), (Axis::V, 10)]]);
    }

    #[test]
    fn a_third_finger_ends_the_two_finger_gesture() {
        let (_, recognizer) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (400, 540), (600, 540)),
            three(20, (400, 540), (600, 540), (500, 300)),
        ]);
        assert_eq!(recognizer.two_finger, TwoFinger::Idle);
    }

    #[test]
    fn distances_are_a_share_of_the_touchpad() {
        let px = Px::new(&TouchGestureConfig::default(), (2000, 1000));
        assert_eq!(px.decide, 80);
    }
}
