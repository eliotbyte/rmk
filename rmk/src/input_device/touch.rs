//! Gesture recognition for multi-touch devices such as trackpads.
//!
//! A touchpad publishes [`TouchEvent`]s, frames of where each finger is.
//! [`TouchGestureProcessor`] turns them into:
//!
//! * cursor motion from one finger, and two-finger scrolling, as [`PointingEvent`]s
//!   on the X/Y and H/V axes for a [`crate::input_device::pointing::PointingProcessor`];
//! * gestures, which press [`KeyboardEventPos::Touch`] positions whose actions live
//!   in the keymap's touch map, per layer like encoders.
//!
//! A gesture without an action on the active layer is not recognized at all, so
//! two fingers moving sideways scroll on a layer without two-finger swipes.

use embassy_futures::select::{Either, select};
use embassy_time::{Instant, Timer};
use rmk_macro::processor;
use rmk_types::action::TouchGesture;

use crate::core_traits::Runnable;
use crate::event::{
    Axis, AxisEvent, AxisValType, EventSubscriber, KeyboardEvent, KeyboardEventPos, PointingEvent, TOUCH_MAX_FINGERS,
    TouchEvent, TouchPos, publish_event_async,
};
use crate::keymap::KeyMap;
use crate::processor::Processor;

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
    /// The same as the PointingProcessor's, so that swipe directions are the cursor's.
    pub invert_x: bool,
    pub invert_y: bool,
    pub swap_xy: bool,
    /// Two fingers moving together scroll.
    pub scroll: bool,
    /// Scroll along both axes at once. Off, a scroll keeps to the axis it started
    /// along until the fingers lift.
    pub scroll_both_axes: bool,
    /// A finger moving this far, in percent of the touchpad's longer side, makes a
    /// touch no tap or hold, and decides between a two-finger scroll and zoom.
    pub decide_percent: u8,
    /// A one-finger tap lifts within this many milliseconds of touching.
    pub tap_ms: u16,
    /// A two- or three-finger tap lifts every finger within this many milliseconds
    /// of the first touching.
    pub multi_finger_tap_ms: u16,
    /// One finger held still this many milliseconds is a hold.
    pub hold_ms: u16,
    /// A zoom needs both fingers moving in opposite directions along the line between
    /// them, within this angle; it's `cos(angle)` in permille (25° is 906).
    pub zoom_cos_permille: u16,
    /// How much the distance between the fingers changes per zoom step, in percent of
    /// the touchpad's longer side.
    pub zoom_step_percent: u8,
    /// How far both fingers move together for a two-finger swipe, in percent of the
    /// touchpad's size in the swipe's direction.
    pub swipe_percent: u8,
    /// A two-finger swipe lifts within this many milliseconds of touching; a longer
    /// move scrolls.
    pub swipe_ms: u16,
    /// A two- or three-finger swipe keeps within this angle of its axis; it's
    /// `cos(angle)` in permille (30° is 866).
    pub swipe_cos_permille: u16,
    /// How far three fingers move together for a three-finger swipe, in percent of the
    /// touchpad's size in the swipe's direction.
    pub three_finger_swipe_percent: u8,
}

impl Default for TouchGestureConfig {
    fn default() -> Self {
        Self {
            device_id: 0,
            touchpad_id: 0,
            invert_x: false,
            invert_y: false,
            swap_xy: false,
            scroll: true,
            scroll_both_axes: false,
            decide_percent: 4,
            tap_ms: 200,
            multi_finger_tap_ms: 300,
            hold_ms: 300,
            zoom_cos_permille: 906,
            zoom_step_percent: 6,
            swipe_percent: 10,
            swipe_ms: 250,
            swipe_cos_permille: 866,
            three_finger_swipe_percent: 15,
        }
    }
}

/// [`TouchGestureConfig`]'s distances in touchpad units, for one touchpad size.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Px {
    size: (u16, u16),
    decide: u32,
    zoom_step: u32,
    /// Swipe distances along X and along Y: two fingers side by side have far less
    /// room across a touchpad's short side than a share of its long side.
    swipe: [u32; 2],
    three_finger_swipe: [u32; 2],
}

impl Px {
    fn new(config: &TouchGestureConfig, size: (u16, u16)) -> Self {
        let percent_of = |span: u16, percent: u8| u32::from(span) * u32::from(percent) / 100;
        let span = size.0.max(size.1);
        let per_axis = |percent| [size.0, size.1].map(|side| percent_of(side, percent).max(1));
        Self {
            size,
            decide: percent_of(span, config.decide_percent),
            zoom_step: percent_of(span, config.zoom_step_percent).max(1),
            swipe: per_axis(config.swipe_percent),
            three_finger_swipe: per_axis(config.three_finger_swipe_percent),
        }
    }

    /// The swipe distance along `dir`, a unit vector on one axis.
    fn swipe_along(&self, dir: Point) -> u32 {
        self.swipe[usize::from(dir.0 == 0)]
    }

    fn three_finger_swipe_along(&self, dir: Point) -> u32 {
        self.three_finger_swipe[usize::from(dir.0 == 0)]
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
            count: event.count,
            slots: event.fingers.map(|f| f.map(|p| (i32::from(p.x), i32::from(p.y)))),
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
    keys: heapless::Vec<(TouchGesture, bool), 4>,
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
    /// A finger went further than a tap allows.
    moved: bool,
    /// Something other than a tap happened: a scroll, zoom, swipe or hold.
    acted: bool,
}

/// A hold: the gesture's key stays down while any finger touches, so another finger
/// can take over when the first runs out of room. The finger that landed last moves
/// the cursor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Drag {
    /// The slot moving the cursor, and where it was last frame.
    lead: Option<(usize, Point)>,
}

impl Drag {
    fn any_finger(frame: &Frame) -> Option<(usize, Point)> {
        frame.slots.iter().enumerate().find_map(|(i, p)| p.map(|p| (i, p)))
    }

    /// This frame's cursor motion. A finger landing takes the lead without moving the
    /// cursor; when the lead lifts, a remaining finger takes over from where it is. A
    /// step longer than `jump` is the touchpad renumbering fingers, not motion.
    fn follow(&mut self, previous: &[Option<Point>; TOUCH_MAX_FINGERS], frame: &Frame, jump: u32) -> Point {
        let landed = (0..TOUCH_MAX_FINGERS).find(|&i| frame.slots[i].is_some() && previous[i].is_none());
        if let Some(i) = landed {
            self.lead = frame.slots[i].map(|p| (i, p));
            return (0, 0);
        }
        match self.lead {
            Some((i, last)) if frame.slots[i].is_some() => {
                let now = frame.slots[i].unwrap_or(last);
                self.lead = Some((i, now));
                let step = sub(now, last);
                if len(step) > jump { (0, 0) } else { step }
            }
            _ => {
                self.lead = Self::any_finger(frame);
                (0, 0)
            }
        }
    }
}

/// The axes a two-finger scroll moves along, fixed when it starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ScrollAxis {
    Both,
    Horizontal,
    Vertical,
}

/// Where a two-finger touch stands. Once it is a scroll or a zoom it stays one until
/// it is no longer exactly two fingers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum TwoFinger {
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
    /// moving far enough (`gesture` fires), or a scroll.
    Flicking {
        start: [Point; 2],
        last: [Point; 2],
        started_ms: u64,
        dir: Point,
        gesture: TouchGesture,
    },
}

/// Where the three-finger part of a touch stands.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ThreeFinger {
    #[default]
    Idle,
    /// Three fingers down since `start`.
    Tracking { start: [Point; 3] },
    /// A three-finger swipe fired; nothing more until the fingers lift.
    Swiped,
}

/// The directions of swipes on the touchpad, before the cursor transforms.
const DIRECTIONS: [Point; 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];

/// Gesture recognition, apart from the event plumbing so it can be tested on its own.
#[derive(Debug)]
struct Recognizer {
    config: TouchGestureConfig,
    px: Px,
    touch: Option<Touch>,
    /// Last frame's slots.
    previous: [Option<Point>; TOUCH_MAX_FINGERS],
    drag: Option<Drag>,
    two_finger: TwoFinger,
    three_finger: ThreeFinger,
}

impl Recognizer {
    fn new(config: TouchGestureConfig) -> Self {
        Self {
            config,
            px: Px::default(),
            touch: None,
            previous: [None; TOUCH_MAX_FINGERS],
            drag: None,
            two_finger: TwoFinger::Idle,
            three_finger: ThreeFinger::Idle,
        }
    }

    /// The gesture a swipe along `dir` on the touchpad makes, as the cursor moves.
    fn swipe_gesture(&self, dir: Point, fingers: u8) -> TouchGesture {
        let (mut x, mut y) = dir;
        if self.config.invert_x {
            x = -x;
        }
        if self.config.invert_y {
            y = -y;
        }
        if self.config.swap_xy {
            (x, y) = (y, x);
        }
        let two = fingers == 2;
        match (x, y) {
            (-1, _) if two => TouchGesture::TwoFingerSwipeLeft,
            (1, _) if two => TouchGesture::TwoFingerSwipeRight,
            (_, -1) if two => TouchGesture::TwoFingerSwipeUp,
            _ if two => TouchGesture::TwoFingerSwipeDown,
            (-1, _) => TouchGesture::ThreeFingerSwipeLeft,
            (1, _) => TouchGesture::ThreeFingerSwipeRight,
            (_, -1) => TouchGesture::ThreeFingerSwipeUp,
            _ => TouchGesture::ThreeFingerSwipeDown,
        }
    }

    /// When a still finger turns into a hold, if one is pending.
    fn deadline(&self) -> Option<u64> {
        match self.touch {
            Some(touch) if self.drag.is_none() && touch.max_fingers == 1 && !touch.moved && !touch.acted => {
                Some(touch.started_ms + u64::from(self.config.hold_ms))
            }
            _ => None,
        }
    }

    /// Start a hold if a still finger has been down long enough and holds have an
    /// action.
    fn check_hold(&mut self, now_ms: u64, bound: &impl Fn(TouchGesture) -> bool, out: &mut Output) -> bool {
        match self.deadline() {
            Some(deadline) if now_ms >= deadline && bound(TouchGesture::Hold) => {
                let _ = out.keys.push((TouchGesture::Hold, true));
                self.drag = Some(Drag {
                    lead: self.previous.iter().enumerate().find_map(|(i, p)| p.map(|p| (i, p))),
                });
                if let Some(touch) = &mut self.touch {
                    touch.acted = true;
                }
                true
            }
            _ => false,
        }
    }

    fn timeout(&mut self, now_ms: u64, bound: &impl Fn(TouchGesture) -> bool) -> Output {
        let mut out = Output::default();
        self.check_hold(now_ms, bound, &mut out);
        out
    }

    fn frame(&mut self, frame: &Frame, size: (u16, u16), bound: &impl Fn(TouchGesture) -> bool) -> Output {
        if self.px.size != size {
            self.px = Px::new(&self.config, size);
        }
        let mut out = Output::default();
        let previous = self.previous;
        self.previous = frame.slots;

        // A hold keeps its key down until no finger is left, and nothing else happens
        // meanwhile.
        if let Some(drag) = &mut self.drag {
            if frame.count == 0 {
                let _ = out.keys.push((TouchGesture::Hold, false));
                self.reset();
            } else {
                out.cursor(drag.follow(&previous, frame, self.px.decide * 4));
            }
            return out;
        }

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
                (Some(start), Some(now)) if len(sub(now, start)) > self.px.decide => touch.moved = true,
                _ => {}
            }
        }

        if self.check_hold(frame.at_ms, bound, &mut out) {
            return out;
        }

        // Three fingers: a tap or a swipe, and nothing else for the rest of the touch, so
        // fingers landing or lifting one by one don't move the cursor or scroll.
        if self.touch.is_some_and(|t| t.max_fingers >= 3) {
            self.two_finger = TwoFinger::Idle;
            self.three_fingers(frame, bound, &mut out);
            return out;
        }

        match frame.count {
            1 => {
                // Leaving two fingers may end a flick.
                self.end_two_fingers(frame.at_ms, &mut out);
                if let Some(i) = frame.slots.iter().position(Option::is_some)
                    && let (Some(now), Some(last)) = (frame.slots[i], previous[i])
                    && previous.iter().flatten().count() == 1
                {
                    out.cursor(sub(now, last));
                }
            }
            2 => self.two_fingers(frame, bound, &mut out),
            _ => {}
        }
        out
    }

    fn reset(&mut self) {
        self.touch = None;
        self.drag = None;
        self.two_finger = TwoFinger::Idle;
        self.three_finger = ThreeFinger::Idle;
    }

    /// The last finger lifted: a tap if the touch was short and still, or the end of
    /// a flick.
    fn lift(&mut self, at_ms: u64, bound: &impl Fn(TouchGesture) -> bool, out: &mut Output) {
        self.end_two_fingers(at_ms, out);
        let Some(touch) = self.touch else {
            return;
        };
        if touch.moved || touch.acted {
            return;
        }
        let elapsed = at_ms.saturating_sub(touch.started_ms);
        let (gesture, limit) = match touch.max_fingers {
            1 => (TouchGesture::Tap, self.config.tap_ms),
            2 => (TouchGesture::TwoFingerTap, self.config.multi_finger_tap_ms),
            3 => (TouchGesture::ThreeFingerTap, self.config.multi_finger_tap_ms),
            _ => return,
        };
        if elapsed <= u64::from(limit) && bound(gesture) {
            out.tap(gesture);
        }
    }

    /// No longer two fingers: a flick fires if it was lifted soon after touching, far
    /// enough along its swipe.
    fn end_two_fingers(&mut self, at_ms: u64, out: &mut Output) {
        if let TwoFinger::Flicking {
            start,
            last,
            started_ms,
            dir,
            gesture,
        } = self.two_finger
            && at_ms.saturating_sub(started_ms) <= u64::from(self.config.swipe_ms)
            && dot(average(start, last), dir) >= i64::from(self.px.swipe_along(dir))
        {
            out.tap(gesture);
        }
        self.two_finger = TwoFinger::Idle;
    }

    fn three_fingers(&mut self, frame: &Frame, bound: &impl Fn(TouchGesture) -> bool, out: &mut Output) {
        let Some(now) = frame.first::<3>().filter(|_| frame.count == 3) else {
            return;
        };
        self.three_finger = match self.three_finger {
            ThreeFinger::Idle => ThreeFinger::Tracking { start: now },
            ThreeFinger::Tracking { start } => {
                let moved_by = average3(start, now);
                let swipe = DIRECTIONS.into_iter().find_map(|dir| {
                    let gesture = self.swipe_gesture(dir, 3);
                    (bound(gesture)
                        && within_angle(moved_by, dir, self.config.swipe_cos_permille.into(), false)
                        && dot(moved_by, dir) >= i64::from(self.px.three_finger_swipe_along(dir)))
                    .then_some(gesture)
                });
                match swipe {
                    Some(gesture) => {
                        out.tap(gesture);
                        if let Some(touch) = &mut self.touch {
                            touch.acted = true;
                        }
                        ThreeFinger::Swiped
                    }
                    None => ThreeFinger::Tracking { start },
                }
            }
            ThreeFinger::Swiped => ThreeFinger::Swiped,
        };
    }

    fn two_fingers(&mut self, frame: &Frame, bound: &impl Fn(TouchGesture) -> bool, out: &mut Output) {
        let Some(now) = frame.first::<2>() else {
            return;
        };
        let zoom = bound(TouchGesture::ZoomIn) || bound(TouchGesture::ZoomOut);
        let scroll_axis = |moved: Point| match moved {
            _ if self.config.scroll_both_axes => ScrollAxis::Both,
            (h, v) if h.abs() > v.abs() => ScrollAxis::Horizontal,
            _ => ScrollAxis::Vertical,
        };
        let scroll = |moved: Point, axis: ScrollAxis, out: &mut Output| {
            if self.config.scroll {
                out.scroll(match axis {
                    ScrollAxis::Both => moved,
                    ScrollAxis::Horizontal => (moved.0, 0),
                    ScrollAxis::Vertical => (0, moved.1),
                });
            }
        };
        let next = match self.two_finger {
            TwoFinger::Idle => TwoFinger::Deciding {
                start: now,
                started_ms: frame.at_ms,
            },
            TwoFinger::Deciding { start, started_ms } => match classify(start, now, &self.px, &self.config, zoom) {
                // Moving together: maybe a swipe if that way has one, a scroll otherwise.
                Some(TwoFingerKind::Scroll) => {
                    let moved = average(start, now);
                    let swipe = DIRECTIONS.into_iter().find_map(|dir| {
                        let gesture = self.swipe_gesture(dir, 2);
                        (bound(gesture) && within_angle(moved, dir, self.config.swipe_cos_permille.into(), false))
                            .then_some((dir, gesture))
                    });
                    match swipe {
                        Some((dir, gesture)) => TwoFinger::Flicking {
                            start,
                            last: now,
                            started_ms,
                            dir,
                            gesture,
                        },
                        None => TwoFinger::Scrolling {
                            last: now,
                            axis: scroll_axis(moved),
                        },
                    }
                }
                Some(TwoFingerKind::Zoom) => TwoFinger::Zooming {
                    base: len(sub(start[1], start[0])),
                },
                None => TwoFinger::Deciding { start, started_ms },
            },
            TwoFinger::Scrolling { last, axis } => {
                scroll(average(last, now), axis, out);
                TwoFinger::Scrolling { last: now, axis }
            }
            TwoFinger::Flicking {
                start,
                started_ms,
                dir,
                gesture,
                ..
            } => {
                let moved = average(start, now);
                let slow = frame.at_ms.saturating_sub(started_ms) > u64::from(self.config.swipe_ms);
                if slow || !within_angle(moved, dir, self.config.swipe_cos_permille.into(), false) {
                    // Too slow or off the swipe's line: a scroll, caught up on the held-back motion.
                    let axis = scroll_axis(moved);
                    scroll(moved, axis, out);
                    TwoFinger::Scrolling { last: now, axis }
                } else {
                    TwoFinger::Flicking {
                        start,
                        last: now,
                        started_ms,
                        dir,
                        gesture,
                    }
                }
            }
            zooming @ TwoFinger::Zooming { .. } => zooming,
        };
        self.two_finger = next;
        // At most one zoom step per frame; the rest follow on the next frames.
        if let TwoFinger::Zooming { base } = &mut self.two_finger {
            let distance = len(sub(now[1], now[0]));
            let step = self.px.zoom_step;
            if distance >= *base + step {
                *base += step;
                if bound(TouchGesture::ZoomIn) {
                    out.tap(TouchGesture::ZoomIn);
                }
            } else if distance + step <= *base {
                *base -= step;
                if bound(TouchGesture::ZoomOut) {
                    out.tap(TouchGesture::ZoomOut);
                }
            }
        }
        if !matches!(self.two_finger, TwoFinger::Idle | TwoFinger::Deciding { .. })
            && let Some(touch) = &mut self.touch
        {
            touch.acted = true;
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TwoFingerKind {
    Scroll,
    Zoom,
}

/// Tell a two-finger touch that moved from `start` to `now` apart: a zoom when both
/// fingers move in opposite directions along the line between them, a scroll when
/// they move the same way, or `None` while that is still unclear.
fn classify(
    start: [Point; 2],
    now: [Point; 2],
    px: &Px,
    config: &TouchGestureConfig,
    zoom: bool,
) -> Option<TwoFingerKind> {
    let d1 = sub(now[0], start[0]);
    let d2 = sub(now[1], start[1]);
    let travel = len(d1) + len(d2);
    if travel < px.decide {
        return None;
    }
    let axis = sub(start[1], start[0]);
    let cos = u32::from(config.zoom_cos_permille.min(1000));
    if zoom
        && len(d1) >= px.decide / 4
        && len(d2) >= px.decide / 4
        && within_angle(d1, d2, cos, true)
        && (within_angle(d1, axis, cos, false) || within_angle(d1, axis, cos, true))
        && (within_angle(d2, axis, cos, false) || within_angle(d2, axis, cos, true))
    {
        return Some(TwoFingerKind::Zoom);
    }
    // Within 45° of each other; failing that, scroll once it has moved a lot, so a
    // touch that never settles still does something harmless.
    if within_angle(d1, d2, 707, false) || travel >= 3 * px.decide {
        return Some(TwoFingerKind::Scroll);
    }
    None
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

/// The average motion of three fingers from `from` to `to`.
fn average3(from: [Point; 3], to: [Point; 3]) -> Point {
    let d = [0, 1, 2].map(|i| sub(to[i], from[i]));
    ((d[0].0 + d[1].0 + d[2].0) / 3, (d[0].1 + d[1].1 + d[2].1) / 3)
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

/// Turns a touchpad's [`TouchEvent`]s into cursor motion, scrolling and gestures.
/// See the [module docs](self).
#[processor(subscribe = [TouchEvent])]
#[::rmk::macros::runnable_generated]
pub struct TouchGestureProcessor<'a> {
    keymap: &'a KeyMap<'a>,
    recognizer: Recognizer,
}

impl<'a> TouchGestureProcessor<'a> {
    pub fn new(keymap: &'a KeyMap<'a>, config: TouchGestureConfig) -> Self {
        Self {
            keymap,
            recognizer: Recognizer::new(config),
        }
    }

    async fn on_touch_event(&mut self, event: TouchEvent) {
        if event.device_id != self.recognizer.config.device_id {
            return;
        }
        let frame = Frame::from_event(&event, Instant::now().as_millis());
        let keymap = self.keymap;
        let id = self.recognizer.config.touchpad_id;
        let out = self
            .recognizer
            .frame(&frame, event.size, &|gesture| keymap.touch_gesture_bound(id, gesture));
        self.publish(out).await;
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

impl Runnable for TouchGestureProcessor<'_> {
    async fn run(&mut self) -> ! {
        let mut sub = <Self as Processor>::subscriber();
        loop {
            match self.recognizer.deadline() {
                Some(deadline) => match select(Timer::at(Instant::from_millis(deadline)), sub.next_event()).await {
                    Either::First(_) => {
                        let keymap = self.keymap;
                        let id = self.recognizer.config.touchpad_id;
                        let out = self.recognizer.timeout(Instant::now().as_millis(), &|gesture| {
                            keymap.touch_gesture_bound(id, gesture)
                        });
                        self.publish(out).await;
                    }
                    Either::Second(event) => self.process(event).await,
                },
                None => {
                    let event = sub.next_event().await;
                    self.process(event).await;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 1000-unit touchpad: deciding at 40, a zoom step every 60, a two-finger swipe of
    /// 100 and a three-finger swipe of 150.
    const SIZE: (u16, u16) = (1000, 1000);

    /// Every gesture but vertical two-finger swipes, so vertical two-finger motion
    /// still scrolls.
    fn bound(gesture: TouchGesture) -> bool {
        !matches!(
            gesture,
            TouchGesture::TwoFingerSwipeUp | TouchGesture::TwoFingerSwipeDown
        )
    }

    enum Step {
        Frame(Frame),
        Timeout(u64),
    }

    /// A frame at `at_ms` with these finger slots.
    fn f(at_ms: u64, slots: &[Option<Point>]) -> Step {
        let mut all = [None; TOUCH_MAX_FINGERS];
        all[..slots.len()].copy_from_slice(slots);
        Step::Frame(Frame {
            count: all.iter().flatten().count() as u8,
            slots: all,
            at_ms,
        })
    }

    fn one(at_ms: u64, a: Point) -> Step {
        f(at_ms, &[Some(a)])
    }

    fn two(at_ms: u64, a: Point, b: Point) -> Step {
        f(at_ms, &[Some(a), Some(b)])
    }

    fn three(at_ms: u64, a: Point, b: Point, c: Point) -> Step {
        f(at_ms, &[Some(a), Some(b), Some(c)])
    }

    fn lift(at_ms: u64) -> Step {
        f(at_ms, &[])
    }

    fn run_with(
        config: TouchGestureConfig,
        bound: impl Fn(TouchGesture) -> bool,
        steps: &[Step],
    ) -> (Vec<Output>, Recognizer) {
        let mut recognizer = Recognizer::new(config);
        let outs = steps
            .iter()
            .map(|step| match step {
                Step::Frame(frame) => recognizer.frame(frame, SIZE, &bound),
                Step::Timeout(at_ms) => recognizer.timeout(*at_ms, &bound),
            })
            .collect();
        (outs, recognizer)
    }

    fn run(steps: &[Step]) -> (Vec<Output>, Recognizer) {
        run_with(TouchGestureConfig::default(), bound, steps)
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
    fn a_still_finger_holds_and_drags() {
        let (outs, recognizer) = run(&[
            one(0, (100, 100)),
            Step::Timeout(299),
            Step::Timeout(300),
            one(310, (105, 101)),
        ]);
        assert_eq!(keys(&outs), vec![(Hold, true)]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 5), (Axis::Y, 1)]]);
        assert!(recognizer.drag.is_some());
        let (outs, recognizer) = run(&[one(0, (100, 100)), Step::Timeout(300), lift(400)]);
        assert_eq!(keys(&outs), tapped(Hold));
        assert!(recognizer.drag.is_none());
    }

    #[test]
    fn a_frame_after_the_hold_time_holds_too() {
        let (outs, _) = run(&[one(0, (100, 100)), one(320, (101, 100))]);
        assert_eq!(keys(&outs), vec![(Hold, true)]);
    }

    #[test]
    fn a_moving_finger_or_one_without_a_hold_action_doesnt_hold() {
        let (outs, recognizer) = run(&[one(0, (100, 100)), one(20, (200, 100)), Step::Timeout(400)]);
        assert!(keys(&outs).is_empty());
        assert_eq!(recognizer.deadline(), None);
        let (outs, _) = run_with(
            TouchGestureConfig::default(),
            |g| g != Hold,
            &[one(0, (100, 100)), Step::Timeout(400), one(410, (104, 100))],
        );
        assert!(keys(&outs).is_empty());
        assert_eq!(axes(&outs), vec![[(Axis::X, 4), (Axis::Y, 0)]]);
    }

    #[test]
    fn a_drag_goes_on_while_any_finger_touches_and_follows_the_last_one() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            Step::Timeout(300),
            f(310, &[Some((100, 100)), Some((300, 300))]), // a second finger lands
            f(320, &[Some((100, 100)), Some((310, 300))]), // and leads
            f(330, &[None, Some((315, 300))]),             // the first lifts
            f(340, &[Some((120, 120)), Some((315, 300))]), // and lands again: leads
            f(350, &[Some((125, 121)), Some((315, 300))]),
            f(360, &[Some((126, 121)), None]), // the second lifts
            lift(370),
        ]);
        assert_eq!(keys(&outs), tapped(Hold));
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
            one(0, (100, 100)),
            Step::Timeout(300),
            f(310, &[Some((100, 100)), Some((300, 300))]),
            f(320, &[Some((102, 100)), None]), // the lead lifts
            f(330, &[Some((107, 100)), None]),
        ]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 5), (Axis::Y, 0)]]);
    }

    #[test]
    fn a_drag_ignores_a_finger_jumping_across_the_touchpad() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            Step::Timeout(300),
            one(310, (600, 100)), // 500 in one frame
            one(320, (604, 100)),
        ]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 4), (Axis::Y, 0)]]);
    }

    #[test]
    fn nothing_else_happens_during_a_drag() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            Step::Timeout(300),
            two(310, (100, 100), (300, 300)),
            two(320, (100, 160), (300, 360)), // would scroll
        ]);
        assert_eq!(keys(&outs), vec![(Hold, true)]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 0), (Axis::Y, 60)]]);
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
    fn two_fingers_moving_apart_along_their_line_zoom_in() {
        let (outs, _) = run(&[
            two(0, (400, 500), (600, 500)),  // 200 apart
            two(10, (380, 501), (620, 499)), // decided: opposite, along the line; 240
            two(20, (340, 500), (660, 500)), // 320: a step past 200 + 60
            two(30, (345, 500), (655, 500)), // 310: short of the next at 320
        ]);
        assert!(axes(&outs).is_empty());
        assert_eq!(keys(&outs), tapped(ZoomIn));
    }

    #[test]
    fn two_fingers_pinching_zoom_out() {
        let (outs, _) = run(&[
            two(0, (300, 500), (700, 500)),  // 400 apart
            two(10, (330, 500), (670, 500)), // decided; 340: a step below 400 - 60
            two(20, (340, 500), (660, 500)), // 320: short of the next at 280
        ]);
        assert_eq!(keys(&outs), tapped(ZoomOut));
    }

    #[test]
    fn opposite_but_across_the_line_is_not_a_zoom() {
        // Rotating: the fingers move apart from each other's path, not along it.
        let (outs, recognizer) = run(&[two(0, (400, 500), (600, 500)), two(10, (400, 470), (600, 530))]);
        assert!(keys(&outs).is_empty());
        assert!(!matches!(recognizer.two_finger, TwoFinger::Zooming { .. }));
    }

    #[test]
    fn one_finger_still_is_not_a_zoom() {
        let (outs, recognizer) = run(&[two(0, (400, 500), (600, 500)), two(10, (400, 500), (650, 500))]);
        assert!(keys(&outs).is_empty());
        assert!(!matches!(recognizer.two_finger, TwoFinger::Zooming { .. }));
    }

    #[test]
    fn without_zoom_actions_spreading_fingers_dont_zoom() {
        let (outs, recognizer) = run_with(
            TouchGestureConfig::default(),
            |g| !matches!(g, ZoomIn | ZoomOut),
            &[two(0, (400, 500), (600, 500)), two(10, (380, 501), (620, 499))],
        );
        assert!(keys(&outs).is_empty());
        assert!(!matches!(recognizer.two_finger, TwoFinger::Zooming { .. }));
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
        let (outs, _) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (400, 540), (600, 540)), // scrolling
            two(20, (350, 540), (650, 540)), // fingers spread: still scrolls
            two(30, (300, 540), (700, 540)),
            lift(40),
            two(100, (400, 500), (600, 500)),
            two(110, (350, 500), (650, 500)), // a new touch zooms: 300, past 260
            two(120, (345, 500), (655, 500)),
        ]);
        // The spread mid-scroll scrolls by zero and doesn't zoom; only the new touch does.
        assert_eq!(keys(&outs), tapped(ZoomIn));
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn a_quick_sideways_flick_swipes_when_the_fingers_lift() {
        let (outs, _) = run(&[
            two(0, (700, 500), (900, 500)),
            two(20, (680, 501), (880, 502)), // decided: together, sideways
            two(60, (600, 505), (800, 505)), // 100: held back, no scroll
            lift(100),                       // a flick: swipe
            two(200, (400, 500), (600, 500)),
            two(220, (420, 499), (620, 498)),
            two(250, (520, 500), (720, 500)), // the other way
            lift(300),
        ]);
        let mut expected = tapped(TwoFingerSwipeLeft);
        expected.extend(tapped(TwoFingerSwipeRight));
        assert_eq!(keys(&outs), expected);
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn swipes_are_named_by_the_cursor_direction() {
        let config = TouchGestureConfig {
            invert_x: true,
            ..TouchGestureConfig::default()
        };
        let (outs, _) = run_with(
            config,
            bound,
            &[
                two(0, (700, 500), (900, 500)),
                two(20, (680, 501), (880, 502)),
                two(60, (600, 505), (800, 505)),
                lift(100),
            ],
        );
        assert_eq!(keys(&outs), tapped(TwoFingerSwipeRight));
    }

    #[test]
    fn a_slow_sideways_move_scrolls_with_the_held_back_motion() {
        let (outs, _) = run(&[
            two(0, (400, 500), (600, 500)),
            two(20, (380, 500), (580, 500)),
            two(300, (300, 500), (500, 500)), // past 250 ms: a scroll
            two(320, (290, 500), (490, 500)),
            lift(400),
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
            two(0, (400, 500), (600, 500)),
            two(20, (380, 500), (580, 500)), // 20 of the 100 a swipe needs
            lift(60),
        ]);
        assert!(keys(&outs).is_empty() && axes(&outs).is_empty());
    }

    #[test]
    fn fingers_resting_before_lifting_dont_swipe() {
        // No frames come while the fingers rest; lifting a second later is no flick.
        let (outs, _) = run(&[
            two(0, (700, 500), (900, 500)),
            two(20, (680, 500), (880, 500)),
            two(60, (580, 500), (780, 500)),
            lift(1000),
        ]);
        assert!(keys(&outs).is_empty());
    }

    #[test]
    fn a_flick_that_turns_vertical_scrolls() {
        let (outs, recognizer) = run(&[
            two(0, (400, 500), (600, 500)),
            two(20, (380, 500), (580, 500)), // sideways: maybe a flick
            two(40, (380, 560), (580, 560)), // now mostly down: scroll
            two(60, (380, 570), (580, 570)),
        ]);
        assert!(keys(&outs).is_empty());
        // Mostly down, so it keeps to vertical, the held-back motion included.
        assert_eq!(
            axes(&outs),
            vec![[(Axis::H, 0), (Axis::V, 60)], [(Axis::H, 0), (Axis::V, 10)]]
        );
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

    #[test]
    fn scroll_both_axes_scrolls_diagonally() {
        let config = TouchGestureConfig {
            scroll_both_axes: true,
            ..TouchGestureConfig::default()
        };
        let (outs, _) = run_with(
            config,
            bound,
            &[
                two(0, (400, 500), (600, 500)),
                two(10, (400, 540), (600, 540)),
                two(20, (450, 545), (650, 545)),
            ],
        );
        assert_eq!(axes(&outs), vec![[(Axis::H, 50), (Axis::V, 5)]]);
    }

    #[test]
    fn scrolling_off_keeps_two_fingers_quiet() {
        let config = TouchGestureConfig {
            scroll: false,
            ..TouchGestureConfig::default()
        };
        let (outs, _) = run_with(
            config,
            bound,
            &[
                two(0, (100, 500), (300, 500)),
                two(10, (100, 520), (300, 522)),
                two(20, (100, 530), (300, 532)),
            ],
        );
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn a_diagonal_two_finger_move_is_not_a_swipe() {
        let (_, recognizer) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (370, 530), (570, 530)), // 45 degrees: outside the 30 of a swipe
        ]);
        assert!(matches!(recognizer.two_finger, TwoFinger::Scrolling { .. }));
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
        let Step::Frame(mut four) = three(20, A, B, C) else {
            unreachable!()
        };
        four.count = 4;
        let (outs, _) = run(&[three(0, A, B, C), Step::Frame(four), lift(100)]);
        assert!(keys(&outs).is_empty());
    }

    #[test]
    fn three_fingers_swipe_once_per_touch() {
        let right = |x: i32| [(A.0 + x, A.1), (B.0 + x, B.1), (C.0 + x, C.1)];
        let down = |y: i32| [(A.0, A.1 + y), (B.0, B.1 + y), (C.0, C.1 + y)];
        let [a, b, c] = right(100);
        let [a2, b2, c2] = right(160);
        let [a3, b3, c3] = right(400);
        let [d, e, g] = down(160);
        let (outs, _) = run(&[
            three(0, A, B, C),
            three(20, a, b, c),    // 100 of the 150 a swipe needs
            three(40, a2, b2, c2), // swipe right
            three(60, a3, b3, c3), // further: nothing more
            lift(80),
            three(200, A, B, C),
            three(220, d, e, g), // swipe down
            lift(240),
        ]);
        let mut expected = tapped(ThreeFingerSwipeRight);
        expected.extend(tapped(ThreeFingerSwipeDown));
        assert_eq!(keys(&outs), expected);
        assert!(axes(&outs).is_empty());
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
        assert_eq!(px.zoom_step, 120);
        // Swipes follow each axis's own size.
        assert_eq!(px.swipe, [200, 100]);
        assert_eq!(px.three_finger_swipe, [300, 150]);
    }
}
