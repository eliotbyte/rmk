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
//!
//! A tap waits out a short window before it clicks, in case another touch follows:
//! tap, then touch and move holds the tap's action while the finger touches, so it
//! drags; two quick taps double-click; and tap, tap, touch and move double-clicks
//! and drags. Two- and three-finger taps drag the same way when the touch after them
//! has as many fingers; such a drag only moves the cursor, and goes on while any of
//! its fingers touches.
//!
//! Optionally a drag outlives a lifted finger briefly, so the finger can be put back
//! down to go on (drag lock), and a two-finger scroll goes on and slows down after
//! the fingers lift quickly (inertia).

use embassy_futures::select::{Either, select};
use embassy_time::{Instant, Timer};
use rmk_macro::processor;
use rmk_types::action::{KeyAction, TouchAction, TouchGesture};

use crate::core_traits::Runnable;
use crate::event::{
    Axis, AxisEvent, AxisValType, EventSubscriber, KeyboardEvent, KeyboardEventPos, LayerChangeEvent, PointingEvent,
    TOUCH_MAX_CONTACTS, TouchEvent, TouchPos, publish_event_async,
};
use crate::processor::Processor;

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
    /// A touch with as many fingers as a tap, starting within this many milliseconds
    /// of the tap lifting, turns the tap into a drag, holding its action until the
    /// touch lifts. A tap clicks only once this has passed. 0 turns tap drags off, so
    /// taps click at once.
    pub tap_drag_ms: u16,
    /// A drag waits this many milliseconds after its fingers lift before it ends, so
    /// a finger put back down goes on with it; a quick still tap then ends it at once.
    /// 0 turns this off.
    pub drag_lock_ms: u16,
    /// A two-finger scroll that ends quickly goes on, slowing down with this time
    /// constant in milliseconds, until a finger touches. 0 turns this off.
    pub scroll_inertia_ms: u16,
}

impl Default for TouchGestureConfig {
    fn default() -> Self {
        Self {
            device_id: 0,
            touchpad_id: 0,
            tap_move_percent: 3,
            tap_min_ms: 0,
            tap_drag_ms: 180,
            drag_lock_ms: 0,
            scroll_inertia_ms: 0,
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

/// The touch after a tap only drags, or taps again, if it lands within this distance
/// of the tap, in percent of the touchpad's longer side.
const TAP_DRAG_DISTANCE_PERCENT: u8 = 8;

/// The recognizer's distances in touchpad units, for one touchpad size.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Px {
    size: (u16, u16),
    decide: u32,
    tap_move: u32,
    tap_drag_distance: u32,
}

impl Px {
    fn new(config: &TouchGestureConfig, size: (u16, u16)) -> Self {
        let percent_of = |span: u16, percent: u8| u32::from(span) * u32::from(percent) / 100;
        let span = size.0.max(size.1);
        Self {
            size,
            decide: percent_of(span, DECIDE_PERCENT),
            tap_move: percent_of(span, config.tap_move_percent),
            tap_drag_distance: percent_of(span, TAP_DRAG_DISTANCE_PERCENT),
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
    keys: heapless::Vec<(TouchGesture, bool), { 2 * MAX_PENDING_TAPS as usize + 2 }>,
    axes: Option<[(Axis, i16); 2]>,
}

impl Output {
    fn taps(&mut self, gesture: TouchGesture, count: u8) {
        for _ in 0..count {
            self.tap(gesture);
        }
    }

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

/// A drag: `gesture`'s key stays down while any finger touches, so another finger
/// can take over when the first runs out of room. The finger that landed last moves
/// the cursor. A touch soon after a tap starts one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Drag {
    gesture: TouchGesture,
    /// The slot moving the cursor, and where it was last frame.
    lead: Option<(usize, Point)>,
    /// When the fingers lifted, while drag lock keeps the drag going.
    lifted_ms: Option<u64>,
    /// The touch since drag lock caught the drag: when it started and how far it
    /// moved, as a quick still one ends the drag.
    relock: Option<(u64, u32)>,
}

impl Drag {
    fn new(gesture: TouchGesture, lead: Option<(usize, Point)>) -> Self {
        Self {
            gesture,
            lead,
            lifted_ms: None,
            relock: None,
        }
    }
}

/// How often inertial scrolling steps.
const INERTIA_TICK_MS: u64 = 16;

/// A scroll faster than this, in percent of the touchpad's longer side per second,
/// goes on after the fingers lift; inertia stops below the second.
const INERTIA_START_PERCENT_PER_S: u32 = 50;
const INERTIA_STOP_PERCENT_PER_S: u32 = 5;

/// A scroll going on after the fingers lifted, slowing down.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Inertia {
    /// Speed in touchpad units per second, on H and V.
    velocity: Point,
    last_ms: u64,
    /// Fractions of a unit, in thousandths, carried to the next step.
    rest: Point,
}

/// The most one-finger taps waiting to click; more in a row click as many.
const MAX_PENDING_TAPS: u8 = 3;

/// Taps of one kind waiting out the tap-drag window before they click.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PendingTaps {
    /// `Tap`, `TwoFingerTap` or `ThreeFingerTap`.
    gesture: TouchGesture,
    count: u8,
    /// When the last of them lifted.
    lifted_ms: u64,
    /// Where the last of them touched.
    at: [Option<Point>; TOUCH_MAX_FINGERS],
    /// The touch that started within the window, until it is another tap or a drag.
    touch: Option<NextTouch>,
}

/// The touch after pending taps.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct NextTouch {
    started_ms: u64,
    /// Its cursor motion, held back meanwhile.
    held: Point,
    /// The most fingers down at once so far.
    fingers: u8,
}

/// How long the fingers of a touch after a multi-finger tap have to land, before a
/// touch with fewer fingers is something else.
const LAND_MS: u64 = 100;

/// How many fingers tap for `gesture`.
fn tap_fingers(gesture: TouchGesture) -> u8 {
    match gesture {
        TouchGesture::TwoFingerTap => 2,
        TouchGesture::ThreeFingerTap => 3,
        _ => 1,
    }
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
    /// Two fingers down at `start` since `started_ms`, not moved far enough to tell.
    /// While one of them rests, the other's motion already moves the cursor, from
    /// `last`; `sent` is how much of each finger's motion went out that way.
    Deciding {
        start: [Point; 2],
        started_ms: u64,
        last: [Point; 2],
        sent: [Point; 2],
    },
    Scrolling {
        last: [Point; 2],
        axis: ScrollAxis,
    },
    /// Only one of the two fingers moves: it moves the cursor. `last` is where both
    /// were on the last frame. Until `JOIN_MS` after the second finger landed at
    /// `started_ms`, the other setting off from `from` still makes it a scroll if it
    /// follows.
    Pointing {
        finger: usize,
        last: [Point; 2],
        started_ms: u64,
        from: [Point; 2],
    },
}

/// How long after a second finger lands it can still join the moving one for a
/// scroll: these often start with one finger, the second put down on the way.
const JOIN_MS: u64 = 300;

/// How long after a finger lands or lifts beside another the touchpad may still
/// report one point between them, gliding there and back.
const GLIDE_MS: u64 = 50;

/// Gesture recognition, apart from the event plumbing so it can be tested on its own.
#[derive(Debug)]
struct Recognizer {
    config: TouchGestureConfig,
    px: Px,
    touch: Option<Touch>,
    /// Last frame's slots.
    previous: [Option<Point>; TOUCH_MAX_FINGERS],
    drag: Option<Drag>,
    pending_taps: Option<PendingTaps>,
    inertia: Option<Inertia>,
    /// When the last frame came.
    previous_ms: u64,
    /// The two-finger scroll's speed in touchpad units per second, and when it last
    /// scrolled, for inertia.
    scroll_velocity: Point,
    scrolled_ms: u64,
    /// How far each slot's glides moved it off its finger, taken off its position
    /// until the finger lifts; and when the number of fingers last changed.
    glide: [Point; TOUCH_MAX_FINGERS],
    count_changed_ms: Option<u64>,
    two_finger: TwoFinger,
}

impl Recognizer {
    fn new(config: TouchGestureConfig) -> Self {
        Self {
            config,
            px: Px::default(),
            touch: None,
            previous: [None; TOUCH_MAX_FINGERS],
            drag: None,
            pending_taps: None,
            inertia: None,
            previous_ms: 0,
            scroll_velocity: (0, 0),
            scrolled_ms: 0,
            glide: [(0, 0); TOUCH_MAX_FINGERS],
            count_changed_ms: None,
            two_finger: TwoFinger::Idle,
        }
    }

    /// `frame` without the touchpad's glides. Right after a finger lands or lifts
    /// beside another, the touchpad reports one point between the two for a few
    /// frames, which glides there and back. No finger moves that far that soon, so a
    /// step longer than `decide` then is a glide, not motion.
    fn unglide(&mut self, mut frame: Frame) -> Frame {
        let counts = [frame.slots, self.previous].map(|slots| slots.iter().flatten().count());
        if counts[0] != counts[1] && counts[0].min(counts[1]) >= 1 {
            self.count_changed_ms = Some(frame.at_ms);
        }
        let gliding = self
            .count_changed_ms
            .is_some_and(|changed_ms| frame.at_ms.saturating_sub(changed_ms) <= GLIDE_MS);
        for ((slot, glide), last) in frame.slots.iter_mut().zip(&mut self.glide).zip(self.previous) {
            let Some(now) = slot else {
                *glide = (0, 0);
                continue;
            };
            *now = sub(*now, *glide);
            if let Some(last) = last
                && gliding
                && len(sub(*now, last)) > self.px.decide
            {
                let step = sub(*now, last);
                *glide = (glide.0 + step.0, glide.1 + step.1);
                *now = last;
            }
        }
        frame
    }

    /// When something happens without a frame: a touch after a tap resting long
    /// enough to drag, taps clicking once the tap-drag window has passed, drag lock
    /// running out, or a step of inertial scrolling.
    fn deadline(&self) -> Option<u64> {
        let taps = self.pending_taps.map(|taps| match taps.touch {
            Some(touch) => touch.started_ms + TAP_MS,
            None => taps.lifted_ms + u64::from(self.config.tap_drag_ms),
        });
        let drag_lock = self
            .drag
            .and_then(|drag| drag.lifted_ms)
            .map(|lifted_ms| lifted_ms + u64::from(self.config.drag_lock_ms));
        let inertia = self.inertia.map(|inertia| inertia.last_ms + INERTIA_TICK_MS);
        [taps, drag_lock, inertia].into_iter().flatten().min()
    }

    /// One step of inertial scrolling, up to `now_ms`.
    fn coast(&mut self, now_ms: u64, out: &mut Output) {
        let Some(inertia) = &mut self.inertia else {
            return;
        };
        let dt = now_ms.saturating_sub(inertia.last_ms).max(1) as i64;
        let tau = i64::from(self.config.scroll_inertia_ms.max(1));
        let step = |v: i32, rest: &mut i32| {
            let total = i64::from(v) * dt + i64::from(*rest);
            *rest = (total % 1000) as i32;
            (total / 1000) as i32
        };
        let moved = (
            step(inertia.velocity.0, &mut inertia.rest.0),
            step(inertia.velocity.1, &mut inertia.rest.1),
        );
        // Exponential slowing: v' = v * tau / (tau + dt).
        let slow = |v: i32| (i64::from(v) * tau / (tau + dt)) as i32;
        inertia.velocity = (slow(inertia.velocity.0), slow(inertia.velocity.1));
        inertia.last_ms = now_ms;
        out.scroll(moved);
        let stop = u32::from(self.px.size.0.max(self.px.size.1)) * INERTIA_STOP_PERCENT_PER_S / 100;
        if len(inertia.velocity) < stop.max(1) {
            self.inertia = None;
        }
    }

    /// Follow the scroll's speed, for inertia.
    fn track_scroll(&mut self, moved: Point, at_ms: u64) {
        let dt = at_ms.saturating_sub(self.previous_ms).max(1) as i32;
        let now = (moved.0 * 1000 / dt, moved.1 * 1000 / dt);
        // Average with the last speed, unless the fingers paused since.
        self.scroll_velocity = if at_ms.saturating_sub(self.scrolled_ms) <= 50 {
            (
                (self.scroll_velocity.0 + now.0) / 2,
                (self.scroll_velocity.1 + now.1) / 2,
            )
        } else {
            now
        };
        self.scrolled_ms = at_ms;
    }

    /// Click the taps waiting out the tap-drag window.
    fn flush_taps(&mut self, out: &mut Output) {
        if let Some(taps) = self.pending_taps.take() {
            out.taps(taps.gesture, taps.count);
        }
    }

    /// The touch after the taps is a drag: all but the last tap click, and the last
    /// one's action stays down while the fingers touch.
    fn start_tap_drag(&mut self, lead: Option<(usize, Point)>, out: &mut Output) {
        if let Some(taps) = self.pending_taps.take() {
            out.taps(taps.gesture, taps.count - 1);
            let _ = out.keys.push((taps.gesture, true));
            self.drag = Some(Drag::new(taps.gesture, lead));
        }
    }

    /// Follow the taps waiting to click and the touch after them. `Some` when the
    /// frame is theirs; `None` hands it on as an ordinary frame.
    fn pending_taps_frame(
        &mut self,
        frame: &Frame,
        previous: &[Option<Point>; TOUCH_MAX_FINGERS],
        out: &mut Output,
    ) -> Option<()> {
        let taps = self.pending_taps?;
        let fingers = tap_fingers(taps.gesture);
        match taps.touch {
            Some(touch) => match frame.count {
                // Lifted before it moved or rested: another tap, unless it only grazed
                // or had fewer fingers.
                0 => {
                    let elapsed = frame.at_ms.saturating_sub(touch.started_ms);
                    if touch.fingers < fingers {
                        self.flush_taps(out);
                    } else if elapsed >= u64::from(self.config.tap_min_ms) {
                        self.pending_taps = Some(PendingTaps {
                            count: (taps.count + 1).min(MAX_PENDING_TAPS),
                            lifted_ms: frame.at_ms,
                            touch: None,
                            ..taps
                        });
                    } else {
                        self.pending_taps = Some(PendingTaps { touch: None, ..taps });
                    }
                    self.reset();
                }
                // Fewer fingers than the taps had, for longer than they take to land:
                // no drag after all.
                count
                    if count < fingers
                        && touch.fingers < fingers
                        && frame.at_ms.saturating_sub(touch.started_ms) > LAND_MS =>
                {
                    self.flush_taps(out);
                    return None;
                }
                count if count <= fingers => {
                    let lead = Drag::any_finger(frame);
                    let step = match lead {
                        Some((i, now)) => previous[i].map_or((0, 0), |last| sub(now, last)),
                        None => (0, 0),
                    };
                    let held = (touch.held.0 + step.0, touch.held.1 + step.1);
                    let fingers_down = touch.fingers.max(count);
                    // Moving further than a tap may is a drag, as in libinput.
                    if fingers_down == fingers && len(held) > self.px.tap_move {
                        self.start_tap_drag(lead, out);
                        out.cursor(held);
                    } else {
                        self.pending_taps = Some(PendingTaps {
                            touch: Some(NextTouch {
                                held,
                                fingers: fingers_down,
                                ..touch
                            }),
                            ..taps
                        });
                    }
                }
                // More fingers: no drag after all.
                _ => {
                    self.flush_taps(out);
                    return None;
                }
            },
            None => {
                let in_window = frame.at_ms.saturating_sub(taps.lifted_ms) <= u64::from(self.config.tap_drag_ms);
                let near = frame.slots.iter().flatten().any(|&p| {
                    taps.at
                        .iter()
                        .flatten()
                        .any(|&tapped| len(sub(p, tapped)) <= self.px.tap_drag_distance)
                });
                if (1..=fingers).contains(&frame.count) && in_window && near {
                    self.pending_taps = Some(PendingTaps {
                        at: frame.slots,
                        touch: Some(NextTouch {
                            started_ms: frame.at_ms,
                            held: (0, 0),
                            fingers: frame.count,
                        }),
                        ..taps
                    });
                } else {
                    self.flush_taps(out);
                    return None;
                }
            }
        }
        Some(())
    }

    fn timeout(&mut self, now_ms: u64) -> Output {
        let mut out = Output::default();
        if self
            .inertia
            .is_some_and(|inertia| now_ms >= inertia.last_ms + INERTIA_TICK_MS)
        {
            self.coast(now_ms, &mut out);
        }
        // Drag lock ran out without a finger coming back: the drag ends.
        if let Some(drag) = self.drag
            && let Some(lifted_ms) = drag.lifted_ms
            && now_ms >= lifted_ms + u64::from(self.config.drag_lock_ms)
        {
            let _ = out.keys.push((drag.gesture, false));
            self.reset();
            return out;
        }
        if let Some(taps) = self.pending_taps {
            match taps.touch {
                // The touch after the taps rested: a drag, with whatever it moved, if
                // it has as many fingers as they had.
                Some(touch) if now_ms >= touch.started_ms + TAP_MS => {
                    if touch.fingers < tap_fingers(taps.gesture) {
                        self.flush_taps(&mut out);
                    } else {
                        let lead = self.previous.iter().enumerate().find_map(|(i, p)| p.map(|p| (i, p)));
                        self.start_tap_drag(lead, &mut out);
                        out.cursor(touch.held);
                    }
                }
                None if now_ms >= taps.lifted_ms + u64::from(self.config.tap_drag_ms) => self.flush_taps(&mut out),
                _ => {}
            }
        }
        out
    }

    fn frame(&mut self, frame: &Frame, size: (u16, u16), bound: &impl Fn(TouchGesture) -> bool) -> Output {
        if self.px.size != size {
            self.px = Px::new(&self.config, size);
        }
        let frame = &self.unglide(*frame);
        let mut out = Output::default();
        let previous = self.previous;
        self.previous = frame.slots;
        let previous_ms = self.previous_ms;
        self.previous_ms = frame.at_ms;

        // A finger touching stops inertia, as on a phone, and isn't a tap.
        if frame.count > 0 && self.inertia.take().is_some() && self.touch.is_none() && self.drag.is_none() {
            self.touch = Some(Touch {
                started_ms: frame.at_ms,
                acted: true,
                ..Touch::default()
            });
        }

        // A drag keeps its key down until no finger is left, and nothing else happens
        // meanwhile.
        if let Some(mut drag) = self.drag {
            // A drag after a multi-finger tap takes as many fingers, so they only move
            // the cursor, and lifting some of them doesn't end it.
            let multi_finger = tap_fingers(drag.gesture) > 1;
            // Two fingers scroll while the drag holds its button, and one of them moving
            // alone moves the cursor, so another finger can take over.
            let two_fingers = frame.count >= 2 && drag.lifted_ms.is_none() && !multi_finger;
            if !two_fingers && self.two_finger != TwoFinger::Idle {
                self.end_two_fingers(frame.at_ms);
                drag.lead = Drag::any_finger(frame);
                if frame.count > 0 {
                    self.drag = Some(drag);
                    return out;
                }
            }
            match (frame.count, drag.lifted_ms) {
                (0, Some(_)) => {}
                (0, None) => {
                    // A quick still touch after drag lock caught the drag ends it.
                    let ended = drag.relock.is_some_and(|(started_ms, travel)| {
                        frame.at_ms.saturating_sub(started_ms) <= TAP_MS && travel <= self.px.tap_move
                    });
                    if self.config.drag_lock_ms > 0 && !ended {
                        drag.lifted_ms = Some(frame.at_ms);
                    } else {
                        let _ = out.keys.push((drag.gesture, false));
                        self.reset();
                        return out;
                    }
                }
                // Fingers back within drag lock: the drag goes on from where they are.
                (count, Some(_)) => {
                    drag.lifted_ms = None;
                    // Two fingers aren't a tap that ends a one-finger drag.
                    let scrolls = count >= 2 && !multi_finger;
                    drag.relock = Some((frame.at_ms, if scrolls { u32::MAX } else { 0 }));
                    drag.lead = Drag::any_finger(frame);
                    if scrolls {
                        self.drag = Some(drag);
                        self.two_fingers(frame, &mut out);
                        return out;
                    }
                }
                _ if two_fingers => {
                    if let Some((_, travel)) = &mut drag.relock {
                        *travel = u32::MAX;
                    }
                    self.drag = Some(drag);
                    self.two_fingers(frame, &mut out);
                    return out;
                }
                _ => {
                    let step = drag.follow(&previous, frame, self.px.decide * 4);
                    if let Some((_, travel)) = &mut drag.relock {
                        *travel = travel.saturating_add(len(step));
                    }
                    out.cursor(step);
                }
            }
            self.drag = Some(drag);
            return out;
        }

        if self.pending_taps_frame(frame, &previous, &mut out).is_some() {
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
            // A scroll pauses while one of its fingers is lifted, to go on when it is
            // put back.
            1 if matches!(self.two_finger, TwoFinger::Scrolling { .. }) => {}
            1 => {
                self.end_two_fingers(frame.at_ms);
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
            2 => {
                // The previous frame's time stands in for the last scroll's on the
                // first one.
                if !matches!(self.two_finger, TwoFinger::Scrolling { .. }) {
                    self.scrolled_ms = previous_ms;
                    self.scroll_velocity = (0, 0);
                }
                // The finger put back on a paused scroll lands anywhere: go on from here.
                if let TwoFinger::Scrolling { axis, .. } = self.two_finger
                    && previous.iter().flatten().count() < 2
                    && let Some(now) = frame.first::<2>()
                {
                    self.two_finger = TwoFinger::Scrolling { last: now, axis };
                    return out;
                }
                self.two_fingers(frame, &mut out);
                if let Some([(Axis::H, h), (Axis::V, v)]) = out.axes {
                    self.previous_ms = previous_ms;
                    self.track_scroll((i32::from(h), i32::from(v)), frame.at_ms);
                    self.previous_ms = frame.at_ms;
                }
            }
            _ => {}
        }
        out
    }

    fn reset(&mut self) {
        self.touch = None;
        self.drag = None;
        self.two_finger = TwoFinger::Idle;
    }

    /// The last finger lifted: a tap if the touch was short and still.
    fn lift(&mut self, at_ms: u64, bound: &impl Fn(TouchGesture) -> bool, out: &mut Output) {
        self.end_two_fingers(at_ms);
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
            if self.config.tap_drag_ms > 0 {
                // Clicks once the tap-drag window passes, unless a touch makes it a drag.
                self.pending_taps = Some(PendingTaps {
                    gesture,
                    count: 1,
                    lifted_ms: at_ms,
                    at: touch.landed,
                    touch: None,
                });
            } else {
                out.tap(gesture);
            }
        }
    }

    /// No longer two fingers: a quick scroll goes on with inertia.
    fn end_two_fingers(&mut self, at_ms: u64) {
        let start = u32::from(self.px.size.0.max(self.px.size.1)) * INERTIA_START_PERCENT_PER_S / 100;
        if matches!(self.two_finger, TwoFinger::Scrolling { .. })
            && self.config.scroll_inertia_ms > 0
            && at_ms.saturating_sub(self.scrolled_ms) <= 50
            && len(self.scroll_velocity) >= start
        {
            self.inertia = Some(Inertia {
                velocity: self.scroll_velocity,
                last_ms: at_ms,
                rest: (0, 0),
            });
        }
        self.two_finger = TwoFinger::Idle;
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
                started_ms: frame.at_ms,
                last: now,
                sent: [(0, 0); 2],
            },
            TwoFinger::Deciding {
                start,
                started_ms,
                last,
                mut sent,
            } => match classify(start, now, &self.px) {
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
                    TwoFinger::Pointing {
                        finger,
                        last: now,
                        started_ms,
                        from: start,
                    }
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
                    TwoFinger::Deciding {
                        start,
                        started_ms,
                        last: now,
                        sent,
                    }
                }
            },
            TwoFinger::Scrolling { last, axis } => {
                scroll(common(last, now), axis, out);
                TwoFinger::Scrolling { last: now, axis }
            }
            TwoFinger::Pointing {
                finger,
                last,
                started_ms,
                from,
            } => {
                let other = 1 - finger;
                let (moved, joined) = (sub(now[finger], from[finger]), sub(now[other], from[other]));
                // The other finger set off the same way soon after landing: a scroll that
                // started with one finger.
                if frame.at_ms.saturating_sub(started_ms) <= JOIN_MS
                    && len(joined) >= self.px.decide
                    && within_angle(moved, joined, 707, false)
                {
                    TwoFinger::Scrolling {
                        last: now,
                        axis: scroll_axis(joined),
                    }
                } else {
                    out.cursor(sub(now[finger], last[finger]));
                    TwoFinger::Pointing {
                        finger,
                        last: now,
                        started_ms,
                        from,
                    }
                }
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
#[::rmk::macros::runnable_generated]
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

impl Runnable for TouchGestureProcessor<'_> {
    async fn run(&mut self) -> ! {
        let mut sub = <Self as Processor>::subscriber();
        loop {
            match self.recognizer.deadline() {
                Some(deadline) => match select(Timer::at(Instant::from_millis(deadline)), sub.next_event()).await {
                    Either::First(_) => {
                        let out = self.recognizer.timeout(Instant::now().as_millis());
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

    /// A 1000-unit touchpad: deciding at 40.
    const SIZE: (u16, u16) = (1000, 1000);

    fn bound(_: TouchGesture) -> bool {
        true
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
                Step::Timeout(at_ms) => recognizer.timeout(*at_ms),
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
        let (outs, _) = run(&[one(0, (100, 100)), one(50, (102, 101)), lift(100), Step::Timeout(300)]);
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
        let (outs, _) = run(&[
            one(0, (300, 500)),
            two(10, (300, 500), (500, 500)),
            lift(150),
            Step::Timeout(330),
        ]);
        assert_eq!(keys(&outs), tapped(TwoFingerTap));
        assert_eq!(keys(&outs[3..4]), tapped(TwoFingerTap));
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn two_quick_two_finger_taps_click_twice() {
        let (a, b) = ((300, 500), (500, 500));
        let (outs, _) = run(&[two(0, a, b), lift(100), two(200, a, b), lift(260), Step::Timeout(440)]);
        assert_eq!(keys(&outs), [tapped(TwoFingerTap), tapped(TwoFingerTap)].concat());
    }

    #[test]
    fn two_finger_tap_then_two_fingers_moving_drag_with_its_button_and_only_move_the_cursor() {
        let (outs, recognizer) = run(&[
            two(0, (300, 500), (500, 500)),
            lift(100), // a two-finger tap: waits
            one(200, (300, 500)),
            two(210, (300, 500), (500, 500)),
            two(220, (335, 500), (535, 500)), // moves: a drag
            two(230, (345, 510), (545, 510)), // together: no scroll
            one(240, (345, 510)),             // one finger lifts: still held
            one(250, (365, 510)),
            lift(300),
        ]);
        assert_eq!(keys(&outs), tapped(TwoFingerTap));
        assert_eq!(keys(&outs[4..5]), vec![(TwoFingerTap, true)]);
        assert_eq!(keys(&outs[8..9]), vec![(TwoFingerTap, false)]);
        assert_eq!(
            axes(&outs),
            vec![
                [(Axis::X, 35), (Axis::Y, 0)],
                [(Axis::X, 10), (Axis::Y, 10)],
                [(Axis::X, 20), (Axis::Y, 0)],
            ]
        );
        assert!(recognizer.drag.is_none() && recognizer.pending_taps.is_none());
    }

    #[test]
    fn two_finger_tap_then_two_fingers_resting_drag_and_drag_lock_lets_the_hand_move() {
        let (a, b) = ((300, 500), (500, 500));
        let (outs, _) = run_with(
            locked(300),
            bound,
            &[
                two(0, a, b),
                lift(100),
                two(200, a, b),
                Step::Timeout(380), // rested: a drag
                two(390, (320, 500), (520, 500)),
                lift(400), // lifted: still held
                Step::Timeout(600),
                two(650, (100, 100), (300, 100)), // back down, elsewhere: no jump
                two(660, (110, 100), (310, 100)),
                lift(900),
                Step::Timeout(1200), // nobody came back: released
            ],
        );
        assert_eq!(keys(&outs), tapped(TwoFingerTap));
        assert_eq!(keys(&outs[3..4]), vec![(TwoFingerTap, true)]);
        assert_eq!(keys(&outs[10..11]), vec![(TwoFingerTap, false)]);
        assert_eq!(
            axes(&outs),
            vec![[(Axis::X, 20), (Axis::Y, 0)], [(Axis::X, 10), (Axis::Y, 0)]]
        );
    }

    #[test]
    fn two_finger_tap_then_one_finger_moving_clicks_and_moves_the_cursor() {
        let (outs, recognizer) = run(&[
            two(0, (300, 500), (500, 500)),
            lift(100),
            one(200, (300, 500)),
            one(250, (310, 500)),
            one(320, (330, 500)), // no second finger by now: no drag
        ]);
        assert_eq!(keys(&outs), tapped(TwoFingerTap));
        assert_eq!(keys(&outs[4..5]), tapped(TwoFingerTap));
        assert_eq!(axes(&outs), vec![[(Axis::X, 20), (Axis::Y, 0)]]);
        assert!(recognizer.drag.is_none() && recognizer.pending_taps.is_none());
    }

    #[test]
    fn three_finger_tap_then_three_fingers_moving_drag_with_its_button() {
        let (a, b, c) = ((300, 500), (450, 480), (600, 500));
        let (outs, _) = run(&[
            three(0, a, b, c),
            lift(100),
            three(200, a, b, c),
            three(210, (335, 500), (485, 480), (635, 500)),
            lift(300),
        ]);
        assert_eq!(keys(&outs), tapped(ThreeFingerTap));
        assert_eq!(keys(&outs[3..4]), vec![(ThreeFingerTap, true)]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 35), (Axis::Y, 0)]]);
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
    fn a_tap_clicks_once_the_tap_drag_window_passes() {
        let (outs, recognizer) = run(&[one(0, (100, 100)), lift(80), Step::Timeout(259)]);
        assert!(keys(&outs).is_empty());
        assert_eq!(recognizer.deadline(), Some(260));
        let (outs, _) = run(&[one(0, (100, 100)), lift(80), Step::Timeout(260)]);
        assert_eq!(keys(&outs), tapped(Tap));
    }

    #[test]
    fn tap_then_touch_and_move_is_one_press_held_while_it_drags() {
        let (outs, recognizer) = run(&[
            one(0, (100, 100)),
            lift(80), // a tap: waits
            one(200, (100, 100)),
            one(210, (150, 104)), // moves: a drag, no click before it
            one(220, (155, 104)),
            lift(700),
        ]);
        assert_eq!(keys(&outs), tapped(Tap));
        assert_eq!(keys(&outs[3..4]), vec![(Tap, true)]);
        assert_eq!(keys(&outs[5..6]), vec![(Tap, false)]);
        // The motion held back until it was a drag goes out with the press.
        assert_eq!(
            axes(&outs),
            vec![[(Axis::X, 50), (Axis::Y, 4)], [(Axis::X, 5), (Axis::Y, 0)]]
        );
        assert!(recognizer.drag.is_none() && recognizer.pending_taps.is_none());
    }

    #[test]
    fn tap_then_touch_and_rest_drags_too() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            lift(80),
            one(200, (100, 100)),
            Step::Timeout(400), // rested past tap_ms: a drag
            one(410, (110, 100)),
            lift(500),
        ]);
        assert_eq!(keys(&outs), tapped(Tap));
        assert_eq!(keys(&outs[3..4]), vec![(Tap, true)]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 10), (Axis::Y, 0)]]);
    }

    #[test]
    fn two_quick_taps_double_click() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            lift(80),
            one(200, (101, 100)),
            lift(260),
            Step::Timeout(460),
        ]);
        let mut expected = tapped(Tap);
        expected.extend(tapped(Tap));
        assert_eq!(keys(&outs), expected);
        assert!(keys(&outs[..4]).is_empty());
        assert!(axes(&outs).is_empty());
    }

    #[test]
    fn tap_tap_touch_and_move_double_clicks_and_drags() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            lift(80),
            one(200, (100, 100)),
            lift(260),
            one(400, (100, 100)),
            one(410, (150, 100)),
            lift(900),
        ]);
        let mut expected = tapped(Tap); // one click
        expected.push((Tap, true)); // then the press of the second, held
        expected.push((Tap, false));
        assert_eq!(keys(&outs), expected);
        assert_eq!(keys(&outs[5..6]), vec![(Tap, true), (Tap, false), (Tap, true)]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 50), (Axis::Y, 0)]]);
    }

    #[test]
    fn a_touch_long_after_a_tap_only_moves_the_cursor() {
        let (outs, _) = run(&[one(0, (100, 100)), lift(80), one(400, (100, 100)), one(410, (110, 100))]);
        // The tap clicks when the late touch shows the window has passed.
        assert_eq!(keys(&outs[2..3]), tapped(Tap));
        assert_eq!(axes(&outs), vec![[(Axis::X, 10), (Axis::Y, 0)]]);
    }

    #[test]
    fn two_fingers_soon_after_a_tap_click_then_scroll() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            lift(80),
            two(200, (100, 500), (300, 500)),
            two(210, (100, 520), (300, 522)),
            two(220, (100, 530), (300, 532)),
        ]);
        assert_eq!(keys(&outs[2..3]), tapped(Tap));
        assert_eq!(axes(&outs), vec![[(Axis::H, 0), (Axis::V, 10)]]);
    }

    #[test]
    fn a_second_finger_on_the_touch_after_a_tap_clicks_and_scrolls() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            lift(80),
            one(200, (100, 110)), // near the tap: might drag
            two(210, (100, 110), (300, 110)),
        ]);
        assert_eq!(keys(&outs[3..4]), tapped(Tap));
    }

    #[test]
    fn a_short_touch_that_moves_is_a_cursor_move_not_a_tap() {
        // 35 is past the 30 a tap may move, though short of the 40 that decides gestures.
        let (outs, _) = run(&[one(0, (100, 100)), one(20, (135, 100)), lift(60), Step::Timeout(500)]);
        assert!(keys(&outs).is_empty());
        assert_eq!(axes(&outs), vec![[(Axis::X, 35), (Axis::Y, 0)]]);
    }

    #[test]
    fn a_touch_away_from_the_tap_clicks_and_moves_the_cursor() {
        let (outs, recognizer) = run(&[
            one(0, (100, 100)),
            lift(80),
            one(150, (300, 100)), // 200 away, past the 80 a tap drag allows
            one(160, (310, 100)),
        ]);
        assert_eq!(keys(&outs[2..3]), tapped(Tap));
        assert_eq!(axes(&outs), vec![[(Axis::X, 10), (Axis::Y, 0)]]);
        assert!(recognizer.pending_taps.is_none() && recognizer.drag.is_none());
    }

    #[test]
    fn a_touch_too_short_for_a_tap_is_a_graze() {
        let config = TouchGestureConfig {
            tap_min_ms: 20,
            ..TouchGestureConfig::default()
        };
        let (outs, _) = run_with(config, bound, &[one(0, (100, 100)), lift(10), Step::Timeout(500)]);
        assert!(keys(&outs).is_empty());
        let (outs, _) = run_with(config, bound, &[one(0, (100, 100)), lift(30), Step::Timeout(500)]);
        assert_eq!(keys(&outs), tapped(Tap));
    }

    #[test]
    fn a_graze_after_a_tap_is_no_second_tap() {
        let config = TouchGestureConfig {
            tap_min_ms: 20,
            ..TouchGestureConfig::default()
        };
        let (outs, _) = run_with(
            config,
            bound,
            &[
                one(0, (100, 100)),
                lift(80),
                one(150, (100, 100)),
                lift(155),
                Step::Timeout(400),
            ],
        );
        assert_eq!(keys(&outs), tapped(Tap));
    }

    #[test]
    fn without_tap_drag_a_tap_clicks_at_once() {
        let config = TouchGestureConfig {
            tap_drag_ms: 0,
            ..TouchGestureConfig::default()
        };
        let (outs, _) = run_with(
            config,
            bound,
            &[one(0, (100, 100)), lift(80), one(200, (100, 100)), one(210, (150, 100))],
        );
        assert_eq!(keys(&outs[1..2]), tapped(Tap));
        assert_eq!(keys(&outs), tapped(Tap));
    }

    fn locked(drag_lock_ms: u16) -> TouchGestureConfig {
        TouchGestureConfig {
            drag_lock_ms,
            ..TouchGestureConfig::default()
        }
    }

    #[test]
    fn drag_lock_keeps_a_drag_going_while_the_finger_is_put_back() {
        let (outs, _) = run_with(
            locked(300),
            bound,
            &[
                one(0, (100, 100)),
                lift(80),
                one(150, (100, 100)),
                one(160, (200, 100)), // dragging
                lift(400),            // lifted: still held
                Step::Timeout(500),
                one(600, (100, 100)), // back down, elsewhere: no jump
                one(610, (130, 100)),
                lift(900),
                Step::Timeout(1199),
                Step::Timeout(1200), // nobody came back: released
            ],
        );
        assert_eq!(keys(&outs), tapped(Tap));
        assert_eq!(keys(&outs[10..11]), vec![(Tap, false)]);
        assert_eq!(
            axes(&outs),
            vec![[(Axis::X, 100), (Axis::Y, 0)], [(Axis::X, 30), (Axis::Y, 0)]]
        );
    }

    #[test]
    fn a_quick_tap_ends_a_locked_drag_at_once() {
        let (outs, recognizer) = run_with(
            locked(300),
            bound,
            &[
                one(0, (100, 100)),
                lift(80),
                one(150, (100, 100)),
                one(160, (200, 100)),
                lift(400),
                one(500, (100, 100)),
                lift(550), // a tap: drop
            ],
        );
        assert_eq!(keys(&outs), tapped(Tap));
        assert_eq!(keys(&outs[6..7]), vec![(Tap, false)]);
        assert!(recognizer.drag.is_none());
    }

    #[test]
    fn without_drag_lock_lifting_ends_the_drag() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            lift(80),
            one(150, (100, 100)),
            one(160, (200, 100)),
            lift(400),
        ]);
        assert_eq!(keys(&outs[4..5]), vec![(Tap, false)]);
    }

    fn inertial(scroll_inertia_ms: u16) -> TouchGestureConfig {
        TouchGestureConfig {
            scroll_inertia_ms,
            ..TouchGestureConfig::default()
        }
    }

    /// A fast vertical two-finger scroll: 30 every 10 ms, 3000 a second, lifted at 50.
    fn fling() -> Vec<Step> {
        vec![
            two(0, (400, 500), (600, 500)),
            two(10, (400, 530), (600, 530)), // decided: scrolling
            two(20, (400, 560), (600, 560)),
            two(30, (400, 590), (600, 590)),
            two(40, (400, 620), (600, 620)),
            lift(50),
        ]
    }

    #[test]
    fn a_fast_scroll_goes_on_and_slows_down_after_lifting() {
        let mut steps = fling();
        steps.extend((1..=200).map(|i| Step::Timeout(50 + i * 16)));
        let (outs, recognizer) = run_with(inertial(300), bound, &steps);
        let coasting: Vec<i16> = outs[6..].iter().filter_map(|out| out.axes).map(|a| a[1].1).collect();
        assert!(coasting[0] >= 40, "first step {}", coasting[0]);
        // Slows down, give or take the fractions carried between steps.
        assert!(
            coasting.windows(2).all(|w| w[1] <= w[0] + 1),
            "slows down: {coasting:?}"
        );
        assert!(coasting.last().unwrap() * 10 < coasting[0]);
        assert!(coasting.iter().all(|v| *v >= 0));
        assert!(recognizer.inertia.is_none(), "stops by itself");
    }

    #[test]
    fn a_touch_stops_inertia_and_isnt_a_tap() {
        let mut steps = fling();
        steps.extend([Step::Timeout(66), one(70, (500, 500)), lift(120), Step::Timeout(1000)]);
        let (outs, recognizer) = run_with(inertial(300), bound, &steps);
        assert!(recognizer.inertia.is_none());
        assert!(keys(&outs).is_empty());
        assert!(outs[8..].iter().all(|out| out.axes.is_none()));
    }

    #[test]
    fn a_slow_or_paused_scroll_has_no_inertia() {
        let (_, recognizer) = run_with(
            inertial(300),
            bound,
            &[
                two(0, (400, 500), (600, 500)),
                two(10, (400, 541), (600, 541)),
                two(110, (400, 545), (600, 545)), // 40 a second
                lift(120),
            ],
        );
        assert!(recognizer.inertia.is_none());
        let mut steps = fling();
        steps.pop();
        steps.push(lift(300)); // rested before lifting
        let (_, recognizer) = run_with(inertial(300), bound, &steps);
        assert!(recognizer.inertia.is_none());
    }

    #[test]
    fn without_inertia_a_scroll_stops_on_lifting() {
        let (_, recognizer) = run(&fling());
        assert!(recognizer.inertia.is_none());
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
    fn a_drag_goes_on_while_any_finger_touches_and_a_moving_one_leads() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            lift(50),
            one(100, (100, 100)),
            Step::Timeout(280),                            // rested: a drag
            f(310, &[Some((100, 100)), Some((300, 300))]), // a second finger lands
            f(370, &[Some((100, 100)), Some((350, 300))]), // and moves alone: it leads
            f(380, &[None, Some((355, 300))]),             // the first lifts
            f(390, &[None, Some((360, 301))]),
            f(400, &[Some((120, 120)), Some((360, 301))]), // and lands again
            f(410, &[Some((120, 120)), None]),             // the second lifts
            f(420, &[Some((125, 120)), None]),
            lift(430),
        ]);
        assert_eq!(keys(&outs), tapped(Tap));
        assert_eq!(
            axes(&outs),
            vec![
                [(Axis::X, 50), (Axis::Y, 0)],
                [(Axis::X, 5), (Axis::Y, 1)],
                [(Axis::X, 5), (Axis::Y, 0)],
            ]
        );
    }

    #[test]
    fn when_the_lead_lifts_the_other_finger_takes_over_without_a_jump() {
        let (outs, _) = run(&[
            one(0, (100, 100)),
            lift(50),
            one(100, (100, 100)),
            Step::Timeout(280), // rested: a drag
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
            lift(50),
            one(100, (100, 100)),
            Step::Timeout(280),   // rested: a drag
            one(310, (600, 100)), // 500 in one frame
            one(320, (604, 100)),
        ]);
        assert_eq!(axes(&outs), vec![[(Axis::X, 4), (Axis::Y, 0)]]);
    }

    #[test]
    fn two_fingers_scroll_during_a_drag_with_its_button_held() {
        let (outs, recognizer) = run(&[
            one(0, (100, 100)),
            lift(50),
            one(100, (100, 100)),
            Step::Timeout(280), // rested: a drag
            two(310, (100, 100), (300, 300)),
            two(370, (100, 160), (300, 360)), // both move: a scroll
            two(380, (100, 170), (300, 370)),
            one(390, (100, 175)), // back to one finger: no jump
            one(400, (110, 175)),
        ]);
        assert_eq!(keys(&outs), vec![(Tap, true)]);
        assert_eq!(
            axes(&outs),
            vec![[(Axis::H, 0), (Axis::V, 10)], [(Axis::X, 10), (Axis::Y, 0)]]
        );
        assert!(recognizer.drag.is_some());
    }

    #[test]
    fn a_locked_drag_scrolls_with_two_fingers_and_goes_on_with_one() {
        let (outs, recognizer) = run_with(
            locked(300),
            bound,
            &[
                one(0, (100, 100)),
                lift(80),
                one(150, (100, 100)),
                one(160, (200, 100)), // dragging
                lift(400),            // held by drag lock
                two(500, (400, 500), (600, 500)),
                two(510, (400, 530), (600, 530)),
                two(520, (400, 540), (600, 540)), // scrolling, button still down
                lift(530),                        // held again
                one(600, (300, 300)),             // a finger: the cursor again
                one(610, (320, 300)),
            ],
        );
        assert_eq!(keys(&outs), vec![(Tap, true)]);
        assert_eq!(
            axes(&outs),
            vec![
                [(Axis::X, 100), (Axis::Y, 0)],
                [(Axis::H, 0), (Axis::V, 10)],
                [(Axis::X, 20), (Axis::Y, 0)],
            ]
        );
        assert!(recognizer.drag.is_some_and(|d| d.lifted_ms.is_none()));
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
        let (outs, recognizer) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (400, 530), (600, 530)), // decided: a scroll
            two(20, (400, 540), (600, 540)), // scrolls 10
            two(30, (400, 600), (600, 540)), // only one moves: no scroll, no cursor
            two(40, (450, 650), (600, 540)),
            one(50, (460, 660)), // the other lifts: still no cursor
            one(60, (480, 680)),
        ]);
        assert_eq!(axes(&outs), vec![[(Axis::H, 0), (Axis::V, 10)]]);
        // Paused until a finger is back or both lift.
        assert!(matches!(recognizer.two_finger, TwoFinger::Scrolling { .. }));
    }

    #[test]
    fn a_scroll_pauses_while_a_finger_is_lifted_and_goes_on_when_it_is_back() {
        let (outs, recognizer) = run(&[
            two(0, (400, 500), (600, 500)),
            two(10, (400, 530), (600, 530)), // decided: a scroll
            two(20, (400, 540), (600, 540)), // scrolls 10
            one(30, (400, 550)),             // one lifts: paused, no cursor
            one(40, (400, 560)),
            two(200, (400, 560), (700, 400)), // back, elsewhere: no jump
            two(210, (400, 570), (700, 410)), // scrolls on at once
        ]);
        assert_eq!(
            axes(&outs),
            vec![[(Axis::H, 0), (Axis::V, 10)], [(Axis::H, 0), (Axis::V, 10)]]
        );
        assert!(matches!(recognizer.two_finger, TwoFinger::Scrolling { .. }));
    }

    #[test]
    fn a_scroll_can_start_with_one_finger_and_the_second_joining() {
        let (outs, recognizer) = run(&[
            one(0, (300, 600)),
            one(10, (300, 580)),             // moving up alone: the cursor
            two(20, (300, 560), (500, 600)), // a second finger lands, still at first
            two(30, (300, 540), (500, 600)),
            two(40, (300, 520), (500, 600)), // only the first has moved: the cursor
            two(60, (300, 500), (500, 570)),
            two(80, (300, 480), (500, 540)), // the second follows: a vertical scroll
            two(90, (300, 470), (500, 530)),
        ]);
        assert!(matches!(
            recognizer.two_finger,
            TwoFinger::Scrolling {
                axis: ScrollAxis::Vertical,
                ..
            }
        ));
        assert_eq!(axes(&outs).last(), Some(&[(Axis::H, 0), (Axis::V, -10)]));
    }

    #[test]
    fn a_resting_finger_moving_late_doesnt_turn_a_cursor_move_into_a_scroll() {
        let (_, recognizer) = run(&[
            two(0, (300, 600), (500, 600)),
            two(10, (300, 570), (500, 600)),
            two(20, (300, 540), (500, 600)),  // the first moves the cursor
            two(400, (300, 500), (500, 560)), // the second moves too, but much later
            two(410, (300, 490), (500, 550)),
        ]);
        assert!(matches!(recognizer.two_finger, TwoFinger::Pointing { .. }));
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

    /// Runs frames of up to two fingers from a recording on a TPS43 (1792 by 2048).
    fn replay(frames: &[(u64, Option<Point>, Option<Point>)]) -> Vec<Output> {
        let mut recognizer = Recognizer::new(TouchGestureConfig::default());
        frames
            .iter()
            .map(|&(at_ms, a, b)| {
                let frame = Frame {
                    count: [a, b].iter().flatten().count() as u8,
                    slots: [a, b, None, None, None],
                    at_ms,
                };
                recognizer.frame(&frame, (1792, 2048), &bound)
            })
            .collect()
    }

    #[test]
    fn the_point_gliding_between_two_fingers_as_one_lands_or_lifts_is_no_cursor_motion() {
        // From a recording on a TPS43 (2048 tall): one finger rests, a second one
        // lifts, lands again and lifts. Each time the touchpad's one point glided
        // about 150 a frame toward the other finger and back, and the cursor with it;
        // the last time without the finger count changing on the way back.
        let frames = [
            (289791, Some((1155, 694)), None),
            (289828, Some((1155, 695)), None),
            (289851, Some((1155, 695)), Some((530, 1167))),
            (289858, Some((1018, 800)), None),
            (289873, Some((880, 905)), None),
            (289881, Some((879, 905)), None),
            (289896, Some((877, 906)), None),
            (289911, Some((1006, 813)), Some((529, 1170))),
            (289926, Some((1136, 720)), Some((529, 1170))),
            (289934, Some((1136, 720)), Some((528, 1172))),
            (289956, Some((1137, 720)), Some((527, 1174))),
            (289994, Some((1138, 719)), Some((524, 1179))),
            (291899, Some((1164, 689)), Some((563, 1265))),
            (291913, Some((1040, 776)), None),
            (291928, Some((922, 859)), None),
            (291936, Some((1036, 763)), None),
            (291943, Some((1144, 672)), None),
            (291958, Some((1144, 671)), None),
        ];
        let steps: Vec<_> = replay(&frames)
            .iter()
            .filter_map(|out| match out.axes {
                Some([(Axis::X, x), (Axis::Y, y)]) => Some((x, y)),
                _ => None,
            })
            .collect();
        assert!(steps.iter().all(|&(x, y)| x.abs() + y.abs() < 40), "{steps:?}");
    }

    #[test]
    fn a_fast_move_long_after_fingers_change_is_cursor_motion() {
        let (outs, _) = run(&[one(0, (100, 100)), one(100, (200, 100)), one(110, (300, 100))]);
        assert_eq!(
            axes(&outs),
            vec![[(Axis::X, 100), (Axis::Y, 0)], [(Axis::X, 100), (Axis::Y, 0)]]
        );
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
            Step::Timeout(330),
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
