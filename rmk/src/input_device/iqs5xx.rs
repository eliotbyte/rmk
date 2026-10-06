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
//! This driver requests only the 10-byte motion block at 0x000C (previous
//! cycle time, gesture events, system info, number of fingers, relative XY).
//! Relative XY is published as cursor movement. The IC's own gestures (§6) are
//! enabled per [`Iqs5xxGestures`]: taps, press-and-hold, swipes and zoom press
//! virtual keys (`KeyboardEventPos::Virtual`), whose actions live in
//! `BehaviorConfig::virtual_keys`; two-finger scroll is published on the H/V
//! axes. Absolute finger data and raw channel data are not read.
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

    state: GestureState,
}

/// Gesture state carried from one cycle to the next.
#[derive(Debug, Default)]
struct GestureState {
    /// Whether the press-and-hold virtual key is down.
    holding: bool,
    /// The two-finger gesture this touch settled on. The IC switches between scroll
    /// and zoom mid-touch (§6.7), so fingers drifting apart while scrolling would
    /// zoom; instead the first one keeps the touch until fewer than two fingers remain.
    two_finger: Option<TwoFinger>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TwoFinger {
    Scroll,
    Zoom,
}

/// The IQS5xx's built-in gestures (§6) to enable, each with the index of the
/// virtual key (`KeyboardEventPos::Virtual`) it presses. `None` leaves the
/// gesture disabled on the IC.
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
    /// Two-finger scroll, published on the H/V axes (§6.5).
    pub scroll: bool,
    /// Pinch apart / together: tapped once per zoom step (§6.6).
    pub zoom_in: Option<u8>,
    pub zoom_out: Option<u8>,
    /// How much the distance between the fingers must change before the first zoom
    /// step, in percent of the trackpad's longer side (§6.6). `None` keeps the IC's
    /// own value, which can be small at this driver's full resolution.
    pub zoom_start_percent: Option<u8>,
    /// The same for every further zoom step.
    pub zoom_step_percent: Option<u8>,
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

    /// Multi-finger Gestures register value, §8.10.22.
    fn multi_finger_enable(&self) -> u8 {
        u8::from(self.two_finger_tap.is_some())
            | u8::from(self.scroll) << 1
            | u8::from(self.zoom_in.is_some() || self.zoom_out.is_some()) << 2
    }
}

/// `percent` of `span` pixels, saturating at the register's range.
fn percent_of(span: u16, percent: u8) -> u16 {
    u16::try_from(u32::from(span) * u32::from(percent) / 100).unwrap_or(u16::MAX)
}

/// The part of one cycle's motion block the gestures depend on.
struct Motion {
    gesture_events_0: u8,
    gesture_events_1: u8,
    fingers: u8,
    dx: i16,
    dy: i16,
}

/// What one cycle turns into: virtual key presses `(index, pressed)` in order, and
/// the axes to publish, X/Y for the cursor or H/V for scrolling.
#[derive(Debug, Default, PartialEq, Eq)]
struct CycleOutput {
    keys: heapless::Vec<(u8, bool), 4>,
    axes: Option<[(Axis, i16); 2]>,
}

/// Decode a cycle's gesture bits (§8.10.1-§8.10.2) against the enabled gestures,
/// updating the state carried between cycles.
fn decode_cycle(gestures: &Iqs5xxGestures, motion: &Motion, state: &mut GestureState) -> CycleOutput {
    let mut out = CycleOutput::default();
    let tap = |key: Option<u8>, out: &mut CycleOutput| {
        if let Some(key) = key {
            let _ = out.keys.push((key, true));
            let _ = out.keys.push((key, false));
        }
    };
    let g0 = motion.gesture_events_0;
    let g1 = motion.gesture_events_1;

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
    if g1 & 0b1 != 0 {
        tap(gestures.two_finger_tap, &mut out);
    }

    if motion.fingers < 2 {
        state.two_finger = None;
    }
    let two_finger = if g1 & 0b100 != 0 {
        Some(TwoFinger::Zoom)
    } else if g1 & 0b10 != 0 {
        Some(TwoFinger::Scroll)
    } else {
        None
    };
    if let Some(gesture) = two_finger {
        match state.two_finger {
            // The other gesture took over mid-touch: drop it, motion included.
            Some(locked) if locked != gesture => return out,
            _ => state.two_finger = Some(gesture),
        }
    }

    // During scroll and zoom the relative registers carry the gesture, not the cursor.
    if g1 & 0b100 != 0 {
        // The zoom step is in relative X, positive when the fingers move apart (§6.6).
        if motion.dx > 0 {
            tap(gestures.zoom_in, &mut out);
        } else if motion.dx < 0 {
            tap(gestures.zoom_out, &mut out);
        }
    } else if motion.dx != 0 || motion.dy != 0 {
        let (h, v) = if g1 & 0b10 != 0 {
            (Axis::H, Axis::V)
        } else {
            (Axis::X, Axis::Y)
        };
        out.axes = Some([(h, motion.dx), (v, motion.dy)]);
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

        // Zoom initial / consecutive distance, 2 bytes each at 0x06CC/0x06CE (§6.6), in
        // pixels of the resolution set above.
        let span = x_resolution.max(y_resolution);
        for (tag, addr, percent) in [
            ("zoom_initial_distance", 0xCC, self.gestures.zoom_start_percent),
            ("zoom_consecutive_distance", 0xCE, self.gestures.zoom_step_percent),
        ] {
            if let Some(percent) = percent {
                let [high, low] = percent_of(span, percent).to_be_bytes();
                i2c_tx(&mut self.i2c, tag, &mut [Operation::Write(&[0x06, addr, high, low])]).await?;
            }
        }

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
        // (§8.10.3-§8.10.4), number of fingers (§5.2.1), relative XY (§5.2.2).
        let mut data = [0u8; 10];
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
                    let out = decode_cycle(&self.gestures, &motion, &mut self.state);
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
        zoom_start_percent: None,
        zoom_step_percent: None,
    };

    /// A cycle with one finger down, or two when it reports a scroll or zoom.
    fn motion(gesture_events_0: u8, gesture_events_1: u8, dx: i16, dy: i16) -> Motion {
        let fingers = if gesture_events_1 & 0b110 != 0 { 2 } else { 1 };
        Motion {
            gesture_events_0,
            gesture_events_1,
            fingers,
            dx,
            dy,
        }
    }

    fn lifted() -> Motion {
        Motion {
            gesture_events_0: 0,
            gesture_events_1: 0,
            fingers: 0,
            dx: 0,
            dy: 0,
        }
    }

    fn keys(out: &CycleOutput) -> &[(u8, bool)] {
        &out.keys
    }

    #[test]
    fn enable_registers_follow_the_datasheet_bit_order() {
        assert_eq!(ALL.single_finger_enable(), 0b11_1111);
        assert_eq!(ALL.multi_finger_enable(), 0b111);
        let swipe_y_neg_only = Iqs5xxGestures {
            swipe_y_neg: Some(0),
            ..Default::default()
        };
        assert_eq!(swipe_y_neg_only.single_finger_enable(), 0b10_0000);
        let zoom_out_only = Iqs5xxGestures {
            zoom_out: Some(0),
            ..Default::default()
        };
        assert_eq!(zoom_out_only.multi_finger_enable(), 0b100);
        assert_eq!(Iqs5xxGestures::default().single_finger_enable(), 0);
        assert_eq!(Iqs5xxGestures::default().multi_finger_enable(), 0);
    }

    #[test]
    fn zoom_distances_are_a_share_of_the_trackpad() {
        assert_eq!(percent_of(2304, 15), 345);
        assert_eq!(percent_of(2304, 0), 0);
        assert_eq!(percent_of(2304, 100), 2304);
        assert_eq!(percent_of(u16::MAX, 255), u16::MAX);
    }

    #[test]
    fn plain_motion_moves_the_cursor() {
        let out = decode_cycle(&ALL, &motion(0, 0, 3, -4), &mut GestureState::default());
        assert!(keys(&out).is_empty());
        assert_eq!(out.axes, Some([(Axis::X, 3), (Axis::Y, -4)]));
    }

    #[test]
    fn taps_press_and_release_their_keys() {
        let out = decode_cycle(&ALL, &motion(0b1, 0, 0, 0), &mut GestureState::default());
        assert_eq!(keys(&out), &[(0, true), (0, false)]);
        assert_eq!(out.axes, None);
        let out = decode_cycle(&ALL, &motion(0, 0b1, 0, 0), &mut GestureState::default());
        assert_eq!(keys(&out), &[(6, true), (6, false)]);
    }

    #[test]
    fn swipes_map_to_their_bits() {
        for (bit, key) in [(2, 2), (3, 3), (4, 5), (5, 4)] {
            let out = decode_cycle(&ALL, &motion(1 << bit, 0, 0, 0), &mut GestureState::default());
            assert_eq!(keys(&out), &[(key, true), (key, false)], "bit {bit}");
        }
    }

    #[test]
    fn press_and_hold_stays_down_and_drags() {
        let mut state = GestureState::default();
        let out = decode_cycle(&ALL, &motion(0b10, 0, 0, 0), &mut state);
        assert_eq!(keys(&out), &[(1, true)]);
        assert!(state.holding);
        // Still held: no key change, the cursor moves.
        let out = decode_cycle(&ALL, &motion(0b10, 0, 5, 1), &mut state);
        assert!(keys(&out).is_empty());
        assert_eq!(out.axes, Some([(Axis::X, 5), (Axis::Y, 1)]));
        let out = decode_cycle(&ALL, &lifted(), &mut state);
        assert_eq!(keys(&out), &[(1, false)]);
        assert!(!state.holding);
    }

    #[test]
    fn two_finger_scroll_goes_to_the_scroll_axes() {
        let out = decode_cycle(&ALL, &motion(0, 0b10, 0, -7), &mut GestureState::default());
        assert!(keys(&out).is_empty());
        assert_eq!(out.axes, Some([(Axis::H, 0), (Axis::V, -7)]));
    }

    #[test]
    fn zoom_taps_in_or_out_by_sign_and_never_moves_the_cursor() {
        let out = decode_cycle(&ALL, &motion(0, 0b100, 12, 0), &mut GestureState::default());
        assert_eq!(keys(&out), &[(7, true), (7, false)]);
        assert_eq!(out.axes, None);
        let out = decode_cycle(&ALL, &motion(0, 0b100, -12, 0), &mut GestureState::default());
        assert_eq!(keys(&out), &[(8, true), (8, false)]);
        assert_eq!(out.axes, None);
    }

    #[test]
    fn a_scroll_is_not_taken_over_by_zoom_until_the_fingers_lift() {
        let mut state = GestureState::default();
        let out = decode_cycle(&ALL, &motion(0, 0b10, 0, -7), &mut state);
        assert_eq!(out.axes, Some([(Axis::H, 0), (Axis::V, -7)]));
        // The fingers drift apart: the IC reports a zoom, which is dropped.
        let out = decode_cycle(&ALL, &motion(0, 0b100, 12, 0), &mut state);
        assert!(keys(&out).is_empty());
        assert_eq!(out.axes, None);
        // Back to scrolling within the same touch.
        let out = decode_cycle(&ALL, &motion(0, 0b10, 0, -3), &mut state);
        assert_eq!(out.axes, Some([(Axis::H, 0), (Axis::V, -3)]));
        // A new touch can zoom.
        decode_cycle(&ALL, &lifted(), &mut state);
        let out = decode_cycle(&ALL, &motion(0, 0b100, 12, 0), &mut state);
        assert_eq!(keys(&out), &[(7, true), (7, false)]);
    }

    #[test]
    fn a_zoom_is_not_taken_over_by_scroll_until_the_fingers_lift() {
        let mut state = GestureState::default();
        decode_cycle(&ALL, &motion(0, 0b100, -12, 0), &mut state);
        let out = decode_cycle(&ALL, &motion(0, 0b10, 0, -7), &mut state);
        assert_eq!(out.axes, None);
        // One finger lifted ends the two-finger gesture.
        decode_cycle(&ALL, &motion(0, 0, 0, 0), &mut state);
        let out = decode_cycle(&ALL, &motion(0, 0b10, 0, -7), &mut state);
        assert_eq!(out.axes, Some([(Axis::H, 0), (Axis::V, -7)]));
    }

    #[test]
    fn gestures_without_a_key_are_ignored() {
        let none = Iqs5xxGestures::default();
        let mut state = GestureState::default();
        let out = decode_cycle(&none, &motion(0b11_1111, 0b001, 2, 2), &mut state);
        assert!(keys(&out).is_empty());
        assert!(!state.holding);
        assert_eq!(out.axes, Some([(Axis::X, 2), (Axis::Y, 2)]));
    }
}
