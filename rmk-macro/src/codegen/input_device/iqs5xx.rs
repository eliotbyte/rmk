use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use rmk_config::resolved::hardware::{
    BoardConfig, ChipModel, ChipSeries, InputDeviceConfig, Iqs5xxConfig,
};

use super::Initializer;

/// Where a trackpad is wired: the central (or a unibody board), or peripheral `n`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Side {
    Central,
    Peripheral(usize),
}

/// The trackpads wired to `side`.
pub(crate) fn trackpads(board: &BoardConfig, side: Side) -> Vec<Iqs5xxConfig> {
    let input_device = match (board, side) {
        (BoardConfig::UniBody(unibody), Side::Central) => Some(unibody.input_device.clone()),
        (BoardConfig::Split(split), Side::Central) => split.central.input_device.clone(),
        (BoardConfig::Split(split), Side::Peripheral(id)) => split
            .peripheral
            .get(id)
            .and_then(|p| p.input_device.clone()),
        (BoardConfig::UniBody(_), Side::Peripheral(_)) => None,
    };
    input_device
        .unwrap_or(InputDeviceConfig::default())
        .iqs5xx
        .unwrap_or_default()
}

/// The IC's gestures, in the order their virtual keys are numbered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Gesture {
    SingleTap,
    PressAndHold,
    SwipeXNeg,
    SwipeXPos,
    SwipeYNeg,
    SwipeYPos,
    TwoFingerTap,
    ZoomIn,
    ZoomOut,
}

const GESTURES: [Gesture; 9] = [
    Gesture::SingleTap,
    Gesture::PressAndHold,
    Gesture::SwipeXNeg,
    Gesture::SwipeXPos,
    Gesture::SwipeYNeg,
    Gesture::SwipeYPos,
    Gesture::TwoFingerTap,
    Gesture::ZoomIn,
    Gesture::ZoomOut,
];

/// The action configured for `gesture`. Swipes are configured by cursor direction,
/// so a sensor-axis swipe goes through the same invert/swap as the cursor.
fn gesture_action(config: &Iqs5xxConfig, gesture: Gesture) -> Option<&String> {
    let g = &config.gestures;
    let sensor = match gesture {
        Gesture::SingleTap => return g.single_tap.as_ref(),
        Gesture::PressAndHold => return g.press_and_hold.as_ref(),
        Gesture::TwoFingerTap => return g.two_finger_tap.as_ref(),
        Gesture::ZoomIn => return g.zoom_in.as_ref(),
        Gesture::ZoomOut => return g.zoom_out.as_ref(),
        Gesture::SwipeXNeg => (-1, 0),
        Gesture::SwipeXPos => (1, 0),
        Gesture::SwipeYNeg => (0, -1),
        Gesture::SwipeYPos => (0, 1),
    };
    let (mut x, mut y) = sensor;
    if config.proc_invert_x {
        x = -x;
    }
    if config.proc_invert_y {
        y = -y;
    }
    if config.proc_swap_xy {
        (x, y) = (y, x);
    }
    // Cursor +Y points down.
    match (x, y) {
        (1, _) => g.swipe_right.as_ref(),
        (-1, _) => g.swipe_left.as_ref(),
        (_, 1) => g.swipe_down.as_ref(),
        _ => g.swipe_up.as_ref(),
    }
}

/// One gesture with an action: on which trackpad, and the action string.
pub(crate) struct GestureKey {
    side: Side,
    device: usize,
    gesture: Gesture,
    pub(crate) action: String,
}

/// Every configured gesture of every trackpad: the central's first, then each
/// peripheral's in order. A gesture's position is its `KeyboardEventPos::Virtual`
/// index, so the half that reads the trackpad and the central that runs the
/// actions derive the same numbering from keyboard.toml.
pub(crate) fn gesture_keys(board: &BoardConfig) -> Vec<GestureKey> {
    let peripherals = match board {
        BoardConfig::Split(split) => split.peripheral.len(),
        BoardConfig::UniBody(_) => 0,
    };
    let sides = core::iter::once(Side::Central).chain((0..peripherals).map(Side::Peripheral));
    let mut keys = Vec::new();
    for side in sides {
        for (device, config) in trackpads(board, side).iter().enumerate() {
            for gesture in GESTURES {
                if let Some(action) = gesture_action(config, gesture) {
                    keys.push(GestureKey {
                        side,
                        device,
                        gesture,
                        action: action.clone(),
                    });
                }
            }
        }
    }
    if keys.len() > usize::from(u8::MAX) + 1 {
        panic!("\n\u{274c} keyboard.toml: at most 256 trackpad gestures can have an action");
    }
    keys
}

/// The `Iqs5xxGestures` of trackpad `device` on `side`.
fn expand_gestures(
    board: &BoardConfig,
    side: Side,
    device: usize,
    config: &Iqs5xxConfig,
) -> TokenStream {
    let keys = gesture_keys(board);
    let key = |gesture| match keys
        .iter()
        .position(|k| k.side == side && k.device == device && k.gesture == gesture)
    {
        Some(idx) => {
            let idx = idx as u8;
            quote! { Some(#idx) }
        }
        None => quote! { None },
    };
    let [
        single_tap,
        press_and_hold,
        swipe_x_neg,
        swipe_x_pos,
        swipe_y_neg,
        swipe_y_pos,
        two_finger_tap,
        zoom_in,
        zoom_out,
    ] = GESTURES.map(key);
    let scroll = config.gestures.scroll;
    quote! {
        ::rmk::input_device::iqs5xx::Iqs5xxGestures {
            single_tap: #single_tap,
            press_and_hold: #press_and_hold,
            swipe_x_neg: #swipe_x_neg,
            swipe_x_pos: #swipe_x_pos,
            swipe_y_neg: #swipe_y_neg,
            swipe_y_pos: #swipe_y_pos,
            two_finger_tap: #two_finger_tap,
            scroll: #scroll,
            zoom_in: #zoom_in,
            zoom_out: #zoom_out,
        }
    }
}

/// Expand IQS5xx device configuration for the trackpads wired to `side`.
/// Returns (device initializers, processor initializers).
pub(crate) fn expand_iqs5xx_device(
    board: &BoardConfig,
    side: Side,
    chip: &ChipModel,
) -> (Vec<Initializer>, Vec<Initializer>) {
    let iqs5xx_config = trackpads(board, side);
    if iqs5xx_config.is_empty() {
        return (Vec::new(), Vec::new());
    }

    match chip.series {
        ChipSeries::Nrf52 | ChipSeries::Rp2040 => {}
        _ => {
            panic!("IQS5xx is only supported on nRF52 and RP2040 chips");
        }
    }

    let mut device_initializers = vec![];
    let mut processor_initializers = vec![];

    for (idx, sensor) in iqs5xx_config.iter().enumerate() {
        let sensor_id = sensor.id.unwrap_or(0);
        let sensor_name = if sensor.name.is_empty() {
            format!("iqs5xx_{}_id{}", idx, sensor_id)
        } else {
            format!("{}_id{}", sensor.name.clone(), sensor_id)
        };

        let device_ident = format_ident!("{}_device", sensor_name);
        let i2c_ident = format_ident!("{}_i2c", sensor_name);
        let i2c_buf_ident = format_ident!("{}_i2c_buf", sensor_name);
        let i2c_buf_cell_ident = format_ident!("{}_I2C_BUF", sensor_name.to_uppercase());
        let rdy_ident = format_ident!("{}_rdy", sensor_name);
        let processor_ident = format_ident!("{}_processor", sensor_name);
        let processor_ident_config = format_ident!("{}_config", processor_ident);

        let instance_ident = format_ident!("{}", sensor.i2c.instance.to_uppercase());
        let sda_ident = format_ident!("{}", sensor.i2c.sda);
        let scl_ident = format_ident!("{}", sensor.i2c.scl);

        let proc_invert_x = sensor.proc_invert_x;
        let proc_invert_y = sensor.proc_invert_y;
        let proc_swap_xy = sensor.proc_swap_xy;
        let gestures = expand_gestures(board, side, idx, sensor);
        let scroll_divisor = sensor.gestures.scroll_divisor.unwrap_or(8);
        let natural_scroll = sensor.gestures.natural_scroll;

        let rdy_init = match (&sensor.rdy, &chip.series) {
            (Some(rdy_pin), ChipSeries::Nrf52) => {
                let rdy_pin_ident = format_ident!("{}", rdy_pin);
                quote! {
                    let #rdy_ident = Some(::embassy_nrf::gpio::Input::new(
                        p.#rdy_pin_ident,
                        ::embassy_nrf::gpio::Pull::None,
                    ));
                }
            }
            (Some(rdy_pin), ChipSeries::Rp2040) => {
                let rdy_pin_ident = format_ident!("{}", rdy_pin);
                quote! {
                    let #rdy_ident = Some(::embassy_rp::gpio::Input::new(
                        p.#rdy_pin_ident,
                        ::embassy_rp::gpio::Pull::None,
                    ));
                }
            }
            (None, ChipSeries::Nrf52) => quote! {
                let #rdy_ident: Option<::embassy_nrf::gpio::Input<'static>> = None;
            },
            (None, ChipSeries::Rp2040) => quote! {
                let #rdy_ident: Option<::embassy_rp::gpio::Input<'static>> = None;
            },
            _ => unreachable!(),
        };

        let device_init = match chip.series {
            ChipSeries::Nrf52 => quote! {
                #rdy_init
                static #i2c_buf_cell_ident: ::static_cell::StaticCell<[u8; 16]> = ::static_cell::StaticCell::new();
                let #i2c_buf_ident = #i2c_buf_cell_ident.init([0u8; 16]);
                let #i2c_ident = ::embassy_nrf::twim::Twim::new(
                    p.#instance_ident,
                    Irqs,
                    p.#sda_ident,
                    p.#scl_ident,
                    ::embassy_nrf::twim::Config::default(),
                    #i2c_buf_ident,
                );
                let mut #device_ident = ::rmk::input_device::iqs5xx::Iqs5xx::new(
                    #sensor_id,
                    #i2c_ident,
                    #rdy_ident,
                )
                .with_gestures(#gestures);
            },
            ChipSeries::Rp2040 => quote! {
                #rdy_init
                let #i2c_ident = ::embassy_rp::i2c::I2c::new_async(
                    p.#instance_ident,
                    p.#scl_ident,
                    p.#sda_ident,
                    Irqs,
                    ::embassy_rp::i2c::Config::default(),
                );
                let mut #device_ident = ::rmk::input_device::iqs5xx::Iqs5xx::new(
                    #sensor_id,
                    #i2c_ident,
                    #rdy_ident,
                )
                .with_gestures(#gestures);
            },
            _ => unreachable!(),
        };

        device_initializers.push(Initializer {
            initializer: device_init,
            var_name: device_ident,
        });

        let processor_init = quote! {
            let #processor_ident_config = ::rmk::input_device::pointing::PointingProcessorConfig {
                device_id: #sensor_id,
                invert_x: #proc_invert_x,
                invert_y: #proc_invert_y,
                swap_xy: #proc_swap_xy,
                device_scroll: ::rmk::input_device::pointing::ScrollConfig {
                    multiplier_x: 1,
                    divisor_x: #scroll_divisor,
                    multiplier_y: 1,
                    divisor_y: #scroll_divisor,
                    invert_x: #natural_scroll,
                    invert_y: #natural_scroll,
                },
            };
            let mut #processor_ident = ::rmk::input_device::pointing::PointingProcessor::new(
                &keymap,
                #processor_ident_config,
            );
        };

        processor_initializers.push(Initializer {
            initializer: processor_init,
            var_name: processor_ident,
        });
    }

    (device_initializers, processor_initializers)
}

/// Generate `bind_interrupts!` entries for the I²C peripherals used by IQS5xx
/// devices on `chip`. Returns an empty token stream if there are no devices.
pub(crate) fn expand_iqs5xx_interrupts(
    chip_series: &ChipSeries,
    iqs5xx_config: &[Iqs5xxConfig],
) -> TokenStream {
    if iqs5xx_config.is_empty() {
        return quote! {};
    }
    let entries = iqs5xx_config.iter().map(|sensor| {
        let instance = format_ident!("{}", sensor.i2c.instance.to_uppercase());
        match chip_series {
            ChipSeries::Nrf52 => quote! {
                #instance => ::embassy_nrf::twim::InterruptHandler<::embassy_nrf::peripherals::#instance>;
            },
            ChipSeries::Rp2040 => {
                let irq = format_ident!("{}_IRQ", sensor.i2c.instance.to_uppercase());
                quote! {
                    #irq => ::embassy_rp::i2c::InterruptHandler<::embassy_rp::peripherals::#instance>;
                }
            }
            _ => quote! {},
        }
    });
    quote! { #(#entries)* }
}

/// `behavior_config.virtual_keys`: the action of every trackpad gesture, by index.
pub(crate) fn expand_virtual_keys(board: &BoardConfig) -> TokenStream {
    let keys = gesture_keys(board);
    if keys.is_empty() {
        return quote! {};
    }
    let actions = keys
        .into_iter()
        .map(|key| super::super::action_parser::parse_key(key.action, &None));
    quote! {
        behavior_config.virtual_keys = {
            const VIRTUAL_KEYS: &[::rmk::types::action::KeyAction] = &[#(#actions),*];
            VIRTUAL_KEYS
        };
    }
}

#[cfg(test)]
mod tests {
    use rmk_config::KeyboardTomlConfig;

    use super::*;

    /// A split board with a trackpad on each half; `central` and `peripheral` go into
    /// their `[...iqs5xx]` tables, gestures included.
    fn split_board(name: &str, central: &str, peripheral: &str) -> BoardConfig {
        let path = std::env::temp_dir().join(format!(
            "rmk-macro-iqs5xx-{name}-{}.toml",
            std::process::id()
        ));
        let toml = format!(
            r#"
[keyboard]
name = "gestures"
vendor_id = 0x4c4b
product_id = 0x4643
chip = "nrf52840"

[layout]
rows = 1
cols = 2

[split]
connection = "ble"

[split.central]
rows = 1
cols = 1
row_offset = 0
col_offset = 0

[split.central.matrix]
matrix_type = "normal"
row_pins = ["P0_02"]
col_pins = ["P0_03"]

[[split.central.input_device.iqs5xx]]
name = "left"
i2c.instance = "TWISPI0"
i2c.sda = "P0_17"
i2c.scl = "P0_20"
{central}

[[split.peripheral]]
rows = 1
cols = 1
row_offset = 0
col_offset = 1

[split.peripheral.matrix]
matrix_type = "normal"
row_pins = ["P0_02"]
col_pins = ["P0_03"]

[[split.peripheral.input_device.iqs5xx]]
name = "right"
i2c.instance = "TWISPI0"
i2c.sda = "P0_17"
i2c.scl = "P0_20"
{peripheral}
"#
        );
        std::fs::write(&path, toml).unwrap();
        let hardware = KeyboardTomlConfig::new_from_toml_path(&path).hardware();
        std::fs::remove_file(&path).ok();
        hardware.unwrap_or_else(|e| panic!("{e}")).board
    }

    fn summary(board: &BoardConfig) -> Vec<(Side, usize, Gesture, String)> {
        gesture_keys(board)
            .into_iter()
            .map(|k| (k.side, k.device, k.gesture, k.action))
            .collect()
    }

    #[test]
    fn gestures_are_numbered_central_first_in_gesture_order() {
        let board = split_board(
            "order",
            "gestures.zoom_out = \"A\"\ngestures.single_tap = \"MouseBtn1\"",
            "gestures.two_finger_tap = \"MouseBtn2\"\ngestures.press_and_hold = \"B\"",
        );
        assert_eq!(
            summary(&board),
            vec![
                (
                    Side::Central,
                    0,
                    Gesture::SingleTap,
                    "MouseBtn1".to_string()
                ),
                (Side::Central, 0, Gesture::ZoomOut, "A".to_string()),
                (
                    Side::Peripheral(0),
                    0,
                    Gesture::PressAndHold,
                    "B".to_string()
                ),
                (
                    Side::Peripheral(0),
                    0,
                    Gesture::TwoFingerTap,
                    "MouseBtn2".to_string()
                ),
            ]
        );
    }

    #[test]
    fn the_peripheral_driver_gets_the_centrals_numbering() {
        let board = split_board(
            "driver",
            "gestures.single_tap = \"MouseBtn1\"",
            "gestures.two_finger_tap = \"MouseBtn2\"\ngestures.scroll = true",
        );
        let config = &trackpads(&board, Side::Peripheral(0))[0];
        let tokens = expand_gestures(&board, Side::Peripheral(0), 0, config).to_string();
        assert!(tokens.contains("two_finger_tap : Some (1u8)"), "{tokens}");
        assert!(tokens.contains("single_tap : None"), "{tokens}");
        assert!(tokens.contains("scroll : true"), "{tokens}");
    }

    #[test]
    fn swipes_follow_the_cursor_through_invert_and_swap() {
        // With X inverted, moving along sensor +X moves the cursor left.
        let board = split_board(
            "swipe",
            "",
            "proc_invert_x = true\ngestures.swipe_left = \"L\"\ngestures.swipe_down = \"D\"",
        );
        assert_eq!(
            summary(&board),
            vec![
                (Side::Peripheral(0), 0, Gesture::SwipeXPos, "L".to_string()),
                (Side::Peripheral(0), 0, Gesture::SwipeYPos, "D".to_string()),
            ]
        );
        // Swapped, sensor -Y is the cursor's -X: left.
        let board = split_board(
            "swap",
            "",
            "proc_swap_xy = true\ngestures.swipe_left = \"L\"",
        );
        assert_eq!(
            summary(&board),
            vec![(Side::Peripheral(0), 0, Gesture::SwipeYNeg, "L".to_string())]
        );
    }

    #[test]
    fn no_gestures_leave_the_virtual_key_table_alone() {
        let board = split_board("none", "", "");
        assert!(gesture_keys(&board).is_empty());
        assert!(expand_virtual_keys(&board).is_empty());
    }
}
