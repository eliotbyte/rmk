//! Initialize default keymap from config
use std::collections::HashMap;

use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use rmk_config::TouchActionsConfig;
use rmk_config::resolved::behavior::MorseProfile;
use rmk_config::resolved::{Behavior, Keymap};

use super::action_parser::parse_key;

/// Read the default keymap setting in `keyboard.toml` and add as a `get_default_keymap` function
/// Also add `get_default_encoder_map`, and `get_default_touch_map` for a board with touchpads
pub(crate) fn expand_default_keymap(keymap: &Keymap, behavior: &Behavior) -> TokenStream2 {
    let profiles: Option<HashMap<String, MorseProfile>> = behavior
        .morse
        .as_ref()
        .map(|m| m.profiles.clone())
        .filter(|p| !p.is_empty());

    let num_encoder = keymap.num_encoder;

    let mut layers = vec![];
    let mut encoder_map = vec![];

    for layer in &keymap.keymap {
        layers.push(expand_layer(layer.clone(), &profiles));
    }

    for encoder_layer in &keymap.encoder_map {
        encoder_map.push(expand_encoder_layer(
            encoder_layer.clone(),
            num_encoder,
            &profiles,
        ));
    }
    encoder_map.resize(
        keymap.keymap.len(),
        quote! { [::rmk::encoder!(::rmk::k!(No), ::rmk::k!(No)); NUM_ENCODER] },
    );

    let touch_map = (keymap.num_touchpad > 0).then(|| {
        let mut touch_map = keymap.touch_map.clone();
        touch_map.resize(keymap.keymap.len(), Vec::new());
        let touch_layers = touch_map
            .iter()
            .map(|layer| expand_touch_layer(layer, keymap.num_touchpad, &profiles));
        quote! {
            pub const fn get_default_touch_map() -> [[::rmk::types::action::TouchAction; NUM_TOUCHPAD]; NUM_LAYER] {
                [#(#touch_layers), *]
            }
        }
    });

    quote! {
        pub const fn get_default_keymap() -> [[[::rmk::types::action::KeyAction; COL]; ROW]; NUM_LAYER] {
            [#(#layers), *]
        }

        pub const fn get_default_encoder_map() -> [[::rmk::types::action::EncoderAction; NUM_ENCODER]; NUM_LAYER] {
            [#(#encoder_map), *]
        }

        #touch_map
    }
}

/// Expand a layer of the touch map: each touchpad's gesture actions. A gesture the
/// layer leaves out is transparent, so layers only list what they change.
pub(crate) fn expand_touch_layer(
    touchpads: &[TouchActionsConfig],
    num_touchpad: usize,
    profiles: &Option<HashMap<String, MorseProfile>>,
) -> TokenStream2 {
    let none = TouchActionsConfig::default();
    let touchpads = (0..num_touchpad).map(|i| {
        let actions = touchpads.get(i).unwrap_or(&none).actions();
        let with = actions.into_iter().filter_map(|(gesture, action)| {
            let gesture = format_ident!("{}", gesture);
            let action = parse_key(action.clone()?, profiles);
            Some(quote! { .with(::rmk::types::action::TouchGesture::#gesture, #action) })
        });
        quote! { ::rmk::types::action::TouchAction::transparent() #(#with)* }
    });
    quote! { [#(#touchpads), *] }
}

/// Expand a layer for keymap
pub(crate) fn expand_layer(
    layer: Vec<Vec<String>>,
    profiles: &Option<HashMap<String, MorseProfile>>,
) -> TokenStream2 {
    let mut rows = vec![];
    for row in layer {
        rows.push(expand_row(row, profiles));
    }
    quote! { [#(#rows), *] }
}

/// Expand a row for keymap
fn expand_row(row: Vec<String>, profiles: &Option<HashMap<String, MorseProfile>>) -> TokenStream2 {
    let mut keys = vec![];
    for key in row {
        keys.push(parse_key(key, profiles));
    }
    quote! { [#(#keys), *] }
}

/// Expand a layer for encoder map
pub(crate) fn expand_encoder_layer(
    encoder_layer: Vec<[String; 2]>,
    num_encoder: usize,
    profiles: &Option<HashMap<String, MorseProfile>>,
) -> TokenStream2 {
    let mut encoders = vec![];

    for encoder in encoder_layer {
        let cw_action = parse_key(encoder[0].clone(), profiles);
        let ccw_action = parse_key(encoder[1].clone(), profiles);
        encoders.push(quote! { ::rmk::encoder!(#cw_action, #ccw_action) });
    }

    // Make sure it configures correct number of encoders
    encoders.resize(
        num_encoder,
        quote! { ::rmk::encoder!(::rmk::k!(No), ::rmk::k!(No)) },
    );

    if encoders.is_empty() {
        // An empty `[]` literal has no element type for Rust to infer when
        // `num_encoder == 0`. Emit the typed `[expr; N]` form instead.
        quote! { [::rmk::encoder!(::rmk::k!(No), ::rmk::k!(No)); NUM_ENCODER] }
    } else {
        quote! { [#(#encoders), *] }
    }
}
