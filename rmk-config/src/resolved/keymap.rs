use crate::{KeyInfo, TouchActionsConfig};

/// Resolved keymap for keymap generation: layer count, per-layer actions,
/// encoder map, plus the matrix-derived per-key info and grid dimensions.
pub struct Keymap {
    pub rows: u8,
    pub cols: u8,
    pub layers: u8,
    pub keymap: Vec<Vec<Vec<String>>>,
    pub encoder_map: Vec<Vec<[String; 2]>>,
    /// Per-layer gesture actions, alias-resolved; a layer without `touch` is empty.
    pub touch_map: Vec<Vec<TouchActionsConfig>>,
    pub key_info: Vec<Vec<KeyInfo>>,
    /// Total number of encoders on the board.
    pub num_encoder: usize,
    /// Total number of touchpads with gestures on the board.
    pub num_touchpad: usize,
}

impl crate::KeyboardTomlConfig {
    /// Resolve the keymap configuration from TOML config.
    pub fn keymap(&self) -> Result<Keymap, String> {
        let (keymap_config, key_info) = self.get_keymap_config()?;
        // Encoders may be spread across split halves; only the board-wide total is used here.
        let num_encoder = self.total_encoders();
        let num_touchpad = self.total_touchpads();

        // Encoder maps are all-or-none; partial lists would leave encoders dead.
        for (i, encoders) in keymap_config.encoder_map.iter().enumerate() {
            if !encoders.is_empty() && encoders.len() != num_encoder {
                return Err(format!(
                    "keyboard.toml: [[keymap.layer]] #{i} lists {} encoders but the board has \
                     {num_encoder} (configure all {num_encoder} or none)",
                    encoders.len()
                ));
            }
        }

        for (i, touchpads) in keymap_config.touch_map.iter().enumerate() {
            if !touchpads.is_empty() && touchpads.len() != num_touchpad {
                return Err(format!(
                    "keyboard.toml: [[keymap.layer]] #{i} lists {} touchpads but the board has \
                     {num_touchpad} with gestures (configure all {num_touchpad} or none)",
                    touchpads.len()
                ));
            }
        }

        Ok(Keymap {
            rows: keymap_config.rows,
            cols: keymap_config.cols,
            layers: keymap_config.layers,
            keymap: keymap_config.keymap,
            encoder_map: keymap_config.encoder_map,
            touch_map: keymap_config.touch_map,
            key_info,
            num_encoder,
            num_touchpad,
        })
    }
}
