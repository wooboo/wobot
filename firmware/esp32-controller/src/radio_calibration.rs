use std::fmt::Write;

use esp_idf_svc::nvs::EspDefaultNvs;

pub const CALIBRATION_CHANNELS: usize = 10;
pub const MAX_CALIBRATION_FORM_SIZE: usize = 1_024;
const STORAGE_KEY: &str = "radio_cal";

#[derive(Debug, Clone)]
pub struct RadioCalibration {
    pub configured: bool,
    pub ranges: [[u16; 3]; CALIBRATION_CHANNELS],
    /// Pedal throttle, steering, strafe, and direction; zero-based indices.
    pub pedal_mapping: [u8; 4],
    /// Centered forward/reverse, steering, and strafe axes; zero-based indices.
    pub right_stick_mapping: [u8; 3],
    /// Servo, ARM, and control-profile selector; zero-based indices.
    pub shared_mapping: [u8; 3],
    /// Bits invert the functions in the corresponding mapping arrays.
    pub pedal_invert_mask: u8,
    pub right_stick_invert_mask: u8,
    pub shared_invert_mask: u8,
    /// Dead zones in per-mille of calibrated travel.
    pub stick_deadband: u16,
    pub switch_deadband: u16,
}

impl Default for RadioCalibration {
    fn default() -> Self {
        Self {
            configured: false,
            ranges: [[0, 16_384, 32_767]; CALIBRATION_CHANNELS],
            pedal_mapping: [2, 0, 3, 5],
            right_stick_mapping: [1, 0, 3],
            shared_mapping: [6, 4, 7],
            pedal_invert_mask: 0,
            right_stick_invert_mask: 0,
            shared_invert_mask: 0,
            stick_deadband: 35,
            switch_deadband: 250,
        }
    }
}

impl RadioCalibration {
    pub fn from_form(body: &[u8]) -> anyhow::Result<Self> {
        let form = std::str::from_utf8(body)?;
        let ranges = parse_ranges(form)?;
        let pedal_mapping = [
            parse_field::<u8>(form, "pedal_throttle")?,
            parse_field::<u8>(form, "pedal_steering")?,
            parse_field::<u8>(form, "pedal_strafe")?,
            parse_field::<u8>(form, "pedal_direction")?,
        ];
        let right_stick_mapping = [
            parse_field::<u8>(form, "stick_throttle")?,
            parse_field::<u8>(form, "stick_steering")?,
            parse_field::<u8>(form, "stick_strafe")?,
        ];
        let shared_mapping = [
            parse_field::<u8>(form, "servo")?,
            parse_field::<u8>(form, "arm")?,
            parse_field::<u8>(form, "mode")?,
        ];

        validate_channel_indices(&pedal_mapping)?;
        validate_channel_indices(&right_stick_mapping)?;
        validate_channel_indices(&shared_mapping)?;
        ensure_unique(&[
            pedal_mapping[0],
            pedal_mapping[1],
            pedal_mapping[2],
            pedal_mapping[3],
            shared_mapping[0],
            shared_mapping[1],
            shared_mapping[2],
        ])?;
        ensure_unique(&[
            right_stick_mapping[0],
            right_stick_mapping[1],
            right_stick_mapping[2],
            shared_mapping[0],
            shared_mapping[1],
            shared_mapping[2],
        ])?;

        let pedal_invert_mask = parse_field::<u8>(form, "pedal_invert")?;
        let right_stick_invert_mask = parse_field::<u8>(form, "stick_invert")?;
        let shared_invert_mask = parse_field::<u8>(form, "shared_invert")?;
        if pedal_invert_mask & !0b1111 != 0
            || right_stick_invert_mask & !0b111 != 0
            || shared_invert_mask & !0b111 != 0
        {
            anyhow::bail!("invalid inversion mask");
        }

        let stick_deadband = parse_field::<u16>(form, "stick_deadband")?;
        if stick_deadband > 300 {
            anyhow::bail!("stick deadband is too large");
        }
        let switch_deadband = parse_field::<u16>(form, "switch_deadband")?;
        if switch_deadband > 450 {
            anyhow::bail!("switch deadband is too large");
        }

        for &(channel, name) in &[
            (pedal_mapping[0], "gaz trybu pedału"),
            (pedal_mapping[1], "skręt trybu pedału"),
            (pedal_mapping[2], "jazda bokiem"),
            (pedal_mapping[3], "kierunek"),
            (right_stick_mapping[0], "jazda przód/tył prawego drążka"),
            (right_stick_mapping[1], "skręt prawego drążka"),
            (
                right_stick_mapping[2],
                "jazda bokiem w trybie prawego drążka",
            ),
            (shared_mapping[0], "serwo"),
            (shared_mapping[1], "ARM"),
            (shared_mapping[2], "wybór trybu"),
        ] {
            let [minimum, _, maximum] = ranges[channel as usize];
            if maximum.saturating_sub(minimum) < 500 {
                anyhow::bail!(
                    "CH{} ({name}) ma zbyt mały zarejestrowany zakres",
                    channel + 1
                );
            }
        }

        for &(channel, name) in &[
            (pedal_mapping[1], "skręt trybu pedału"),
            (pedal_mapping[2], "jazda bokiem"),
            (right_stick_mapping[0], "jazda przód/tył prawego drążka"),
            (right_stick_mapping[1], "skręt prawego drążka"),
            (
                right_stick_mapping[2],
                "jazda bokiem w trybie prawego drążka",
            ),
        ] {
            let [minimum, center, maximum] = ranges[channel as usize];
            if center.saturating_sub(minimum) < 250 || maximum.saturating_sub(center) < 250 {
                anyhow::bail!(
                    "CH{} ({name}) nie ma zakresu po obu stronach pozycji środkowej",
                    channel + 1
                );
            }
        }

        Ok(Self {
            configured: true,
            ranges,
            pedal_mapping,
            right_stick_mapping,
            shared_mapping,
            pedal_invert_mask,
            right_stick_invert_mask,
            shared_invert_mask,
            stick_deadband,
            switch_deadband,
        })
    }

    pub fn encode(&self) -> String {
        let mut ranges = String::with_capacity(192);
        for (index, [minimum, center, maximum]) in self.ranges.iter().enumerate() {
            if index > 0 {
                ranges.push(';');
            }
            write!(&mut ranges, "{minimum},{center},{maximum}").expect("write to String");
        }
        format!(
            "v4|{ranges}|{},{},{},{}|{},{},{}|{},{},{}|{},{},{}|{}|{}",
            self.pedal_mapping[0],
            self.pedal_mapping[1],
            self.pedal_mapping[2],
            self.pedal_mapping[3],
            self.right_stick_mapping[0],
            self.right_stick_mapping[1],
            self.right_stick_mapping[2],
            self.shared_mapping[0],
            self.shared_mapping[1],
            self.shared_mapping[2],
            self.pedal_invert_mask,
            self.right_stick_invert_mask,
            self.shared_invert_mask,
            self.stick_deadband,
            self.switch_deadband
        )
    }

    pub fn decode(encoded: &str) -> anyhow::Result<Self> {
        if encoded.starts_with("v4|") {
            return Self::decode_v4(encoded);
        }
        if encoded.starts_with("v3|") {
            return Self::decode_v3(encoded);
        }
        if encoded.starts_with("v2|") {
            return Self::decode_v2(encoded);
        }
        Self::decode_legacy(encoded)
    }

    fn decode_v4(encoded: &str) -> anyhow::Result<Self> {
        let parts: Vec<&str> = encoded.split('|').collect();
        if parts.len() != 8 || parts[0] != "v4" {
            anyhow::bail!("invalid stored calibration");
        }
        Self::decode_profile_form(&parts, parts[6], parts[7], true)
    }

    fn decode_v3(encoded: &str) -> anyhow::Result<Self> {
        let parts: Vec<&str> = encoded.split('|').collect();
        if parts.len() != 8 || parts[0] != "v3" {
            anyhow::bail!("invalid stored calibration");
        }
        Self::decode_profile_form(&parts, parts[6], parts[7], false)
    }

    fn decode_v2(encoded: &str) -> anyhow::Result<Self> {
        let parts: Vec<&str> = encoded.split('|').collect();
        if parts.len() != 7 || parts[0] != "v2" {
            anyhow::bail!("invalid stored calibration");
        }
        Self::decode_profile_form(&parts, parts[6], "250", false)
    }

    fn decode_profile_form(
        parts: &[&str],
        stick_deadband: &str,
        switch_deadband: &str,
        has_strafe: bool,
    ) -> anyhow::Result<Self> {
        let pedal = split_exact::<4>(parts[2])?;
        let shared = split_exact::<3>(parts[4])?;
        let invert = split_exact::<3>(parts[5])?;
        let stick: Vec<&str> = parts[3].split(',').collect();
        let expected_stick_fields = if has_strafe { 3 } else { 2 };
        if stick.len() != expected_stick_fields {
            anyhow::bail!("invalid stored calibration");
        }
        let stick_strafe = if has_strafe {
            stick[2].parse::<u8>()?
        } else {
            choose_migrated_strafe(pedal[2], &[stick[0], stick[1]], &shared)?
        };
        let stick_invert = if has_strafe {
            invert[1].parse::<u8>()?
        } else {
            invert[1].parse::<u8>()? | (invert[0].parse::<u8>()? & 0b100)
        };
        let form = format!(
            "ranges={}&pedal_throttle={}&pedal_steering={}&pedal_strafe={}&pedal_direction={}&stick_throttle={}&stick_steering={}&stick_strafe={stick_strafe}&servo={}&arm={}&mode={}&pedal_invert={}&stick_invert={}&shared_invert={}&stick_deadband={stick_deadband}&switch_deadband={switch_deadband}",
            parts[1],
            pedal[0],
            pedal[1],
            pedal[2],
            pedal[3],
            stick[0],
            stick[1],
            shared[0],
            shared[1],
            shared[2],
            invert[0],
            stick_invert,
            invert[2]
        );
        Self::from_form(form.as_bytes())
    }

    fn decode_legacy(encoded: &str) -> anyhow::Result<Self> {
        let parts: Vec<&str> = encoded.split('|').collect();
        if parts.len() != 4 {
            anyhow::bail!("invalid stored calibration");
        }
        let legacy = split_exact::<6>(parts[1])?;
        let old_invert: u8 = parts[2].parse()?;
        let mut used = [false; CALIBRATION_CHANNELS];
        for value in legacy {
            let channel: usize = value.parse()?;
            if channel >= CALIBRATION_CHANNELS {
                anyhow::bail!("invalid stored calibration");
            }
            used[channel] = true;
        }
        let stick_throttle = if !used[1] {
            1
        } else {
            used.iter()
                .position(|channel_used| !channel_used)
                .ok_or_else(|| anyhow::anyhow!("no free channel for right stick"))?
        };
        used[stick_throttle] = true;
        let mode = used
            .iter()
            .position(|channel_used| !channel_used)
            .ok_or_else(|| anyhow::anyhow!("no free channel for mode selector"))?;
        let pedal_invert = old_invert & 0b1111;
        let stick_invert = old_invert & 0b110;
        let shared_invert = ((old_invert >> 4) & 0b1) | ((old_invert >> 4) & 0b10);
        let form = format!(
            "ranges={}&pedal_throttle={}&pedal_steering={}&pedal_strafe={}&pedal_direction={}&stick_throttle={stick_throttle}&stick_steering={}&stick_strafe={}&servo={}&arm={}&mode={mode}&pedal_invert={pedal_invert}&stick_invert={stick_invert}&shared_invert={shared_invert}&stick_deadband={}&switch_deadband=250",
            parts[0],
            legacy[0],
            legacy[1],
            legacy[2],
            legacy[3],
            legacy[1],
            legacy[2],
            legacy[4],
            legacy[5],
            parts[3]
        );
        Self::from_form(form.as_bytes())
    }

    pub fn to_json(&self) -> String {
        let ranges = self
            .ranges
            .iter()
            .map(|[minimum, center, maximum]| format!("[{minimum},{center},{maximum}]"))
            .collect::<Vec<_>>()
            .join(",");
        format!(
            "{{\"ok\":true,\"configured\":{},\"ranges\":[{}],\"mapping\":{{\"pedal\":{{\"throttle\":{},\"steering\":{},\"strafe\":{},\"direction\":{}}},\"right_stick\":{{\"throttle\":{},\"steering\":{},\"strafe\":{}}},\"shared\":{{\"servo\":{},\"arm\":{},\"mode\":{}}}}},\"invert\":{{\"pedal\":{},\"right_stick\":{},\"shared\":{}}},\"deadband\":{},\"switch_deadband\":{}}}",
            self.configured,
            ranges,
            self.pedal_mapping[0],
            self.pedal_mapping[1],
            self.pedal_mapping[2],
            self.pedal_mapping[3],
            self.right_stick_mapping[0],
            self.right_stick_mapping[1],
            self.right_stick_mapping[2],
            self.shared_mapping[0],
            self.shared_mapping[1],
            self.shared_mapping[2],
            self.pedal_invert_mask,
            self.right_stick_invert_mask,
            self.shared_invert_mask,
            self.stick_deadband,
            self.switch_deadband
        )
    }
}

pub fn load(store: &EspDefaultNvs) -> anyhow::Result<RadioCalibration> {
    let mut buffer = [0_u8; 768];
    let Some(encoded) = store.get_str(STORAGE_KEY, &mut buffer)? else {
        return Ok(RadioCalibration::default());
    };
    match RadioCalibration::decode(encoded) {
        Ok(calibration) => Ok(calibration),
        Err(error) => {
            log::warn!("Ignoring invalid stored radio calibration: {error:#}");
            Ok(RadioCalibration::default())
        }
    }
}

pub fn save(store: &EspDefaultNvs, calibration: &RadioCalibration) -> anyhow::Result<()> {
    store.set_str(STORAGE_KEY, &calibration.encode())?;
    Ok(())
}

fn parse_ranges(form: &str) -> anyhow::Result<[[u16; 3]; CALIBRATION_CHANNELS]> {
    let encoded = field(form, "ranges").ok_or_else(|| anyhow::anyhow!("missing ranges"))?;
    let items: Vec<&str> = encoded.split(';').collect();
    if items.len() != CALIBRATION_CHANNELS {
        anyhow::bail!("expected {CALIBRATION_CHANNELS} channel ranges");
    }
    let mut ranges = [[0_u16; 3]; CALIBRATION_CHANNELS];
    for (index, item) in items.into_iter().enumerate() {
        let values = split_exact::<3>(item)?;
        for (part, value) in values.into_iter().enumerate() {
            ranges[index][part] = value.parse()?;
        }
        let [minimum, center, maximum] = ranges[index];
        if maximum > 0x7fff || minimum > center || center > maximum {
            anyhow::bail!("invalid values for channel {}", index + 1);
        }
    }
    Ok(ranges)
}

fn choose_migrated_strafe(
    preferred: &str,
    stick_channels: &[&str],
    shared_channels: &[&str; 3],
) -> anyhow::Result<u8> {
    let mut used = [false; CALIBRATION_CHANNELS];
    for value in stick_channels.iter().chain(shared_channels.iter()) {
        let channel: usize = value.parse()?;
        if channel >= CALIBRATION_CHANNELS {
            anyhow::bail!("invalid stored calibration");
        }
        used[channel] = true;
    }
    let preferred: usize = preferred.parse()?;
    if preferred < CALIBRATION_CHANNELS && !used[preferred] {
        return Ok(preferred as u8);
    }
    used.iter()
        .position(|channel_used| !channel_used)
        .map(|channel| channel as u8)
        .ok_or_else(|| anyhow::anyhow!("no free channel for strafe"))
}

fn validate_channel_indices(channels: &[u8]) -> anyhow::Result<()> {
    if channels
        .iter()
        .any(|channel| *channel as usize >= CALIBRATION_CHANNELS)
    {
        anyhow::bail!("channel mapping is out of range");
    }
    Ok(())
}

fn ensure_unique(channels: &[u8]) -> anyhow::Result<()> {
    for left in 0..channels.len() {
        for right in left + 1..channels.len() {
            if channels[left] == channels[right] {
                anyhow::bail!("each active function must use a different channel");
            }
        }
    }
    Ok(())
}

fn split_exact<const N: usize>(value: &str) -> anyhow::Result<[&str; N]> {
    value
        .split(',')
        .collect::<Vec<_>>()
        .try_into()
        .map_err(|_| anyhow::anyhow!("invalid stored calibration"))
}

fn field<'a>(form: &'a str, wanted: &str) -> Option<&'a str> {
    form.split('&').find_map(|item| {
        let (key, value) = item.split_once('=')?;
        (key == wanted).then_some(value)
    })
}

fn parse_field<T>(form: &str, name: &str) -> anyhow::Result<T>
where
    T: std::str::FromStr,
    T::Err: std::error::Error + Send + Sync + 'static,
{
    field(form, name)
        .ok_or_else(|| anyhow::anyhow!("missing {name}"))?
        .parse()
        .map_err(anyhow::Error::from)
}
