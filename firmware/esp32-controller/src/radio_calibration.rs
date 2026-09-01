use std::fmt::Write;

use esp_idf_svc::nvs::EspDefaultNvs;

pub const CALIBRATION_CHANNELS: usize = 10;
pub const MAX_CALIBRATION_FORM_SIZE: usize = 1_024;
const STORAGE_KEY: &str = "radio_cal";

#[derive(Debug, Clone)]
pub struct RadioCalibration {
    pub configured: bool,
    pub ranges: [[u16; 3]; CALIBRATION_CHANNELS],
    /// Throttle, steering, strafe, direction, servo, ARM; zero-based indices.
    pub mapping: [u8; 6],
    /// Bits 0..=5 invert throttle, steering, strafe, direction, servo, and ARM.
    pub invert_mask: u8,
    /// Deadband in per-mille of calibrated travel.
    pub deadband: u16,
}

impl Default for RadioCalibration {
    fn default() -> Self {
        Self {
            configured: false,
            ranges: [[0, 16_384, 32_767]; CALIBRATION_CHANNELS],
            mapping: [2, 0, 3, 5, 6, 4],
            invert_mask: 0,
            deadband: 35,
        }
    }
}

impl RadioCalibration {
    pub fn from_form(body: &[u8]) -> anyhow::Result<Self> {
        let form = std::str::from_utf8(body)?;
        let ranges = field(form, "ranges").ok_or_else(|| anyhow::anyhow!("missing ranges"))?;
        let mut parsed_ranges = [[0_u16; 3]; CALIBRATION_CHANNELS];
        let range_items: Vec<&str> = ranges.split(';').collect();
        if range_items.len() != CALIBRATION_CHANNELS {
            anyhow::bail!("expected {CALIBRATION_CHANNELS} channel ranges");
        }
        for (index, item) in range_items.into_iter().enumerate() {
            let values: Vec<&str> = item.split(',').collect();
            if values.len() != 3 {
                anyhow::bail!("invalid range for channel {}", index + 1);
            }
            for (part, value) in values.into_iter().enumerate() {
                parsed_ranges[index][part] = value.parse()?;
            }
            let [minimum, center, maximum] = parsed_ranges[index];
            if maximum > 0x7fff || minimum > center || center > maximum {
                anyhow::bail!("invalid values for channel {}", index + 1);
            }
        }

        let mapping = [
            parse_field::<u8>(form, "throttle")?,
            parse_field::<u8>(form, "steering")?,
            parse_field::<u8>(form, "strafe")?,
            parse_field::<u8>(form, "direction")?,
            parse_field::<u8>(form, "servo")?,
            parse_field::<u8>(form, "arm")?,
        ];
        if mapping
            .iter()
            .any(|channel| *channel as usize >= CALIBRATION_CHANNELS)
        {
            anyhow::bail!("channel mapping is out of range");
        }
        for left in 0..mapping.len() {
            for right in left + 1..mapping.len() {
                if mapping[left] == mapping[right] {
                    anyhow::bail!("each function must use a different channel");
                }
            }
        }

        let invert_mask = parse_field::<u8>(form, "invert")?;
        if invert_mask & !0b11_1111 != 0 {
            anyhow::bail!("invalid inversion mask");
        }
        let deadband = parse_field::<u16>(form, "deadband")?;
        if deadband > 300 {
            anyhow::bail!("deadband is too large");
        }

        let function_names = ["gaz", "skręt", "jazda bokiem", "kierunek", "serwo", "ARM"];
        for (function, &channel) in mapping.iter().enumerate() {
            let [minimum, _, maximum] = parsed_ranges[channel as usize];
            if maximum.saturating_sub(minimum) < 500 {
                anyhow::bail!(
                    "CH{} ({}) ma zbyt mały zarejestrowany zakres",
                    channel + 1,
                    function_names[function]
                );
            }
        }

        // Only centered stick axes need travel around the captured resting
        // value. Direction and ARM derive their safe middle zones solely from
        // measured endpoints; their resting positions may be at an extreme.
        for &(axis, name) in &[(mapping[1], "skręt"), (mapping[2], "jazda bokiem")] {
            let [minimum, center, maximum] = parsed_ranges[axis as usize];
            if center.saturating_sub(minimum) < 250 || maximum.saturating_sub(center) < 250 {
                anyhow::bail!(
                    "CH{} ({name}) nie ma zakresu po obu stronach pozycji środkowej",
                    axis + 1
                );
            }
        }

        Ok(Self {
            configured: true,
            ranges: parsed_ranges,
            mapping,
            invert_mask,
            deadband,
        })
    }

    pub fn encode(&self) -> String {
        let mut encoded = String::with_capacity(192);
        for (index, [minimum, center, maximum]) in self.ranges.iter().enumerate() {
            if index > 0 {
                encoded.push(';');
            }
            write!(&mut encoded, "{minimum},{center},{maximum}").expect("write to String");
        }
        write!(
            &mut encoded,
            "|{},{},{},{},{},{}|{}|{}",
            self.mapping[0],
            self.mapping[1],
            self.mapping[2],
            self.mapping[3],
            self.mapping[4],
            self.mapping[5],
            self.invert_mask,
            self.deadband
        )
        .expect("write to String");
        encoded
    }

    pub fn decode(encoded: &str) -> anyhow::Result<Self> {
        let parts: Vec<&str> = encoded.split('|').collect();
        if parts.len() != 4 {
            anyhow::bail!("invalid stored calibration");
        }
        let form = format!(
            "ranges={}&throttle={}&steering={}&strafe={}&direction={}&servo={}&arm={}&invert={}&deadband={}",
            parts[0],
            parts[1].split(',').next().unwrap_or(""),
            parts[1].split(',').nth(1).unwrap_or(""),
            parts[1].split(',').nth(2).unwrap_or(""),
            parts[1].split(',').nth(3).unwrap_or(""),
            parts[1].split(',').nth(4).unwrap_or(""),
            parts[1].split(',').nth(5).unwrap_or(""),
            parts[2],
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
            "{{\"ok\":true,\"configured\":{},\"ranges\":[{}],\"mapping\":{{\"throttle\":{},\"steering\":{},\"strafe\":{},\"direction\":{},\"servo\":{},\"arm\":{}}},\"invert\":{},\"deadband\":{}}}",
            self.configured,
            ranges,
            self.mapping[0],
            self.mapping[1],
            self.mapping[2],
            self.mapping[3],
            self.mapping[4],
            self.mapping[5],
            self.invert_mask,
            self.deadband
        )
    }
}

pub fn load(store: &EspDefaultNvs) -> anyhow::Result<RadioCalibration> {
    let mut buffer = [0_u8; 512];
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
