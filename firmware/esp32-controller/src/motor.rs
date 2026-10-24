use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::Duration,
};

use wobot_core::{
    ChannelCalibration, ControlProfile, DriveDirection, DriveInput, DriveMode, DriveOutput,
    DriveStopReason, HatDriveController, MotorCommand, WheelSpeeds, FLAG_ARMED,
};

use crate::{arduino_ota::ArduinoOta, radio::RadioService, radio_calibration::RadioCalibration};

const CONTROL_PERIOD: Duration = Duration::from_millis(20);
const MAX_STEP_PER_PERIOD: i16 = 40;

#[derive(Debug, Clone)]
pub struct MotorSnapshot {
    pub sequence: u8,
    pub control_profile: Option<ControlProfile>,
    pub output: Option<DriveOutput>,
    pub sent: bool,
    pub error: Option<String>,
}

#[derive(Clone)]
pub struct MotorService {
    state: Arc<Mutex<MotorSnapshot>>,
    require_disarm: Arc<AtomicBool>,
    maintenance: Arc<AtomicBool>,
}

pub struct MotorMaintenanceGuard {
    service: MotorService,
}

impl Drop for MotorMaintenanceGuard {
    fn drop(&mut self) {
        self.service.end_maintenance();
    }
}

impl MotorService {
    pub fn start(
        radio: RadioService,
        calibration: Arc<Mutex<RadioCalibration>>,
        arduino: Arc<Mutex<ArduinoOta>>,
    ) -> Self {
        let service = Self {
            state: Arc::new(Mutex::new(MotorSnapshot {
                sequence: 0,
                control_profile: None,
                output: None,
                sent: false,
                error: None,
            })),
            require_disarm: Arc::new(AtomicBool::new(true)),
            maintenance: Arc::new(AtomicBool::new(false)),
        };
        let state = Arc::clone(&service.state);
        let require_disarm = Arc::clone(&service.require_disarm);
        let maintenance = Arc::clone(&service.maintenance);
        if let Err(error) = thread::Builder::new()
            .name("wobot-motors".into())
            .stack_size(8 * 1024)
            .spawn(move || {
                control_loop(
                    &state,
                    &require_disarm,
                    &maintenance,
                    &radio,
                    &calibration,
                    &arduino,
                )
            })
        {
            if let Ok(mut state) = service.state.lock() {
                state.error = Some(format!("Nie udało się uruchomić sterowania: {error}"));
            }
        }
        service
    }

    pub fn snapshot(&self) -> MotorSnapshot {
        self.state
            .lock()
            .expect("motor state lock poisoned")
            .clone()
    }

    /// Forces a fresh DISARM observation before any subsequent movement.
    pub fn require_disarm(&self) {
        self.require_disarm.store(true, Ordering::Release);
    }

    pub fn begin_maintenance(&self) {
        self.require_disarm();
        self.maintenance.store(true, Ordering::Release);
    }

    pub fn end_maintenance(&self) {
        self.require_disarm();
        self.maintenance.store(false, Ordering::Release);
    }

    pub fn maintenance(&self) -> MotorMaintenanceGuard {
        self.begin_maintenance();
        MotorMaintenanceGuard {
            service: self.clone(),
        }
    }
}

fn control_loop(
    state: &Arc<Mutex<MotorSnapshot>>,
    require_disarm: &AtomicBool,
    maintenance: &AtomicBool,
    radio: &RadioService,
    calibration: &Arc<Mutex<RadioCalibration>>,
    arduino: &Arc<Mutex<ArduinoOta>>,
) {
    let mut controller = HatDriveController::new(MAX_STEP_PER_PERIOD);
    let mut sequence = 0_u8;

    loop {
        if maintenance.load(Ordering::Acquire) {
            thread::sleep(CONTROL_PERIOD);
            continue;
        }
        if require_disarm.swap(false, Ordering::AcqRel) {
            controller.require_disarm();
        }

        let decoded = calibration
            .lock()
            .ok()
            .and_then(|calibration| decode_input(&radio.snapshot(), &calibration));
        if let Some((_, profile)) = decoded {
            controller.select_control_profile(profile);
        }
        let input = decoded.map(|(input, _)| input);
        let output = controller.update(input);
        let command = MotorCommand {
            sequence,
            flags: if output.armed { FLAG_ARMED } else { 0 },
            wheels: output.wheels,
        };

        let (sent, error) = match arduino.try_lock() {
            Ok(link) => match link.send_motor_command(command) {
                Ok(()) => (true, None),
                Err(error) => (false, Some(format!("UART Arduino: {error}"))),
            },
            Err(std::sync::TryLockError::WouldBlock) => (false, None),
            Err(std::sync::TryLockError::Poisoned(_)) => (
                false,
                Some("Blokada UART Arduino jest uszkodzona".to_owned()),
            ),
        };
        if sent {
            sequence = sequence.wrapping_add(1);
        }
        if let Ok(mut state) = state.lock() {
            *state = MotorSnapshot {
                sequence,
                control_profile: decoded.map(|(_, profile)| profile),
                output: Some(output),
                sent,
                error,
            };
        }
        thread::sleep(CONTROL_PERIOD);
    }
}

fn decode_input(
    snapshot: &crate::radio::RadioSnapshot,
    calibration: &RadioCalibration,
) -> Option<(DriveInput, ControlProfile)> {
    if !snapshot.connected || !calibration.configured {
        return None;
    }
    let highest_channel = calibration
        .pedal_mapping
        .iter()
        .chain(calibration.right_stick_mapping.iter())
        .chain(calibration.shared_mapping.iter())
        .copied()
        .max()? as usize;
    if snapshot.channel_count <= highest_channel {
        return None;
    }

    let channel = |index: u8, inverted: bool| {
        let index = index as usize;
        let [minimum, neutral, maximum] = calibration.ranges[index];
        (
            ChannelCalibration {
                minimum,
                neutral,
                maximum,
                inverted,
            },
            snapshot.channels[index],
        )
    };

    let (profile_calibration, profile_raw) = channel(
        calibration.shared_mapping[2],
        calibration.shared_invert_mask & (1 << 2) != 0,
    );
    let profile = profile_calibration
        .control_profile_with_deadband(profile_raw, calibration.switch_deadband)
        .ok()?;
    let (arm_calibration, arm_raw) = channel(
        calibration.shared_mapping[1],
        calibration.shared_invert_mask & (1 << 1) != 0,
    );
    let arm = arm_calibration
        .arm_with_deadband(arm_raw, calibration.switch_deadband)
        .ok()?;

    let input = match profile {
        ControlProfile::Pedal => {
            let (throttle_calibration, throttle_raw) = channel(
                calibration.pedal_mapping[0],
                calibration.pedal_invert_mask & (1 << 0) != 0,
            );
            let (steering_calibration, steering_raw) = channel(
                calibration.pedal_mapping[1],
                calibration.pedal_invert_mask & (1 << 1) != 0,
            );
            let (strafe_calibration, strafe_raw) = channel(
                calibration.pedal_mapping[2],
                calibration.pedal_invert_mask & (1 << 2) != 0,
            );
            let (direction_calibration, direction_raw) = channel(
                calibration.pedal_mapping[3],
                calibration.pedal_invert_mask & (1 << 3) != 0,
            );
            DriveInput {
                throttle: throttle_calibration.unipolar(throttle_raw).ok()?,
                steering: steering_calibration
                    .bipolar(steering_raw, calibration.stick_deadband as i16)
                    .ok()?,
                strafe: strafe_calibration
                    .bipolar(strafe_raw, calibration.stick_deadband as i16)
                    .ok()?,
                direction: direction_calibration
                    .direction_with_deadband(direction_raw, calibration.switch_deadband)
                    .ok()?,
                arm,
            }
        }
        ControlProfile::RightStick => {
            let (throttle_calibration, throttle_raw) = channel(
                calibration.right_stick_mapping[0],
                calibration.right_stick_invert_mask & (1 << 0) != 0,
            );
            let (steering_calibration, steering_raw) = channel(
                calibration.right_stick_mapping[1],
                calibration.right_stick_invert_mask & (1 << 1) != 0,
            );
            let (strafe_calibration, strafe_raw) = channel(
                calibration.right_stick_mapping[2],
                calibration.right_stick_invert_mask & (1 << 2) != 0,
            );
            DriveInput {
                throttle: throttle_calibration
                    .bipolar(throttle_raw, calibration.stick_deadband as i16)
                    .ok()?,
                steering: steering_calibration
                    .bipolar(steering_raw, calibration.stick_deadband as i16)
                    .ok()?,
                strafe: strafe_calibration
                    .bipolar(strafe_raw, calibration.stick_deadband as i16)
                    .ok()?,
                direction: DriveDirection::Forward,
                arm,
            }
        }
    };
    Some((input, profile))
}

pub fn status_json(snapshot: &MotorSnapshot) -> String {
    let Some(output) = snapshot.output else {
        return r#"{"ok":true,"ready":false}"#.to_owned();
    };
    let mode = match output.mode {
        DriveMode::Stopped => "stopped",
        DriveMode::CarLike => "car_like",
        DriveMode::Strafe => "strafe",
    };
    let reason = output.stop_reason.map(reason_name);
    let reason = reason
        .map(|reason| format!("\"{reason}\""))
        .unwrap_or_else(|| "null".to_owned());
    let WheelSpeeds {
        front_left,
        front_right,
        rear_left,
        rear_right,
    } = output.wheels;
    let error = snapshot
        .error
        .as_deref()
        .map(|error| format!("\"{}\"", crate::escape_json(error)))
        .unwrap_or_else(|| "null".to_owned());
    let control_profile = match snapshot.control_profile {
        Some(ControlProfile::Pedal) => "\"pedal\"",
        Some(ControlProfile::RightStick) => "\"right_stick\"",
        None => "null",
    };
    format!(
        "{{\"ok\":true,\"ready\":true,\"armed\":{},\"control_profile\":{control_profile},\"mode\":\"{mode}\",\"stop_reason\":{reason},\"wheels\":[{front_left},{front_right},{rear_left},{rear_right}],\"sequence\":{},\"sent\":{},\"error\":{error}}}",
        output.armed, snapshot.sequence, snapshot.sent
    )
}

fn reason_name(reason: DriveStopReason) -> &'static str {
    match reason {
        DriveStopReason::InputUnavailable => "input_unavailable",
        DriveStopReason::Disarmed => "disarmed",
        DriveStopReason::AwaitingDisarm => "awaiting_disarm",
        DriveStopReason::ControlsNotIdle => "controls_not_idle",
        DriveStopReason::Neutral => "neutral",
        DriveStopReason::ModeChange => "mode_change",
    }
}
