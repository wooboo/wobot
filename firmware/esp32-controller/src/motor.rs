use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::Duration,
};

use wobot_core::{
    ChannelCalibration, DriveInput, DriveMode, DriveOutput, DriveStopReason, HatDriveController,
    MotorCommand, WheelSpeeds, FLAG_ARMED,
};

use crate::{arduino_ota::ArduinoOta, radio::RadioService, radio_calibration::RadioCalibration};

const CONTROL_PERIOD: Duration = Duration::from_millis(20);
const MAX_STEP_PER_PERIOD: i16 = 40;

#[derive(Debug, Clone)]
pub struct MotorSnapshot {
    pub sequence: u8,
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

        let input = calibration
            .lock()
            .ok()
            .and_then(|calibration| decode_input(&radio.snapshot(), &calibration));
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
) -> Option<DriveInput> {
    if !snapshot.connected || !calibration.configured {
        return None;
    }
    let highest_channel = calibration.mapping.iter().copied().max()? as usize;
    if snapshot.channel_count <= highest_channel {
        return None;
    }

    let channel = |function: usize| {
        let index = calibration.mapping[function] as usize;
        let [minimum, neutral, maximum] = calibration.ranges[index];
        (
            ChannelCalibration {
                minimum,
                neutral,
                maximum,
                inverted: calibration.invert_mask & (1 << function) != 0,
            },
            snapshot.channels[index],
        )
    };

    let (throttle_calibration, throttle_raw) = channel(0);
    let (steering_calibration, steering_raw) = channel(1);
    let (strafe_calibration, strafe_raw) = channel(2);
    let (direction_calibration, direction_raw) = channel(3);
    let (arm_calibration, arm_raw) = channel(5);
    Some(DriveInput {
        throttle: throttle_calibration.unipolar(throttle_raw).ok()?,
        steering: steering_calibration
            .bipolar(steering_raw, calibration.deadband as i16)
            .ok()?,
        strafe: strafe_calibration
            .bipolar(strafe_raw, calibration.deadband as i16)
            .ok()?,
        direction: direction_calibration.direction(direction_raw).ok()?,
        arm: arm_calibration.arm(arm_raw).ok()?,
    })
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
    format!(
        "{{\"ok\":true,\"ready\":true,\"armed\":{},\"mode\":\"{mode}\",\"stop_reason\":{reason},\"wheels\":[{front_left},{front_right},{rear_left},{rear_right}],\"sequence\":{},\"sent\":{},\"error\":{error}}}",
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
