#![no_std]

pub const COMMAND_MIN: i16 = -1_000;
pub const COMMAND_MAX: i16 = 1_000;
pub const CAR_STEERING_AUTHORITY: i16 = 700;
pub const DEFAULT_SWITCH_DEADBAND: u16 = 250;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElrsFrameError {
    TooShort,
    WrongFrameType,
    InvalidChannelCount,
    WrongLength,
    InvalidChannelValue,
}

/// A decoded ExpressLRS Wi-Fi Joystick channel frame.
///
/// Wire format: frame type, channel count, then little-endian `u16` channels.
/// ELRS maps stick values to the inclusive range `0..=0x7fff`.
#[derive(Debug, Clone, Copy)]
pub struct ElrsChannels<'a> {
    count: usize,
    payload: &'a [u8],
}

impl<'a> ElrsChannels<'a> {
    pub const FRAME_CHANNELS: u8 = 1;
    pub const MAX_CHANNELS: usize = 16;

    pub fn parse(packet: &'a [u8]) -> Result<Self, ElrsFrameError> {
        if packet.len() < 2 {
            return Err(ElrsFrameError::TooShort);
        }
        if packet[0] != Self::FRAME_CHANNELS {
            return Err(ElrsFrameError::WrongFrameType);
        }

        let count = packet[1] as usize;
        if !(1..=Self::MAX_CHANNELS).contains(&count) {
            return Err(ElrsFrameError::InvalidChannelCount);
        }
        if packet.len() != 2 + count * 2 {
            return Err(ElrsFrameError::WrongLength);
        }

        let frame = Self {
            count,
            payload: &packet[2..],
        };
        for index in 0..count {
            if frame.raw(index).expect("validated index") > 0x7fff {
                return Err(ElrsFrameError::InvalidChannelValue);
            }
        }
        Ok(frame)
    }

    pub const fn len(&self) -> usize {
        self.count
    }

    pub const fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn raw(&self, index: usize) -> Option<u16> {
        if index >= self.count {
            return None;
        }
        let offset = index * 2;
        Some(u16::from_le_bytes([
            self.payload[offset],
            self.payload[offset + 1],
        ]))
    }

    /// Maps a channel to `-1000..=1000`, applying a deadband expressed in
    /// the same per-mille units and rescaling the remainder to the full range.
    pub fn bipolar(&self, index: usize, deadband: i16) -> Option<i16> {
        let raw = self.raw(index)? as i32;
        let mapped = (raw * 2_000 / 0x7fff) - 1_000;
        Some(apply_deadband(mapped as i16, deadband))
    }
}

pub fn apply_deadband(value: i16, deadband: i16) -> i16 {
    let deadband = deadband.clamp(0, 999) as i32;
    let value = value.clamp(COMMAND_MIN, COMMAND_MAX) as i32;
    let magnitude = value.abs();
    if magnitude <= deadband {
        return 0;
    }

    let rescaled = (magnitude - deadband) * 1_000 / (1_000 - deadband);
    (rescaled * value.signum()) as i16
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationError {
    InvalidRange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelCalibration {
    pub minimum: u16,
    pub neutral: u16,
    pub maximum: u16,
    pub inverted: bool,
}

impl ChannelCalibration {
    /// Maps a stick used as a gas pedal to `0..=1000`.
    pub fn unipolar(self, raw: u16) -> Result<i16, CalibrationError> {
        if self.minimum >= self.maximum {
            return Err(CalibrationError::InvalidRange);
        }
        let raw = raw.clamp(self.minimum, self.maximum) as i32;
        let minimum = self.minimum as i32;
        let span = (self.maximum - self.minimum) as i32;
        let value = (raw - minimum) * 1_000 / span;
        Ok(if self.inverted { 1_000 - value } else { value } as i16)
    }

    /// Maps a centered axis to `-1000..=1000`, preserving asymmetric travel.
    pub fn bipolar(self, raw: u16, deadband: i16) -> Result<i16, CalibrationError> {
        if !(self.minimum < self.neutral && self.neutral < self.maximum) {
            return Err(CalibrationError::InvalidRange);
        }
        let raw = raw.clamp(self.minimum, self.maximum);
        let mapped = if raw <= self.neutral {
            -((self.neutral - raw) as i32 * 1_000 / (self.neutral - self.minimum) as i32)
        } else {
            (raw - self.neutral) as i32 * 1_000 / (self.maximum - self.neutral) as i32
        };
        let mapped = if self.inverted { -mapped } else { mapped };
        Ok(apply_deadband(mapped as i16, deadband))
    }

    /// Decodes a three-position direction switch independently of arming.
    pub fn direction(self, raw: u16) -> Result<DriveDirection, CalibrationError> {
        self.direction_with_deadband(raw, DEFAULT_SWITCH_DEADBAND)
    }

    pub fn direction_with_deadband(
        self,
        raw: u16,
        deadband: u16,
    ) -> Result<DriveDirection, CalibrationError> {
        if self.minimum >= self.maximum {
            return Err(CalibrationError::InvalidRange);
        }
        let raw = raw.clamp(self.minimum, self.maximum);
        let (low_threshold, high_threshold) =
            switch_thresholds(self.minimum, self.maximum, deadband);
        let direction = if raw < low_threshold {
            DriveDirection::Reverse
        } else if raw > high_threshold {
            DriveDirection::Forward
        } else {
            DriveDirection::Neutral
        };
        Ok(if self.inverted {
            direction.reversed()
        } else {
            direction
        })
    }

    /// Decodes a dedicated three-position ARM switch fail-safe.
    ///
    /// Only the configured extreme position arms. The middle position and the
    /// opposite extreme both disarm, so a single ambiguous value cannot arm.
    pub fn arm(self, raw: u16) -> Result<ArmState, CalibrationError> {
        self.arm_with_deadband(raw, DEFAULT_SWITCH_DEADBAND)
    }

    pub fn arm_with_deadband(self, raw: u16, deadband: u16) -> Result<ArmState, CalibrationError> {
        if self.minimum >= self.maximum {
            return Err(CalibrationError::InvalidRange);
        }
        let raw = raw.clamp(self.minimum, self.maximum);
        let (low_threshold, high_threshold) =
            switch_thresholds(self.minimum, self.maximum, deadband);
        let armed = if self.inverted {
            raw < low_threshold
        } else {
            raw > high_threshold
        };
        Ok(if armed {
            ArmState::Armed
        } else {
            ArmState::Disarmed
        })
    }

    /// Selects the alternate control profile only at one switch extreme.
    ///
    /// A three-position switch therefore keeps its middle position on the
    /// pedal profile. A two-position switch works as expected at its two
    /// endpoints. This mirrors the fail-safe ARM decoder: an ambiguous value
    /// can never unexpectedly select the alternate profile.
    pub fn control_profile(self, raw: u16) -> Result<ControlProfile, CalibrationError> {
        self.control_profile_with_deadband(raw, DEFAULT_SWITCH_DEADBAND)
    }

    pub fn control_profile_with_deadband(
        self,
        raw: u16,
        deadband: u16,
    ) -> Result<ControlProfile, CalibrationError> {
        if self.minimum >= self.maximum {
            return Err(CalibrationError::InvalidRange);
        }
        let raw = raw.clamp(self.minimum, self.maximum);
        let (low_threshold, high_threshold) =
            switch_thresholds(self.minimum, self.maximum, deadband);
        let joystick = if self.inverted {
            raw < low_threshold
        } else {
            raw > high_threshold
        };
        Ok(if joystick {
            ControlProfile::RightStick
        } else {
            ControlProfile::Pedal
        })
    }
}

fn switch_thresholds(minimum: u16, maximum: u16, deadband: u16) -> (u16, u16) {
    let minimum = minimum as u32;
    let span = maximum as u32 - minimum;
    let midpoint = minimum + span / 2;
    let offset = span * deadband.min(499) as u32 / 1_000;
    ((midpoint - offset) as u16, (midpoint + offset) as u16)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveDirection {
    Reverse,
    Neutral,
    Forward,
}

impl DriveDirection {
    pub const fn reversed(self) -> Self {
        match self {
            Self::Reverse => Self::Forward,
            Self::Neutral => Self::Neutral,
            Self::Forward => Self::Reverse,
        }
    }

    pub const fn sign(self) -> i16 {
        match self {
            Self::Reverse => -1,
            Self::Neutral => 0,
            Self::Forward => 1,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmState {
    Disarmed,
    Armed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlProfile {
    Pedal,
    RightStick,
}

impl ArmState {
    pub const fn is_armed(self) -> bool {
        matches!(self, Self::Armed)
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct WheelSpeeds {
    pub front_left: i16,
    pub front_right: i16,
    pub rear_left: i16,
    pub rear_right: i16,
}

impl WheelSpeeds {
    pub const STOPPED: Self = Self {
        front_left: 0,
        front_right: 0,
        rear_left: 0,
        rear_right: 0,
    };

    /// Differential, car-like motion. Steering changes the inside/outside
    /// wheel speeds proportionally to throttle, so steering at zero throttle
    /// cannot rotate the robot in place.
    pub fn car_like(throttle: i16, steering: i16) -> Self {
        let throttle = throttle.clamp(COMMAND_MIN, COMMAND_MAX) as i32;
        let steering = steering.clamp(COMMAND_MIN, COMMAND_MAX) as i32
            * CAR_STEERING_AUTHORITY as i32
            / COMMAND_MAX as i32;
        let left = throttle * (1_000 + steering) / 1_000;
        let right = throttle * (1_000 - steering) / 1_000;
        let (left, right) = normalize_pair(left, right);

        Self {
            front_left: left,
            rear_left: left,
            front_right: right,
            rear_right: right,
        }
    }

    /// Standard four-wheel mecanum inverse kinematics. Motor polarity and
    /// wheel orientation are calibrated in the motor-controller firmware.
    pub fn mecanum(forward: i16, strafe: i16, turn: i16) -> Self {
        let forward = forward.clamp(COMMAND_MIN, COMMAND_MAX) as i32;
        let strafe = strafe.clamp(COMMAND_MIN, COMMAND_MAX) as i32;
        let turn = turn.clamp(COMMAND_MIN, COMMAND_MAX) as i32;
        normalize_four([
            forward + strafe + turn,
            forward - strafe - turn,
            forward - strafe + turn,
            forward + strafe - turn,
        ])
    }
}

fn normalize_pair(left: i32, right: i32) -> (i16, i16) {
    let peak = left.abs().max(right.abs()).max(1_000);
    ((left * 1_000 / peak) as i16, (right * 1_000 / peak) as i16)
}

fn normalize_four(values: [i32; 4]) -> WheelSpeeds {
    let peak = values
        .iter()
        .map(|value| value.abs())
        .max()
        .unwrap_or(1_000)
        .max(1_000);
    let scale = |value: i32| (value * 1_000 / peak) as i16;
    WheelSpeeds {
        front_left: scale(values[0]),
        front_right: scale(values[1]),
        rear_left: scale(values[2]),
        rear_right: scale(values[3]),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlewLimiter {
    value: i16,
    max_step: i16,
}

impl SlewLimiter {
    pub fn new(max_step: i16) -> Self {
        Self {
            value: 0,
            max_step: max_step.clamp(1, COMMAND_MAX),
        }
    }

    pub fn update(&mut self, target: i16) -> i16 {
        let target = target.clamp(COMMAND_MIN, COMMAND_MAX);
        let delta = (target as i32 - self.value as i32)
            .clamp(-(self.max_step as i32), self.max_step as i32);
        self.value += delta as i16;
        self.value
    }

    pub fn reset(&mut self) {
        self.value = 0;
    }
}

pub const ARM_IDLE_THRESHOLD: i16 = 50;
pub const STRAFE_ENTER_THRESHOLD: i16 = 180;
pub const STRAFE_EXIT_THRESHOLD: i16 = 100;
pub const AXIS_DOMINANCE_MARGIN: i16 = 100;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriveInput {
    pub throttle: i16,
    pub steering: i16,
    pub strafe: i16,
    pub direction: DriveDirection,
    pub arm: ArmState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveMode {
    Stopped,
    CarLike,
    Strafe,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveStopReason {
    InputUnavailable,
    Disarmed,
    AwaitingDisarm,
    ControlsNotIdle,
    Neutral,
    ModeChange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriveOutput {
    pub armed: bool,
    pub wheels: WheelSpeeds,
    pub mode: DriveMode,
    pub stop_reason: Option<DriveStopReason>,
}

impl DriveOutput {
    const fn stopped(armed: bool, reason: DriveStopReason) -> Self {
        Self {
            armed,
            wheels: WheelSpeeds::STOPPED,
            mode: DriveMode::Stopped,
            stop_reason: Some(reason),
        }
    }
}

/// Safety state and motion shaping for the two-PWM TSCINBUNY motor HAT.
///
/// The board can independently reverse four motors, but the two motors on
/// each side share one PWM duty. Consequently car-like arcs and pure mecanum
/// strafing are representable, while arbitrary forward/strafe blending is
/// not. A one-cycle stop separates those two modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HatDriveController {
    saw_disarm: bool,
    armed: bool,
    control_profile: Option<ControlProfile>,
    mode: DriveMode,
    left: SlewLimiter,
    right: SlewLimiter,
    strafe: SlewLimiter,
}

impl HatDriveController {
    pub fn new(max_step: i16) -> Self {
        Self {
            saw_disarm: false,
            armed: false,
            control_profile: None,
            mode: DriveMode::Stopped,
            left: SlewLimiter::new(max_step),
            right: SlewLimiter::new(max_step),
            strafe: SlewLimiter::new(max_step),
        }
    }

    /// Changes the radio control profile and invalidates the ARM latch.
    /// Returning to motion always requires a fresh physical DISARM observation.
    pub fn select_control_profile(&mut self, profile: ControlProfile) {
        if self.control_profile != Some(profile) {
            self.control_profile = Some(profile);
            self.require_disarm();
        }
    }

    pub fn update(&mut self, input: Option<DriveInput>) -> DriveOutput {
        let Some(input) = input else {
            self.require_disarm();
            return DriveOutput::stopped(false, DriveStopReason::InputUnavailable);
        };

        if !input.arm.is_armed() {
            self.saw_disarm = true;
            self.armed = false;
            self.stop_motion();
            return DriveOutput::stopped(false, DriveStopReason::Disarmed);
        }

        if !self.saw_disarm {
            self.armed = false;
            self.stop_motion();
            return DriveOutput::stopped(false, DriveStopReason::AwaitingDisarm);
        }

        if !self.armed {
            if input.throttle.abs() > ARM_IDLE_THRESHOLD || input.strafe != 0 {
                self.stop_motion();
                return DriveOutput::stopped(false, DriveStopReason::ControlsNotIdle);
            }
            self.armed = true;
        }

        if input.direction == DriveDirection::Neutral {
            self.stop_motion();
            return DriveOutput::stopped(true, DriveStopReason::Neutral);
        }

        let strafe = input.strafe.abs();
        let throttle = input.throttle.abs();
        let wanted_mode = if (self.mode == DriveMode::Strafe
            && strafe >= STRAFE_EXIT_THRESHOLD
            && strafe >= throttle)
            || (strafe >= STRAFE_ENTER_THRESHOLD
                && strafe as i32 >= throttle as i32 + AXIS_DOMINANCE_MARGIN as i32)
        {
            DriveMode::Strafe
        } else {
            DriveMode::CarLike
        };
        if self.mode != wanted_mode {
            self.stop_motion();
            self.mode = wanted_mode;
            return DriveOutput::stopped(true, DriveStopReason::ModeChange);
        }

        let wheels = match wanted_mode {
            DriveMode::CarLike => {
                let target =
                    WheelSpeeds::car_like(input.throttle * input.direction.sign(), input.steering);
                let left = self.left.update(target.front_left);
                let right = self.right.update(target.front_right);
                WheelSpeeds {
                    front_left: left,
                    rear_left: left,
                    front_right: right,
                    rear_right: right,
                }
            }
            DriveMode::Strafe => {
                let speed = self.strafe.update(input.strafe);
                WheelSpeeds::mecanum(0, speed, 0)
            }
            DriveMode::Stopped => WheelSpeeds::STOPPED,
        };

        DriveOutput {
            armed: true,
            wheels,
            mode: wanted_mode,
            stop_reason: None,
        }
    }

    /// Invalidates the arm latch after startup, radio loss, or maintenance.
    pub fn require_disarm(&mut self) {
        self.saw_disarm = false;
        self.armed = false;
        self.stop_motion();
    }

    fn stop_motion(&mut self) {
        self.mode = DriveMode::Stopped;
        self.left.reset();
        self.right.reset();
        self.strafe.reset();
    }
}

pub const MOTOR_FRAME_LEN: usize = 14;
pub const MOTOR_FRAME_START: u8 = 0xa5;
pub const MOTOR_FRAME_END: u8 = 0x5a;
pub const MOTOR_PROTOCOL_VERSION: u8 = 1;
pub const FLAG_ARMED: u8 = 1 << 0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorCommand {
    pub sequence: u8,
    pub flags: u8,
    pub wheels: WheelSpeeds,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorFrameError {
    WrongBoundary,
    UnsupportedVersion,
    BadChecksum,
    OutOfRange,
}

impl MotorCommand {
    pub fn encode(self) -> [u8; MOTOR_FRAME_LEN] {
        let mut frame = [0_u8; MOTOR_FRAME_LEN];
        frame[0] = MOTOR_FRAME_START;
        frame[1] = MOTOR_PROTOCOL_VERSION;
        frame[2] = self.sequence;
        frame[3] = self.flags;
        for (index, speed) in [
            self.wheels.front_left,
            self.wheels.front_right,
            self.wheels.rear_left,
            self.wheels.rear_right,
        ]
        .iter()
        .enumerate()
        {
            let bytes = speed.to_le_bytes();
            frame[4 + index * 2] = bytes[0];
            frame[5 + index * 2] = bytes[1];
        }
        frame[12] = crc8(&frame[1..12]);
        frame[13] = MOTOR_FRAME_END;
        frame
    }

    pub fn decode(frame: &[u8; MOTOR_FRAME_LEN]) -> Result<Self, MotorFrameError> {
        if frame[0] != MOTOR_FRAME_START || frame[13] != MOTOR_FRAME_END {
            return Err(MotorFrameError::WrongBoundary);
        }
        if frame[1] != MOTOR_PROTOCOL_VERSION {
            return Err(MotorFrameError::UnsupportedVersion);
        }
        if crc8(&frame[1..12]) != frame[12] {
            return Err(MotorFrameError::BadChecksum);
        }

        let read_speed = |offset: usize| i16::from_le_bytes([frame[offset], frame[offset + 1]]);
        let wheels = WheelSpeeds {
            front_left: read_speed(4),
            front_right: read_speed(6),
            rear_left: read_speed(8),
            rear_right: read_speed(10),
        };
        if [
            wheels.front_left,
            wheels.front_right,
            wheels.rear_left,
            wheels.rear_right,
        ]
        .iter()
        .any(|speed| !(COMMAND_MIN..=COMMAND_MAX).contains(speed))
        {
            return Err(MotorFrameError::OutOfRange);
        }

        Ok(Self {
            sequence: frame[2],
            flags: frame[3],
            wheels,
        })
    }
}

/// CRC-8/ATM (polynomial 0x07, initial value 0x00).
pub fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0_u8;
    for byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_elrs_wifi_channels() {
        let packet = [1, 3, 0, 0, 0, 0x40, 0xff, 0x7f];
        let channels = ElrsChannels::parse(&packet).unwrap();
        assert_eq!(channels.len(), 3);
        assert_eq!(channels.raw(0), Some(0));
        assert_eq!(channels.raw(1), Some(0x4000));
        assert_eq!(channels.raw(2), Some(0x7fff));
        assert_eq!(channels.bipolar(0, 0), Some(-1_000));
        assert_eq!(channels.bipolar(1, 20), Some(0));
        assert_eq!(channels.bipolar(2, 0), Some(1_000));
    }

    #[test]
    fn calibrated_throttle_is_a_unipolar_gas_pedal() {
        let channel = ChannelCalibration {
            minimum: 1_000,
            neutral: 1_000,
            maximum: 2_000,
            inverted: false,
        };
        assert_eq!(channel.unipolar(1_000), Ok(0));
        assert_eq!(channel.unipolar(1_500), Ok(500));
        assert_eq!(channel.unipolar(2_000), Ok(1_000));
        assert_eq!(
            ChannelCalibration {
                inverted: true,
                ..channel
            }
            .unipolar(1_000),
            Ok(1_000)
        );
    }

    #[test]
    fn calibrated_centered_axis_preserves_asymmetric_travel() {
        let channel = ChannelCalibration {
            minimum: 1_000,
            neutral: 1_400,
            maximum: 2_000,
            inverted: false,
        };
        assert_eq!(channel.bipolar(1_000, 0), Ok(-1_000));
        assert_eq!(channel.bipolar(1_400, 35), Ok(0));
        assert_eq!(channel.bipolar(2_000, 0), Ok(1_000));
    }

    #[test]
    fn direction_switch_has_an_independent_neutral() {
        let channel = ChannelCalibration {
            minimum: 1_000,
            neutral: 1_500,
            maximum: 2_000,
            inverted: false,
        };
        assert_eq!(channel.direction(1_000), Ok(DriveDirection::Reverse));
        assert_eq!(channel.direction(1_500), Ok(DriveDirection::Neutral));
        assert_eq!(channel.direction(2_000), Ok(DriveDirection::Forward));

        // The resting position captured by the wizard must not define the
        // direction dead zone; only the measured endpoints do.
        let resting_at_minimum = ChannelCalibration {
            neutral: 1_000,
            ..channel
        };
        assert_eq!(
            resting_at_minimum.direction(1_000),
            Ok(DriveDirection::Reverse)
        );
        assert_eq!(
            resting_at_minimum.direction(1_500),
            Ok(DriveDirection::Neutral)
        );
        assert_eq!(
            resting_at_minimum.direction(2_000),
            Ok(DriveDirection::Forward)
        );
    }

    #[test]
    fn dedicated_arm_switch_only_arms_at_one_extreme() {
        let channel = ChannelCalibration {
            minimum: 1_000,
            neutral: 1_500,
            maximum: 2_000,
            inverted: false,
        };
        assert_eq!(channel.arm(1_000), Ok(ArmState::Disarmed));
        assert_eq!(channel.arm(1_500), Ok(ArmState::Disarmed));
        assert_eq!(channel.arm(2_000), Ok(ArmState::Armed));
        assert_eq!(
            ChannelCalibration {
                inverted: true,
                ..channel
            }
            .arm(1_000),
            Ok(ArmState::Armed)
        );

        // Capturing a safe resting position at an extreme must not invalidate
        // ARM calibration; the middle value is irrelevant to this decoder.
        let resting_at_minimum = ChannelCalibration {
            neutral: 1_000,
            ..channel
        };
        assert_eq!(resting_at_minimum.arm(1_000), Ok(ArmState::Disarmed));
        assert_eq!(resting_at_minimum.arm(1_500), Ok(ArmState::Disarmed));
        assert_eq!(resting_at_minimum.arm(2_000), Ok(ArmState::Armed));
    }

    #[test]
    fn control_profile_switch_uses_only_one_extreme_for_right_stick() {
        let channel = ChannelCalibration {
            minimum: 1_000,
            neutral: 1_500,
            maximum: 2_000,
            inverted: false,
        };
        assert_eq!(channel.control_profile(1_000), Ok(ControlProfile::Pedal));
        assert_eq!(channel.control_profile(1_500), Ok(ControlProfile::Pedal));
        assert_eq!(
            channel.control_profile(2_000),
            Ok(ControlProfile::RightStick)
        );
        assert_eq!(
            ChannelCalibration {
                inverted: true,
                ..channel
            }
            .control_profile(1_000),
            Ok(ControlProfile::RightStick)
        );
    }

    #[test]
    fn switch_deadband_controls_the_safe_middle_zone() {
        let channel = ChannelCalibration {
            minimum: 1_000,
            neutral: 1_500,
            maximum: 2_000,
            inverted: false,
        };

        assert_eq!(
            channel.direction_with_deadband(1_650, 250),
            Ok(DriveDirection::Neutral)
        );
        assert_eq!(
            channel.direction_with_deadband(1_650, 100),
            Ok(DriveDirection::Forward)
        );
        assert_eq!(
            channel.arm_with_deadband(1_650, 250),
            Ok(ArmState::Disarmed)
        );
        assert_eq!(channel.arm_with_deadband(1_650, 100), Ok(ArmState::Armed));
        assert_eq!(
            channel.control_profile_with_deadband(1_650, 250),
            Ok(ControlProfile::Pedal)
        );
        assert_eq!(
            channel.control_profile_with_deadband(1_650, 100),
            Ok(ControlProfile::RightStick)
        );
    }

    #[test]
    fn full_elrs_range_keeps_switch_center_safe() {
        let channel = ChannelCalibration {
            minimum: 19,
            neutral: 16_393,
            maximum: 32_767,
            inverted: false,
        };

        assert_eq!(channel.direction(16_393), Ok(DriveDirection::Neutral));
        assert_eq!(channel.arm(16_393), Ok(ArmState::Disarmed));
        assert_eq!(channel.direction(19), Ok(DriveDirection::Reverse));
        assert_eq!(channel.direction(32_767), Ok(DriveDirection::Forward));
        assert_eq!(channel.arm(32_767), Ok(ArmState::Armed));
    }

    #[test]
    fn rejects_malformed_elrs_frames() {
        assert_eq!(
            ElrsChannels::parse(&[2, 1, 0, 0]).unwrap_err(),
            ElrsFrameError::WrongFrameType
        );
        assert_eq!(
            ElrsChannels::parse(&[1, 2, 0, 0]).unwrap_err(),
            ElrsFrameError::WrongLength
        );
        assert_eq!(
            ElrsChannels::parse(&[1, 1, 0xff, 0xff]).unwrap_err(),
            ElrsFrameError::InvalidChannelValue
        );
    }

    #[test]
    fn car_like_steering_cannot_pivot() {
        assert_eq!(WheelSpeeds::car_like(0, 1_000), WheelSpeeds::STOPPED);
        assert_eq!(
            WheelSpeeds::car_like(500, 1_000),
            WheelSpeeds {
                front_left: 850,
                rear_left: 850,
                front_right: 150,
                rear_right: 150,
            }
        );
    }

    #[test]
    fn full_car_like_steering_keeps_both_sides_moving() {
        let wheels = WheelSpeeds::car_like(500, 1_000);
        assert!(wheels.front_left > 0);
        assert!(wheels.front_right > 0);
        assert_eq!(wheels.front_left, wheels.rear_left);
        assert_eq!(wheels.front_right, wheels.rear_right);
    }

    #[test]
    fn mecanum_output_is_normalized() {
        let wheels = WheelSpeeds::mecanum(1_000, 1_000, 1_000);
        assert_eq!(wheels.front_left, 1_000);
        assert!(wheels.front_right.abs() <= 1_000);
        assert!(wheels.rear_left.abs() <= 1_000);
        assert!(wheels.rear_right.abs() <= 1_000);
    }

    #[test]
    fn slew_limiter_bounds_each_update() {
        let mut limiter = SlewLimiter::new(75);
        assert_eq!(limiter.update(1_000), 75);
        assert_eq!(limiter.update(1_000), 150);
        assert_eq!(limiter.update(-1_000), 75);
    }

    fn drive_input(
        arm: ArmState,
        direction: DriveDirection,
        throttle: i16,
        strafe: i16,
    ) -> DriveInput {
        DriveInput {
            throttle,
            steering: 0,
            strafe,
            direction,
            arm,
        }
    }

    #[test]
    fn drive_requires_disarm_after_boot_and_radio_loss() {
        let mut drive = HatDriveController::new(100);
        let armed = drive_input(ArmState::Armed, DriveDirection::Forward, 0, 0);
        assert_eq!(
            drive.update(Some(armed)).stop_reason,
            Some(DriveStopReason::AwaitingDisarm)
        );

        let disarmed = drive_input(ArmState::Disarmed, DriveDirection::Forward, 0, 0);
        assert_eq!(
            drive.update(Some(disarmed)).stop_reason,
            Some(DriveStopReason::Disarmed)
        );
        assert!(drive.update(Some(armed)).armed);

        assert_eq!(
            drive.update(None).stop_reason,
            Some(DriveStopReason::InputUnavailable)
        );
        assert_eq!(
            drive.update(Some(armed)).stop_reason,
            Some(DriveStopReason::AwaitingDisarm)
        );
    }

    #[test]
    fn drive_refuses_to_arm_with_non_idle_controls() {
        let mut drive = HatDriveController::new(100);
        let disarmed = drive_input(ArmState::Disarmed, DriveDirection::Forward, 0, 0);
        drive.update(Some(disarmed));

        let unsafe_arm = drive_input(ArmState::Armed, DriveDirection::Forward, 500, 0);
        assert_eq!(
            drive.update(Some(unsafe_arm)).stop_reason,
            Some(DriveStopReason::ControlsNotIdle)
        );

        let safe_arm = drive_input(ArmState::Armed, DriveDirection::Forward, 0, 0);
        assert!(drive.update(Some(safe_arm)).armed);
    }

    #[test]
    fn changing_control_profile_requires_a_fresh_disarm() {
        let mut drive = HatDriveController::new(100);
        drive.select_control_profile(ControlProfile::Pedal);
        let disarmed = drive_input(ArmState::Disarmed, DriveDirection::Forward, 0, 0);
        let armed = drive_input(ArmState::Armed, DriveDirection::Forward, 0, 0);
        drive.update(Some(disarmed));
        assert!(drive.update(Some(armed)).armed);

        drive.select_control_profile(ControlProfile::RightStick);
        assert_eq!(
            drive.update(Some(armed)).stop_reason,
            Some(DriveStopReason::AwaitingDisarm)
        );
        drive.update(Some(disarmed));
        assert!(drive.update(Some(armed)).armed);
    }

    #[test]
    fn hat_drive_outputs_only_representable_motor_patterns() {
        let mut drive = HatDriveController::new(100);
        drive.update(Some(drive_input(
            ArmState::Disarmed,
            DriveDirection::Forward,
            0,
            0,
        )));
        drive.update(Some(drive_input(
            ArmState::Armed,
            DriveDirection::Forward,
            0,
            0,
        )));

        // First tick selects car mode; the next one ramps its two PWM duties.
        drive.update(Some(drive_input(
            ArmState::Armed,
            DriveDirection::Forward,
            500,
            0,
        )));
        let car = drive.update(Some(drive_input(
            ArmState::Armed,
            DriveDirection::Forward,
            500,
            0,
        )));
        assert_eq!(car.mode, DriveMode::CarLike);
        assert_eq!(car.wheels.front_left, car.wheels.rear_left);
        assert_eq!(car.wheels.front_right, car.wheels.rear_right);

        // Switching to strafe inserts a full stop, then ramps equal magnitudes.
        let switching = drive.update(Some(drive_input(
            ArmState::Armed,
            DriveDirection::Forward,
            0,
            500,
        )));
        assert_eq!(switching.wheels, WheelSpeeds::STOPPED);
        let strafe = drive.update(Some(drive_input(
            ArmState::Armed,
            DriveDirection::Forward,
            0,
            500,
        )));
        assert_eq!(strafe.mode, DriveMode::Strafe);
        assert_eq!(
            strafe.wheels.front_left.abs(),
            strafe.wheels.rear_left.abs()
        );
        assert_eq!(
            strafe.wheels.front_right.abs(),
            strafe.wheels.rear_right.abs()
        );
    }

    #[test]
    fn incidental_strafe_does_not_interrupt_car_like_motion() {
        let mut drive = HatDriveController::new(100);
        drive.update(Some(drive_input(
            ArmState::Disarmed,
            DriveDirection::Forward,
            0,
            0,
        )));
        drive.update(Some(drive_input(
            ArmState::Armed,
            DriveDirection::Forward,
            0,
            0,
        )));
        drive.update(Some(drive_input(
            ArmState::Armed,
            DriveDirection::Forward,
            500,
            0,
        )));

        let output = drive.update(Some(drive_input(
            ArmState::Armed,
            DriveDirection::Forward,
            500,
            200,
        )));
        assert_eq!(output.mode, DriveMode::CarLike);
        assert_eq!(output.stop_reason, None);
    }

    #[test]
    fn motor_command_round_trips_and_detects_corruption() {
        let command = MotorCommand {
            sequence: 42,
            flags: FLAG_ARMED,
            wheels: WheelSpeeds::mecanum(400, -200, 100),
        };
        let mut encoded = command.encode();
        assert_eq!(MotorCommand::decode(&encoded), Ok(command));

        encoded[6] ^= 0x80;
        assert_eq!(
            MotorCommand::decode(&encoded),
            Err(MotorFrameError::BadChecksum)
        );
    }
}
