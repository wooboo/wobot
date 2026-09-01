use std::{fmt, io::Read};

use esp_idf_svc::{ota::EspOta, sys::EspError};

pub const ESP32_OTA_SLOT_SIZE: usize = 0x40_0000;
pub const ARDUINO_APPLICATION_LIMIT: usize = 0x7e00;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdateTarget {
    Esp32,
    Arduino,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdateStatus {
    Idle,
    Installing {
        target: UpdateTarget,
        written: usize,
        total: usize,
    },
    PendingReboot,
    Failed,
}

#[derive(Debug)]
pub enum UpdateError {
    RobotArmed,
    EmptyImage,
    ImageTooLarge { actual: usize, limit: usize },
    Truncated { expected: usize, actual: usize },
    TrailingData,
    Read(std::io::Error),
    Esp(EspError),
}

impl fmt::Display for UpdateError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for UpdateError {}

impl From<EspError> for UpdateError {
    fn from(error: EspError) -> Self {
        Self::Esp(error)
    }
}

/// Owns the ESP-IDF OTA state and keeps transport code independent from slots.
///
/// HTTP, a local CLI, or a future pull updater only provide a validated payload
/// reader. Dropping an unfinished ESP-IDF update automatically aborts it.
pub struct UpdateManager {
    ota: EspOta,
    status: UpdateStatus,
    robot_armed: bool,
}

impl UpdateManager {
    pub fn new() -> Result<Self, UpdateError> {
        Ok(Self {
            ota: EspOta::new()?,
            status: UpdateStatus::Idle,
            robot_armed: false,
        })
    }

    pub const fn status(&self) -> UpdateStatus {
        self.status
    }

    pub fn set_robot_armed(&mut self, armed: bool) {
        self.robot_armed = armed;
    }

    /// Streams one ESP-IDF application image into the inactive OTA slot.
    ///
    /// This expects an application image, not a merged 4 MiB flash dump.
    pub fn install_esp32<R: Read>(
        &mut self,
        payload_len: usize,
        payload: &mut R,
    ) -> Result<(), UpdateError> {
        if self.robot_armed {
            return Err(UpdateError::RobotArmed);
        }
        if payload_len == 0 {
            return Err(UpdateError::EmptyImage);
        }
        if payload_len > ESP32_OTA_SLOT_SIZE {
            return Err(UpdateError::ImageTooLarge {
                actual: payload_len,
                limit: ESP32_OTA_SLOT_SIZE,
            });
        }

        self.status = UpdateStatus::Installing {
            target: UpdateTarget::Esp32,
            written: 0,
            total: payload_len,
        };

        let result = self.install_esp32_inner(payload_len, payload);
        self.status = if result.is_ok() {
            UpdateStatus::PendingReboot
        } else {
            UpdateStatus::Failed
        };
        result
    }

    fn install_esp32_inner<R: Read>(
        &mut self,
        payload_len: usize,
        payload: &mut R,
    ) -> Result<(), UpdateError> {
        let mut update = self.ota.initiate_update_with_known_size(payload_len)?;
        let mut buffer = [0_u8; 4096];
        let mut written = 0;

        while written < payload_len {
            let wanted = buffer.len().min(payload_len - written);
            let count = payload
                .read(&mut buffer[..wanted])
                .map_err(UpdateError::Read)?;
            if count == 0 {
                return Err(UpdateError::Truncated {
                    expected: payload_len,
                    actual: written,
                });
            }

            update.write(&buffer[..count])?;
            written += count;
            self.status = UpdateStatus::Installing {
                target: UpdateTarget::Esp32,
                written,
                total: payload_len,
            };
        }

        if payload.read(&mut buffer[..1]).map_err(UpdateError::Read)? != 0 {
            return Err(UpdateError::TrailingData);
        }

        update.finish()?.activate()?;
        Ok(())
    }

    /// Call only after the new firmware has passed its startup self-test.
    pub fn confirm_running_image(&mut self) -> Result<(), UpdateError> {
        self.ota.mark_running_slot_valid()?;
        self.status = UpdateStatus::Idle;
        Ok(())
    }
}
