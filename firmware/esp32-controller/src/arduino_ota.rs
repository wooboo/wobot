use std::{
    fmt,
    io::Read,
    thread,
    time::{Duration, Instant},
};

use esp_idf_svc::{
    hal::{delay::TickType, uart::UartDriver},
    partition::EspPartition,
    sys::{EspError, ESP_ERR_TIMEOUT},
};
use wobot_core::MotorCommand;

pub const ARDUINO_APPLICATION_LIMIT: usize = 0x7e00;
const STAGING_PARTITION_SIZE: usize = 0x1_0000;
const FLASH_PAGE_SIZE: usize = 128;
const RESPONSE_TIMEOUT: u64 = 750;
const SYNC_WINDOW: Duration = Duration::from_secs(20);

const STK_OK: u8 = 0x10;
const STK_INSYNC: u8 = 0x14;
const CRC_EOP: u8 = 0x20;

#[derive(Debug)]
pub enum ArduinoOtaError {
    EmptyImage,
    ImageTooLarge {
        actual: usize,
        limit: usize,
    },
    Truncated {
        expected: usize,
        actual: usize,
    },
    TrailingData,
    MissingStagingPartition,
    NoStagedImage,
    VerificationFailed {
        offset: usize,
    },
    BootloaderTimeout {
        attempts: u16,
        bytes_seen: u32,
        last_len: u8,
        last: [u8; 2],
    },
    ResponseTimeout,
    UnexpectedResponse {
        first: u8,
        second: u8,
    },
    Partition(EspError),
    Uart(EspError),
    Read(std::io::Error),
}

impl fmt::Display for ArduinoOtaError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for ArduinoOtaError {}

pub struct ArduinoOta {
    partition: EspPartition,
    uart: UartDriver<'static>,
    staged_len: Option<usize>,
}

impl ArduinoOta {
    pub fn new(uart: UartDriver<'static>) -> Result<Self, ArduinoOtaError> {
        let partition = unsafe { EspPartition::new("avr_ota") }
            .map_err(ArduinoOtaError::Partition)?
            .ok_or(ArduinoOtaError::MissingStagingPartition)?;
        Ok(Self {
            partition,
            uart,
            staged_len: None,
        })
    }

    pub const fn staged_len(&self) -> Option<usize> {
        self.staged_len
    }

    pub fn send_motor_command(&self, command: MotorCommand) -> Result<(), ArduinoOtaError> {
        self.write_all(&command.encode())
    }

    pub fn stage<R: Read>(
        &mut self,
        payload_len: usize,
        payload: &mut R,
    ) -> Result<(), ArduinoOtaError> {
        if payload_len == 0 {
            return Err(ArduinoOtaError::EmptyImage);
        }
        if payload_len > ARDUINO_APPLICATION_LIMIT {
            return Err(ArduinoOtaError::ImageTooLarge {
                actual: payload_len,
                limit: ARDUINO_APPLICATION_LIMIT,
            });
        }

        self.staged_len = None;
        self.partition
            .erase(0, STAGING_PARTITION_SIZE)
            .map_err(ArduinoOtaError::Partition)?;

        let mut buffer = [0_u8; 4096];
        let mut verification = [0_u8; 4096];
        let mut written = 0;
        while written < payload_len {
            let wanted = buffer.len().min(payload_len - written);
            let count = payload
                .read(&mut buffer[..wanted])
                .map_err(ArduinoOtaError::Read)?;
            if count == 0 {
                return Err(ArduinoOtaError::Truncated {
                    expected: payload_len,
                    actual: written,
                });
            }
            self.partition
                .write(written, &buffer[..count])
                .map_err(ArduinoOtaError::Partition)?;
            self.partition
                .read(written, &mut verification[..count])
                .map_err(ArduinoOtaError::Partition)?;
            if verification[..count] != buffer[..count] {
                return Err(ArduinoOtaError::VerificationFailed { offset: written });
            }
            written += count;
        }

        if payload
            .read(&mut buffer[..1])
            .map_err(ArduinoOtaError::Read)?
            != 0
        {
            return Err(ArduinoOtaError::TrailingData);
        }

        self.staged_len = Some(payload_len);
        Ok(())
    }

    /// Waits for a manual Arduino reset, writes every Optiboot page, then reads
    /// every page back before leaving programming mode.
    pub fn flash(&mut self) -> Result<(), ArduinoOtaError> {
        let payload_len = self.staged_len.ok_or(ArduinoOtaError::NoStagedImage)?;
        self.sync()?;

        let mut expected = [0xff_u8; FLASH_PAGE_SIZE];
        let mut actual = [0_u8; FLASH_PAGE_SIZE];
        for offset in (0..payload_len).step_by(FLASH_PAGE_SIZE) {
            expected.fill(0xff);
            let count = FLASH_PAGE_SIZE.min(payload_len - offset);
            self.partition
                .read(offset, &mut expected[..count])
                .map_err(ArduinoOtaError::Partition)?;
            self.load_address(offset)?;
            self.program_page(&expected)?;
        }

        for offset in (0..payload_len).step_by(FLASH_PAGE_SIZE) {
            expected.fill(0xff);
            let count = FLASH_PAGE_SIZE.min(payload_len - offset);
            self.partition
                .read(offset, &mut expected[..count])
                .map_err(ArduinoOtaError::Partition)?;
            self.load_address(offset)?;
            self.read_page(&mut actual)?;
            if actual != expected {
                return Err(ArduinoOtaError::VerificationFailed { offset });
            }
        }

        self.command(&[0x51, CRC_EOP])?;
        thread::sleep(Duration::from_millis(150));
        Ok(())
    }

    fn sync(&mut self) -> Result<(), ArduinoOtaError> {
        let deadline = Instant::now() + SYNC_WINDOW;
        let mut attempts = 0_u16;
        let mut bytes_seen = 0_u32;
        let mut last = [0_u8; 2];
        let mut last_len = 0_u8;
        while Instant::now() < deadline {
            attempts = attempts.saturating_add(1);
            // UART2 on this ESP32-S3 board can report ESP_ERR_TIMEOUT from
            // uart_flush_input even when the RX path is healthy. A stale byte
            // is harmless here: every attempt still requires the exact
            // Optiboot INSYNC/OK pair before programming can start.
            if let Err(error) = self.uart.clear_rx() {
                log::debug!("UART RX flush unavailable while syncing Arduino: {error}");
            }
            self.write_all(&[0x30, CRC_EOP])?;
            let mut response = [0_u8; 2];
            let received = self.read_until(&mut response, 120)?;
            if received > 0 {
                bytes_seen = bytes_seen.saturating_add(received as u32);
                last = response;
                last_len = received as u8;
            }
            if received == response.len() && response == [STK_INSYNC, STK_OK] {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(80));
        }
        Err(ArduinoOtaError::BootloaderTimeout {
            attempts,
            bytes_seen,
            last_len,
            last,
        })
    }

    fn load_address(&mut self, byte_offset: usize) -> Result<(), ArduinoOtaError> {
        let word_address = (byte_offset / 2) as u16;
        let [low, high] = word_address.to_le_bytes();
        self.command(&[0x55, low, high, CRC_EOP])
    }

    fn program_page(&mut self, page: &[u8; FLASH_PAGE_SIZE]) -> Result<(), ArduinoOtaError> {
        let mut command = [0_u8; FLASH_PAGE_SIZE + 5];
        command[..4].copy_from_slice(&[0x64, 0, FLASH_PAGE_SIZE as u8, b'F']);
        command[4..4 + FLASH_PAGE_SIZE].copy_from_slice(page);
        command[FLASH_PAGE_SIZE + 4] = CRC_EOP;
        self.command(&command)
    }

    fn read_page(&mut self, page: &mut [u8; FLASH_PAGE_SIZE]) -> Result<(), ArduinoOtaError> {
        self.write_all(&[0x74, 0, FLASH_PAGE_SIZE as u8, b'F', CRC_EOP])?;
        let mut marker = [0_u8; 1];
        self.read_exact(&mut marker, RESPONSE_TIMEOUT)?;
        if marker[0] != STK_INSYNC {
            return Err(ArduinoOtaError::UnexpectedResponse {
                first: marker[0],
                second: 0,
            });
        }
        self.read_exact(page, RESPONSE_TIMEOUT)?;
        self.read_exact(&mut marker, RESPONSE_TIMEOUT)?;
        if marker[0] != STK_OK {
            return Err(ArduinoOtaError::UnexpectedResponse {
                first: STK_INSYNC,
                second: marker[0],
            });
        }
        Ok(())
    }

    fn command(&mut self, bytes: &[u8]) -> Result<(), ArduinoOtaError> {
        self.write_all(bytes)?;
        let mut response = [0_u8; 2];
        self.read_exact(&mut response, RESPONSE_TIMEOUT)?;
        if response != [STK_INSYNC, STK_OK] {
            return Err(ArduinoOtaError::UnexpectedResponse {
                first: response[0],
                second: response[1],
            });
        }
        Ok(())
    }

    fn write_all(&self, bytes: &[u8]) -> Result<(), ArduinoOtaError> {
        let mut written = 0;
        while written < bytes.len() {
            written += self
                .uart
                .write(&bytes[written..])
                .map_err(ArduinoOtaError::Uart)?;
        }
        // With the software TX ring buffer disabled, `write` blocks while
        // feeding the hardware FIFO. Optiboot cannot answer until it receives
        // CRC_EOP, so the following read is the transmission-completion
        // barrier. Avoid uart_wait_tx_done here: UART2 on this ESP32-S3 board
        // never raises the TX_DONE semaphore and returns ESP_ERR_TIMEOUT.
        Ok(())
    }

    fn read_exact(&self, buffer: &mut [u8], timeout_ms: u64) -> Result<(), ArduinoOtaError> {
        if self.read_until(buffer, timeout_ms)? != buffer.len() {
            return Err(ArduinoOtaError::ResponseTimeout);
        }
        Ok(())
    }

    fn read_until(&self, buffer: &mut [u8], timeout_ms: u64) -> Result<usize, ArduinoOtaError> {
        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
        let mut read = 0;
        while read < buffer.len() {
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            let timeout = deadline.saturating_duration_since(now).as_millis() as u64;
            match self
                .uart
                .read(&mut buffer[read..], TickType::new_millis(timeout).ticks())
            {
                Ok(count) => read += count,
                // A read timeout means "no bytes in this attempt", not a
                // broken UART. The caller owns the overall sync deadline.
                Err(error) if error.code() == ESP_ERR_TIMEOUT => break,
                Err(error) => return Err(ArduinoOtaError::Uart(error)),
            }
        }
        Ok(read)
    }
}
