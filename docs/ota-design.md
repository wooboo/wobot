# OTA design

## Goal

One upload path updates either processor without exposing ESP-IDF partition
details or the AVR STK500v1 protocol to HTTP, CLI, or future cloud callers.

## Module and interface

`UpdateManager` is the deep module. Its external interface is intentionally
small:

```rust,ignore
fn install<R: Read>(manifest: &FirmwareManifest, payload: &mut R)
    -> Result<InstallOutcome, UpdateError>;
fn status() -> UpdateStatus;
```

The manifest selects `Esp32` or `Arduino`, carries the payload length, version,
SHA-256 digest, and Ed25519 signature. The implementation owns validation,
staging, progress, retries, boot selection, and rollback. HTTP upload and a
future pull-based updater call the same interface.

## Common package

Use one streamable `.wobotfw` format:

1. fixed magic and format version;
2. target (`esp32` or `arduino`);
3. semantic firmware version;
4. payload length;
5. SHA-256 payload digest;
6. Ed25519 signature over the header fields and payload digest;
7. raw target payload.

An ESP32 payload is an ESP-IDF application image, not a merged 4 MB flash
image. An Arduino payload is the application region only and must not overlap
the Optiboot bootloader at `0x7e00..0x7fff`.

## ESP32 adapter

- Write the stream to the inactive OTA slot through ESP-IDF OTA operations.
- Validate the image before selecting it for the next boot.
- Enable bootloader rollback.
- On first boot, perform a short self-test: NVS access, Wi-Fi initialization,
  UART initialization, and control task startup.
- Mark the image valid only after the self-test. A crash, reset, or failed test
  rolls back to the previous slot.

The 4 MB layout gives each application slot 1.5 MB and reserves staging space
for the AVR image.

With `esp-idf-sys`, the custom partition table is supplied to `espflash`
during the initial USB installation rather than selected in
`sdkconfig.defaults`. The first installation must flash the rollback-enabled
bootloader, `partitions.csv`, and the application into `ota_0`. Later ESP32 OTA
uploads contain only the application image and alternate between `ota_0` and
`ota_1`.

## Arduino adapter

1. Validate and fully stage the image in ESP32 flash.
2. Stop motor commands and keep the robot disarmed.
3. Assert Arduino RESET from an ESP32 GPIO.
4. Speak Optiboot's STK500v1 subset at 115200 baud.
5. Program 128-byte flash pages below `0x7e00`.
6. Read every page back and compare it with the staged image.
7. Reset Arduino and wait for a valid controller handshake.

The installed UNO bootloader is byte-for-byte compatible with the standard
512-byte Arduino Optiboot image (apart from HEX padding). It deliberately skips
the bootloader after a watchdog reset, so a software-only reset path cannot
recover from a damaged application.

### Required reset circuit

Use a small NPN transistor rather than connecting a 5 V Arduino RESET signal
directly to a 3.3 V ESP32 pin:

```text
ESP32 GPIO13 -- 4.7k -- base  2N3904 / BC547
ESP32 GND  ------------- emitter
Arduino RESET ---------- collector
Arduino GND ------------ ESP32 GND
```

GPIO13 is available when the ESP32-CAM SD-card interface is unused. A 100k
base-to-emitter pulldown is optional and keeps reset inactive during ESP32 boot.

Until this wire exists, a software jump into Optiboot may be used only as a
development convenience. USB remains the recovery route.

## Failure rules

- Never update while motor control is armed.
- Reject unknown targets, oversized images, bad hashes, bad signatures, and
  version downgrades before changing boot selection or AVR flash.
- A disconnected HTTP client aborts the active ESP32 OTA write.
- The AVR image is staged completely before the first AVR page is erased.
- ESP32 rollback is automatic; AVR retry remains possible through hardware
  RESET because Optiboot is never overwritten.
- Only one update may run at a time.

## Delivery stages

1. Initial USB installation establishes the OTA bootloader and partition map.
2. The ESP32 HTTP endpoint streams a signed ESP32 application image into the
   inactive slot through `UpdateManager`.
3. The Arduino endpoint stages a signed AVR image in `avr_ota`, then uses the
   reset circuit and STK500v1 adapter to program and verify it.
4. USB remains the recovery path if the ESP32 bootloader or partition table is
   ever intentionally replaced.
