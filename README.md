# MEMS_MIC_TEST

ESP32-S3 firmware for a custom PCB carrying a PDM MEMS microphone and a
microSD card slot. The goal: record audio to the SD card, triggerable by a
physical button and (working, as of 2026-07-07) Bluetooth LE.

## Status

**Working.** [ESP32_S3_SD_Recorder](ESP32_S3_SD_Recorder/) is the active
firmware — flashed and validated on the bench hardware (last 2026-10-01). Everything
else in this repo is earlier exploratory/diagnostic work kept for reference.

## Directory map

| Path | What it is |
|---|---|
| [ESP32_S3_SD_Recorder/](ESP32_S3_SD_Recorder/) | **Current firmware.** Records PDM mic audio to WAV files on the SD card. Button or BLE triggered. See its own README for full details. |
| [ESP32_S3_USB_Mic/](ESP32_S3_USB_Mic/) | Earlier firmware that exposes the PDM mic as a USB Audio Class (UAC) microphone over native USB, instead of recording to SD. Superseded by the SD recorder for this project's goals, but useful if you need the ESP32 to show up as a Windows recording device again. |
| [SD_Card_SPI_Diagnostic/](SD_Card_SPI_Diagnostic/) | Standalone diagnostic sketch used to bring up and debug the SD card over SPI on this PCB (raw SPI probing, low-level SD command probing, mount-speed sweep). This is what validated the SD wiring on 2026-07-07 before the recorder was built. Re-run this first if the SD card ever stops mounting. |
| [ESP32_devkit_Mems_Test_2.ino](ESP32_devkit_Mems_Test_2.ino) | Earliest PDM mic test: streams raw PCM over Serial (921600 baud) between `REC_START`/`REC_STOP` markers, no SD card involved. Paired with `mictest.py`. |
| [mictest.py](mictest.py) | Python/pyserial companion script for `ESP32_devkit_Mems_Test_2.ino` — waits for the button-press marker, captures raw PCM over serial, writes it to a WAV file on the PC. Not used by the current SD-recorder firmware (which needs no PC in the loop). |
| `button_recording.wav`, `esp32_pdm_test.wav` | Sample recordings captured during early bring-up/testing. |
| [damping_tests/](damping_tests/) | Damping test recordings `REC_00007`–`REC_00034` copied off the SD card, with `SHA256SUMS.txt` of the originals. Timestamps on these are 1980 (no clock was set when they were recorded). |
| [bench_tests/](bench_tests/) | 2026-10-01 bench recordings used to verify capture speed: `REC_00002.WAV` has 1/2/3kHz test tones at 1s/3s/5s. |
| [SD_Card_testing.ino](SD_Card_testing.ino) | Very first basic SPI SD test (from the original GitHub upload); superseded by `SD_Card_SPI_Diagnostic/`. |

## Hardware (custom ESP32-S3 PCB)

Confirmed pinout, shared across the sketches above:

| Signal | GPIO |
|---|---|
| PDM mic clock | 4 |
| PDM mic data | 5 |
| Record button | 35 |
| REC LED | 40 |
| Level-meter LED | 41 |
| SD status LED | 42 |
| SD SPI SCK | 12 |
| SD SPI MOSI | 13 |
| SD SPI MISO | 11 |
| SD SPI CS | 14 |

There is **no dedicated SD card-detect (CD) pin** wired on this board revision
(the SD socket was jury-rigged) — card presence is inferred in software. See
the SD recorder's README for how.

Chip: ESP32-S3 (QFN56, rev v0.2), 8MB embedded PSRAM, native USB (shows up as
`ESP32 Family Device` for flashing). Board profile in Arduino: **ESP32S3 Dev
Module**.

## Build/flash quickstart

All current work is built and flashed via `arduino-cli` (the copy bundled
with Arduino IDE 2 works fine and doesn't require a separate install):

```
"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
```

See [ESP32_S3_SD_Recorder/README.md](ESP32_S3_SD_Recorder/README.md) for the
exact FQBN/board options and commands — they matter here because `CDCOnBoot`
has to be `cdc` for `Serial` output to show up on the native USB port, and the
board needs `PartitionScheme=huge_app` for the BLE stack to fit alongside the
audio buffer pool.

**Gotcha:** if `arduino-cli upload` fails with "port busy / Access is denied"
on Windows, it's usually the Arduino IDE's own background `arduino-cli daemon`
process holding the Serial Monitor connection open — not the IDE window
itself. Find and stop that specific process (`Get-CimInstance Win32_Process
-Filter "Name='arduino-cli.exe'"` in PowerShell) rather than closing the IDE.
