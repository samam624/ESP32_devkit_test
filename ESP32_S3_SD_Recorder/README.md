# ESP32_S3_SD_Recorder

Records the onboard PDM MEMS microphone to WAV files on the SD card.
Start/stop by pressing the physical button or by sending a command over
Bluetooth LE. Validated working on the bench PCB (2026-07-07; re-validated
2026-10-01). The board has enumerated as COM13 and COM3 on different
machines/sessions — check Device Manager and substitute below.

## What it does

- 48kHz stereo, 16-bit PCM, written as standard WAV files: `/REC_00001.WAV`,
  `/REC_00002.WAV`, ... (auto-incrementing, never overwrites).
- Press the button once to start a new recording, press again to stop.
- Same start/stop trigger available over BLE (see below) — useful for
  triggering from a laptop/phone without touching the board.
- Three LEDs report status at a glance (see **LEDs** below).

## Pinout

| Signal | GPIO |
|---|---|
| PDM mic clock | 4 |
| PDM mic data | 5 |
| Record button | 35 (`INPUT_PULLUP`, active low) |
| REC LED | 40 |
| Level-meter LED | 41 |
| SD status LED | 42 |
| SD SPI SCK | 12 |
| SD SPI MOSI | 13 |
| SD SPI MISO | 11 |
| SD SPI CS | 14 |

No hardware SD card-detect pin — see **SD card presence detection** below.

## LEDs

| LED | Behavior |
|---|---|
| REC (40) | Off when idle. **Blinks** (400ms toggle) while recording — a tally-light style indicator, not solid-on. |
| Level (41) | PWM'd proportional to input level (gated/attack-release smoothed), regardless of recording state — lets you check mic health any time. |
| SD status (42) | Solid on while the card is considered present and mounted; off if it's missing/failed. Updates live (see below), not just at boot. |

## SD card presence detection (no CD pin)

This PCB revision has no dedicated card-detect pin (the SD socket was
jury-rigged), so `updateSdPresence()` in the firmware infers presence in
software instead of reading a hardware signal:

- While idle **and mounted**, every `SD_CHECK_INTERVAL_MS` (currently
  **50ms**) it does a cheap `SD.open("/")` probe. If that fails, the card is
  marked absent and the LED turns off.
- While idle **and not mounted**, it retries a full `mountSd()` on a slower
  `SD_REMOUNT_INTERVAL_MS` cadence (currently **1000ms**) — see "Boot-time
  mount failure" below for why this is deliberately much slower than the
  50ms presence probe.
- This check is **skipped entirely while actively recording** so it never
  competes with the SD write for the SPI bus.
- If a **write fails during an active recording** (the strongest possible
  signal the card was just pulled), the firmware marks the card gone
  immediately rather than waiting for the next poll.

If you ever need a longer/shorter interval, they're the two `#define`s near
the top of the sketch (`SD_CHECK_INTERVAL_MS`, `SD_REMOUNT_INTERVAL_MS`).

### Boot-time mount failure (fixed 2026-07-08)

Observed once: after flashing and hitting reset, the SD LED never lit even
with a card inserted, and the level-meter LED (41) got stuck on. Unplugging
and reinserting the card fixed it immediately. Two contributing issues, both
addressed:

1. **The card's own SPI state machine can get stuck.** If the board resets
   while a prior SD transaction is unfinished (e.g. the reset button), the
   card can be left waiting for the rest of that command and will silently
   ignore a fresh `CMD0` — a physical unplug/replug clears this by
   power-cycling the card. `mountSd()` now sends the SD spec's required
   >=74 clock pulses (CS high, MOSI high) before each mount attempt
   (`wakeSdSpiBus()`), which flushes the same stuck state in software. This
   may not cover every possible stuck state a card can get into, but it's
   the standard recovery sequence and costs a negligible ~10 SPI bytes per
   attempt.
2. **The old code retried a full `mountSd()` (all 7 SPI speeds) every 50ms
   while unmounted.** Each `SD.begin()` against a non-responding card can
   block for a while, so back-to-back retries at 50ms could starve `loop()`
   of time for anything else — including `updateLevelMeter()`, which is why
   the level LED appeared to freeze at whatever value it last computed
   rather than decaying back down. This is what `SD_REMOUNT_INTERVAL_MS`
   fixes: the expensive remount path now runs at most once a second,
   independent of the cheap 50ms presence probe used once a card is mounted.

### Hot-inserted cards mounting at low speed (fixed 2026-10-01)

**This was the cause of the intermittent sped-up / glitchy recordings.** A
card inserted while the board is running gets probed by the 1s remount loop,
often before it has finished seating/powering up. The 20MHz attempt fails,
`mountSd()` falls through to whatever slower speed answers first (observed:
4, 8, 10 and 16MHz), and the card then stayed at that speed until removed.
At 4MHz a 4KB write averaged ~10.5ms against a 21.3ms real-time budget, with
occasional 100ms+ stalls draining the buffer pool to half — slower mounts
(or a few more stalls) exhaust it and audio is dropped, which plays back sped
up with clicks. A card present at boot was unaffected, which is why it seemed
random (and why resetting the board appeared to help).

Fix (see `SD_SETTLE_MS` in the sketch):
- When a newly detected card mounts below full speed, wait `SD_SETTLE_MS`
  (500ms) and re-sweep from 20MHz.
- If it's still below full speed, re-sweep up to `SD_UPGRADE_MAX_ATTEMPTS`
  (3) more times while idle, every `SD_UPGRADE_INTERVAL_MS` (3s). Never
  during a recording. If the card genuinely can't do 20MHz it keeps the best
  speed that works.

Bench result after the fix: four pull/reinsert cycles first mounted at
8/16/16/10MHz, all came up to 20MHz after the settle delay, and all four
recordings had zero dropped audio (avg write ~3.3ms per chunk).

## Surviving a card pulled mid-recording (fixed 2026-07-08)

The WAV header's data-size field is written once when a recording starts
(as `0`, since the final size isn't known yet) and was previously only
corrected once, in `stopRecording()`. If the card was pulled before that
final write could happen, the file on disk kept its `dataSize=0` header
forever — even though the audio data was physically written to it — and
most players treat that as an empty/corrupt file, discarding everything.

`updateWavHeaderInPlace()` now rewrites the header with the current byte
count on the same 1-second cadence as the existing periodic flush
(`FLUSH_INTERVAL_MS`), seeking back to the append position afterward so it
doesn't disturb normal writing. A card pulled mid-recording now costs at
most ~1s of audio at the tail instead of the entire file; recordings before
and after the pull are unaffected either way.

## Filenames surviving a flaky mid-session remount (fixed 2026-07-08)

Filenames (`REC_00001.WAV`, ...) used to be chosen by scanning from 1 on
every recording, using `SD.exists()` to find the first free slot. That's
fine across a real power cycle, but on a bench run with a marginal card
connection we saw the SD interface remount mid-session (unrelated to the
button/BLE) after a `SD card removed` event, and the very next recording
picked `REC_00001.WAV` again — even though it had already been used earlier
in the same session. Since `startRecording()` opens files with
`FILE_WRITE`, which truncates whatever's already there, that meant the
earlier take was silently overwritten.

`nextRecordingPath()` now scans forward from a RAM-tracked
`nextRecordingIndex` that only ever advances, instead of always restarting
at 1. A real power-on still starts it at 1 and lets `SD.exists()` skip
whatever's already on the card from previous sessions, so numbering across
reboots is unchanged. But within a single boot, an index is never revisited
once used — so even if a flaky remount makes `SD.exists()` misreport an
already-recorded file as free, it can no longer be reused and overwritten.

## Bluetooth LE control

The board advertises as **`ESP32 SD Recorder`**. GATT layout:

| Characteristic | UUID | Properties | Purpose |
|---|---|---|---|
| Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | — | Container for the characteristics below |
| Control | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | Write | Write `0x01`/`'1'` to start, `0x00`/`'0'` to stop, anything else toggles |
| Status | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | Read, Notify | 10-byte status payload (see below); notifies on every state change **and** on a 500ms heartbeat |
| Time | `6e400004-b5a3-f393-e0a9-e50e24dcca9e` | Write | 4-byte uint32 LE: local wall-clock time as seconds since 1970 (Unix time shifted by the sender's UTC offset). Sets the clock used for WAV file timestamps |

**File timestamps:** the PCB has no RTC, so the clock starts at 1970 on every
power-up and files get FAT's minimum date (1980-01-01, which Windows shows as
Dec 31 1979). `ble_control.html` writes the Time characteristic on every
connect, so connect the remote once after powering the board and before
recording. The clock survives a reset but not a power loss. Recordings made
before any BLE connection (button only) still get the 1980 date.

### Status payload (10 bytes, little-endian)

| Bytes | Field | Meaning |
|---|---|---|
| `[0]` | flags | bit0 recording · bit1 SD ready · bit2 low space · bit3 error present |
| `[1-4]` | bytes written | uint32, bytes written to the current/last recording |
| `[5-8]` | free space | uint32, MB free on the SD card |
| `[9]` | error code | `0`=none, `1`=SD not ready, `2`=write failed, `3`=card removed, `4`=no free filename, `5`=file open failed, `6`=auto-stopped (low space), `7`=start refused (low space) |

The heartbeat (independent of state changes) exists so a client can tell
"idle and fine" apart from "the link went stale" — see below — and can watch
the byte counter climb in real time as proof that audio is actually being
written to the card, not just that the record button was pressed.

**Low-space handling:** below 200MB free, the low-space flag is set (client
shows a soft warning) but recording continues; below 50MB, a new recording is
refused outright and an in-progress one is stopped automatically rather than
letting the card fill up and corrupt the last file (`LOW_SPACE_WARN_MB` /
`LOW_SPACE_STOP_MB` in the sketch).

### `ble_control.html` — one-button remote

A self-contained Web Bluetooth control page (no build step, no server, no
external dependencies — everything is inlined in one HTML file). Connect,
then tap one button to start/stop. Beyond the elapsed timer and technical
log, it actively cross-checks that the recording is real rather than just
trusting the button state:

- **Audio saved** tile — duration and size computed from the device's
  running byte counter, not from a client-side clock. If this stops
  climbing while "Recording" is shown, something is wrong even though no
  explicit error fired.
- **Free space** tile — live free space on the card, flagged when it's
  getting low.
- **Stale-link detection** — if no status heartbeat arrives for 2.5s while
  connected, a warning banner appears; recording confirmation can't be
  trusted until the link recovers.
- **Stall detection** — if "Recording" is active but the byte counter hasn't
  moved in 4s, a warning banner appears even without a device-reported error.
- **Device error banner** — SD card not ready, write failed, card removed,
  storage too low, etc. are surfaced as plain-language warnings, not just
  logged.
- **Disconnect-while-recording warning** — if the BLE link drops while a
  recording was in progress, a persistent banner says so (the file on the
  card may be incomplete) until you reconnect.

All of the above are also written to the collapsible technical log for a
full timeline when debugging the link itself.

**How to use it:**
1. Open `ble_control.html` directly in **Chrome or Edge** — double-click it,
   or drag it into an open browser window/tab.
2. Confirm the address bar shows a `file:///...` URL (i.e. it's its own
   top-level tab). This matters: Web Bluetooth is **not available to embedded
   pages** (e.g. Claude Artifact previews, VS Code's Simple Browser, any
   iframe) — the browser blocks it by security policy regardless of the
   page's own code. It has to be a real top-level tab.
3. Tap **Connect**, pick "ESP32 SD Recorder" from the device chooser.
4. Tap the big button to start/stop.

**Browser support:** Chrome/Edge on desktop or Android. **iOS Safari does not
support Web Bluetooth at all** — use the free **Bluefy** browser app on
iPhone/iPad instead.

**Using it on another machine:** the file is fully self-contained, so just
copy `ble_control.html` to any computer with Bluetooth and open it the same
way — no install, no server, no re-pairing step beyond what the page does
itself.

### Adding WiFi control later

The trigger plumbing is already factored out so a WiFi handler is a small
addition, not a rewrite: `toggleRecording()` / `startRecording()` /
`stopRecording()` in the sketch are the only entry points, and they must only
run on `loop()`, the sole SD card user. The BLE write callback runs on the
Bluetooth task on core 0, so it pushes a `RecordCommand` onto `commandQueue`
and `loop()` executes it. (Calling `stopRecording()` straight from the
callback, as an earlier version did, could close the file mid-write and
produce a spurious "SD write failed" when stopping from the remote.) A future
WiFi command handler (HTTP endpoint, WebSocket, MQTT, whatever) should queue
commands the same way. Not implemented yet.

## Architecture note: why audio capture is split across both cores

Early versions of this firmware read one chunk of PDM audio and immediately
`SD.write()`'d it, both in `loop()`. SD card writes are occasionally slow
(FAT bookkeeping, wear leveling) — sometimes 100ms+. The ESP32's I2S DMA ring
buffer only holds ~30ms of audio by default, so whenever a write stalled,
incoming mic audio had nowhere to go and got silently dropped. The recording
still claimed 48kHz, but actually contained less audio than the elapsed real
time — so on playback it sounded **sped up, with clicks/pops** at every drop.

The fix, and the current architecture:

- A dedicated FreeRTOS task (`i2sReaderTask`), pinned to **core 0**, does
  nothing but continuously pull audio off the I2S/PDM peripheral. It never
  touches the SD card, so it can never be blocked by a slow write.
- A **32-buffer pool** (4KB each, 128KB total, ~680ms of audio headroom) sits
  between the reader task and `loop()` as a queue-backed producer/consumer
  pipeline. The reader task fills buffers and hands them off; `loop()` (core
  1) is the only thing that touches the SD card, pulling filled buffers out
  and writing them whenever it gets to them.
- The I2S driver's own DMA queue was also deepened (`dma_desc_num=8`,
  `dma_frame_num=1024`) as a second line of defense.
- The SD SPI clock auto-negotiates the fastest speed the wiring supports,
  trying 20MHz down to 400kHz (see `kSdSpeeds` in the sketch). On this board
  it mounts at the full 20MHz.

Net effect: an occasional slow SD write just eats into the buffer pool's
headroom instead of dropping samples. As long as the *average* write speed
keeps up with the audio bitrate (it comfortably does at 20MHz SPI), recordings
come out clean.

### Capture diagnostics (serial)

Every `stopRecording()` prints a summary over serial, e.g.:

```
Recording stopped (1540096 bytes)
  diag: wall 8010 ms, audio 8021 ms (100.1% of wall time)
  diag: dropped 0 bytes (0 ms), DMA overflows 0
  diag: SD 20000000 Hz, write avg 3502 us / max 6800 us per 4096-byte chunk (budget 21333 us), 0 writes >50ms, min free buffers 30/32
```

- **audio vs wall time** should be ~100%. Less means audio went missing.
- **dropped** is audio the reader task discarded because the pool was full
  (SD writer fell behind). **DMA overflows** means the reader itself was
  starved of CPU. Both should be 0.
- **SD Hz** should be 20000000; lower means a slow mount (see above).
- **min free buffers** is how close the pool came to running out.

**Bench-test hook:** sending `r` over serial toggles recording, same as the
button — handy for scripted tests without touching the board.

## Building and flashing

Built with `arduino-cli` (the copy bundled with Arduino IDE 2 works fine):

```
CLI="/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
SKETCH="path/to/ESP32_S3_SD_Recorder"
FQBN="esp32:esp32:esp32s3:UploadSpeed=921600,USBMode=hwcdc,CDCOnBoot=cdc,MSCOnBoot=default,DFUOnBoot=default,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=4M,PartitionScheme=huge_app,DebugLevel=none,PSRAM=disabled,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default"

"$CLI" compile -b "$FQBN" "$SKETCH"
"$CLI" upload -b "$FQBN" -p COM3 "$SKETCH"   # or whatever port the board is on
```

Notes on the board options, since defaults silently break things here:
- **`CDCOnBoot=cdc`** is required — with it left at the default (disabled),
  `Serial` output doesn't reach the native USB port at all, even though the
  board still enumerates and flashes fine. (Board: **ESP32S3 Dev Module**.)
- **`PartitionScheme=huge_app`** gives enough app-partition room for the BLE
  stack alongside the audio buffer pool (~671KB flash, ~48% RAM at last
  build — comfortable headroom on both).
- **`PSRAM=disabled`** — not currently used by this firmware (the 128KB
  audio buffer pool lives in internal SRAM), even though the chip has 8MB of
  embedded PSRAM available if a future feature needs it.

**Windows gotcha:** if upload fails with `Could not open COM13, the port is
busy` / `Access is denied`, it's almost always the Arduino IDE's own
background `arduino-cli daemon` process holding the Serial Monitor connection
open — not the IDE window itself. Find and stop just that process rather than
closing the IDE:

```powershell
Get-CimInstance Win32_Process -Filter "Name='arduino-cli.exe'" | Select-Object ProcessId, CommandLine
Stop-Process -Id <that PID> -Force
```

### Verifying it after flashing

There's no need for a Serial Monitor session open (which would just cause
the port-busy problem above) — a one-off Python snippet resets the board and
captures the boot log without leaving the port open:

```python
import serial, time
ser = serial.Serial('COM13', 115200, timeout=1)
ser.dtr = False
ser.rts = True
time.sleep(0.1)
ser.rts = False
time.sleep(0.1)
data = b''
start = time.time()
while time.time() - start < 6:
    chunk = ser.read(256)
    if chunk:
        data += chunk
ser.close()
print(data.decode(errors='replace'))
```

A healthy boot looks like:

```
SD mounted at 20000000 Hz
SD card mounted, 30436 MB total
BLE advertising as "ESP32 SD Recorder"
SD recorder ready. Press the button to start/stop recording.
```

## Known-good hardware state (as of 2026-07-07)

- SD card mounts at the full 20MHz SPI clock — signal integrity on this PCB
  is good, no need to fall back to a slower speed.
- Card capacity seen: 30,436MB (~32GB card).
- No hardware CD pin; software presence detection handles hot-plug instead.
