// ESP32-S3 custom PCB: PDM MEMS microphone recorded to a local SD card (SPI).
//
// Combines the PDM capture from ESP32_S3_USB_Mic/ESP32_S3_USB_Mic.ino with the
// SD/SPI pinout validated in SD_Card_SPI_Diagnostic/SD_Card_SPI_Diagnostic.ino.
//
// Press the button (or send a BLE command) once to start recording a new WAV
// file, again to stop. The REC LED blinks while recording, like a normal
// camcorder/recorder tally light. The LEVEL LED shows input level. The SD
// LED is solid while the card is mounted and ready, and updates live: there's
// no hardware card-detect pin wired on this PCB, so presence is inferred in
// software by updateSdPresence() (periodic probe when idle, immediate on a
// failed write during recording).
//
// Audio is captured on a dedicated FreeRTOS task (pinned to core 0) that never
// blocks on SD I/O, so a slow SD write can't starve the I2S DMA and drop
// samples (the cause of sped-up/glitchy recordings). Captured buffers are
// handed to loop() (core 1) through a queue-backed buffer pool; loop() is the
// only thing that touches the SD card.
//
// Remote trigger: a BLE service exposes a write characteristic (start/stop/
// toggle) and a notify characteristic (recording state) — see the "BLE
// control" section. BLE commands are queued to loop() (see commandQueue)
// rather than run in the BLE callback, since only loop() may touch the SD
// card. A future WiFi trigger should do the same: push onto commandQueue from
// its own handler; no other changes should be required.
//
// Build with the ESP32 Arduino core using:
//   Board: ESP32S3 Dev Module

#include <Arduino.h>
#include <driver/i2s_pdm.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---- PDM microphone pins (from ESP32_S3_USB_Mic.ino) ----
#define PDM_CLK_PIN       4
#define PDM_DATA_PIN      5
#define BUTTON_PIN        35
#define REC_LED_PIN       40
#define LEVEL_LED_PIN     41
#define SD_STATUS_LED_PIN 42

// ---- SD card SPI pins (from SD_Card_SPI_Diagnostic.ino) ----
// No dedicated card-detect pin is wired on this build of the PCB, so presence
// is inferred in software instead (see updateSdPresence()).
#define SD_SCK            12
#define SD_MOSI           13
#define SD_MISO           11
#define SD_CS             14

#define SAMPLE_RATE       48000
#define CHANNELS          2
#define BITS_PER_SAMPLE   16

// Audio buffer pool shared between the I2S reader task and loop()'s SD writer.
// 32 buffers x 4096 bytes = 128KB, ~680ms of audio headroom to absorb slow SD
// writes without ever blocking the I2S read and losing samples.
#define BUFFER_FRAMES     1024
#define BUFFER_BYTES      (BUFFER_FRAMES * CHANNELS * (BITS_PER_SAMPLE / 8))
#define NUM_BUFFERS       32

#define DEBOUNCE_MS       50
#define REC_BLINK_MS      400

#define LEVEL_GAIN        6.0f
#define LEVEL_THRESHOLD   0.035f
#define LEVEL_ATTACK      0.25f
#define LEVEL_RELEASE     0.06f

#define FLUSH_INTERVAL_MS 1000
// Cheap "is the mounted card still answering" probe, while mounted.
#define SD_CHECK_INTERVAL_MS 50
// Full remount-attempt cadence while no card is mounted. Deliberately much
// slower than SD_CHECK_INTERVAL_MS: mountSd() sweeps every SPI speed and
// each SD.begin() can block for a while against a card that isn't
// responding, so retrying it back-to-back at 50ms starves loop() of time
// for audio/BLE/button servicing for as long as the card stays absent.
#define SD_REMOUNT_INTERVAL_MS 1000

// A card hot-inserted while the board is running often gets probed while its
// contacts are still seating / it's still powering up: the fast SPI speeds
// fail and mountSd() settles on a slow one (seen: 4MHz instead of 20MHz),
// which then sticks until the card is removed. At low speeds the average SD
// write barely keeps up with the audio, so recordings start dropping audio
// (sped-up / glitchy playback). To mount at full speed every time:
//   - a hot-insert mount below full speed waits SD_SETTLE_MS and re-sweeps;
//   - if still below full speed, it re-sweeps up to SD_UPGRADE_MAX_ATTEMPTS
//     more times while idle, every SD_UPGRADE_INTERVAL_MS.
#define SD_SETTLE_MS            500
#define SD_UPGRADE_INTERVAL_MS  3000
#define SD_UPGRADE_MAX_ATTEMPTS 3

// How often the free-space estimate is refreshed from the card while idle
// (skipped while recording — see updateFreeSpace()).
#define FREE_SPACE_CHECK_INTERVAL_MS 2000

// A new recording is refused below this much free space, and an in-progress
// one is auto-stopped if it drops below this while recording (rather than
// running the card completely out and corrupting the last file).
#define LOW_SPACE_STOP_MB 50
// Below this (but above the stop threshold), recording is still allowed but
// the status payload carries a warning flag so the remote can show it.
#define LOW_SPACE_WARN_MB 200

// Status is pushed on every state change immediately, plus on this cadence
// regardless of state so the remote can confirm the link is alive and see
// the byte counter actually incrementing while recording.
#define STATUS_NOTIFY_INTERVAL_MS 500

#define BLE_DEVICE_NAME       "ESP32 SD Recorder"
#define BLE_SERVICE_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_CONTROL_CHAR_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_STATUS_CHAR_UUID  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_TIME_CHAR_UUID    "6e400004-b5a3-f393-e0a9-e50e24dcca9e"

// No RTC on this PCB, so the clock starts at the 1970 epoch on every boot and
// FAT file timestamps come out as 1980-01-01 (Windows shows Dec 31 1979). The
// remote sets the clock over BLE on connect (see BleTimeCallbacks); until
// then, files get that placeholder date.
volatile bool clockSet = false;

// ---------------------------------------------------------------------------
// Status payload (10 bytes, all multi-byte fields little-endian):
//   [0]   flag bitfield (see STATUS_FLAG_*)
//   [1-4] bytes written to the current/last recording (uint32)
//   [5-8] free space on the SD card, in MB (uint32)
//   [9]   latched error code (see ErrorCode)
// Sent on every state change and on a fixed heartbeat cadence so the remote
// can distinguish "idle and fine" from "link has gone stale".
// ---------------------------------------------------------------------------
#define STATUS_FLAG_RECORDING 0x01
#define STATUS_FLAG_SD_READY  0x02
#define STATUS_FLAG_LOW_SPACE 0x04
#define STATUS_FLAG_ERROR     0x08

enum ErrorCode : uint8_t {
  ERR_NONE = 0,
  ERR_SD_NOT_READY = 1,       // start refused: card not mounted
  ERR_SD_WRITE_FAILED = 2,    // write failed mid-recording, stopped
  ERR_SD_CARD_REMOVED = 3,    // card disappeared while idle
  ERR_NO_FREE_FILENAME = 4,   // REC_00001..REC_99999 all taken
  ERR_FILE_OPEN_FAILED = 5,   // SD.open() failed
  ERR_LOW_SPACE_STOPPED = 6,  // auto-stopped, ran low on space
  ERR_LOW_SPACE_REFUSED = 7,  // start refused, too little space to begin
};

struct AudioChunk {
  uint8_t *data;
  size_t len;
};

SPIClass sdSPI(FSPI);
i2s_chan_handle_t rx_handle;

static uint8_t audioPool[NUM_BUFFERS][BUFFER_BYTES];
static QueueHandle_t freeQueue;
static QueueHandle_t filledQueue;

// Recording commands received over BLE. The BLE callbacks run on the
// Bluetooth stack's task (core 0), so they must not touch recordingFile
// directly — closing the file there while loop() is mid-write on core 1
// makes that write fail (ERR_SD_WRITE_FAILED) and drops sdReady. Commands
// are queued here instead and executed by loop(), the only SD user.
enum RecordCommand : uint8_t { CMD_STOP = 0, CMD_START = 1, CMD_TOGGLE = 2 };
static QueueHandle_t commandQueue;

// Per-recording capture diagnostics, printed by stopRecording(). Sped-up /
// glitchy playback means audio went missing between the mic and the card;
// these say where. diagDropBytes counts audio the reader task had to discard
// because every pool buffer was still waiting on the SD writer;
// diagDmaOverflows counts times the I2S DMA ring itself overran (reader task
// starved of CPU).
volatile uint32_t diagDropBytes = 0;
volatile uint32_t diagDmaOverflows = 0;
uint32_t diagMaxWriteUs = 0;
uint64_t diagTotalWriteUs = 0;
uint32_t diagSlowWrites = 0;   // writes that took longer than 50ms
UBaseType_t diagMinFreeBuffers = NUM_BUFFERS;
unsigned long recordStartMs = 0;
uint32_t sdMountedHz = 0;

bool sdReady = false;
bool recording = false;
File recordingFile;
uint32_t dataBytesWritten = 0;
unsigned long lastFlushMs = 0;
unsigned long lastSdCheckMs = 0;
unsigned long lastSdRemountMs = 0;
unsigned long lastSdUpgradeMs = 0;
uint8_t sdUpgradeAttempts = 0;

// Next filename index to try, tracked in RAM so it only ever moves forward
// within a boot session — see nextRecordingPath() for why.
uint32_t nextRecordingIndex = 1;

// Free space + error tracking (see status payload doc above)
uint32_t freeSpaceMB = 0;
uint64_t freeSpaceAtRecordStartBytes = 0;
ErrorCode errorCode = ERR_NONE;
unsigned long lastFreeSpaceCheckMs = 0;
unsigned long lastStatusNotifyMs = 0;

bool lastButton = HIGH;
unsigned long lastDebounceMs = 0;
unsigned long lastRecBlinkMs = 0;
bool recBlinkState = false;
float levelSmoothed = 0.0f;

BLECharacteristic *bleStatusCharacteristic = nullptr;
bool bleClientConnected = false;

// ---------------------------------------------------------------------------
// WAV header
// ---------------------------------------------------------------------------

static void writeWavHeader(File &f, uint32_t dataSize) {
  uint32_t byteRate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
  uint16_t blockAlign = CHANNELS * (BITS_PER_SAMPLE / 8);
  uint32_t chunkSize = 36 + dataSize;

  uint8_t header[44];
  memcpy(header + 0, "RIFF", 4);
  memcpy(header + 4, &chunkSize, 4);
  memcpy(header + 8, "WAVE", 4);
  memcpy(header + 12, "fmt ", 4);
  uint32_t subchunk1Size = 16;
  memcpy(header + 16, &subchunk1Size, 4);
  uint16_t audioFormat = 1;
  memcpy(header + 20, &audioFormat, 2);
  uint16_t numChannels = CHANNELS;
  memcpy(header + 22, &numChannels, 2);
  uint32_t sampleRate = SAMPLE_RATE;
  memcpy(header + 24, &sampleRate, 4);
  memcpy(header + 28, &byteRate, 4);
  memcpy(header + 32, &blockAlign, 2);
  uint16_t bitsPerSample = BITS_PER_SAMPLE;
  memcpy(header + 34, &bitsPerSample, 2);
  memcpy(header + 36, "data", 4);
  memcpy(header + 40, &dataSize, 4);

  f.seek(0);
  f.write(header, sizeof(header));
}

// Rewrites the header with the data size known so far, then restores the
// write position so appending continues normally. Called on the same
// cadence as the periodic flush (see loop()) so that if the card is pulled
// mid-recording, the file on disk still has a header claiming (approximately)
// the audio actually present in it — a WAV header stuck at dataSize=0 (its
// value from file creation) makes most players treat the whole file as
// empty/corrupt even though the audio bytes are physically there. Worst case
// this leaves the header up to one flush interval stale, i.e. a pulled card
// costs at most ~1s of the recording instead of the entire file.
static void updateWavHeaderInPlace(File &f, uint32_t dataSize) {
  size_t appendPos = f.position();
  writeWavHeader(f, dataSize);
  f.seek(appendPos);
}

// ---------------------------------------------------------------------------
// SD card
// ---------------------------------------------------------------------------

// Highest speed first; SD.begin() simply fails and we fall back if the wiring
// on this PCB can't hold the signal integrity at a given clock.
static const uint32_t kSdSpeeds[] = {
  20000000, 16000000, 10000000, 8000000, 4000000, 1000000, 400000
};
static const size_t kSdSpeedCount = sizeof(kSdSpeeds) / sizeof(kSdSpeeds[0]);

// Per the SD spec, a card expects >=74 clock cycles with CS deasserted and
// MOSI high before it will accept CMD0. If the board was reset (e.g. the
// reset button) while a prior SD transaction was left unfinished, the card's
// internal state machine can be left waiting for the rest of that command
// and will silently ignore a fresh CMD0 until it either times out on its own
// or gets these clocks — which is effectively what a physical unplug/replug
// does by power-cycling the card. Sending them explicitly here means a stuck
// card can recover without needing to be physically reseated.
static void wakeSdSpiBus() {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  sdSPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  for (int i = 0; i < 10; i++) {
    sdSPI.transfer(0xFF);
  }
  sdSPI.endTransaction();
}

static bool mountSd() {
  for (size_t i = 0; i < kSdSpeedCount; i++) {
    SD.end();
    sdSPI.end();
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    wakeSdSpiBus();

    if (SD.begin(SD_CS, sdSPI, kSdSpeeds[i])) {
      sdMountedHz = kSdSpeeds[i];
      Serial.printf("SD mounted at %lu Hz\r\n", (unsigned long)kSdSpeeds[i]);
      return true;
    }
  }

  return false;
}

// Scans forward from nextRecordingIndex rather than always restarting the
// search at 1. On a real power-on, nextRecordingIndex starts at 1 and
// SD.exists() skips over whatever the card already has from previous
// sessions, same as before. But within a single boot, the index only ever
// advances — it's never reset — so a mid-session SD remount (see
// updateSdPresence(), which can happen after a flaky reconnect) can't land
// back on a filename already used this session. That matters because
// startRecording() opens with FILE_WRITE, which silently truncates whatever
// is already at that path: if a marginal remount ever made SD.exists()
// falsely report an already-recorded file as absent, scanning from 1 again
// would reuse and overwrite it. Scanning from a monotonic counter instead
// makes that impossible regardless of what SD.exists() reports.
static bool nextRecordingPath(char *outPath, size_t outPathLen) {
  while (nextRecordingIndex <= 99999) {
    snprintf(outPath, outPathLen, "/REC_%05lu.WAV", (unsigned long)nextRecordingIndex);
    uint32_t candidate = nextRecordingIndex;
    nextRecordingIndex++; // reserve it now so it's never retried, even if this attempt fails below

    if (!SD.exists(outPath)) {
      return true;
    }
    Serial.printf("REC_%05lu.WAV already exists, skipping\r\n", (unsigned long)candidate);
  }
  return false;
}

// Keeps sdReady and the status LED in lockstep so they can never drift apart.
static void setSdReady(bool ready) {
  sdReady = ready;
  digitalWrite(SD_STATUS_LED_PIN, sdReady ? HIGH : LOW);
}

// There's no hardware card-detect pin wired on this PCB, so presence is
// inferred: while mounted, periodically confirm the card still answers a
// cheap directory read (SD_CHECK_INTERVAL_MS); while absent, periodically
// retry a full mount so a card inserted later is picked up automatically
// (SD_REMOUNT_INTERVAL_MS — deliberately slower, see its comment). Skipped
// entirely while recording so it never competes with the active write for
// the SPI bus.
static void updateSdPresence() {
  if (recording) {
    return;
  }

  bool present;

  if (sdReady) {
    if (millis() - lastSdCheckMs < SD_CHECK_INTERVAL_MS) {
      return;
    }
    lastSdCheckMs = millis();

    present = false;
    File root = SD.open("/");
    if (root) {
      present = true;
      root.close();
    }

    // Still below full SPI speed after a hot-insert: try for full speed
    // again while idle (see SD_SETTLE_MS).
    if (present && sdMountedHz < kSdSpeeds[0] &&
        sdUpgradeAttempts < SD_UPGRADE_MAX_ATTEMPTS &&
        millis() - lastSdUpgradeMs >= SD_UPGRADE_INTERVAL_MS) {
      lastSdUpgradeMs = millis();
      sdUpgradeAttempts++;
      Serial.printf("SD at %lu Hz, retrying for full speed (attempt %u/%u)\r\n",
                    (unsigned long)sdMountedHz, sdUpgradeAttempts, SD_UPGRADE_MAX_ATTEMPTS);
      present = mountSd();
    }
  } else {
    if (millis() - lastSdRemountMs < SD_REMOUNT_INTERVAL_MS) {
      return;
    }
    lastSdRemountMs = millis();

    present = mountSd();
    if (present && sdMountedHz < kSdSpeeds[0]) {
      // Probably caught the card mid-insertion; let it settle and re-sweep.
      delay(SD_SETTLE_MS);
      present = mountSd();
    }
    sdUpgradeAttempts = 0;
    lastSdUpgradeMs = millis();
  }

  if (present != sdReady) {
    setSdReady(present);
    Serial.println(present ? "SD card detected" : "SD card removed");
    if (!present) {
      errorCode = ERR_SD_CARD_REMOVED;
    } else if (errorCode == ERR_SD_CARD_REMOVED) {
      // The condition that caused this latched error is gone now that the
      // card answers again; don't keep showing a stale warning forever.
      errorCode = ERR_NONE;
    }
    notifyBleStatus();
  }
}

// Refreshes the free-space estimate from the card itself. Only runs while
// idle (recording keeps its own cheap running estimate below, so this never
// competes with an active write for the SPI bus — same reasoning as
// updateSdPresence()).
static void updateFreeSpace() {
  if (recording || !sdReady) {
    return;
  }
  if (millis() - lastFreeSpaceCheckMs < FREE_SPACE_CHECK_INTERVAL_MS) {
    return;
  }
  lastFreeSpaceCheckMs = millis();

  uint64_t total = SD.totalBytes();
  uint64_t used = SD.usedBytes();
  uint64_t freeBytes = (total > used) ? (total - used) : 0;
  freeSpaceMB = (uint32_t)(freeBytes / (1024ULL * 1024ULL));
}

// ---------------------------------------------------------------------------
// BLE status notification
// ---------------------------------------------------------------------------

static void notifyBleStatus() {
  if (!bleStatusCharacteristic) {
    return;
  }

  uint8_t flags = 0;
  if (recording) flags |= STATUS_FLAG_RECORDING;
  if (sdReady) flags |= STATUS_FLAG_SD_READY;
  if (sdReady && freeSpaceMB < LOW_SPACE_WARN_MB) flags |= STATUS_FLAG_LOW_SPACE;
  if (errorCode != ERR_NONE) flags |= STATUS_FLAG_ERROR;

  uint8_t payload[10];
  payload[0] = flags;
  memcpy(payload + 1, &dataBytesWritten, 4);
  memcpy(payload + 5, &freeSpaceMB, 4);
  payload[9] = (uint8_t)errorCode;

  bleStatusCharacteristic->setValue(payload, sizeof(payload));
  if (bleClientConnected) {
    bleStatusCharacteristic->notify();
  }
}

// ---------------------------------------------------------------------------
// Recording control
// ---------------------------------------------------------------------------

static void startRecording() {
  if (recording) {
    return;
  }

  // A fresh attempt always clears whatever error was last latched, so the
  // remote doesn't keep showing a stale failure after the user retries.
  errorCode = ERR_NONE;

  if (!sdReady) {
    errorCode = ERR_SD_NOT_READY;
    notifyBleStatus();
    Serial.println("Recording refused: SD card not ready");
    return;
  }

  uint64_t total = SD.totalBytes();
  uint64_t used = SD.usedBytes();
  uint64_t freeBytes = (total > used) ? (total - used) : 0;
  if (freeBytes < (uint64_t)LOW_SPACE_STOP_MB * 1024ULL * 1024ULL) {
    errorCode = ERR_LOW_SPACE_REFUSED;
    freeSpaceMB = (uint32_t)(freeBytes / (1024ULL * 1024ULL));
    notifyBleStatus();
    Serial.println("Recording refused: free space too low");
    return;
  }

  char path[32];
  if (!nextRecordingPath(path, sizeof(path))) {
    errorCode = ERR_NO_FREE_FILENAME;
    notifyBleStatus();
    Serial.println("No free recording filename slots on SD card");
    return;
  }

  recordingFile = SD.open(path, FILE_WRITE);
  if (!recordingFile) {
    errorCode = ERR_FILE_OPEN_FAILED;
    notifyBleStatus();
    Serial.printf("Failed to open %s for recording\r\n", path);
    return;
  }

  writeWavHeader(recordingFile, 0);
  dataBytesWritten = 0;
  freeSpaceAtRecordStartBytes = freeBytes;
  freeSpaceMB = (uint32_t)(freeBytes / (1024ULL * 1024ULL));
  lastFlushMs = millis();
  diagDropBytes = 0;
  diagDmaOverflows = 0;
  diagMaxWriteUs = 0;
  diagTotalWriteUs = 0;
  diagSlowWrites = 0;
  diagMinFreeBuffers = NUM_BUFFERS;
  recordStartMs = millis();
  recording = true;
  lastRecBlinkMs = millis();
  recBlinkState = true;
  digitalWrite(REC_LED_PIN, HIGH);
  notifyBleStatus();
  if (clockSet) {
    time_t now = time(NULL);
    struct tm t;
    localtime_r(&now, &t);
    char stamp[24];
    strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &t);
    Serial.printf("Recording started: %s (%s)\r\n", path, stamp);
  } else {
    Serial.printf("Recording started: %s (clock not set, file will be dated 1980)\r\n", path);
  }
}

static void stopRecording() {
  if (!recording) {
    return;
  }

  writeWavHeader(recordingFile, dataBytesWritten);
  recordingFile.close();
  recording = false;
  digitalWrite(REC_LED_PIN, LOW);
  notifyBleStatus();
  Serial.printf("Recording stopped (%lu bytes)\r\n", (unsigned long)dataBytesWritten);

  const uint32_t bytesPerSec = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
  unsigned long elapsedMs = millis() - recordStartMs;
  uint32_t chunks = dataBytesWritten / BUFFER_BYTES;
  Serial.printf("  diag: wall %lu ms, audio %lu ms (%.1f%% of wall time)\r\n",
                elapsedMs,
                (unsigned long)((uint64_t)dataBytesWritten * 1000 / bytesPerSec),
                elapsedMs ? 100.0 * dataBytesWritten / ((double)bytesPerSec * elapsedMs / 1000.0) : 0.0);
  Serial.printf("  diag: dropped %lu bytes (%lu ms), DMA overflows %lu\r\n",
                (unsigned long)diagDropBytes,
                (unsigned long)((uint64_t)diagDropBytes * 1000 / bytesPerSec),
                (unsigned long)diagDmaOverflows);
  Serial.printf("  diag: SD %lu Hz, write avg %lu us / max %lu us per %d-byte chunk (budget %lu us), %lu writes >50ms, min free buffers %lu/%d\r\n",
                (unsigned long)sdMountedHz,
                (unsigned long)(chunks ? diagTotalWriteUs / chunks : 0),
                (unsigned long)diagMaxWriteUs, BUFFER_BYTES,
                (unsigned long)((uint64_t)BUFFER_BYTES * 1000000 / bytesPerSec),
                (unsigned long)diagSlowWrites,
                (unsigned long)diagMinFreeBuffers, NUM_BUFFERS);
}

static void toggleRecording() {
  if (recording) {
    stopRecording();
  } else {
    startRecording();
  }
}

// ---------------------------------------------------------------------------
// BLE control
//
// A single GATT service exposes:
//   - a WRITE characteristic: write 0x01/'1' to start, 0x00/'0' to stop, or
//     anything else to toggle (mirrors the physical button).
//   - a NOTIFY characteristic: broadcasts 0x01 while recording, 0x00 when idle.
//   - a WRITE time characteristic: sets the clock used for file timestamps
//     (see BleTimeCallbacks).
//
// A future WiFi trigger can live alongside this the same way: have its
// handler push a RecordCommand onto commandQueue (not call startRecording()
// etc. directly, since it would run off loop()'s core too).
// ---------------------------------------------------------------------------

class BleControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = characteristic->getValue();
    if (value.length() == 0) {
      return;
    }

    uint8_t cmd = (uint8_t)value[0];
    RecordCommand command;
    if (cmd == '1' || cmd == 1) {
      command = CMD_START;
    } else if (cmd == '0' || cmd == 0) {
      command = CMD_STOP;
    } else {
      command = CMD_TOGGLE;
    }
    // Executed by loop(); see commandQueue for why this can't run here.
    xQueueSend(commandQueue, &command, 0);
  }
};

// Time-set characteristic: the remote writes the current *local* wall-clock
// time as a uint32 little-endian count of seconds since 1970 (i.e. Unix time
// already shifted by the remote's UTC offset). The system clock is kept in
// that local time with TZ left at UTC, so localtime() — which FatFS uses for
// file timestamps — returns local wall-clock time, matching how FAT (and
// Windows) interpret those timestamps. Survives soft resets, not power loss.
class BleTimeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = characteristic->getValue();
    if (value.length() < 4) {
      return;
    }

    uint32_t localSeconds;
    memcpy(&localSeconds, value.c_str(), 4);

    struct timeval tv = { (time_t)localSeconds, 0 };
    settimeofday(&tv, NULL);
    clockSet = true;

    struct tm t;
    time_t now = (time_t)localSeconds;
    localtime_r(&now, &t);
    char stamp[24];
    strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &t);
    Serial.printf("Clock set over BLE: %s\r\n", stamp);
  }
};

class BleServerCallbacksImpl : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    (void)server;
    bleClientConnected = true;
    Serial.println("BLE client connected");
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    bleClientConnected = false;
    Serial.println("BLE client disconnected, resuming advertising");
    BLEDevice::startAdvertising();
  }
};

static void beginBle() {
  BLEDevice::init(BLE_DEVICE_NAME);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new BleServerCallbacksImpl());

  BLEService *service = server->createService(BLE_SERVICE_UUID);

  BLECharacteristic *controlCharacteristic = service->createCharacteristic(
    BLE_CONTROL_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  controlCharacteristic->setCallbacks(new BleControlCallbacks());

  BLECharacteristic *timeCharacteristic = service->createCharacteristic(
    BLE_TIME_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  timeCharacteristic->setCallbacks(new BleTimeCallbacks());

  bleStatusCharacteristic = service->createCharacteristic(
    BLE_STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  bleStatusCharacteristic->addDescriptor(new BLE2902());
  notifyBleStatus();

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.printf("BLE advertising as \"%s\"\r\n", BLE_DEVICE_NAME);
}

// ---------------------------------------------------------------------------
// Button + LEDs
// ---------------------------------------------------------------------------

static void checkButton() {
  bool button = digitalRead(BUTTON_PIN);

  if (button != lastButton && millis() - lastDebounceMs > DEBOUNCE_MS) {
    lastDebounceMs = millis();
    lastButton = button;

    if (button == LOW) {
      toggleRecording();
    }
  }
}

static void updateLevelMeter(const int16_t *samples, size_t sampleCount) {
  int64_t sumSquares = 0;

  for (size_t i = 0; i < sampleCount; i++) {
    int32_t v = samples[i];
    sumSquares += (int64_t)v * v;
  }

  float rms = sqrtf((float)sumSquares / (float)sampleCount) / 32768.0f;
  float gated = 0.0f;

  if (rms > LEVEL_THRESHOLD) {
    gated = (rms - LEVEL_THRESHOLD) / (1.0f - LEVEL_THRESHOLD);
    gated *= LEVEL_GAIN;
    if (gated > 1.0f) {
      gated = 1.0f;
    }
  }

  float smoothing = gated > levelSmoothed ? LEVEL_ATTACK : LEVEL_RELEASE;
  levelSmoothed = (1.0f - smoothing) * levelSmoothed + smoothing * gated;

  analogWrite(LEVEL_LED_PIN, (int)(levelSmoothed * 255.0f));
}

static void updateRecordLed() {
  if (!recording) {
    digitalWrite(REC_LED_PIN, LOW);
    return;
  }

  if (millis() - lastRecBlinkMs >= REC_BLINK_MS) {
    lastRecBlinkMs = millis();
    recBlinkState = !recBlinkState;
    digitalWrite(REC_LED_PIN, recBlinkState ? HIGH : LOW);
  }
}

// ---------------------------------------------------------------------------
// PDM microphone
// ---------------------------------------------------------------------------

static bool IRAM_ATTR onI2sRecvOverflow(i2s_chan_handle_t handle, i2s_event_data_t *event, void *ctx) {
  (void)handle;
  (void)event;
  (void)ctx;
  diagDmaOverflows++;
  return false;
}

static void beginPdmMic() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  // Deeper DMA queue than the default gives the driver its own cushion
  // (on top of the app-level buffer pool) before any data could be dropped.
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = BUFFER_FRAMES;
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

  i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_16BIT,
      I2S_SLOT_MODE_STEREO
    ),
    .gpio_cfg = {
      .clk = (gpio_num_t)PDM_CLK_PIN,
      .din = (gpio_num_t)PDM_DATA_PIN,
      .invert_flags = {
        .clk_inv = false,
      },
    },
  };

  ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg));

  i2s_event_callbacks_t cbs = {};
  cbs.on_recv_q_ovf = onI2sRecvOverflow;
  ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx_handle, &cbs, NULL));

  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

// ---------------------------------------------------------------------------
// I2S reader task (core 0) — never touches the SD card, so it can always
// keep draining the I2S DMA on schedule regardless of how long a pending SD
// write takes on the other core.
// ---------------------------------------------------------------------------

static void i2sReaderTask(void *param) {
  (void)param;

  static uint8_t scratch[BUFFER_BYTES];

  for (;;) {
    uint8_t *buf = NULL;
    if (xQueueReceive(freeQueue, &buf, 0) != pdTRUE) {
      // Pool exhausted: the SD writer has fallen ~680ms behind. Keep draining
      // the DMA into scratch (so the loss is counted here rather than hidden
      // as a DMA overflow) and go back to waiting for a free buffer.
      size_t dropped = 0;
      i2s_channel_read(rx_handle, scratch, BUFFER_BYTES, &dropped, portMAX_DELAY);
      diagDropBytes += dropped;
      continue;
    }

    size_t bytesRead = 0;
    esp_err_t err = i2s_channel_read(rx_handle, buf, BUFFER_BYTES, &bytesRead, portMAX_DELAY);

    if (err == ESP_OK && bytesRead > 0) {
      AudioChunk chunk = { buf, bytesRead };
      xQueueSend(filledQueue, &chunk, portMAX_DELAY);
    } else {
      xQueueSend(freeQueue, &buf, portMAX_DELAY);
    }
  }
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(REC_LED_PIN, OUTPUT);
  pinMode(LEVEL_LED_PIN, OUTPUT);
  pinMode(SD_STATUS_LED_PIN, OUTPUT);

  digitalWrite(REC_LED_PIN, LOW);
  analogWrite(LEVEL_LED_PIN, 0);
  digitalWrite(SD_STATUS_LED_PIN, LOW);

  Serial.begin(115200);
  delay(500);

  beginPdmMic();

  freeQueue = xQueueCreate(NUM_BUFFERS, sizeof(uint8_t *));
  filledQueue = xQueueCreate(NUM_BUFFERS, sizeof(AudioChunk));
  for (int i = 0; i < NUM_BUFFERS; i++) {
    uint8_t *ptr = audioPool[i];
    xQueueSend(freeQueue, &ptr, 0);
  }

  commandQueue = xQueueCreate(4, sizeof(RecordCommand));

  xTaskCreatePinnedToCore(i2sReaderTask, "i2s_reader", 4096, NULL, 10, NULL, 0);

  setSdReady(mountSd());
  if (sdReady) {
    Serial.printf("SD card mounted, %llu MB total\r\n",
                  SD.cardSize() / (1024ULL * 1024ULL));
  } else {
    Serial.println("SD card mount failed, recording disabled");
  }

  beginBle();

  Serial.println("SD recorder ready. Press the button to start/stop recording.");
}

void loop() {
  RecordCommand command;
  while (xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
    if (command == CMD_START) {
      startRecording();
    } else if (command == CMD_STOP) {
      stopRecording();
    } else {
      toggleRecording();
    }
  }

  // Bench-test hook: 'r' over serial toggles recording, same as the button.
  while (Serial.available()) {
    if (Serial.read() == 'r') {
      toggleRecording();
    }
  }

  checkButton();
  updateRecordLed();
  updateSdPresence();
  updateFreeSpace();

  // Heartbeat: pushed on this fixed cadence regardless of state changes, so
  // the remote can tell "still connected and fine" apart from "link has gone
  // stale" and watch the byte counter actually climb during a recording.
  // Placed before the queue-timeout return so it isn't skipped when no
  // audio chunk happens to be ready yet.
  if (millis() - lastStatusNotifyMs >= STATUS_NOTIFY_INTERVAL_MS) {
    lastStatusNotifyMs = millis();
    notifyBleStatus();
  }

  AudioChunk chunk;
  if (xQueueReceive(filledQueue, &chunk, pdMS_TO_TICKS(50)) != pdTRUE) {
    return;
  }

  size_t sampleCount = chunk.len / sizeof(int16_t);
  updateLevelMeter((const int16_t *)chunk.data, sampleCount);

  if (recording) {
    UBaseType_t freeBuffers = uxQueueMessagesWaiting(freeQueue);
    if (freeBuffers < diagMinFreeBuffers) {
      diagMinFreeBuffers = freeBuffers;
    }

    uint32_t writeStartUs = micros();
    size_t written = recordingFile.write(chunk.data, chunk.len);
    uint32_t writeUs = micros() - writeStartUs;
    diagTotalWriteUs += writeUs;
    if (writeUs > diagMaxWriteUs) diagMaxWriteUs = writeUs;
    if (writeUs > 50000) diagSlowWrites++;
    dataBytesWritten += written;

    if (written != chunk.len) {
      Serial.println("SD write short/failed, stopping recording");
      errorCode = ERR_SD_WRITE_FAILED;
      stopRecording();
      setSdReady(false);
    } else {
      // Cheap arithmetic estimate (no SD query) so a long recording still
      // gets a live free-space readout without contending with the write
      // for the SPI bus — see updateFreeSpace() for the idle-time version.
      uint64_t estFreeBytes = (freeSpaceAtRecordStartBytes > dataBytesWritten)
        ? (freeSpaceAtRecordStartBytes - dataBytesWritten)
        : 0;
      freeSpaceMB = (uint32_t)(estFreeBytes / (1024ULL * 1024ULL));

      if (estFreeBytes < (uint64_t)LOW_SPACE_STOP_MB * 1024ULL * 1024ULL) {
        Serial.println("Free space critically low, stopping recording");
        errorCode = ERR_LOW_SPACE_STOPPED;
        stopRecording();
      } else if (millis() - lastFlushMs >= FLUSH_INTERVAL_MS) {
        lastFlushMs = millis();
        updateWavHeaderInPlace(recordingFile, dataBytesWritten);
        recordingFile.flush();
      }
    }
  }

  xQueueSend(freeQueue, &chunk.data, portMAX_DELAY);
}
