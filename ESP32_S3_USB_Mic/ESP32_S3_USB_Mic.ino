// ESP32-S3 DevKitC-N8R8 PDM MEMS microphone to USB Audio Class device.
//
// Build with the ESP32 Arduino core using:
//   Board: ESP32S3 Dev Module
//   USB Mode: USB-OTG (TinyUSB)
//   USB CDC On Boot: Disabled
//
// The PDM pins match ESP32_devkit_Mems_Test_2.ino in this directory.
// Windows accepts this as a UAC1 headset, so Audacity should use:
//   Microphone (ESP32 PDM STEREO)

#include <Arduino.h>
#include <driver/i2s_pdm.h>
#include <math.h>
#include "USB.h"
#include "USBAudioCard.h"

#define PDM_CLK_PIN       4
#define PDM_DATA_PIN      5
#define BUTTON_PIN        35
#define REC_LED_PIN       40
#define LEVEL_LED_PIN     41

#define SAMPLE_RATE       48000
#define CHANNELS          2
#define USB_FRAME_MS      1
#define AUDIO_SAMPLES     ((SAMPLE_RATE * CHANNELS * USB_FRAME_MS) / 1000)

#define BLINK_MS          250
#define DEBOUNCE_MS       50

#define LEVEL_GAIN        6.0f
#define LEVEL_THRESHOLD   0.035f
#define LEVEL_ATTACK      0.25f
#define LEVEL_RELEASE     0.06f

i2s_chan_handle_t rx_handle;

// The speaker path is intentionally exposed but discarded. On Windows, the
// headset descriptor starts cleanly; the microphone endpoint carries the MEMS data.
USBAudioCard uac(SAMPLE_RATE, UAC_BPS_16, UAC_SPK_STEREO, UAC_MIC_STEREO);

bool micStreaming = false;
bool muted = false;
bool lastButton = HIGH;
unsigned long lastDebounceMs = 0;
unsigned long lastBlinkMs = 0;
bool blinkState = false;
float levelSmoothed = 0.0f;

static void updateLevelMeter(const int16_t *samples, size_t sampleCount) {
  int64_t sumSquares = 0;

  for (size_t i = 0; i < sampleCount; i++) {
    int32_t v = samples[i];
    sumSquares += (int64_t)v * v;
  }

  float rms = sqrtf((float)sumSquares / (float)sampleCount) / 32768.0f;
  float gated = 0.0f;

  if (!muted && rms > LEVEL_THRESHOLD) {
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

static void swapStereoChannels(int16_t *samples, size_t sampleCount) {
  for (size_t i = 0; i + 1 < sampleCount; i += 2) {
    int16_t left = samples[i];
    samples[i] = samples[i + 1];
    samples[i + 1] = left;
  }
}

static void updateRecordLed() {
  if (!micStreaming) {
    digitalWrite(REC_LED_PIN, LOW);
    return;
  }

  if (!muted) {
    digitalWrite(REC_LED_PIN, HIGH);
    return;
  }

  if (millis() - lastBlinkMs >= BLINK_MS) {
    lastBlinkMs = millis();
    blinkState = !blinkState;
    digitalWrite(REC_LED_PIN, blinkState ? HIGH : LOW);
  }
}

static void checkButton() {
  bool button = digitalRead(BUTTON_PIN);

  if (button != lastButton && millis() - lastDebounceMs > DEBOUNCE_MS) {
    lastDebounceMs = millis();
    lastButton = button;

    if (button == LOW) {
      muted = !muted;
      if (muted) {
        analogWrite(LEVEL_LED_PIN, 0);
        levelSmoothed = 0.0f;
      }
      Serial.printf("USB mic %s\r\n", muted ? "muted" : "live");
    }
  }
}

static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  (void)arg;

  if (event_base == ARDUINO_USB_EVENTS) {
    switch (event_id) {
      case ARDUINO_USB_STARTED_EVENT:
        Serial.println("USB started");
        break;
      case ARDUINO_USB_STOPPED_EVENT:
        Serial.println("USB stopped");
        micStreaming = false;
        break;
      case ARDUINO_USB_SUSPEND_EVENT:
        Serial.println("USB suspended");
        micStreaming = false;
        break;
      case ARDUINO_USB_RESUME_EVENT:
        Serial.println("USB resumed");
        break;
      default:
        break;
    }
    return;
  }

  if (event_base == ARDUINO_USB_AUDIO_CARD_EVENTS) {
    arduino_usb_audio_card_event_data_t *data = (arduino_usb_audio_card_event_data_t *)event_data;

    if (event_id == ARDUINO_USB_AUDIO_CARD_INTERFACE_ENABLE_EVENT &&
        data->interface_enable.interface == UAC_INTERFACE_MIC) {
      micStreaming = data->interface_enable.enable;
      lastBlinkMs = millis();
      blinkState = true;
      Serial.printf("USB mic stream %s\r\n", micStreaming ? "enabled" : "disabled");
    } else if (event_id == ARDUINO_USB_AUDIO_CARD_SAMPLE_RATE_EVENT) {
      Serial.printf("USB host sample rate: %lu\r\n", data->sample_rate.rate);
    }
  }
}

static void onSpeakerData(void *data, uint16_t len) {
  (void)data;
  (void)len;
}

static void beginPdmMic() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
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
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(REC_LED_PIN, OUTPUT);
  pinMode(LEVEL_LED_PIN, OUTPUT);

  digitalWrite(REC_LED_PIN, LOW);
  analogWrite(LEVEL_LED_PIN, 0);

  Serial.begin(115200);
  delay(500);

  beginPdmMic();

  uac.onEvent(usbEventCallback);
  uac.onData(onSpeakerData);
  if (!uac.begin()) {
    Serial.println("USB audio init failed");
  }

  USB.PID(0x1002);
  USB.firmwareVersion(0x0101);
  USB.serialNumber("ESP32-PDM-STEREO-001");
  USB.productName("ESP32 PDM STEREO");
  USB.manufacturerName("UC Davis MEMS Mic Test");
  USB.onEvent(usbEventCallback);
  USB.begin();

  Serial.println("USB microphone firmware ready");
}

void loop() {
  checkButton();
  updateRecordLed();

  int16_t samples[AUDIO_SAMPLES];
  size_t bytesRead = 0;

  esp_err_t err = i2s_channel_read(
    rx_handle,
    samples,
    sizeof(samples),
    &bytesRead,
    pdMS_TO_TICKS(5)
  );

  if (err != ESP_OK || bytesRead == 0) {
    return;
  }

  size_t sampleCount = bytesRead / sizeof(int16_t);
  updateLevelMeter(samples, sampleCount);
  swapStereoChannels(samples, sampleCount);

  if (muted) {
    memset(samples, 0, bytesRead);
  }

  if (micStreaming) {
    uac.write(samples, (uint16_t)bytesRead);
  }
}
