#include <driver/i2s_pdm.h>
#include <math.h>

#define PDM_CLK_PIN       4
#define PDM_DATA_PIN      5
#define BUTTON_PIN        35
#define REC_LED_PIN       40
#define LEVEL_LED_PIN     41

#define SAMPLE_RATE       16000
#define BUFFER_SAMPLES    512

#define BLINK_MS          250
#define DEBOUNCE_MS       50

#define LEVEL_GAIN        6.0f
#define LEVEL_THRESHOLD   0.035f
#define LEVEL_ATTACK      0.25f
#define LEVEL_RELEASE     0.06f

i2s_chan_handle_t rx_handle;

bool recording = false;
bool lastButton = HIGH;
unsigned long lastDebounceMs = 0;
unsigned long lastBlinkMs = 0;
bool blinkState = false;

float levelSmoothed = 0.0f;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(REC_LED_PIN, OUTPUT);
  pinMode(LEVEL_LED_PIN, OUTPUT);

  digitalWrite(REC_LED_PIN, LOW);
  analogWrite(LEVEL_LED_PIN, 0);

  Serial.begin(921600);
  Serial.setTimeout(10);
  delay(2000);

  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

  i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_16BIT,
      I2S_SLOT_MODE_STEREO
    ),
    .gpio_cfg = {
      .clk = GPIO_NUM_4,
      .din = GPIO_NUM_5,
      .invert_flags = {
        .clk_inv = false,
      },
    },
  };

  ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

void toggleRecording() {
  recording = !recording;

  if (recording) {
    Serial.print("REC_START\n");
    lastBlinkMs = millis();
    blinkState = true;
    digitalWrite(REC_LED_PIN, HIGH);
  } else {
    Serial.print("REC_STOP\n");
    digitalWrite(REC_LED_PIN, LOW);
    analogWrite(LEVEL_LED_PIN, 0);
    levelSmoothed = 0.0f;
  }
}

void checkButton() {
  bool button = digitalRead(BUTTON_PIN);

  if (button != lastButton && millis() - lastDebounceMs > DEBOUNCE_MS) {
    lastDebounceMs = millis();
    lastButton = button;

    if (button == LOW) {
      toggleRecording();
    }
  }
}

void updateBlinkLed() {
  if (!recording) {
    digitalWrite(REC_LED_PIN, LOW);
    return;
  }

  if (millis() - lastBlinkMs >= BLINK_MS) {
    lastBlinkMs = millis();
    blinkState = !blinkState;
    digitalWrite(REC_LED_PIN, blinkState ? HIGH : LOW);
  }
}

void updateLevelMeter(int16_t *samples, size_t sampleCount) {
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

  int duty = (int)(levelSmoothed * 255.0f);
  analogWrite(LEVEL_LED_PIN, duty);
}

void loop() {
  checkButton();
  updateBlinkLed();

  if (!recording) {
    delay(2);
    return;
  }

  int16_t samples[BUFFER_SAMPLES];
  size_t bytes_read = 0;

  esp_err_t err = i2s_channel_read(
    rx_handle,
    samples,
    sizeof(samples),
    &bytes_read,
    20
  );

  if (err != ESP_OK || bytes_read == 0) {
    return;
  }

  size_t sampleCount = bytes_read / sizeof(int16_t);
  updateLevelMeter(samples, sampleCount);

  if (Serial) {
    Serial.write((uint8_t*)samples, bytes_read);
  }
}