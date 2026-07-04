#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SD_SCK   12
#define SD_MOSI  13
#define SD_MISO  11
#define SD_CS    14
#define SD_CD    10

SPIClass sdSPI(FSPI);

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("--- ESP32-S3 Basic SPI SD Card Test ---");
  Serial.printf("SCK=%d MOSI=%d MISO=%d CS=%d\n", SD_SCK, SD_MOSI, SD_MISO, SD_CS);

  // Set CS high before starting
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  // Start SPI bus on your custom pins
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  Serial.println("Trying SD.begin...");

  // Slower init speed to make marginal wiring/board issues easier to catch
  if (!SD.begin(SD_CS, sdSPI, 400000)) {
    Serial.println("SD Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("Card type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC/SDXC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Card size: %llu MB\n", cardSize);

  Serial.println("Opening test file for write...");

  File file = SD.open("/spi_test.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  file.println("ESP32-S3 SPI SD card test successful");
  file.close();
  Serial.println("Wrote /spi_test.txt");

  Serial.println("Opening test file for read...");

  file = SD.open("/spi_test.txt", FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.println("File contents:");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();

  Serial.println();
  Serial.println("Done.");
}

void loop() {
}