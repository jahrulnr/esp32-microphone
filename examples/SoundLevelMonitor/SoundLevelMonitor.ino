/**
 * SoundLevelMonitor.ino
 * 
 * Example demonstrating how to monitor sound levels and detect sound events
 * using either analog or I2S microphones. This example includes debouncing
 * logic to avoid repeated triggers from continuous sounds.
 * 
 * Features:
 * - Continuously monitors audio levels
 * - Detects sound events above a specified threshold
 * - Implements debounce logic for more reliable detection
 * - Visual feedback using the built-in LED
 * 
 * This example supports both microphone types - uncomment the appropriate section.
 * 
 * Library: https://github.com/jahrulnr/esp32-microphone
 */

#include <Arduino.h>

// Uncomment ONE of these includes based on your microphone type
#include "AnalogMicrophone.h"
//#include "I2SMicrophone.h"

// Pin definitions
#define LED_PIN 2  // Built-in LED pin for ESP32 DevKit

// === FOR ANALOG MICROPHONE ===
#define ANALOG_MIC_PIN 36
// === FOR I2S MICROPHONE ===
#define I2S_SD_PIN GPIO_NUM_32
#define I2S_SCK_PIN GPIO_NUM_25
#define I2S_WS_PIN GPIO_NUM_33

// Sound detection parameters
#define SAMPLE_RATE 8000       // Lower sample rate for level monitoring
#define SOUND_THRESHOLD 2000   // Threshold for analog mic (0-4095)
#define I2S_SOUND_THRESHOLD 5000 // Threshold for I2S mic (0-32767)
#define DEBOUNCE_TIME 1000     // Minimum time between sound events (ms)
#define DETECTION_WINDOW 500   // Time window for detecting sound (ms)

// Uncomment ONE of these lines based on your microphone type
AnalogMicrophone* mic = nullptr;
//I2SMicrophone* mic = nullptr;

// Variables for sound event detection
unsigned long lastSoundTime = 0;
bool soundDetected = false;

// Function to initialize the chosen microphone
bool initMicrophone() {
  // Uncomment ONE of these sections based on your microphone type
  
  // === FOR ANALOG MICROPHONE ===
  mic = new AnalogMicrophone(ANALOG_MIC_PIN);
  if (!mic->init(SAMPLE_RATE)) {
    Serial.println("Failed to initialize analog microphone!");
    return false;
  }
  return true;
  
  /* === FOR I2S MICROPHONE ===
  mic = new I2SMicrophone(I2S_SD_PIN, I2S_SCK_PIN, I2S_WS_PIN);
  esp_err_t err = mic->init(SAMPLE_RATE);
  if (err != ESP_OK) {
    Serial.printf("Failed to initialize I2S microphone: %s\n", esp_err_to_name(err));
    return false;
  }
  err = mic->start();
  if (err != ESP_OK) {
    Serial.printf("Failed to start I2S microphone: %s\n", esp_err_to_name(err));
    return false;
  }
  return true;
  */
}

// Function to read the current sound level
int getSoundLevel() {
  // Uncomment ONE of these sections based on your microphone type
  
  // === FOR ANALOG MICROPHONE ===
  return mic->readPeakLevel(DETECTION_WINDOW);
  
  /* === FOR I2S MICROPHONE ===
  return mic->readLevel();
  */
}

// Check if the sound exceeds the threshold
bool isSoundAboveThreshold(int level) {
  // Uncomment ONE of these sections based on your microphone type
  
  // === FOR ANALOG MICROPHONE ===
  return level > SOUND_THRESHOLD;
  
  /* === FOR I2S MICROPHONE ===
  return level > I2S_SOUND_THRESHOLD;
  */
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nSound Level Monitor Example");
  
  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize microphone
  Serial.println("Initializing microphone...");
  if (!initMicrophone()) {
    Serial.println("Microphone initialization failed!");
    while (1) {
      // Blink LED to indicate failure
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }
  
  Serial.println("Microphone initialized successfully");
  Serial.println("Monitoring sound levels...");
  Serial.println("-----------------------------------");
}

void loop() {
  // Read current sound level
  int level = getSoundLevel();
  
  // Current timestamp
  unsigned long currentTime = millis();
  
  // Check if sound is above threshold and debounce time has passed
  if (isSoundAboveThreshold(level) && (currentTime - lastSoundTime > DEBOUNCE_TIME)) {
    // Sound event detected
    soundDetected = true;
    lastSoundTime = currentTime;
    
    // Turn on LED
    digitalWrite(LED_PIN, HIGH);
    
    // Log the event
    Serial.print("Sound event detected! Level: ");
    Serial.println(level);
    
    // Here you could trigger other actions:
    // - Send notification
    // - Start recording
    // - Activate other system components
  }
  
  // If in sound detected state and debounce time has passed, reset
  if (soundDetected && (currentTime - lastSoundTime > 500)) {
    soundDetected = false;
    digitalWrite(LED_PIN, LOW);
  }
  
  // Print sound level periodically
  static unsigned long lastPrintTime = 0;
  if (currentTime - lastPrintTime > 500) {
    Serial.print("Sound level: ");
    Serial.println(level);
    lastPrintTime = currentTime;
  }
  
  // Small delay to prevent overwhelming the serial output
  delay(10);
}
