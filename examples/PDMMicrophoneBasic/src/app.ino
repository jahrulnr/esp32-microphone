/**
 * PDMMicrophoneBasic.ino
 * 
 * Basic example for using the PDMMicrophone class with PDM MEMS microphones
 * such as SPM1423, MP34DT05, or similar PDM microphones.
 * 
 * This example:
 * 1. Initializes a PDM microphone
 * 2. Starts the I2S PDM channel
 * 3. Reads audio levels
 * 4. Reads audio samples for processing
 * 
 * Hardware connections for PDM microphone:
 * - Connect microphone VDD to 3.3V
 * - Connect microphone GND to ground
 * - Connect microphone CLK to GPIO25 (or your chosen pin)
 * - Connect microphone DATA to GPIO32 (or your chosen pin)
 * - Connect microphone L/R to GND for left channel or VDD for right channel
 * 
 * Library: https://github.com/jahrulnr/esp32-microphone
 */

#include <Arduino.h>
#include "PDMMicrophone.h"

// Pin definitions for PDM microphone - change these to match your wiring
// #define PDM_DATA_PIN GPIO_NUM_41  // PDM data input pin
// #define PDM_CLK_PIN GPIO_NUM_42   // PDM clock pin
#define PDM_DATA_PIN GPIO_NUM_41  // PDM data input pin
#define PDM_CLK_PIN GPIO_NUM_42   // PDM clock pin
#define I2S_PORT I2S_NUM_0        // I2S port (PDM only works on I2S_NUM_0)

// Audio configuration
#define SAMPLE_RATE 16000                     // 16kHz sample rate
#define BIT_DEPTH I2S_DATA_BIT_WIDTH_16BIT    // 16-bit samples (only option for PDM)
#define CHANNELS I2S_SLOT_MODE_MONO           // Mono channel

// Create PDM microphone instance
PDMMicrophone* mic = nullptr;

// For level visualization
const int numLevels = 20;
const int levelThreshold = 32768 / numLevels;

void printAudioLevel(int level) {
  // Normalize level to 0-numLevels range
  int normalizedLevel = map(level, 0, 32767, 0, numLevels);
  
  // Print a bar graph
  Serial.print("PDM Level [");
  for (int i = 0; i < numLevels; i++) {
    if (i < normalizedLevel) {
      Serial.print("#");
    } else {
      Serial.print(" ");
    }
  }
  Serial.print("] ");
  Serial.println(level);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to open
  Serial.println("\nPDM Microphone Example");
  
  // Create PDM microphone instance
  mic = new PDMMicrophone(PDM_DATA_PIN, PDM_CLK_PIN, I2S_PORT);
  
  Serial.println("Initializing PDM microphone...");
  esp_err_t err = mic->init(SAMPLE_RATE, BIT_DEPTH, CHANNELS);
  if (err != ESP_OK) {
    Serial.printf("Failed to initialize PDM microphone: %s\n", esp_err_to_name(err));
    while (1) { delay(100); } // Halt if initialization failed
  }
  
  Serial.println("Starting PDM microphone...");
  err = mic->start();
  if (err != ESP_OK) {
    Serial.printf("Failed to start PDM microphone: %s\n", esp_err_to_name(err));
    while (1) { delay(100); } // Halt if start failed
  }
  
  // Allow time for the microphone to stabilize
  delay(500);
  
  Serial.println("PDM microphone ready!");
  Serial.printf("Configuration: %dHz, %d-bit, %s\n", 
               mic->getSampleRate(),
               16, // PDM is always 16-bit
               (mic->getChannelMode() == I2S_SLOT_MODE_MONO) ? "Mono" : "Stereo");
  Serial.printf("PDM Frequency: %dHz\n", mic->getPDMFrequency());
  Serial.println("Monitoring audio levels...");
  Serial.println("-----------------------------------");
}

void loop() {
  // Read current audio level
  int level = mic->readLevel();
  
  // Print level visualization
  static unsigned long lastLevelTime = 0;
  if (millis() - lastLevelTime > 200) { // Every 200ms
    printAudioLevel(level);
    lastLevelTime = millis();
  }
  
  // Demonstrate reading audio samples
  static unsigned long lastSampleTime = 0;
  if (millis() - lastSampleTime > 5000) { // Every 5 seconds
    const int bufferSize = 512;
    int16_t buffer[bufferSize];
    
    Serial.println("\nReading audio samples...");
    int samplesRead = mic->readSamples(buffer, bufferSize, 100); // 100ms timeout
    
    if (samplesRead > 0) {
      Serial.printf("Read %d samples\n", samplesRead);
      
      // Calculate some basic audio metrics
      int32_t sum = 0;
      int16_t minSample = 32767;
      int16_t maxSample = -32768;
      
      for (int i = 0; i < samplesRead; i++) {
        sum += buffer[i];
        if (buffer[i] < minSample) minSample = buffer[i];
        if (buffer[i] > maxSample) maxSample = buffer[i];
      }
      
      float average = (float)sum / samplesRead;
      
      Serial.printf("Min: %d, Max: %d, Avg: %.2f\n", minSample, maxSample, average);
      
      // Print first 10 samples for demonstration
      Serial.println("Sample values (first 10):");
      for (int i = 0; i < min(10, samplesRead); i++) {
        Serial.print(buffer[i]);
        Serial.print(" ");
      }
      Serial.println("\n");
    } else {
      Serial.println("Failed to read samples or timeout occurred");
    }
    
    lastSampleTime = millis();
  }
  
  // Small delay to prevent overwhelming the CPU
  delay(10);
}
