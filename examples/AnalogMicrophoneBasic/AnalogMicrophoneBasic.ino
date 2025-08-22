/**
 * AnalogMicrophoneBasic.ino
 * 
 * Basic example for using the AnalogMicrophone class with an analog microphone 
 * such as MAX9814 or MAX4466.
 * 
 * This example:
 * 1. Initializes an analog microphone
 * 2. Reads audio levels
 * 3. Detects sounds above a threshold
 * 4. Reads audio samples for processing
 * 
 * Hardware connections:
 * - Connect microphone OUT pin to an analog input (GPIO36 in this example)
 * - Optional: Connect microphone GAIN pin to a digital output for gain control
 * - Optional: Connect microphone A/R (attack/release) pin for timing control
 * 
 * Library: https://github.com/jahrulnr/esp32-microphone
 */

#include <Arduino.h>
#include "AnalogMicrophone.h"

// Pin definitions - change these to match your wiring
#define MIC_PIN 36        // Analog input connected to microphone output
#define GAIN_PIN -1       // Optional: GPIO pin for gain control (or -1 if not connected)
#define AR_PIN -1         // Optional: GPIO pin for attack/release control (or -1 if not connected)

// Audio configuration
#define SAMPLE_RATE 16000 // 16kHz sample rate
#define SOUND_THRESHOLD 2000 // Threshold for sound detection

// Create microphone instance
AnalogMicrophone* mic = nullptr;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to open
  Serial.println("\nAnalog Microphone Example");
  
  // Create and initialize microphone
  mic = new AnalogMicrophone(MIC_PIN, GAIN_PIN, AR_PIN);
  
  Serial.println("Initializing microphone...");
  if (!mic->init(SAMPLE_RATE)) {
    Serial.println("Failed to initialize microphone!");
    while (1) { delay(100); } // Halt if initialization failed
  }
  
  Serial.println("Microphone initialized successfully");
  Serial.println("Calibrating baseline noise level...");
  
  // Calibrate the microphone to establish baseline noise level
  // Make sure the environment is quiet during calibration
  int baseline = mic->calibrateBaseline(1000);
  Serial.print("Baseline noise level: ");
  Serial.println(baseline);
  
  // If using gain control pin, you can set gain level
  if (GAIN_PIN >= 0) {
    Serial.println("Setting gain to 40dB (LOW)");
    mic->setGain(LOW); // LOW = 40dB, HIGH = 50dB, or use INPUT mode for 60dB
  }
  
  Serial.println("Ready! Monitoring audio levels...");
  Serial.println("-----------------------------------");
}

void loop() {
  // Read current audio level
  int level = mic->readLevel();
  
  // Read peak level over 100ms
  int peakLevel = mic->readPeakLevel(100);
  
  // Read average level over 100ms
  int avgLevel = mic->readAverageLevel(100);
  
  // Print levels every second
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) {
    Serial.print("Audio levels - Current: ");
    Serial.print(level);
    Serial.print(", Peak: ");
    Serial.print(peakLevel);
    Serial.print(", Average: ");
    Serial.println(avgLevel);
    lastPrintTime = millis();
  }
  
  // Check for sound detection
  if (mic->isSoundDetected(SOUND_THRESHOLD)) {
    // Only print when first detected to avoid flooding the serial monitor
    static bool soundActive = false;
    if (!soundActive) {
      Serial.println("*** Sound detected! ***");
      soundActive = true;
    }
  } else {
    static bool soundActive = true;
    if (soundActive) {
      soundActive = false;
    }
  }
  
  // Demonstrate reading audio samples
  // This is typically done when sound is detected or on a regular interval
  static unsigned long lastSampleTime = 0;
  if (millis() - lastSampleTime > 5000) { // Every 5 seconds
    const int bufferSize = 512;
    int16_t buffer[bufferSize];
    
    Serial.println("Reading audio samples...");
    int samplesRead = mic->readSamples(buffer, bufferSize);
    
    Serial.print("Read ");
    Serial.print(samplesRead);
    Serial.println(" samples");
    
    // Print first 10 samples for demonstration
    Serial.println("Sample values (first 10):");
    for (int i = 0; i < min(10, samplesRead); i++) {
      Serial.print(buffer[i]);
      Serial.print(" ");
    }
    Serial.println();
    
    lastSampleTime = millis();
  }
  
  // Small delay to prevent overwhelming the serial output
  delay(10);
}
