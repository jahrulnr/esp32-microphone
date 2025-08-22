# ESP32 Microphone Library

A versatile audio input library for ESP32 microcontrollers supporting both analog and digital I2S microphones.

## Overview

This library simplifies audio capture for any ESP32-based project:
- Universal support for common microphone types
- High-quality audio sampling optimized for voice applications
- Simple API with both basic and advanced functions
- Ready for integration with audio processing systems

Perfect for voice control, sound detection, audio recording, and acoustic monitoring applications.

## Features

- **Universal Microphone Support**: Compatible with various microphone hardware
- **Flexible Configuration**: Adapt to different audio requirements
- **Simple API**: Consistent interface for both microphone types
- **Audio Analysis**: Built-in functions for sound level detection
- **Efficient Performance**: Optimized for real-time applications
- **Hardware Abstraction**: Isolates your application from hardware details

## Basic Usage

### Analog Microphone

```cpp
#include "AnalogMicrophone.h"

// Create an analog microphone instance
// Parameters: analog pin, optional gain pin, optional attack/release pin
AnalogMicrophone* mic = new AnalogMicrophone(MIC_PIN);

void setup() {
  Serial.begin(115200);
  
  // Initialize with desired sample rate
  if (!mic->init(16000)) {
    Serial.println("Microphone initialization failed");
    return;
  }
  
  Serial.println("Microphone ready");
}

void loop() {
  // Get current audio level
  int level = mic->readLevel();
  
  // Detect sound above threshold
  if (mic->isSoundDetected(2000)) {
    // Handle sound detection
  }
  
  // Read audio samples
  int16_t buffer[512];
  int samplesRead = mic->readSamples(buffer, 512);
  
  // Process audio data as needed
  
  delay(100);
}
```

### I2S Digital Microphone

```cpp
#include "I2SMicrophone.h"

// Create an I2S microphone instance
// Parameters: data pin, clock pin, word select pin, I2S port
I2SMicrophone* mic = new I2SMicrophone(DATA_PIN, CLOCK_PIN, WS_PIN);

void setup() {
  Serial.begin(115200);
  
  // Initialize with desired configuration
  esp_err_t err = mic->init(16000);  // 16kHz sample rate
  if (err != ESP_OK) {
    Serial.println("Microphone initialization failed");
    return;
  }
  
  // Start the I2S interface
  mic->start();
  
  Serial.println("Microphone ready");
}

void loop() {
  // Read audio samples
  int16_t buffer[512];
  int samplesRead = mic->readSamples(buffer, 512);
  
  if (samplesRead > 0) {
    // Process audio data as needed
  }
  
  delay(100);
}
```

## API Reference

### AnalogMicrophone Class

#### Constructor
```cpp
AnalogMicrophone(int analogPin, int gainPin = -1, int attackReleasePin = -1)
```
- `analogPin`: ADC pin connected to microphone output
- `gainPin`: Optional gain control pin
- `attackReleasePin`: Optional attack/release control pin

#### Core Methods
- `bool init(uint16_t sampleRate = 16000)`: Initialize the microphone
- `int readLevel()`: Read current audio level
- `int readPeakLevel(int durationMs = 100)`: Read peak level over time
- `int readAverageLevel(int durationMs = 100)`: Read average level over time
- `bool isSoundDetected(int threshold = 2000)`: Check if sound is above threshold
- `int readSamples(int16_t* buffer, size_t sampleCount, uint32_t timeoutMs = 100)`: Read audio samples into buffer

#### Configuration Methods
- `void setGain(int gainLevel)`: Set microphone gain (LOW = 40dB, HIGH = 50dB, etc.)
- `void setAttackRelease(bool attackRelease)`: Set attack/release time
- `int calibrateBaseline(int samplingTime = 1000)`: Calibrate background noise level
- `bool setSampleRate(uint16_t sampleRate)`: Set sample rate
- `uint16_t getSampleRate() const`: Get current sample rate
- `bool isInitialized() const`: Check if microphone is initialized

### I2SMicrophone Class

#### Constructor
```cpp
I2SMicrophone(gpio_num_t dataPin, gpio_num_t clockPin, gpio_num_t wordSelectPin, i2s_port_t portNum = I2S_NUM_0)
```
- `dataPin`: I2S data input pin (DIN/SD)
- `clockPin`: I2S bit clock pin (BCLK/SCK)
- `wordSelectPin`: I2S word select pin (WS/LRCLK)
- `portNum`: I2S port number (I2S_NUM_0 or I2S_NUM_1)

#### Core Methods
- `esp_err_t init(uint32_t sampleRate = 16000, i2s_data_bit_width_t bitsPerSample = I2S_DATA_BIT_WIDTH_16BIT, i2s_slot_mode_t channels = I2S_SLOT_MODE_MONO)`: Initialize I2S
- `esp_err_t start()`: Start I2S channel
- `esp_err_t stop()`: Stop I2S channel
- `int readLevel()`: Read current audio level
- `int readSamples(int16_t* buffer, size_t sampleCount, uint32_t timeoutMs = 100)`: Read audio samples

#### Advanced Methods
- `esp_err_t readAudioData(void* buffer, size_t bufferSize, size_t* bytesRead, uint32_t timeoutMs = 100)`: Low-level audio reading
- `esp_err_t readSamples(int16_t* buffer, size_t sampleCount, size_t* samplesRead, uint32_t timeoutMs = 100)`: Read with detailed info
- `esp_err_t preloadDMA()`: Preload DMA buffers to reduce initial latency
- `size_t calculateBufferSize(uint32_t durationMs)`: Calculate buffer size for duration

#### Status Methods
- `bool isInitialized() const`: Check if I2S is initialized
- `bool isActive() const`: Check if I2S channel is active
- `uint32_t getSampleRate() const`: Get sample rate
- `i2s_data_bit_width_t getBitsPerSample() const`: Get bits per sample
- `i2s_slot_mode_t getChannelMode() const`: Get channel mode

## Advanced Usage

### Audio Processing Integration

```cpp
// Initialize microphone
I2SMicrophone* mic = new I2SMicrophone(DATA_PIN, CLOCK_PIN, WS_PIN);
mic->init(16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
mic->start();

// Audio processing loop
void processAudio() {
  const int bufferSize = 512;
  int16_t buffer[bufferSize];
  
  while (true) {
    int samplesRead = mic->readSamples(buffer, bufferSize);
    if (samplesRead > 0) {
      // Process audio data with your algorithm
      processAudioData(buffer, samplesRead);
    }
    // Small delay to prevent CPU hogging
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
```

### Sound Event Detection

```cpp
// Initialize microphone
AnalogMicrophone* mic = new AnalogMicrophone(MIC_PIN);
mic->init(8000);  // 8kHz sample rate

// Monitor for sound events
void detectSoundEvents() {
  const int threshold = 2000;
  const int cooldownTime = 1000;  // ms
  unsigned long lastEventTime = 0;
  
  while (true) {
    int level = mic->readPeakLevel(100);
    unsigned long currentTime = millis();
    
    // Detect events with cooldown
    if (level > threshold && (currentTime - lastEventTime > cooldownTime)) {
      // Sound event detected!
      handleSoundEvent();
      lastEventTime = currentTime;
    }
    
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
```

## Why Choose This Library?

1. **Universal Support**: Works with both analog and digital microphone hardware
2. **Consistent API**: Same core functions across different microphone types
3. **Performance Optimized**: Designed for reliable real-time audio applications
4. **Easy Integration**: Drop-in solution for voice and audio projects
5. **Customizable**: Extensive configuration options for specific requirements

## Compatibility

- All ESP32 family: ESP32, ESP32-S2, ESP32-S3, ESP32-C3
- Arduino ESP32 and ESP-IDF frameworks
- Compatible with common microphone hardware:
  - I2S MEMS microphones (INMP441, SPM1423, etc.)
  - Analog microphones (MAX9814, MAX4466, etc.)

## Examples

The library includes several example applications:
- Basic audio capture and level monitoring
- Sound event detection
- Audio recording and playback
- Integration with audio processing systems

## License

MIT
