# ESP32 Microphone Library

A versatile audio input library for ESP32 microcontrollers supporting analog microphones, digital I2S microphones, and PDM microphones.

## Overview

This library simplifies audio capture for any ESP32-based project:
- Universal support for common microphone types (Analog, I2S Standard, and PDM)
- High-quality audio sampling optimized for voice applications
- Simple API with both basic and advanced functions
- Ready for integration with audio processing systems
- Uses modern ESP-IDF v5+ APIs for optimal performance

Perfect for voice control, sound detection, audio recording, and acoustic monitoring applications.

## Features

- **Universal Microphone Support**: Compatible with various microphone hardware
  - Analog microphones (MAX9814, MAX4466, etc.)
  - I2S Standard digital microphones (INMP441, SPM1423, etc.)
  - PDM (Pulse Density Modulation) microphones (SPM1423, MP34DT05, etc.)
- **Modern ESP-IDF v5+ Support**: Uses latest I2S Standard and PDM APIs
- **Flexible Configuration**: Adapt to different audio requirements
- **Simple API**: Consistent interface across all microphone types
- **Audio Analysis**: Built-in functions for sound level detection
- **Efficient Performance**: Optimized for real-time applications
- **Hardware Abstraction**: Isolates your application from hardware details

## Microphone Types Comparison

| Feature | Analog | I2S Standard | PDM |
|---------|--------|--------------|-----|
| **Wiring Complexity** | Simple (1 wire) | Moderate (3 wires) | Simple (2 wires) |
| **Audio Quality** | Good | Excellent | Excellent |
| **Digital Noise Immunity** | Low | High | High |
| **ESP32 ADC Required** | Yes | No | No |
| **GPIO Pins Required** | 1 (+optional gain/AR) | 3 (Data, Clock, WS) | 2 (Data, Clock) |
| **Supported ESP32 Ports** | Any ADC pin | I2S_NUM_0, I2S_NUM_1 | I2S_NUM_0 only |
| **Sample Rate Range** | Limited by ADC | High (up to 48kHz+) | High (up to 48kHz+) |
| **Power Consumption** | Low-Medium | Medium | Low-Medium |
| **Cost** | Low | Medium | Medium |
| **Best For** | Simple projects, battery use | High-quality audio, stereo | High-quality audio, simple wiring |

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

### PDM Digital Microphone

```cpp
#include "PDMMicrophone.h"

// Create a PDM microphone instance
// Parameters: data pin, clock pin, I2S port (only I2S_NUM_0 supports PDM)
PDMMicrophone* mic = new PDMMicrophone(DATA_PIN, CLOCK_PIN);

void setup() {
  Serial.begin(115200);
  
  // Initialize with desired configuration
  esp_err_t err = mic->init(16000);  // 16kHz sample rate
  if (err != ESP_OK) {
    Serial.println("PDM microphone initialization failed");
    return;
  }
  
  // Start the PDM interface
  mic->start();
  
  Serial.println("PDM microphone ready");
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
- `int readSamplesWithTiming(int16_t* buffer, size_t sampleCount)`: Read samples with precise timing control

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
- `int readSamples(int16_t* buffer, size_t sampleCount, uint32_t timeoutMs = 100)`: Read audio samples (automatically converts to 16-bit)

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

### PDMMicrophone Class

#### Constructor
```cpp
PDMMicrophone(gpio_num_t dataPin, gpio_num_t clockPin, i2s_port_t portNum = I2S_NUM_0)
```
- `dataPin`: PDM data input pin (DIN/DATA)
- `clockPin`: PDM clock pin (CLK)
- `portNum`: I2S port number (I2S_NUM_0 only - PDM is only supported on I2S0)

#### Core Methods
- `esp_err_t init(uint32_t sampleRate = 16000, i2s_data_bit_width_t bitsPerSample = I2S_DATA_BIT_WIDTH_16BIT, i2s_slot_mode_t channels = I2S_SLOT_MODE_MONO)`: Initialize PDM
- `esp_err_t start()`: Start PDM channel
- `esp_err_t stop()`: Stop PDM channel
- `int readLevel()`: Read current audio level
- `int readSamples(int16_t* buffer, size_t sampleCount, uint32_t timeoutMs = 100)`: Read audio samples

#### Advanced Methods
- `esp_err_t readAudioData(void* buffer, size_t bufferSize, size_t* bytesRead, uint32_t timeoutMs = 100)`: Low-level audio reading
- `esp_err_t readSamples(int16_t* buffer, size_t sampleCount, size_t* samplesRead, uint32_t timeoutMs = 100)`: Read with detailed info
- `esp_err_t preloadDMA()`: Preload DMA buffers to reduce initial latency
- `size_t calculateBufferSize(uint32_t durationMs)`: Calculate buffer size for duration
- `uint32_t getPDMFrequency() const`: Get the effective PDM frequency being used
- `esp_err_t setHighPassFilter(bool enable)`: Enable/disable high-pass filter

#### Status Methods
- `bool isInitialized() const`: Check if PDM is initialized
- `bool isActive() const`: Check if PDM channel is active
- `uint32_t getSampleRate() const`: Get sample rate
- `i2s_data_bit_width_t getBitsPerSample() const`: Get bits per sample (always 16-bit for PDM)
- `i2s_slot_mode_t getChannelMode() const`: Get channel mode

## Advanced Usage

### Audio Processing Integration

```cpp
// Initialize microphone (example with I2S)
I2SMicrophone* mic = new I2SMicrophone(DATA_PIN, CLOCK_PIN, WS_PIN);
mic->init(16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
mic->start();

// For PDM microphone, use:
// PDMMicrophone* mic = new PDMMicrophone(DATA_PIN, CLOCK_PIN);
// mic->init(16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
// mic->start();

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

1. **Universal Support**: Works with analog, I2S Standard, and PDM microphone hardware
2. **Consistent API**: Same core functions across different microphone types
3. **Modern ESP-IDF**: Uses ESP-IDF v5+ APIs for optimal performance and reliability
4. **Performance Optimized**: Designed for reliable real-time audio applications
5. **Easy Integration**: Drop-in solution for voice and audio projects
6. **Customizable**: Extensive configuration options for specific requirements
7. **Future-Proof**: Built with modern ESP32 development practices

## Compatibility

- All ESP32 family: ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6, ESP32-H2
- Arduino ESP32 and ESP-IDF frameworks
- Requires ESP-IDF v5.0+ for I2S Standard and PDM support
- Compatible with common microphone hardware:
  - **I2S Standard MEMS microphones**: INMP441, SPM1423, ICS43432, etc.
  - **PDM MEMS microphones**: SPM1423, MP34DT05, MP34DT01, etc.
  - **Analog microphones**: MAX9814, MAX4466, etc.

### Hardware Notes
- **PDM microphones**: Only supported on I2S_NUM_0 port
- **I2S Standard microphones**: Supported on both I2S_NUM_0 and I2S_NUM_1 ports
- **Analog microphones**: Can use any ADC-capable GPIO pin

## Requirements

### Software Requirements
- **ESP-IDF v5.0+**: Required for I2S Standard and PDM support
- **Arduino ESP32 Core**: v2.0.5+ (includes ESP-IDF v4.4+, but v5.0+ recommended)
- **PlatformIO**: Latest version with ESP32 platform

### Hardware Requirements
- ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6, or ESP32-H2 development board
- Supported microphone hardware (see compatibility list above)
- Proper wiring and power supply for chosen microphone type

## Examples

The library includes several example applications:
- **AnalogMicrophoneBasic**: Basic analog microphone usage with MAX9814/MAX4466
- **I2SMicrophoneBasic**: I2S Standard microphone usage with INMP441/SPM1423
- **PDMMicrophoneBasic**: PDM microphone usage with SPM1423/MP34DT05
- **SoundLevelMonitor**: Comprehensive sound detection with multiple microphone types
- Basic audio capture and level monitoring
- Sound event detection with debouncing
- Audio recording and playback
- Integration with audio processing systems

## Troubleshooting

### Common Issues

**I2S/PDM microphone not working:**
- Ensure ESP-IDF version is 5.0+
- Check that GPIO pins are correctly connected
- Verify power supply (3.3V) is stable
- For PDM: Only use I2S_NUM_0 port

**Analog microphone noisy or not working:**
- Check that the analog pin supports ADC
- Ensure proper grounding
- Try different ADC resolution settings
- Use shielded cables for long connections

**Audio quality issues:**
- Try different sample rates (16kHz recommended for voice)
- Check for electrical interference
- Ensure adequate power supply
- Consider using digital microphones for better noise immunity

## License

MIT
