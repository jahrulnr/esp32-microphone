#include "AnalogMicrophone.h"

AnalogMicrophone::AnalogMicrophone(int analogPin, int gainPin, int attackReleasePin) 
    : _analogPin(analogPin), _gainPin(gainPin), _attackReleasePin(attackReleasePin), 
      _initialized(false), _baselineLevel(0), _sampleRate(8000), _lastSampleTime(0) {
}

AnalogMicrophone::~AnalogMicrophone() {
    // No special cleanup needed for analog pins
}

bool AnalogMicrophone::init(uint16_t sampleRate) {
    if (_initialized) {
        return true; // Already initialized
    }

    // Validate analog pin
    if (_analogPin < 0) {
        return false;
    }

    // Set up gain control pin if specified
    if (_gainPin >= 0) {
        pinMode(_gainPin, OUTPUT);
        digitalWrite(_gainPin, LOW); // Default to 40dB gain
    }

    // Set up attack/release control pin if specified
    if (_attackReleasePin >= 0) {
        pinMode(_attackReleasePin, OUTPUT);
        digitalWrite(_attackReleasePin, LOW); // Default to fast attack/release
    }

    // Set sample rate
    _sampleRate = sampleRate;
    
    // Set ADC resolution to 12 bits for better precision
    analogReadResolution(10);
    
    // Allow some time for the microphone amplifier to stabilize
    delay(100);

    // Calibrate baseline noise level
    _baselineLevel = calibrateBaseline(500);

    _initialized = true;
    _lastSampleTime = micros();
    return true;
}

int AnalogMicrophone::readLevel() {
    if (!_initialized) {
        return -1;
    }

    return analogRead(_analogPin);
}

int AnalogMicrophone::readPeakLevel(int durationMs) {
    if (!_initialized) {
        return -1;
    }

    int peakLevel = 0;
    unsigned long startTime = millis();
    
    while (millis() - startTime < durationMs) {
        int currentLevel = analogRead(_analogPin);
        if (currentLevel > peakLevel) {
            peakLevel = currentLevel;
        }
        delayMicroseconds(100); // Small delay to prevent overwhelming the ADC
    }

    return peakLevel;
}

int AnalogMicrophone::readAverageLevel(int durationMs) {
    if (!_initialized) {
        return -1;
    }

    long totalLevel = 0;
    int sampleCount = 0;
    unsigned long startTime = millis();
    
    while (millis() - startTime < durationMs) {
        totalLevel += analogRead(_analogPin);
        sampleCount++;
        delayMicroseconds(100); // Small delay to prevent overwhelming the ADC
    }

    return sampleCount > 0 ? (int)(totalLevel / sampleCount) : 0;
}

bool AnalogMicrophone::isSoundDetected(int threshold) {
    if (!_initialized) {
        return false;
    }

    int currentLevel = readLevel();
    return (currentLevel - _baselineLevel) > threshold;
}

void AnalogMicrophone::setGain(int gainLevel) {
    if (_gainPin >= 0) {
        if (gainLevel == LOW) {
            digitalWrite(_gainPin, LOW);  // 40dB gain
        } else if (gainLevel == HIGH) {
            digitalWrite(_gainPin, HIGH); // 50dB gain
        } else {
            // For 60dB gain, set pin as input (floating)
            pinMode(_gainPin, INPUT);
        }
    }
}

void AnalogMicrophone::setAttackRelease(bool attackRelease) {
    if (_attackReleasePin >= 0) {
        digitalWrite(_attackReleasePin, attackRelease ? HIGH : LOW);
    }
}

int AnalogMicrophone::calibrateBaseline(int samplingTime) {
    if (!_initialized && samplingTime <= 0) {
        return 0;
    }

    long totalLevel = 0;
    int sampleCount = 0;
    unsigned long startTime = millis();
    
    // Sample the environment noise for the specified time
    while (millis() - startTime < samplingTime) {
        totalLevel += analogRead(_analogPin);
        sampleCount++;
        delay(5); // 5ms delay between samples
    }

    int baseline = sampleCount > 0 ? (int)(totalLevel / sampleCount) : 0;
    
    if (_initialized) {
        _baselineLevel = baseline;
    }
    
    return baseline;
}

bool AnalogMicrophone::isInitialized() const {
    return _initialized;
}

int* AnalogMicrophone::readSamples(int samples, int delayMs) {
    if (!_initialized || samples <= 0) {
        return nullptr;
    }

    int* sampleArray = new int[samples];
    
    for (int i = 0; i < samples; i++) {
        sampleArray[i] = analogRead(_analogPin);
        if (delayMs > 0) {
            vTaskDelay(pdMS_TO_TICKS(delayMs));
        }
    }

    return sampleArray;
}

int AnalogMicrophone::readSamples(int16_t* buffer, size_t sampleCount, uint32_t timeoutMs){
    if (!buffer || sampleCount == 0) {
        return ESP_FAIL;
    }

    if (!_initialized || !buffer || sampleCount == 0) {
        return 0;
    }
    
    // Calculate time between samples in microseconds
    unsigned long microsPerSample = 1000000 / _sampleRate;
    size_t samplesRead = 0;
    
    for (size_t i = 0; i < sampleCount; i++) {
        // Wait until it's time to take the next sample
        unsigned long currentTime = micros();
        unsigned long elapsedMicros = currentTime - _lastSampleTime;
        
        if (elapsedMicros < microsPerSample) {
            // Need to wait
            delayMicroseconds(microsPerSample - elapsedMicros);
            currentTime = micros();
        }
        
        // Read the sample
        int rawSample = analogRead(_analogPin);
        // Convert to 16-bit signed integer (-32768 to 32767)
        buffer[i] = (int16_t)map(rawSample, 0, 4095, -32768, 32767);
        
        samplesRead++;
        _lastSampleTime = currentTime;
    }
    
    return samplesRead;
}

bool AnalogMicrophone::setSampleRate(uint16_t sampleRate) {
    if (sampleRate < 1000 || sampleRate > 48000) {
        // Limit to reasonable range
        return false;
    }
    
    _sampleRate = sampleRate;
    return true;
}

uint16_t AnalogMicrophone::getSampleRate() const {
    return _sampleRate;
}

int AnalogMicrophone::readSamplesWithTiming(int16_t* buffer, size_t sampleCount) {
    if (!_initialized || !buffer || sampleCount == 0) {
        return 0;
    }
    
    // Calculate time between samples in microseconds
    unsigned long microsPerSample = 1000000 / _sampleRate;
    size_t samplesRead = 0;
    
    for (size_t i = 0; i < sampleCount; i++) {
        // Wait until it's time to take the next sample
        unsigned long currentTime = micros();
        unsigned long elapsedMicros = currentTime - _lastSampleTime;
        
        if (elapsedMicros < microsPerSample) {
            // Need to wait
            delayMicroseconds(microsPerSample - elapsedMicros);
            currentTime = micros();
        }
        
        // Read the sample
        int rawSample = analogRead(_analogPin);
        // Convert to 16-bit signed integer (-32768 to 32767)
        buffer[i] = (int16_t)map(rawSample, 0, 4095, -32768, 32767);
        
        samplesRead++;
        _lastSampleTime = currentTime;
    }
    
    return samplesRead;
}
