#pragma once

#include <Arduino.h>
#include "driver/i2s_pdm.h"
#include "driver/gpio.h"
#include "esp_log.h"

/**
 * PDMMicrophone class for PDM microphone input using ESP-IDF v5+ I2S PDM API
 * 
 * This implementation uses the new ESP-IDF v5.x I2S PDM driver which provides
 * better performance, cleaner API, and improved resource management compared to
 * the legacy I2S driver. PDM (Pulse Density Modulation) is commonly used in
 * MEMS microphones and provides high-quality audio with simple wiring.
 */
class PDMMicrophone {
public:
    /**
     * Constructor for I2S PDM microphone
     * 
     * @param dataPin PDM data input pin (DIN/DATA)
     * @param clockPin PDM clock pin (CLK)
     * @param portNum I2S port number (I2S_NUM_0 only - PDM is only supported on I2S0)
     */
    PDMMicrophone(gpio_num_t dataPin, gpio_num_t clockPin, 
                  i2s_port_t portNum = I2S_NUM_0);

    /**
     * Destructor - properly cleans up I2S PDM resources
     */
    ~PDMMicrophone();

    /**
     * Initialize the I2S PDM microphone
     * 
     * @param sampleRate Sample rate in Hz (8000, 16000, 22050, 44100, 48000)
     * @param bitsPerSample Bits per sample (16 only for PDM)
     * @param channels Number of channels (1 for mono, 2 for stereo)
     * @return ESP_OK if successful, error code otherwise
     */
    esp_err_t init(uint32_t sampleRate = 16000, i2s_data_bit_width_t bitsPerSample = I2S_DATA_BIT_WIDTH_16BIT, 
                   i2s_slot_mode_t channels = I2S_SLOT_MODE_MONO);

    /**
     * Start the I2S PDM channel (begin receiving data)
     * 
     * @return ESP_OK if successful, error code otherwise
     */
    esp_err_t start();

    /**
     * Stop the I2S PDM channel
     * 
     * @return ESP_OK if successful, error code otherwise
     */
    esp_err_t stop();

    /**
     * Read audio samples from the PDM microphone
     * 
     * @param buffer Buffer to store audio samples
     * @param bufferSize Size of buffer in bytes
     * @param bytesRead Pointer to store actual bytes read
     * @param timeoutMs Timeout in milliseconds
     * @return ESP_OK if successful, error code otherwise
     */
    esp_err_t readAudioData(void* buffer, size_t bufferSize, size_t* bytesRead, 
                           uint32_t timeoutMs = 100);

    /**
     * Read audio samples into an int16_t buffer (convenience method)
     * 
     * @param buffer Buffer to store audio samples (int16_t)
     * @param sampleCount Number of samples to read
     * @param samplesRead Pointer to store actual samples read
     * @param timeoutMs Timeout in milliseconds
     * @return ESP_OK if successful, error code otherwise
     */
    esp_err_t readSamples(int16_t* buffer, size_t sampleCount, size_t* samplesRead, 
                         uint32_t timeoutMs = 100);

    /**
     * Read audio samples and return count (for compatibility with old API)
     * 
     * @param buffer Buffer to store audio samples (int16_t)
     * @param sampleCount Number of samples to read
     * @param timeoutMs Timeout in milliseconds
     * @return Number of samples read, or -1 on error
     */
    int readSamples(int16_t* buffer, size_t sampleCount, uint32_t timeoutMs = 100);

    /**
     * Read current audio level (peak value from a sample buffer)
     * 
     * @return Peak audio level (0-32767), or -1 on error
     */
    int readLevel();

    /**
     * Check if the microphone is properly initialized
     * 
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;

    /**
     * Check if the I2S PDM channel is currently active
     * 
     * @return true if active, false otherwise
     */
    bool isActive() const;

    /**
     * Get current sample rate
     * 
     * @return Sample rate in Hz
     */
    uint32_t getSampleRate() const;

    /**
     * Get current bits per sample
     * 
     * @return Bits per sample (always 16 for PDM)
     */
    i2s_data_bit_width_t getBitsPerSample() const;

    /**
     * Get current channel mode
     * 
     * @return Channel mode (mono/stereo)
     */
    i2s_slot_mode_t getChannelMode() const;

    /**
     * Preload DMA buffers to reduce initial latency
     * 
     * @return ESP_OK if successful, error code otherwise
     */
    esp_err_t preloadDMA();

    /**
     * Calculate the optimal buffer size for given duration
     * 
     * @param durationMs Duration in milliseconds
     * @return Buffer size in bytes
     */
    size_t calculateBufferSize(uint32_t durationMs) const;

    /**
     * Get the effective PDM frequency being used
     * 
     * @return PDM frequency in Hz
     */
    uint32_t getPDMFrequency() const;

    /**
     * Enable/disable high-pass filter (if supported by hardware)
     * 
     * @param enable True to enable high-pass filter
     * @return ESP_OK if successful, error code otherwise
     */
    esp_err_t setHighPassFilter(bool enable);

private:
    static const char* TAG;

    // Hardware configuration
    gpio_num_t _dataPin;
    gpio_num_t _clockPin;
    i2s_port_t _portNum;

    // I2S PDM configuration
    uint32_t _sampleRate;
    i2s_data_bit_width_t _bitsPerSample;
    i2s_slot_mode_t _channelMode;

    // I2S handles
    i2s_chan_handle_t _rxHandle;

    // State flags
    bool _initialized;
    bool _active;
    bool _highPassEnabled;

    /**
     * Configure I2S PDM channel
     * 
     * @return ESP_OK if successful, error code otherwise
     */
    esp_err_t configureChannel();

    /**
     * Get bytes per sample based on bit width
     * 
     * @return Bytes per sample (always 2 for PDM)
     */
    size_t getBytesPerSample() const;

    /**
     * Get number of channels as integer
     * 
     * @return Number of channels (1 or 2)
     */
    size_t getChannelCount() const;

    /**
     * Calculate optimal PDM configuration parameters
     * 
     * @param sampleRate Desired sample rate
     * @return ESP_OK if parameters are valid
     */
    esp_err_t calculatePDMParams(uint32_t sampleRate);
};
