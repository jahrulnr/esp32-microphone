#include "PDMMicrophone.h"
#include "esp_err.h"
#include "esp_check.h"
#include <algorithm>

const char* PDMMicrophone::TAG = "PDMMicrophone";

PDMMicrophone::PDMMicrophone(gpio_num_t dataPin, gpio_num_t clockPin, i2s_port_t portNum)
    : _dataPin(dataPin)
    , _clockPin(clockPin)
    , _portNum(portNum)
    , _sampleRate(16000)
    , _bitsPerSample(I2S_DATA_BIT_WIDTH_16BIT)
    , _channelMode(I2S_SLOT_MODE_MONO)
    , _rxHandle(nullptr)
    , _initialized(false)
    , _active(false)
    , _highPassEnabled(true)
{
    ESP_LOGI(TAG, "PDMMicrophone created with data pin %d, clock pin %d", dataPin, clockPin);
}

PDMMicrophone::~PDMMicrophone() {
    if (_active) {
        stop();
    }
    
    if (_rxHandle) {
        esp_err_t err = i2s_del_channel(_rxHandle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete I2S channel: %s", esp_err_to_name(err));
        }
        _rxHandle = nullptr;
    }
    
    _initialized = false;
    ESP_LOGI(TAG, "PDMMicrophone destroyed");
}

esp_err_t PDMMicrophone::init(uint32_t sampleRate, i2s_data_bit_width_t bitsPerSample, 
                              i2s_slot_mode_t channels) {
    if (_initialized) {
        ESP_LOGW(TAG, "PDM microphone already initialized");
        return ESP_OK;
    }

    // Validate parameters
    if (bitsPerSample != I2S_DATA_BIT_WIDTH_16BIT) {
        ESP_LOGE(TAG, "PDM mode only supports 16-bit samples");
        return ESP_ERR_INVALID_ARG;
    }

    if (_portNum != I2S_NUM_0) {
        ESP_LOGE(TAG, "PDM is only supported on I2S_NUM_0");
        return ESP_ERR_INVALID_ARG;
    }

    _sampleRate = sampleRate;
    _bitsPerSample = bitsPerSample;
    _channelMode = channels;

    ESP_LOGI(TAG, "Initializing PDM microphone: %luHz, %d-bit, %s", 
             _sampleRate, 16, (_channelMode == I2S_SLOT_MODE_MONO) ? "Mono" : "Stereo");

    // Step 1: Create I2S channel
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;  // Enable auto clear to avoid noise
    
    esp_err_t err = i2s_new_channel(&chan_cfg, nullptr, &_rxHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(err));
        return err;
    }

    // Step 2: Configure PDM RX mode
    err = configureChannel();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PDM channel: %s", esp_err_to_name(err));
        i2s_del_channel(_rxHandle);
        _rxHandle = nullptr;
        return err;
    }

    _initialized = true;
    ESP_LOGI(TAG, "PDM microphone initialized successfully");
    return ESP_OK;
}

esp_err_t PDMMicrophone::configureChannel() {
    if (!_rxHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    // PDM RX configuration
    i2s_pdm_rx_config_t pdm_rx_cfg = {
#if SOC_I2S_SUPPORTS_PDM2PCM
        // Use PCM format if hardware supports PDM to PCM conversion
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(_sampleRate),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, _channelMode),
#else
        // Use raw PDM format for older hardware
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(_sampleRate * 64), // Oversample
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, _channelMode),
#endif
        .gpio_cfg = {
            .clk = _clockPin,
            .din = _dataPin,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    esp_err_t err = i2s_channel_init_pdm_rx_mode(_rxHandle, &pdm_rx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PDM RX mode: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "PDM channel configured successfully");
    return ESP_OK;
}

esp_err_t PDMMicrophone::start() {
    if (!_initialized) {
        ESP_LOGE(TAG, "PDM microphone not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (_active) {
        ESP_LOGW(TAG, "PDM microphone already active");
        return ESP_OK;
    }

    esp_err_t err = i2s_channel_enable(_rxHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(err));
        return err;
    }

    _active = true;
    
    // Preload DMA buffers to reduce initial latency
    preloadDMA();
    
    ESP_LOGI(TAG, "PDM microphone started");
    return ESP_OK;
}

esp_err_t PDMMicrophone::stop() {
    if (!_active) {
        return ESP_OK;
    }

    esp_err_t err = i2s_channel_disable(_rxHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable I2S channel: %s", esp_err_to_name(err));
        return err;
    }

    _active = false;
    ESP_LOGI(TAG, "PDM microphone stopped");
    return ESP_OK;
}

esp_err_t PDMMicrophone::readAudioData(void* buffer, size_t bufferSize, size_t* bytesRead, 
                                       uint32_t timeoutMs) {
    if (!_active) {
        ESP_LOGE(TAG, "PDM microphone not active");
        return ESP_ERR_INVALID_STATE;
    }

    if (!buffer || !bytesRead) {
        ESP_LOGE(TAG, "Invalid buffer or bytesRead pointer");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2s_channel_read(_rxHandle, buffer, bufferSize, bytesRead, 
                                     pdMS_TO_TICKS(timeoutMs));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Failed to read audio data: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t PDMMicrophone::readSamples(int16_t* buffer, size_t sampleCount, size_t* samplesRead, 
                                     uint32_t timeoutMs) {
    if (!buffer || !samplesRead) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t bytesToRead = sampleCount * getBytesPerSample() * getChannelCount();
    size_t bytesRead = 0;
    
    esp_err_t err = readAudioData(buffer, bytesToRead, &bytesRead, timeoutMs);
    
    *samplesRead = bytesRead / (getBytesPerSample() * getChannelCount());
    
    return err;
}

int PDMMicrophone::readSamples(int16_t* buffer, size_t sampleCount, uint32_t timeoutMs) {
    size_t samplesRead = 0;
    esp_err_t err = readSamples(buffer, sampleCount, &samplesRead, timeoutMs);
    
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        return -1;
    }
    
    return static_cast<int>(samplesRead);
}

int PDMMicrophone::readLevel() {
    const int bufferSize = 128;
    int16_t buffer[bufferSize];
    
    int samplesRead = readSamples(buffer, bufferSize, 50); // 50ms timeout
    
    if (samplesRead <= 0) {
        return -1;
    }
    
    // Find peak absolute value
    int16_t peak = 0;
    for (int i = 0; i < samplesRead; i++) {
        int16_t abs_val = abs(buffer[i]);
        if (abs_val > peak) {
            peak = abs_val;
        }
    }
    
    return peak;
}

bool PDMMicrophone::isInitialized() const {
    return _initialized;
}

bool PDMMicrophone::isActive() const {
    return _active;
}

uint32_t PDMMicrophone::getSampleRate() const {
    return _sampleRate;
}

i2s_data_bit_width_t PDMMicrophone::getBitsPerSample() const {
    return _bitsPerSample;
}

i2s_slot_mode_t PDMMicrophone::getChannelMode() const {
    return _channelMode;
}

esp_err_t PDMMicrophone::preloadDMA() {
    if (!_active) {
        return ESP_ERR_INVALID_STATE;
    }

    // Preload with silence to initialize DMA buffers
    const int preloadSize = 512;
    int16_t silenceBuffer[preloadSize] = {0};
    size_t bytesRead = 0;
    
    // Try to read some data to initialize the DMA chain
    esp_err_t err = i2s_channel_read(_rxHandle, silenceBuffer, sizeof(silenceBuffer), 
                                     &bytesRead, pdMS_TO_TICKS(10));
    
    // It's okay if this times out - we just want to prime the DMA
    return ESP_OK;
}

size_t PDMMicrophone::calculateBufferSize(uint32_t durationMs) const {
    size_t samplesPerMs = _sampleRate / 1000;
    size_t totalSamples = samplesPerMs * durationMs * getChannelCount();
    return totalSamples * getBytesPerSample();
}

uint32_t PDMMicrophone::getPDMFrequency() const {
#if SOC_I2S_SUPPORTS_PDM2PCM
    return _sampleRate * 64; // Typical oversampling ratio
#else
    return _sampleRate * 64; // Raw PDM frequency
#endif
}

esp_err_t PDMMicrophone::setHighPassFilter(bool enable) {
    _highPassEnabled = enable;
    
    // Note: High-pass filter configuration would depend on specific hardware
    // This is a placeholder for future implementation
    ESP_LOGI(TAG, "High-pass filter %s", enable ? "enabled" : "disabled");
    
    return ESP_OK;
}

size_t PDMMicrophone::getBytesPerSample() const {
    return 2; // PDM mode is always 16-bit = 2 bytes
}

size_t PDMMicrophone::getChannelCount() const {
    return (_channelMode == I2S_SLOT_MODE_MONO) ? 1 : 2;
}

esp_err_t PDMMicrophone::calculatePDMParams(uint32_t sampleRate) {
    // Validate sample rate for PDM
    if (sampleRate < 8000 || sampleRate > 48000) {
        ESP_LOGE(TAG, "Sample rate %lu Hz is not supported for PDM", sampleRate);
        return ESP_ERR_INVALID_ARG;
    }
    
    // PDM typically works with specific sample rates
    const uint32_t supportedRates[] = {8000, 16000, 22050, 44100, 48000};
    bool rateSupported = false;
    
    for (size_t i = 0; i < sizeof(supportedRates) / sizeof(supportedRates[0]); i++) {
        if (sampleRate == supportedRates[i]) {
            rateSupported = true;
            break;
        }
    }
    
    if (!rateSupported) {
        ESP_LOGW(TAG, "Sample rate %lu Hz may not be optimal for PDM", sampleRate);
    }
    
    return ESP_OK;
}
