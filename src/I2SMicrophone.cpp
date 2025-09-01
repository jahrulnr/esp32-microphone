#include "I2SMicrophone.h"
#include <cstring>

const char* I2SMicrophone::TAG = "I2SMicrophone";

I2SMicrophone::I2SMicrophone(gpio_num_t dataPin, gpio_num_t clockPin, gpio_num_t wordSelectPin, 
                         i2s_port_t portNum)
    : _dataPin(dataPin), _clockPin(clockPin), _wordSelectPin(wordSelectPin), _portNum(portNum),
      _sampleRate(16000), _bitsPerSample(I2S_DATA_BIT_WIDTH_16BIT), _channelMode(I2S_SLOT_MODE_MONO),
      _rxHandle(nullptr), _initialized(false), _active(false), 
      _lastLevel(nullptr), _lastBufferLevel(-1) {
    
    ESP_LOGI(TAG, "I2SMicrophone created for port %d, pins: DATA=%d, CLK=%d, WS=%d", 
             _portNum, _dataPin, _clockPin, _wordSelectPin);
}

I2SMicrophone::~I2SMicrophone() {
    if (_active) {
        stop();
    }
    
    if (_initialized && _rxHandle) {
        esp_err_t ret = i2s_del_channel(_rxHandle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete I2S channel: %s", esp_err_to_name(ret));
        }
        _rxHandle = nullptr;
        _initialized = false;
    }
    
    ESP_LOGI(TAG, "I2SMicrophone destroyed");
}

esp_err_t I2SMicrophone::init(uint32_t sampleRate, i2s_data_bit_width_t bitsPerSample, 
                           i2s_slot_mode_t channels) {
    if (_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    _sampleRate = sampleRate;
    _bitsPerSample = bitsPerSample;
    _channelMode = channels;

    ESP_LOGI(TAG, "Initializing I2S Standard: %lu Hz, %d-bit, %s", 
             _sampleRate, 
             (_bitsPerSample == I2S_DATA_BIT_WIDTH_16BIT) ? 16 : 
             (_bitsPerSample == I2S_DATA_BIT_WIDTH_24BIT) ? 24 : 32,
             (_channelMode == I2S_SLOT_MODE_MONO) ? "mono" : "stereo");

    esp_err_t ret = configureChannel();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    _initialized = true;
    ESP_LOGI(TAG, "I2S Standard initialized successfully");
    return ESP_OK;
}

esp_err_t I2SMicrophone::configureChannel() {
    // Step 1: Create I2S channel configuration
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(_portNum, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;  // Auto clear DMA buffer
    chan_cfg.dma_desc_num = 6;   // Number of DMA descriptors
    chan_cfg.dma_frame_num = 240; // Number of frames in each DMA descriptor
    
    esp_err_t ret = i2s_new_channel(&chan_cfg, nullptr, &_rxHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 2: Configure I2S Standard mode
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = _sampleRate,
            .clk_src = I2S_CLK_SRC_DEFAULT,    // Use default clock source
            .mclk_multiple = I2S_MCLK_MULTIPLE_256  // MCLK = sample_rate * 256
        },
        .slot_cfg = {
            .data_bit_width = _bitsPerSample,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,  // Auto-adjust slot width
            .slot_mode = _channelMode,
            .slot_mask = (_channelMode == I2S_SLOT_MODE_MONO) ? I2S_STD_SLOT_LEFT : I2S_STD_SLOT_BOTH,
            .ws_width = _bitsPerSample,
            .ws_pol = false,    // WS signal polarity (false = low for left channel)
            .bit_shift = true,  // Enable bit shift for standard I2S
            .left_align = true, // Left-aligned data
            .big_endian = false, // Little endian
            .bit_order_lsb = false  // MSB first
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    // Master clock not used for simple microphones
            .bclk = _clockPin,          // Bit clock
            .ws = _wordSelectPin,       // Word select (LRCLK)
            .dout = I2S_GPIO_UNUSED,    // Data out not used for RX
            .din = _dataPin,            // Data in
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(_rxHandle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S standard mode: %s", esp_err_to_name(ret));
        i2s_del_channel(_rxHandle);
        _rxHandle = nullptr;
        return ret;
    }

    ESP_LOGI(TAG, "I2S channel configured successfully");
    return ESP_OK;
}

esp_err_t I2SMicrophone::start() {
    if (!_initialized || !_rxHandle) {
        ESP_LOGE(TAG, "I2S not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (_active) {
        ESP_LOGW(TAG, "I2S already active");
        return ESP_OK;
    }

    esp_err_t ret = i2s_channel_enable(_rxHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    _active = true;
    ESP_LOGI(TAG, "I2S channel started");
    return ESP_OK;
}

esp_err_t I2SMicrophone::stop() {
    if (!_initialized || !_rxHandle) {
        ESP_LOGE(TAG, "I2S not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!_active) {
        ESP_LOGW(TAG, "I2S already stopped");
        return ESP_OK;
    }

    esp_err_t ret = i2s_channel_disable(_rxHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    _active = false;
    ESP_LOGI(TAG, "I2S channel stopped");
    return ESP_OK;
}

esp_err_t I2SMicrophone::readAudioData(void* buffer, size_t bufferSize, size_t* bytesRead, 
                                    uint32_t timeoutMs) {
    if (!_initialized || !_rxHandle) {
        ESP_LOGE(TAG, "I2S not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!_active) {
        ESP_LOGE(TAG, "I2S channel not active");
        return ESP_ERR_INVALID_STATE;
    }

    if (!buffer || !bytesRead) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    TickType_t timeout = (timeoutMs == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
    
    esp_err_t ret = i2s_channel_read(_rxHandle, buffer, bufferSize, bytesRead, timeout);
    if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Failed to read I2S data: %s", esp_err_to_name(ret));
        return ret;
    }

    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGD(TAG, "I2S read timeout");
        *bytesRead = 0;
    }

    _lastBufferLevel = bufferSize;
    _lastLevel = (int16_t*)buffer;
    return ret;
}

esp_err_t I2SMicrophone::readSamples(int16_t* buffer, size_t sampleCount, size_t* samplesRead, 
                                  uint32_t timeoutMs) {
    if (!buffer || !samplesRead) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    size_t bytesPerSample = getBytesPerSample() * getChannelCount();
    size_t bufferSize = sampleCount * bytesPerSample;
    size_t bytesRead = 0;

    esp_err_t ret = readAudioData(buffer, bufferSize, &bytesRead, timeoutMs);
    
    *samplesRead = bytesRead / bytesPerSample;
    
    // Convert to 16-bit if necessary
    if (_bitsPerSample == I2S_DATA_BIT_WIDTH_32BIT && ret == ESP_OK && *samplesRead > 0) {
        // Convert 32-bit samples to 16-bit by taking upper 16 bits
        int32_t* buffer32 = (int32_t*)buffer;
        for (size_t i = 0; i < *samplesRead; i++) {
            buffer[i] = (int16_t)(buffer32[i] >> 16);
        }
    } else if (_bitsPerSample == I2S_DATA_BIT_WIDTH_24BIT && ret == ESP_OK && *samplesRead > 0) {
        // Convert 24-bit samples to 16-bit by taking upper 16 bits of 24-bit
        uint8_t* buffer24 = (uint8_t*)buffer;
        for (size_t i = 0; i < *samplesRead; i++) {
            // 24-bit sample is stored in 32-bit container, take upper 16 bits
            int32_t sample24 = (buffer24[i*4+2] << 16) | (buffer24[i*4+1] << 8) | buffer24[i*4];
            if (buffer24[i*4+2] & 0x80) sample24 |= 0xFF000000; // Sign extend
            buffer[i] = (int16_t)(sample24 >> 8);
        }
    }

    return ret;
}

int I2SMicrophone::readSamples(int16_t* buffer, size_t sampleCount, uint32_t timeoutMs) {
    size_t samplesRead = 0;
    esp_err_t ret = readSamples(buffer, sampleCount, &samplesRead, timeoutMs);
    
    if (ret == ESP_OK) {
        return (int)samplesRead;
    } else if (ret == ESP_ERR_TIMEOUT) {
        return 0;  // Timeout is not an error, just no data
    } else {
        ESP_LOGE(TAG, "readSamples failed: %s", esp_err_to_name(ret));
        return -1;  // Error
    }
}

int I2SMicrophone::readLevel() {
    int level = 0;
    for(int i=0; i < _lastBufferLevel; i++) {
        level = max(abs(_lastLevel[i]), level);
    }

    return level;
}

bool I2SMicrophone::isInitialized() const {
    return _initialized;
}

bool I2SMicrophone::isActive() const {
    return _active;
}

uint32_t I2SMicrophone::getSampleRate() const {
    return _sampleRate;
}

i2s_data_bit_width_t I2SMicrophone::getBitsPerSample() const {
    return _bitsPerSample;
}

i2s_slot_mode_t I2SMicrophone::getChannelMode() const {
    return _channelMode;
}

esp_err_t I2SMicrophone::preloadDMA() {
    if (!_initialized || !_rxHandle || !_active) {
        ESP_LOGE(TAG, "I2S not ready for DMA preload");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2s_channel_preload_data(_rxHandle, nullptr, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to preload DMA: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "DMA buffers preloaded");
    return ESP_OK;
}

size_t I2SMicrophone::calculateBufferSize(uint32_t durationMs) const {
    size_t samplesPerMs = _sampleRate / 1000;
    size_t totalSamples = samplesPerMs * durationMs;
    size_t bytesPerSample = getBytesPerSample() * getChannelCount();
    
    return totalSamples * bytesPerSample;
}

size_t I2SMicrophone::getBytesPerSample() const {
    switch (_bitsPerSample) {
        case I2S_DATA_BIT_WIDTH_16BIT:
            return 2;
        case I2S_DATA_BIT_WIDTH_24BIT:
            return 4;  // 24-bit samples are stored in 32-bit containers
        case I2S_DATA_BIT_WIDTH_32BIT:
            return 4;
        default:
            return 2;
    }
}

size_t I2SMicrophone::getChannelCount() const {
    return (_channelMode == I2S_SLOT_MODE_MONO) ? 1 : 2;
}
