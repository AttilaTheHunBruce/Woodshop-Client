/*
 * current_sense.cpp
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * Uses the ESP32 I2S peripheral in ADC mode to sample GPIO36 at
 * CURRENT_SAMPLE_RATE (20 kHz) via DMA.  A dedicated FreeRTOS task
 * pinned to Core 1 drains the DMA buffers and maintains two
 * exponential filters:
 *
 *   _mean_filtered  — slow DC bias tracker  (tau ≈ 1 s)
 *   _ms_filtered    — fast mean-square RMS² (tau ≈ 10 ms)
 *
 * Why I2S instead of a timer + ADC ISR?
 *   The ESP32 ADC cannot be clocked faster than ~3-5 kHz reliably from
 *   a software timer because the ISR latency jitter corrupts sample
 *   timing.  The I2S peripheral has a dedicated DMA engine that clocks
 *   the ADC at a precise rate entirely in hardware, delivering clean
 *   20 kHz samples with no CPU involvement during capture.
 *
 * Why Core 1?
 *   Core 0 runs the Arduino/FreeRTOS scheduler and WiFi stack.
 *   Offloading the RMS math to Core 1 keeps Core 0 latency low for
 *   the RFID SPI transactions and WiFi messaging.
 */

#include "current_sense.h"
#include "config.h"
#include <driver/i2s.h>
#include <math.h>

// ── I2S port and ADC channel ──────────────────────────────────────────────────
// GPIO36 = ADC1_CH0
#define I2S_PORT        I2S_NUM_0
#define ADC_CHANNEL     ADC1_CHANNEL_0   // GPIO36

// ── Filter alphas ─────────────────────────────────────────────────────────────
// At 20 kHz with DMA_BUF_LEN=512 samples per callback:
//   each callback covers 512/20000 = 25.6 ms
// ALPHA_MS = 0.5  -> tau ≈ 25.6 ms  (fast, tracks load changes)
// ALPHA_MEAN = 0.02 -> tau ≈ 1.28 s  (slow DC bias tracker)
#define ALPHA_MS        0.5f
#define ALPHA_MEAN      0.02f

// ── Shared results (written by Core 1 task, read by Core 0 tasks) ─────────────
static volatile float _rms_amps    = 0.0f;
static volatile float _bias_volts  = 0.0f;
static volatile bool  _machine_on  = false;
static SemaphoreHandle_t _result_mutex = NULL;

// Internal filter state (Core 1 only — no mutex needed)
static float _mean_filtered = 2048.0f;  // seed at mid-scale
static float _ms_filtered   = 0.0f;

// Pre-computed threshold in count² space (avoids sqrt in hot path)
static float _threshold_ms_on;
static float _threshold_ms_off;

// ── I2S ADC init ──────────────────────────────────────────────────────────────
static void _i2s_adc_init() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate          = CURRENT_SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = CURRENT_DMA_BUF_COUNT,
        .dma_buf_len          = CURRENT_DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0,
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL));
    ESP_ERROR_CHECK(i2s_adc_enable(I2S_PORT));
}

// ── Core 1 processing task ────────────────────────────────────────────────────
static void _current_task(void* arg) {
    // Each I2S sample is 16 bits.  The ADC result is in the lower 12 bits
    // (bits 11:0); bits 15:12 are the channel tag — mask them off.
    const size_t buf_bytes = CURRENT_DMA_BUF_LEN * sizeof(uint16_t);
    uint16_t* buf = (uint16_t*)malloc(buf_bytes);
    if (!buf) {
        Serial.println("[current] malloc failed — task exiting");
        vTaskDelete(NULL);
    }

    // Pre-compute threshold in count² space
    float counts_on  = CURRENT_THRESHOLD_AMPS / CURRENT_AMPS_PER_COUNT;
    float counts_off = counts_on * 0.8f;    // 80% hysteresis
    _threshold_ms_on  = counts_on  * counts_on;
    _threshold_ms_off = counts_off * counts_off;

    bool machine_on_local = false;

    for (;;) {
        size_t bytes_read = 0;
        // i2s_read blocks until DMA_BUF_LEN samples are ready
        i2s_read(I2S_PORT, buf, buf_bytes, &bytes_read, portMAX_DELAY);

        size_t n = bytes_read / sizeof(uint16_t);
        if (n == 0) continue;

        // --- Compute batch mean and mean-square in count space ---
        int bias = (int)_mean_filtered;
        double acc_sum = 0.0;
        double acc_sq  = 0.0;

        for (size_t i = 0; i < n; i++) {
            int raw = (int)(buf[i] & 0x0FFF);   // strip channel tag
            acc_sum += raw;
            int s    = raw - bias;
            acc_sq  += (double)s * s;
        }

        float batch_mean = (float)(acc_sum / n);
        float batch_ms   = (float)(acc_sq  / n);

        // --- Update exponential filters ---
        _mean_filtered = ALPHA_MEAN * batch_mean + (1.0f - ALPHA_MEAN) * _mean_filtered;
        _ms_filtered   = ALPHA_MS   * batch_ms   + (1.0f - ALPHA_MS)   * _ms_filtered;

        // --- Hysteretic machine-on detection ---
        if (!machine_on_local && _ms_filtered > _threshold_ms_on)
            machine_on_local = true;
        else if (machine_on_local && _ms_filtered < _threshold_ms_off)
            machine_on_local = false;

        // --- Publish results (mutex-protected) ---
        float rms    = sqrtf(_ms_filtered) * CURRENT_AMPS_PER_COUNT;
        float bias_v = _mean_filtered * (CURRENT_VREF / CURRENT_ADC_MAX);

        if (xSemaphoreTake(_result_mutex, 0) == pdTRUE) {
            _rms_amps   = rms;
            _bias_volts = bias_v;
            _machine_on = machine_on_local;
            xSemaphoreGive(_result_mutex);
        }

        // --- Periodic diagnostic (every ~2 s) ---
        // Uncomment to re-enable current sensor diagnostics on Serial.
        // Expected with test load (14Vrms / 50Ω / 100 turns ratio):
        //   bias  ≈ 1.65 V  (mid-rail)
        //   rms   ≈ 0.280 A
        //   on    = true
        // static uint32_t _diag_count = 0;
        // if (++_diag_count >= 78) {    // 78 × 25.6ms ≈ 2s
        //     _diag_count = 0;
        //     Serial.printf("[current] bias=%.3fV  rms=%.3fA  on=%s\n",
        //                   bias_v, rms, machine_on_local ? "YES" : "no");
        // }
    }
}

// ── Public API ────────────────────────────────────────────────────────────────
void current_sense_init() {
    _result_mutex = xSemaphoreCreateMutex();
    _i2s_adc_init();
    // Pin task to Core 1, priority 4 (higher than Core 0 tasks)
    xTaskCreatePinnedToCore(_current_task, "current", 4096, NULL, 4, NULL, 1);
    Serial.println("[current] I2S ADC started at " + String(CURRENT_SAMPLE_RATE) + " Hz on Core 1");
}

float current_get_rms_amps() {
    float v = 0.0f;
    if (xSemaphoreTake(_result_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        v = _rms_amps;
        xSemaphoreGive(_result_mutex);
    }
    return v;
}

float current_get_bias_volts() {
    float v = 0.0f;
    if (xSemaphoreTake(_result_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        v = _bias_volts;
        xSemaphoreGive(_result_mutex);
    }
    return v;
}

bool current_machine_is_on() {
    bool v = false;
    if (xSemaphoreTake(_result_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        v = _machine_on;
        xSemaphoreGive(_result_mutex);
    }
    return v;
}
