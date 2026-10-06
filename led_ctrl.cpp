/*
 * led_ctrl.cpp
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * Single FreeRTOS task updates all four LEDs every LED_TICK_MS (100ms).
 * led_set() is safe to call from any task — protected by a mutex.
 */

#include "led_ctrl.h"
#include "config.h"

static const int _pins[4] = {
    PIN_LED_RED, PIN_LED_GREEN, PIN_LED_YELLOW, PIN_LED_WHITE
};

typedef struct {
    int  mode;
    int  tick;
    bool state;
} led_state_t;

static led_state_t       _leds[4];
static SemaphoreHandle_t _mutex = NULL;

void led_init() {
    _mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < 4; i++) {
        pinMode(_pins[i], OUTPUT);
        digitalWrite(_pins[i], LOW);
        _leds[i] = {LED_OFF, 0, false};
    }
}

void led_set(int led, int mode) {
    if (led < 0 || led > 3) return;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _leds[led].mode  = mode;
        _leds[led].tick  = 0;
        _leds[led].state = (mode == LED_ON);
        xSemaphoreGive(_mutex);
    }
}

void task_led(void* arg) {
    led_init();
    Serial.println("[led] task started");

    // Startup flash — all LEDs twice to confirm power-on
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) digitalWrite(_pins[j], HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        for (int j = 0; j < 4; j++) digitalWrite(_pins[j], LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    Serial.println("[led] startup flash done");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LED_TICK_MS));

        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;

        for (int i = 0; i < 4; i++) {
            led_state_t* L = &_leds[i];
            bool out;

            if (L->mode == LED_ON) {
                out = true;
            } else if (L->mode == LED_OFF) {
                out = false;
            } else {
                // Blink: toggle every L->mode ticks
                L->tick++;
                if (L->tick >= L->mode) {
                    L->tick  = 0;
                    L->state = !L->state;
                }
                out = L->state;
            }
            digitalWrite(_pins[i], out ? HIGH : LOW);
        }

        xSemaphoreGive(_mutex);
    }
}
