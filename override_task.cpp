/*
 * override_task.cpp
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * Reads the two override switches every 20ms, debounces by requiring
 * OVERRIDE_DEBOUNCE_COUNT consecutive identical samples, then updates
 * g_session.override_flags. task_relay picks those flags up and drives
 * the physical relays.
 *
 * Switches are wired with internal pullups, active LOW: LOW = pressed
 * = override active. We invert in software so the flag bit means
 * "override is asserted" rather than "pin is low."
 *
 * WHITE LED indicates override state so the operator can see at a
 * glance that a manual bypass is active:
 *   SLOW   — main override only
 *   MEDIUM — blast override only
 *   FAST   — both active
 *   OFF    — no overrides
 */

#include "override_task.h"
#include "led_ctrl.h"
#include "config.h"
#include "diag.h"

static uint8_t _debounce_count_main  = 0;
static uint8_t _debounce_count_blast = 0;
static bool    _stable_main  = false;
static bool    _stable_blast = false;

// Apply debounce to a raw reading. Returns true if the stable state changed.
static bool _debounce(bool raw, bool& stable, uint8_t& count) {
    if (raw == stable) {
        count = 0;
        return false;
    }
    count++;
    if (count >= OVERRIDE_DEBOUNCE_COUNT) {
        stable = raw;
        count = 0;
        return true;
    }
    return false;
}

// LED scheme (two LEDs, independent):
//   RED   — fast flash when main override is asserted, off otherwise
//   WHITE — medium flash when blast override is asserted, off otherwise
//
// Note: task_rfid also drives RED (for card read failures / signature
// errors). If main override is asserted while a bad card is presented,
// the two writers will trade control of RED — task_override's next tick
// (20ms) reasserts its state. That's acceptable because the override
// condition is the more important one to display, and the RFID error
// flash is a transient event that gets overwritten within one tick anyway.
static void _update_leds(uint8_t flags) {
    led_set(LED_RED,   (flags & OVR_FLAG_MAIN)  ? LED_FAST   : LED_OFF);
    led_set(LED_WHITE, (flags & OVR_FLAG_BLAST) ? LED_MEDIUM : LED_OFF);
}

void task_override(void* arg) {
    Serial.println("[override] task started");

    uint8_t last_flags = 0xFF;   // force first update

    for (;;) {
        diag_mark(DIAG_MARK_OVERRIDE_POLL);

        // Switches are active LOW
        bool raw_main  = (digitalRead(PIN_SW_MAIN_OVERRIDE)  == LOW);
        bool raw_blast = (digitalRead(PIN_SW_BLAST_OVERRIDE) == LOW);

        bool changed  = _debounce(raw_main,  _stable_main,  _debounce_count_main);
        changed      |= _debounce(raw_blast, _stable_blast, _debounce_count_blast);

        uint8_t flags = 0;
        if (_stable_main)  flags |= OVR_FLAG_MAIN;
        if (_stable_blast) flags |= OVR_FLAG_BLAST;

        if (flags != last_flags) {
            if (xSemaphoreTake(g_session_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_session.override_flags = flags;
                xSemaphoreGive(g_session_mutex);
            }
            _update_leds(flags);
            Serial.print("[override] main=");
            Serial.print(_stable_main  ? "ON " : "off");
            Serial.print(" blast=");
            Serial.println(_stable_blast ? "ON " : "off");
            last_flags = flags;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
