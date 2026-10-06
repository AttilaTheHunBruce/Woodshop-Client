/*
 * relay_ctrl.cpp
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * Single owner of PIN_RELAY_MAIN and PIN_RELAY_BLAST. Runs at 50Hz.
 *
 * Decision table:
 *   main_on  = (session.active && session.authorized) || override.main
 *   blast_on = main_on || override.blast || (blast_delay timer active)
 *
 * When main transitions ON->OFF, arm a one-shot drop-out timer so the
 * blast gate keeps running for g_blast_delay_ms (default 30s, adjustable
 * via admin card) to clear the dust
 * from the machine. Any new main-on event before the timer expires just
 * keeps the blast relay on — no special handling needed because blast_on
 * is re-evaluated every tick.
 *
 * The relay pins were set to OUTPUT/LOW in setup() in the .ino; this task
 * only does digitalWrite() from here on out.
 */

#include "relay_ctrl.h"
#include "node_config.h"
#include "config.h"
#include "diag.h"

// Blast gate hold-off: after a card turns the main relay on, the blast gate
// waits this long before following it. The server's allow/deny reply arrives
// in well under a second, so a rejected card (relay dropped by wifi_task)
// never starts the blast gate and never arms the dust-clearing run-on timer.
// Admin/override-main is NOT delayed. Set to 0 to restore the old behavior.
#ifndef BLAST_START_HOLD_MS
#define BLAST_START_HOLD_MS  1500UL
#endif

static bool _last_main_on = false;
static bool _last_blast_from_main = false;
static uint32_t _main_on_at = 0;

void task_relay(void* arg) {
    Serial.println("[relay] task started");

    uint32_t blast_off_at = 0;   // millis() timestamp when blast may drop

    for (;;) {
        diag_mark(DIAG_MARK_RELAY_TICK);

        // --- Snapshot session state ---
        bool     active = false;
        bool     authed = false;
        uint8_t  ovr    = 0;

        if (xSemaphoreTake(g_session_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            active = g_session.active;
            authed = g_session.authorized;
            ovr    = g_session.override_flags;
            xSemaphoreGive(g_session_mutex);
        }

        bool main_on  = (active && authed) || (ovr & OVR_FLAG_MAIN);

        // --- Arm blast drop-out timer on main ON->OFF transition ---
        // Uses the NVS-backed runtime value, not the compile-time default,
        // so admin-card updates take effect on the next drop-out event.
        uint32_t now = millis();
        if (_last_main_on && !main_on) {
            diag_mark(DIAG_MARK_RELAY_MAIN_OFF);
            // Run the blast gate on only if it was actually running -- a card
            // rejected before the hold-off expired never started it.
            if (_last_blast_from_main) {
                blast_off_at = now + g_blast_delay_ms;
                Serial.print("[relay] main off -- blast gate will run ");
                Serial.print(g_blast_delay_ms / 1000);
                Serial.println("s more");
            } else {
                Serial.println("[relay] main off -- blast gate never started, no run-on");
            }
        } else if (!_last_main_on && main_on) {
            // Coil energize -- inrush current draw happens right here.
            diag_mark(DIAG_MARK_RELAY_MAIN_ON);
            _main_on_at = now;
        }
        _last_main_on = main_on;

        bool blast_from_main = main_on &&
            ((ovr & OVR_FLAG_MAIN) || (now - _main_on_at) >= BLAST_START_HOLD_MS);
        _last_blast_from_main = blast_from_main;

        bool blast_timer_active = (now < blast_off_at);
        bool blast_on = blast_from_main || (ovr & OVR_FLAG_BLAST) || blast_timer_active;

        // --- Drive the pins ---
        digitalWrite(PIN_RELAY_MAIN,  main_on  ? HIGH : LOW);
        digitalWrite(PIN_RELAY_BLAST, blast_on ? HIGH : LOW);

        vTaskDelay(pdMS_TO_TICKS(20));   // 50 Hz
    }
}
