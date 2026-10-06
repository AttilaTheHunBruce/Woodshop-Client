/*
 * session_task.cpp
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * Runs at SESSION_SAMPLE_MS (200ms). While g_session.active is true:
 *
 *   - Watches current_machine_is_on() for edges; increments g_session.starts
 *     on OFF->ON and g_session.stops on ON->OFF.
 *   - Accumulates g_session.duration_ms by adding the sample interval
 *     whenever the machine is on (so this is *machine runtime*, not total
 *     session length — matches what the server log wants).
 *   - Maintains a running mean of current_get_rms_amps() over just the
 *     samples where the machine is on, exposed as g_session.avg_current.
 *   - Skips the first INRUSH_IGNORE_MS after each machine-on edge so
 *     motor inrush current doesn't inflate the average.
 *
 * Local state (edge detector, sum/count for the mean) is reset the first
 * time we observe a fresh session.  task_rfid zero-initializes the
 * counters when it sets active=true, so we just watch for that transition
 * ourselves to know when to reset the edge-detector memory.
 */

#include "session_task.h"
#include "current_sense.h"
#include "config.h"
#include "diag.h"

// How long to ignore current samples after a machine-on edge.
// Motor inrush at 60 Hz typically settles within 2-5 cycles (33-83 ms).
// 100 ms gives comfortable margin; increase if you see elevated averages
// on tools with large motors or compressors.
#define INRUSH_IGNORE_MS  100

void task_session(void* arg) {
    Serial.println("[session] task started");

    bool     last_active    = false;
    bool     last_on        = false;   // previous machine_is_on() reading
    double   sum_current    = 0.0;     // for running mean (double for precision)
    uint32_t count_current  = 0;
    uint32_t machine_on_ms  = 0;       // millis() when last start edge occurred

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SESSION_SAMPLE_MS));
        diag_mark(DIAG_MARK_SESSION_TICK);

        bool active = false;
        if (xSemaphoreTake(g_session_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            active = g_session.active;
            xSemaphoreGive(g_session_mutex);
        }

        // Session transition: reset local edge/accumulator state on a
        // fresh card insert so last session's data doesn't leak into this one.
        if (active && !last_active) {
            last_on       = false;
            sum_current   = 0.0;
            count_current = 0;
            machine_on_ms = 0;
            Serial.println("[session] new session started");
        }
        if (!active && last_active) {
            Serial.print("[session] session ended: starts=");
            // Read back under mutex for the log line
            if (xSemaphoreTake(g_session_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                Serial.print(g_session.starts);
                Serial.print(" stops=");
                Serial.print(g_session.stops);
                Serial.print(" runtime=");
                Serial.print(g_session.duration_ms / 1000);
                Serial.print("s avg_current=");
                Serial.print(g_session.avg_current, 3);
                Serial.println("A");
                xSemaphoreGive(g_session_mutex);
            } else {
                Serial.println("(mutex busy)");
            }
        }
        last_active = active;

        if (!active) continue;

        // --- Sample the sensor ---
        bool  on   = current_machine_is_on();
        float amps = current_get_rms_amps();

        // --- Edge detection ---
        bool start_edge = on && !last_on;
        bool stop_edge  = !on && last_on;
        last_on = on;

        if (start_edge) {
            machine_on_ms = millis();
            Serial.println("[session] machine START");
        }
        if (stop_edge) {
            Serial.println("[session] machine STOP");
        }

        // --- Running mean of RMS current (machine on, inrush window elapsed) ---
        // Skip the first INRUSH_IGNORE_MS after each start edge so motor
        // inrush doesn't inflate the average. Duration still accumulates
        // from the moment the machine turns on.
        bool inrush_window = (machine_on_ms > 0) &&
                             ((millis() - machine_on_ms) < INRUSH_IGNORE_MS);

        if (on && !inrush_window) {
            sum_current += amps;
            count_current++;
        }

        float mean_amps = (count_current > 0)
                          ? (float)(sum_current / count_current)
                          : 0.0f;

        // --- Commit to shared state ---
        if (xSemaphoreTake(g_session_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (start_edge && g_session.starts < 255) g_session.starts++;
            if (stop_edge  && g_session.stops  < 255) g_session.stops++;
            if (on) g_session.duration_ms += SESSION_SAMPLE_MS;
            g_session.avg_current = mean_amps;
            xSemaphoreGive(g_session_mutex);
        }
    }
}
