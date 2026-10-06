/*
 *    Arduino 2.3.10
 *       board: ESP32 Dev Module
 *
 * woodshop_esp32.ino
 * Woodshop Access Control System — ESP32 Arduino IDE
 *
 * External library dependency:
 *   - Elechouse PN532 library (PN532, PN532_SPI)
 *   - rweather/Crypto (Ed25519) — "Crypto" in the Arduino library manager
 *   - bblanchon/ArduinoJson — "ArduinoJson" in the Arduino library manager
 *     (ota_task.cpp; parses the small firmware manifest object)
 *
 * Startup order (IMPORTANT — do not reorder without understanding why):
 *
 *   1. current_sense_init()  — Core 1 I2S DMA (doesn't touch SPI/WiFi pins)
 *   2. task_led              — low priority, just GPIO
 *   3. task_rfid             — created but immediately blocks on g_system_settled
 *   4. task_override         — reads switches, updates shared state
 *   5. task_relay            — drives main/blast relays (single owner)
 *   6. task_session          — accumulates run-time stats while active
 *   7. xSemaphoreGive(g_system_settled)
 *                            — unblocks task_rfid; it now performs the PN532
 *                              hard reset (GPIO13 → RSTPD_N) followed by full
 *                              software init.  Doing this AFTER the other tasks
 *                              are running means the power rails and system bus
 *                              have settled before the PN532 is touched.
 *   8. (wait for g_rfid_ready — PN532 must be fully initialized before the
 *       WiFi radio comes up; radio noise during SAMConfig causes intermittent
 *       failures on marginal power supplies)
 *   9. task_wifi             — connects to server, sends protocol messages
 *
 * Task ownership of shared resources:
 *   - PIN_RELAY_MAIN/BLAST         : task_relay only
 *   - PIN_SW_MAIN/BLAST_OVERRIDE   : task_override only
 *   - PN532 SPI bus                : task_rfid only
 *   - GPIO36 ADC (I2S)             : current_sense (Core 1) only
 *   - LEDs                         : any task via led_set() (mutex-protected)
 *   - g_session                    : any task via g_session_mutex
 */

#include "config.h"
#include "diag.h"
#include "node_config.h"
#include "led_ctrl.h"
#include "current_sense.h"
#include "rfid_task.h"
#include "override_task.h"
#include "relay_ctrl.h"
#include "session_task.h"
#include "wifi_task.h"
#include "ota_task.h"

#include <esp_task_wdt.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ── FreeRTOS task handles ─────────────────────────────────────────────────────
static TaskHandle_t h_rfid     = NULL;
static TaskHandle_t h_led      = NULL;
static TaskHandle_t h_wifi     = NULL;
static TaskHandle_t h_override = NULL;
static TaskHandle_t h_relay    = NULL;
static TaskHandle_t h_session  = NULL;
static TaskHandle_t h_ota      = NULL;

// ── Shared state ──────────────────────────────────────────────────────────────
volatile session_t g_session;
SemaphoreHandle_t  g_session_mutex;

// ── Startup synchronization ───────────────────────────────────────────────────
// g_system_settled: given by setup() after all non-WiFi tasks are created.
//   task_rfid blocks on this before touching the PN532, so the hard reset and
//   init happen once the system is in a stable operating state.
//
// g_rfid_ready: given by task_rfid after PN532 init completes (success or fail).
//   setup() waits on this before creating task_wifi, ensuring the WiFi radio
//   does not start while the PN532 SPI bus is being initialized.
SemaphoreHandle_t  g_system_settled;
SemaphoreHandle_t  g_rfid_ready;

// ── Disable the Task Watchdog Timer (DEV_BUILD only — see config.h) ─────────
// During development we don't want silent reboots from the 5-second TWDT;
// we'd rather see the chip actually hang so we can observe what it's doing.
//
// IMPORTANT: as of Aug 2026 this is gated behind DEV_BUILD in setup() below.
// It used to run unconditionally, alongside an unconditional brown-out-
// detector disable (WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0)) that a stale
// version of this comment claimed left the BOD "enabled" — it did not; that
// register write disables it outright. Running both unconditionally in a
// field-installed unit is almost certainly why reboots there show no clean
// reset_reason and no discernible pattern: a sagging supply just does
// whatever undefined thing an unprotected core does instead of a logged
// ESP_RST_BROWNOUT, and a hung task spins forever instead of a logged
// ESP_RST_TASK_WDT. See diag.h.
//
// Rather than deinit'ing the watchdog (which has a broken teardown path in
// ESP-IDF 5.x that asserts on already-unsubscribed idle tasks), we just
// reconfigure it with an effectively-infinite timeout and no idle cores
// subscribed. This is the approach Espressif officially recommends.
static void _disable_task_watchdog() {
    esp_task_wdt_config_t cfg = {
        .timeout_ms     = 0xFFFFFFFF,   // ~49 days = effectively disabled
        .idle_core_mask = 0,            // don't watch either core's idle
        .trigger_panic  = false,
    };
    esp_err_t err = esp_task_wdt_reconfigure(&cfg);
    Serial.print("[main] task watchdog relaxed (reconfigure=");
    Serial.print(err == ESP_OK ? "OK" : "ERR");
    Serial.println(")");
    Serial.flush();
}

void setup() {
#ifdef DEV_BUILD
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // DEV ONLY: brown-out detector disabled
#endif
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  Woodshop Access Control (ESP32)");
    Serial.print("  Firmware version: ");
    Serial.println(FW_VERSION);
#ifdef DEV_BUILD
    Serial.println("  *** DEV_BUILD: BOD + task watchdog disabled ***");
#endif
    Serial.println("========================================");
    Serial.println();
    Serial.flush();

#ifdef DEV_BUILD
    _disable_task_watchdog();
#else
    Serial.println("[main] production build: brown-out detector + task watchdog active");
    Serial.flush();
#endif

    // Reboot diagnostics: capture why THIS boot happened (reset reason) and
    // what the firmware was doing right before it (RTC breadcrumb from the
    // previous run), append to the persistent ring buffer, dump history to
    // Serial. Deliberately called before node_config_init() and anything
    // else, so a crash during config load or PN532/WiFi bring-up is still
    // preceded by a clean diag_init() -- diag_mark(DIAG_MARK_BOOT) is the
    // floor value; anything else means a task got at least that far after
    // boot before this run ended.
    diag_init();

    // Load machine_number and blast_delay_ms from NVS (or seed defaults on
    // first boot). Must happen before any task reads g_machine_number /
    // g_blast_delay_ms. The "Machine #N" banner below reflects the loaded
    // value, not the compile-time default.
    node_config_init();

    Serial.print("[main] Machine #");
    Serial.println(g_machine_number);
    Serial.flush();

    // Output pins — all start LOW (relays de-energized, safe state).
    // task_relay will take over writing these from now on.
    pinMode(PIN_RELAY_MAIN,  OUTPUT); digitalWrite(PIN_RELAY_MAIN,  LOW);
    pinMode(PIN_RELAY_BLAST, OUTPUT); digitalWrite(PIN_RELAY_BLAST, LOW);
    pinMode(PIN_RELAY_AUX,   OUTPUT); digitalWrite(PIN_RELAY_AUX,   LOW);

    // Override switches, internal pullup, active LOW
    pinMode(PIN_SW_MAIN_OVERRIDE,  INPUT_PULLUP);
    pinMode(PIN_SW_BLAST_OVERRIDE, INPUT_PULLUP);

    // Shared state init
    memset((void*)&g_session, 0, sizeof(g_session));
    g_session_mutex  = xSemaphoreCreateMutex();
    g_system_settled = xSemaphoreCreateBinary();
    g_rfid_ready     = xSemaphoreCreateBinary();

    // Current sensor (Core 1 I2S DMA) — safe to start first, uses no pins
    // or peripherals that the other tasks touch.
    current_sense_init();

    // Diag heartbeat — updates the RTC breadcrumb every DIAG_HEARTBEAT_MS.
    // Touches only RTC no-init memory, no pins/peripherals, safe to start
    // this early.
    diag_start_heartbeat_task();

    // LEDs first so subsequent tasks can report status visually.
    xTaskCreatePinnedToCore(task_led, "led", 2048, NULL, 2, &h_led, 0);
    Serial.println("[main] led task created");
    Serial.flush();
    delay(200);

    // Create task_rfid now so it can begin stack/heap allocation, but it will
    // block immediately on g_system_settled and not touch the PN532 until we
    // give that semaphore below.
    xTaskCreatePinnedToCore(task_rfid, "rfid", 12288, NULL, 3, &h_rfid, 0);
    Serial.println("[main] rfid task created (blocked on g_system_settled)");
    Serial.flush();

    // Create the remaining non-WiFi tasks.  By the time all of these are
    // running the 3V3 rail and SPI bus are stable.
    xTaskCreatePinnedToCore(task_override, "override", 2048, NULL, 2, &h_override, 0);
    Serial.println("[main] override task created");

    xTaskCreatePinnedToCore(task_relay, "relay", 2048, NULL, 3, &h_relay, 0);
    Serial.println("[main] relay task created");

    xTaskCreatePinnedToCore(task_session, "session", 3072, NULL, 2, &h_session, 0);
    Serial.println("[main] session task created");
    Serial.flush();

    // Signal task_rfid that the system is settled and it may proceed with the
    // PN532 hard reset and initialization sequence.
    Serial.println("[main] signaling g_system_settled — PN532 init will begin");
    Serial.flush();
    xSemaphoreGive(g_system_settled);

    // Wait for task_rfid to complete PN532 init before starting WiFi.
    // The WiFi radio produces RF noise that can cause SAMConfig to fail on
    // boards with marginal power supplies.  20 s is ample; the PN532 init
    // sequence completes in well under 5 s when successful.
    if (xSemaphoreTake(g_rfid_ready, pdMS_TO_TICKS(30000)) == pdTRUE) {
        Serial.println("[main] RFID init complete — starting WiFi task");
    } else {
        Serial.println("[main] RFID init TIMEOUT — starting WiFi task anyway");
    }
    Serial.flush();

    // WiFi last — only task that touches the radio; everything else is stable.
    xTaskCreatePinnedToCore(task_wifi, "wifi", 8192, NULL, 2, &h_wifi, 0);
    Serial.println("[main] wifi task created");

    // OTA — background maintenance, lowest priority. It checks WiFi.status()
    // itself before doing anything, so it's fine to create it immediately
    // after task_wifi rather than waiting for a confirmed connection here.
    // Stack sized like task_wifi's — HTTPClient + Update + JSON parsing live
    // on this task's stack too.
    xTaskCreatePinnedToCore(task_ota, "ota", 8192, NULL, 1, &h_ota, 0);
    Serial.println("[main] ota task created");

    diag_mark(DIAG_MARK_BOOT_DONE);
    Serial.println("[main] All tasks started.");
    Serial.println();
    Serial.flush();
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
