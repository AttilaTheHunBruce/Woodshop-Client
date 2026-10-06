/*
 * diag.h
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * Reboot diagnostics: WHY did this board just reset, and what was it doing
 * at the time?
 *
 * Background / motivation (Aug 2026): units installed in the field, wired
 * to all peripherals but with no machine drawing current, reboot several
 * times in rapid succession with no discernible pattern. Between reboots
 * the board is fully functional (reads cards, drives relays/LEDs
 * correctly), so this does not look like a hang -- it looks like something
 * is resetting the chip. There is no way to have a laptop watching Serial
 * when this happens in the field, so this module captures enough context
 * at boot time, persists it, and (via wifi_task) phones it home, so a
 * reboot storm that happened last week is still fully diagnosable today.
 *
 * Three layers, cheapest/fastest first:
 *
 *   1. RTC_NOINIT_ATTR breadcrumb (diag_rtc_state_t, in diag.cpp)
 *      Lives in RTC "no-init" memory, which is NOT cleared by a software
 *      reset, panic, task/interrupt watchdog reset, or (usually) a clean
 *      brown-out reset -- only by a true power-on (VDD_RTC actually lost
 *      power). Every task calls diag_mark() at points worth remembering;
 *      a dedicated low-priority heartbeat task updates a tick count and
 *      free-heap watermark every DIAG_HEARTBEAT_MS. Writing here costs a
 *      few RTC-memory bus cycles -- no flash wear, no blocking, safe to
 *      call from any task as often as you like.
 *
 *   2. NVS ring buffer (Preferences, namespace "diag")
 *      Once per boot, diag_init() reads esp_reset_reason() (why THIS boot
 *      happened) and pairs it with whatever survived in the RTC breadcrumb
 *      from the PREVIOUS run (what it was doing right before this reset),
 *      then appends one diag_record_t to a fixed-capacity ring buffer in
 *      flash. This survives full power loss, at the cost of one small
 *      flash write per boot (not per event) -- flash wear is a non-issue
 *      at that rate.
 *
 *      IMPORTANT CAVEAT: if the RTC domain itself lost power (a hard
 *      power-on reset, or a brownout deep enough to take VDD_RTC down with
 *      it), the breadcrumb is gone and prev_run_valid will read 0 --
 *      reset_reason is still captured correctly either way. Seeing
 *      prev_run_valid==0 paired with reset_reason==BROWNOUT or POWERON
 *      across most of the ring is itself a strong diagnostic signal:
 *      it means the supply rail is dropping hard enough to reset the RTC
 *      domain, not just glitching the digital core -- point a scope at
 *      3V3 rather than looking for a firmware bug.
 *
 *   3. WiFi auto-report (wifi_task.cpp)
 *      Once WiFi comes up after a boot that has a pending report,
 *      diag_pending_report_json() is POSTed to the Pi. See the server-side
 *      spec in claude/diag-server-endpoint-spec.md. This is what makes the
 *      log readable without ever touching the unit again -- if the Pi is
 *      unreachable the record still sits safely in the on-device ring
 *      buffer and prints over Serial at every boot.
 */
#pragma once
#include <Arduino.h>

// ── Build mode ────────────────────────────────────────────────────────────
// See DEV_BUILD in config.h. diag.cpp behaves identically either way; only
// Client.ino's setup() changes what it does with the BOD/TWDT.

// ── Ring buffer sizing ────────────────────────────────────────────────────
#define DIAG_LOG_CAPACITY   16      // boot records kept in NVS (ring buffer)
#define DIAG_HEARTBEAT_MS   2000    // RTC breadcrumb heartbeat period

// ── Breadcrumb codes ──────────────────────────────────────────────────────
// "What was the board doing right before it reset?" -- each task calls
// diag_mark() with one of these at points worth remembering. Steady-state
// loops mark every iteration (the write is essentially free), so the value
// captured is always "whatever was true in the last couple of RTC-memory
// writes before power/execution stopped," which for a fast reset is
// effectively "what was happening at the moment of reset."
typedef enum {
    DIAG_MARK_UNKNOWN = 0,     // RTC breadcrumb didn't survive (see prev_run_valid)
    DIAG_MARK_BOOT,             // diag_init() itself, very start of setup()
    DIAG_MARK_BOOT_DONE,        // setup() completed, all tasks running

    DIAG_MARK_LED_INIT,
    DIAG_MARK_LED_TICK,

    DIAG_MARK_RFID_WAIT_SETTLED,
    DIAG_MARK_RFID_HARD_RESET,      // RSTO toggle -- PN532 power-on self-test window
    DIAG_MARK_RFID_SOFT_INIT,       // SPI.begin() / nfc.begin() / getFirmwareVersion
    DIAG_MARK_RFID_SAMCONFIG,       // RF frontend enable -- ~150mA spike, see rfid_task.cpp
    DIAG_MARK_RFID_POLL_IDLE,       // steady-state poll, no card
    DIAG_MARK_RFID_CARD_READ,       // reading NTAG pages off a present card
    DIAG_MARK_RFID_INIT_GIVEUP,     // about to call ESP.restart() -- retries exhausted

    DIAG_MARK_OVERRIDE_POLL,

    DIAG_MARK_RELAY_TICK,
    DIAG_MARK_RELAY_MAIN_ON,        // main relay energizing -- coil inrush
    DIAG_MARK_RELAY_MAIN_OFF,

    DIAG_MARK_SESSION_TICK,

    DIAG_MARK_WIFI_CONNECTING,      // WiFi.begin() / radio TX burst
    DIAG_MARK_WIFI_RECONNECTING,
    DIAG_MARK_WIFI_SEND,            // TCP message to master_server.py
    DIAG_MARK_WIFI_IDLE,            // connected, waiting on next event

    DIAG_MARK_OTA_MANIFEST_FETCH,
    DIAG_MARK_OTA_DOWNLOAD,         // streaming firmware.bin -- HTTP + flash write current
    DIAG_MARK_OTA_FLASH_WRITE,
    DIAG_MARK_OTA_WAIT_IDLE_REBOOT,
    DIAG_MARK_OTA_REBOOT,           // about to call ESP.restart() -- verified image flashed

    DIAG_MARK_COUNT
} diag_mark_t;

// Human-readable label for a diag_mark_t (for Serial dump / JSON report).
const char* diag_mark_str(uint8_t mark);

// Human-readable label for an esp_reset_reason_t value.
const char* diag_reset_reason_str(uint8_t reason);

// Human-readable label for a rtc_get_reset_reason() RESET_REASON value
// (finer-grained than esp_reset_reason_t -- e.g. distinguishes
// RTCWDT_BROWN_OUT_RESET from a generic watchdog reset).
const char* diag_rtc_reason_str(uint8_t reason);

// One entry in the persisted ring buffer.
typedef struct __attribute__((packed)) {
    uint32_t boot_num;             // monotonic boot counter (this boot)
    uint8_t  reset_reason;         // esp_reset_reason_t for THIS boot
    uint8_t  rtc_reason_cpu0;      // rtc_get_reset_reason(0) -- finer-grained
    uint8_t  rtc_reason_cpu1;      // rtc_get_reset_reason(1)
    uint8_t  prev_run_valid;       // 1 if the fields below came from a live
                                    // RTC breadcrumb; 0 if RTC domain lost
                                    // power and everything below is unknown
    uint8_t  last_mark;            // diag_mark_t at (or just before) the reset
    uint16_t heartbeat_count;      // heartbeats completed by the previous run
    uint32_t free_heap_last;       // free heap at last heartbeat
    uint32_t free_heap_min;        // low-water mark for the previous run
} diag_record_t;

// ── Public API ────────────────────────────────────────────────────────────

// Call once, as early as possible in setup() (right after Serial.begin()).
// Captures the reset reason for this boot, pairs it with whatever survived
// in RTC memory from the previous run, appends to the NVS ring buffer, and
// prints the full history to Serial. Also arms the pending WiFi report.
void diag_init();

// Starts the heartbeat task (call once from setup(), after diag_init()).
// FreeRTOS task, tiny stack, priority 1, pin to Core 0.
void diag_start_heartbeat_task();

// Cheap breadcrumb update -- safe to call from any task, any frequency.
void diag_mark(uint8_t mark);

// Prints the full ring buffer (oldest to newest) to Serial as a table.
void diag_dump_serial();

// True if this boot produced a report that hasn't been confirmed sent yet.
bool diag_has_pending_report();

// Renders the pending report as a compact JSON object into `out` (NUL
// terminated). Returns false (and leaves `out` untouched) if there is no
// pending report or the buffer is too small. `machine_number` and
// `fw_version` are folded in so the server doesn't have to correlate by IP.
bool diag_pending_report_json(char* out, size_t out_len,
                               uint8_t machine_number, const char* fw_version);

// Call after the pending report has been POSTed successfully (2xx). Clears
// the pending flag so wifi_task doesn't keep re-sending it every reconnect.
void diag_mark_report_sent();
