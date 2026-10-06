/*
 * diag.cpp
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * See diag.h for the design rationale. Short version: capture why we just
 * rebooted (esp_reset_reason + per-CPU rtc_get_reset_reason) and what the
 * firmware was doing right before it happened (RTC-memory breadcrumb),
 * persist a rolling history of that in NVS, and hand the newest entry to
 * wifi_task to phone home.
 */

#include "diag.h"
#include <Preferences.h>
#include <esp_system.h>
#include <string.h>       // memset
#include <stdio.h>        // snprintf
#include <rom/rtc.h>     // RESET_REASON, rtc_get_reset_reason()
                          // NOTE: on some Arduino-ESP32 core versions this
                          // header lives at "esp32/rom/rtc.h" instead -- if
                          // this include fails to resolve, swap it for that
                          // path (the RESET_REASON enum and function
                          // signature are identical either way).

static const char* DIAG_NS = "diag";

// ── RTC "no-init" breadcrumb ─────────────────────────────────────────────
// Survives any reset that doesn't take the RTC power domain down with it
// (software reset, panic, task/interrupt/RTC watchdog, a clean brown-out
// trip). Lost on a true power-on (or a brownout deep enough to drag VDD_RTC
// down too) -- diag_init() detects that via the magic number and records
// prev_run_valid=0 rather than trusting stale/zeroed memory.
#define DIAG_RTC_MAGIC 0xD1A6B007UL

typedef struct {
    uint32_t magic;
    uint16_t heartbeat_count;
    uint8_t  last_mark;
    uint32_t free_heap_last;
    uint32_t free_heap_min;
} diag_rtc_state_t;

static RTC_NOINIT_ATTR diag_rtc_state_t s_rtc;

// ── Pending WiFi report (RAM only -- one boot's worth) ───────────────────
static diag_record_t s_pending;
static bool          s_pending_valid = false;

// ── Label tables ──────────────────────────────────────────────────────────
const char* diag_mark_str(uint8_t mark) {
    switch ((diag_mark_t)mark) {
        case DIAG_MARK_UNKNOWN:            return "unknown";
        case DIAG_MARK_BOOT:               return "boot";
        case DIAG_MARK_BOOT_DONE:          return "boot_done";
        case DIAG_MARK_LED_INIT:           return "led_init";
        case DIAG_MARK_LED_TICK:           return "led_tick";
        case DIAG_MARK_RFID_WAIT_SETTLED:  return "rfid_wait_settled";
        case DIAG_MARK_RFID_HARD_RESET:    return "rfid_hard_reset";
        case DIAG_MARK_RFID_SOFT_INIT:     return "rfid_soft_init";
        case DIAG_MARK_RFID_SAMCONFIG:     return "rfid_samconfig";
        case DIAG_MARK_RFID_POLL_IDLE:     return "rfid_poll_idle";
        case DIAG_MARK_RFID_CARD_READ:     return "rfid_card_read";
        case DIAG_MARK_RFID_INIT_GIVEUP:   return "rfid_init_giveup_reboot";
        case DIAG_MARK_OVERRIDE_POLL:      return "override_poll";
        case DIAG_MARK_RELAY_TICK:         return "relay_tick";
        case DIAG_MARK_RELAY_MAIN_ON:      return "relay_main_on";
        case DIAG_MARK_RELAY_MAIN_OFF:     return "relay_main_off";
        case DIAG_MARK_SESSION_TICK:       return "session_tick";
        case DIAG_MARK_WIFI_CONNECTING:    return "wifi_connecting";
        case DIAG_MARK_WIFI_RECONNECTING:  return "wifi_reconnecting";
        case DIAG_MARK_WIFI_SEND:          return "wifi_send";
        case DIAG_MARK_WIFI_IDLE:          return "wifi_idle";
        case DIAG_MARK_OTA_MANIFEST_FETCH: return "ota_manifest_fetch";
        case DIAG_MARK_OTA_DOWNLOAD:       return "ota_download";
        case DIAG_MARK_OTA_FLASH_WRITE:    return "ota_flash_write";
        case DIAG_MARK_OTA_WAIT_IDLE_REBOOT: return "ota_wait_idle_reboot";
        case DIAG_MARK_OTA_REBOOT:         return "ota_reboot";
        default:                           return "other";
    }
}

const char* diag_reset_reason_str(uint8_t reason) {
    switch ((esp_reset_reason_t)reason) {
        case ESP_RST_UNKNOWN:   return "UNKNOWN";
        case ESP_RST_POWERON:   return "POWERON";        // cold boot / VDD_RTC lost power
        case ESP_RST_EXT:       return "EXT_PIN";         // EN pin pulled low
        case ESP_RST_SW:        return "SW (ESP.restart)";
        case ESP_RST_PANIC:     return "PANIC";           // crash/abort/assert
        case ESP_RST_INT_WDT:   return "INTERRUPT_WDT";   // ISR ran too long
        case ESP_RST_TASK_WDT:  return "TASK_WDT";        // a task starved the idle task
        case ESP_RST_WDT:       return "OTHER_WDT";       // RTC WDT / other hardware WDT
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP_WAKE";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";         // supply sagged below threshold
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "OTHER";
    }
}

const char* diag_rtc_reason_str(uint8_t reason) {
    switch ((RESET_REASON)reason) {
        case POWERON_RESET:          return "POWERON";
        case SW_RESET:               return "SW";
        case OWDT_RESET:             return "OWDT";
        case DEEPSLEEP_RESET:        return "DEEPSLEEP";
        case SDIO_RESET:             return "SDIO";
        case TG0WDT_SYS_RESET:       return "TG0WDT_SYS";
        case TG1WDT_SYS_RESET:       return "TG1WDT_SYS";
        case RTCWDT_SYS_RESET:       return "RTCWDT_SYS";
        case INTRUSION_RESET:        return "INTRUSION";
        case TGWDT_CPU_RESET:        return "TGWDT_CPU";
        case SW_CPU_RESET:           return "SW_CPU";
        case RTCWDT_CPU_RESET:       return "RTCWDT_CPU";
        case EXT_CPU_RESET:          return "EXT_CPU";
        case RTCWDT_BROWN_OUT_RESET: return "RTCWDT_BROWNOUT";  // the smoking gun to watch for
        case RTCWDT_RTC_RESET:       return "RTCWDT_RTC";
        default:                     return "OTHER";
    }
}

// ── NVS ring buffer ───────────────────────────────────────────────────────
static void _append_to_ring(const diag_record_t& rec) {
    Preferences prefs;
    if (!prefs.begin(DIAG_NS, false)) {
        Serial.println("[diag] NVS open FAILED — reboot history not persisted this boot");
        return;
    }

    diag_record_t buf[DIAG_LOG_CAPACITY];
    size_t expect = sizeof(buf);
    size_t got = (prefs.getBytesLength("log") == expect)
                     ? prefs.getBytes("log", buf, expect)
                     : 0;
    if (got != expect) {
        memset(buf, 0, sizeof(buf));   // first boot ever, or layout changed
    }

    uint8_t head = prefs.getUChar("head", 0);
    if (head >= DIAG_LOG_CAPACITY) head = 0;   // guard against a corrupted value
    buf[head] = rec;
    head = (head + 1) % DIAG_LOG_CAPACITY;

    prefs.putBytes("log", buf, sizeof(buf));
    prefs.putUChar("head", head);
    prefs.end();
}

// Reads the ring buffer back and calls cb(rec, is_most_recent) oldest-first.
template <typename Fn>
static void _for_each_record(Fn cb) {
    Preferences prefs;
    if (!prefs.begin(DIAG_NS, true)) return;

    diag_record_t buf[DIAG_LOG_CAPACITY];
    size_t expect = sizeof(buf);
    size_t got = (prefs.getBytesLength("log") == expect)
                     ? prefs.getBytes("log", buf, expect)
                     : 0;
    uint8_t head = prefs.getUChar("head", 0);
    prefs.end();

    if (got != expect) return;   // nothing recorded yet
    if (head >= DIAG_LOG_CAPACITY) head = 0;

    // Oldest entry is at `head` (the slot about to be overwritten next);
    // walk forward from there so output prints in chronological order.
    for (uint8_t i = 0; i < DIAG_LOG_CAPACITY; i++) {
        uint8_t idx = (head + i) % DIAG_LOG_CAPACITY;
        if (buf[idx].boot_num == 0) continue;   // never-written slot
        cb(buf[idx], i == DIAG_LOG_CAPACITY - 1);
    }
}

// ── Public API ────────────────────────────────────────────────────────────
void diag_init() {
    esp_reset_reason_t reason = esp_reset_reason();
    RESET_REASON rtc0 = rtc_get_reset_reason(0);
    RESET_REASON rtc1 = rtc_get_reset_reason(1);

    bool prev_valid = (s_rtc.magic == DIAG_RTC_MAGIC);

    Preferences prefs;
    uint32_t boot_num = 1;
    if (prefs.begin(DIAG_NS, false)) {
        boot_num = prefs.getUInt("bootn", 0) + 1;
        prefs.putUInt("bootn", boot_num);
        prefs.end();
    }

    diag_record_t rec = {0};
    rec.boot_num        = boot_num;
    rec.reset_reason     = (uint8_t)reason;
    rec.rtc_reason_cpu0  = (uint8_t)rtc0;
    rec.rtc_reason_cpu1  = (uint8_t)rtc1;
    rec.prev_run_valid   = prev_valid ? 1 : 0;
    rec.last_mark        = prev_valid ? s_rtc.last_mark        : DIAG_MARK_UNKNOWN;
    rec.heartbeat_count  = prev_valid ? s_rtc.heartbeat_count   : 0;
    rec.free_heap_last   = prev_valid ? s_rtc.free_heap_last    : 0;
    rec.free_heap_min    = prev_valid ? s_rtc.free_heap_min     : 0;

    _append_to_ring(rec);

    s_pending = rec;
    s_pending_valid = true;

    // Reset the breadcrumb for THIS run.
    s_rtc.magic           = DIAG_RTC_MAGIC;
    s_rtc.heartbeat_count = 0;
    s_rtc.last_mark       = DIAG_MARK_BOOT;
    s_rtc.free_heap_last  = ESP.getFreeHeap();
    s_rtc.free_heap_min   = s_rtc.free_heap_last;

    Serial.print("[diag] boot #"); Serial.print(boot_num);
    Serial.print(" reset_reason="); Serial.print(diag_reset_reason_str(rec.reset_reason));
    Serial.print(" rtc0="); Serial.print(diag_rtc_reason_str(rec.rtc_reason_cpu0));
    Serial.print(" rtc1="); Serial.println(diag_rtc_reason_str(rec.rtc_reason_cpu1));
    if (prev_valid) {
        Serial.print("[diag] previous run: last_mark=");
        Serial.print(diag_mark_str(rec.last_mark));
        Serial.print(" heartbeats="); Serial.print(rec.heartbeat_count);
        Serial.print(" free_heap_last="); Serial.print(rec.free_heap_last);
        Serial.print(" free_heap_min="); Serial.println(rec.free_heap_min);
    } else {
        Serial.println("[diag] previous run: RTC breadcrumb lost "
                        "(power-on, or a brownout deep enough to reset VDD_RTC too)");
    }

    diag_dump_serial();
}

static void _diag_heartbeat_task(void* arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DIAG_HEARTBEAT_MS));
        uint32_t heap = ESP.getFreeHeap();
        s_rtc.heartbeat_count++;
        s_rtc.free_heap_last = heap;
        if (heap < s_rtc.free_heap_min) s_rtc.free_heap_min = heap;
    }
}

void diag_start_heartbeat_task() {
    xTaskCreatePinnedToCore(_diag_heartbeat_task, "diag_hb", 1536, NULL, 1, NULL, 0);
    Serial.println("[diag] heartbeat task started");
}

void diag_mark(uint8_t mark) {
    s_rtc.last_mark = mark;
}

void diag_dump_serial() {
    Serial.println("[diag] ── reboot history (oldest first) ──────────────────────────");
    Serial.println("[diag] boot  reset        rtc0/rtc1            valid  last_mark              hb    heap_last  heap_min");
    _for_each_record([](const diag_record_t& r, bool) {
        char line[160];
        snprintf(line, sizeof(line),
                 "[diag] %4lu  %-12s %-10s/%-10s %-5s  %-22s %-5u %-10lu %lu",
                 (unsigned long)r.boot_num,
                 diag_reset_reason_str(r.reset_reason),
                 diag_rtc_reason_str(r.rtc_reason_cpu0),
                 diag_rtc_reason_str(r.rtc_reason_cpu1),
                 r.prev_run_valid ? "yes" : "NO",
                 diag_mark_str(r.last_mark),
                 (unsigned)r.heartbeat_count,
                 (unsigned long)r.free_heap_last,
                 (unsigned long)r.free_heap_min);
        Serial.println(line);
    });
    Serial.println("[diag] ─────────────────────────────────────────────────────────────");
    Serial.flush();
}

bool diag_has_pending_report() {
    return s_pending_valid;
}

bool diag_pending_report_json(char* out, size_t out_len,
                               uint8_t machine_number, const char* fw_version) {
    if (!s_pending_valid) return false;

    int n = snprintf(out, out_len,
        "{"
        "\"machine\":%u,"
        "\"fw_version\":\"%s\","
        "\"boot_num\":%lu,"
        "\"reset_reason\":\"%s\","
        "\"rtc_reason_cpu0\":\"%s\","
        "\"rtc_reason_cpu1\":\"%s\","
        "\"prev_run_valid\":%s,"
        "\"last_mark\":\"%s\","
        "\"heartbeat_count\":%u,"
        "\"free_heap_last\":%lu,"
        "\"free_heap_min\":%lu,"
        "\"uptime_ms\":%lu"
        "}",
        machine_number,
        fw_version,
        (unsigned long)s_pending.boot_num,
        diag_reset_reason_str(s_pending.reset_reason),
        diag_rtc_reason_str(s_pending.rtc_reason_cpu0),
        diag_rtc_reason_str(s_pending.rtc_reason_cpu1),
        s_pending.prev_run_valid ? "true" : "false",
        diag_mark_str(s_pending.last_mark),
        (unsigned)s_pending.heartbeat_count,
        (unsigned long)s_pending.free_heap_last,
        (unsigned long)s_pending.free_heap_min,
        (unsigned long)millis());

    return (n > 0 && (size_t)n < out_len);
}

void diag_mark_report_sent() {
    s_pending_valid = false;
}
