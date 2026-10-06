/*
 * ota_task.cpp
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * Over-the-air firmware update.
 *
 * This replaces the old ESP32-C3/MicroPython ota.py, which worked by
 * downloading individual changed .py source files and rebooting so the
 * interpreter re-read them. That trick doesn't exist for us anymore: this
 * firmware is one linked C binary, so "update" means fetching a whole new
 * firmware.bin and flashing it — there's no such thing as replacing one
 * .cpp file on a running device.
 *
 * The ESP32 Arduino core has first-class support for exactly this. Flash is
 * normally partitioned with two app slots (ota_0 / ota_1) plus a small
 * otadata partition that records which one is active. Update.write() (via
 * writeStream() here) streams the new image into the INACTIVE slot while
 * this code keeps running untouched out of the active one — a bad download
 * can't corrupt what's currently running. Only once the new image is fully
 * written and its MD5 verified does Update.end() flip otadata; the new
 * image only actually starts executing after ESP.restart().
 *
 * IMPORTANT PREREQUISITE: Tools > Partition Scheme in the Arduino IDE must
 * be set to a scheme with two OTA app partitions and enough room for this
 * firmware's compiled size (e.g. any of the "...with OTA" schemes for a 4MB
 * board). If it's set to a single-app scheme, Update.begin() will fail with
 * "not enough space" and this whole file is a no-op. Also worth enabling,
 * separately, is the ESP-IDF bootloader's app-rollback option
 * (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE) so a new image that can't get back
 * on WiFi reverts itself to the last-known-good image on its own next boot,
 * without a truck roll to the shop.
 *
 * Trigger: the binary protocol's 8-byte response has always reserved byte 7
 * as update_available (see wifi_task.cpp / master_server.py) — wifi_task
 * calls ota_notify_update_flagged() when it sees that bit set, which wakes
 * this task for an immediate check. That alone isn't sufficient though: a
 * machine nobody has badged into for days would never see a response at
 * all, so this task ALSO polls GET /firmware/manifest.json on its own timer
 * (OTA_CHECK_INTERVAL_MS) regardless of card activity.
 *
 * Self-cancelling by design: every check compares the manifest's "version"
 * field against our own compiled-in FW_VERSION (config.h) and does nothing
 * if they already match. The server's UPDATE_AVAILABLE flag
 * (master_server.py) is a simple global boolean an admin flips on for the
 * whole rollout and remembers to flip off later — without this version
 * compare, every node would re-download and reflash itself on every
 * subsequent reboot until that happens. With it, a node that's already
 * current just no-ops even while the server keeps broadcasting
 * update_available=1 to everyone.
 *
 * Safety: flashing the inactive partition doesn't touch the relay GPIOs or
 * anything task_relay/task_rfid are doing — but the reboot to actually run
 * the new image does interrupt everything for a couple seconds, which would
 * drop relay power if it happened while a machine is mid-cut. So once a
 * download is verified, this task waits for g_session.active to go false
 * (no card session open) before calling ESP.restart().
 *
 * External library dependency (new):
 *   - ArduinoJson (bblanchon) — Arduino Library Manager. Used only to parse
 *     the small, fixed-shape manifest object; not for arbitrary JSON.
 *
 * Failure indication (Aug 2026): a failure inside the download+flash block
 * (as opposed to a routine manifest-fetch hiccup or "nothing staged yet")
 * blinks red+white together for OTA_FAIL_LED_MS and POSTs a report to the
 * Pi's /diag/report endpoint. See _report_ota_failure() below for why it's
 * timeboxed rather than persistent, and why it doesn't reuse diag.h's
 * boot-report machinery.
 */

#include "ota_task.h"
#include "config.h"
#include "diag.h"
#include "led_ctrl.h"
#include "node_config.h"

#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>

// ── Download-in-progress indicator (white, fast) ─────────────────────────────
// White is already used by override_task.cpp for "blast gate override
// engaged" (LED_MEDIUM, 800ms period) -- but only in override mode, not
// during a normal automatic blast-gate cycle, so it isn't lit most of the
// time. LED_FAST (400ms period) is visibly quicker than override's
// LED_MEDIUM on the same LED, so the two situations stay distinguishable
// even though they share a color. Doesn't touch green/yellow at all, so no
// interaction with rfid_task.cpp's card/session indicators.
//
// RAII rather than explicit led_set(LED_WHITE, LED_OFF) calls at each of
// _check_for_update()'s several early-return points below: a stack object's
// destructor runs on every exit from its scope, success or early return, so
// there's no exit path that can forget to clear it. This is deliberately
// more defensive than the server's first attempt at an equivalent LED
// (app.py's white/GPIO27 download indicator), which relied on a single
// explicit "turn it off" callback and ended up stuck on when that callback
// didn't fire.
struct _DownloadLedGuard {
    _DownloadLedGuard()  { led_set(LED_WHITE, LED_FAST); }
    ~_DownloadLedGuard() { led_set(LED_WHITE, LED_OFF); }
};

// ── Fast-check request flag ──────────────────────────────────────────────────
// Set by wifi_task (any task, really) when a server response flags
// update_available=1. Cleared once task_ota has acted on it. A plain
// volatile bool is fine here — it's a single-writer-per-edge coalescing
// flag, not a queue; missing a redundant set costs nothing since the flag
// stays true until consumed.
static volatile bool s_fast_check_requested = false;

void ota_notify_update_flagged() {
    s_fast_check_requested = true;
}

// ── Derive the Pi's HTTP (Flask) base URL from our own subnet ────────────────
// Same trick wifi_task.cpp uses for the TCP protocol port — same Pi, just a
// different port (FIRMWARE_HTTP_PORT instead of SERVER_PORT).
static void _get_firmware_base_url(char* out, size_t len) {
    IPAddress my_ip = WiFi.localIP();
    snprintf(out, len, "http://%d.%d.%d.%d:%d",
             my_ip[0], my_ip[1], my_ip[2], SERVER_HOST_BYTE, FIRMWARE_HTTP_PORT);
}

// ── Failed-upgrade reporting ──────────────────────────────────────────────────
// Called from every early-return inside _check_for_update()'s download+flash
// block (an update was available and this node committed to fetching it --
// as opposed to a routine manifest-fetch hiccup or "nothing staged yet",
// neither of which counts as a failed upgrade attempt and shouldn't alarm on
// every occurrence).
//
// Two things happen, and in this order -- LED first, deliberately blocking,
// then the report:
//   1. Red+white blink together at LED_FAST for OTA_FAIL_LED_MS (config.h),
//      then both clear. Timeboxed rather than persistent-until-cleared: red
//      is already shared, with no coordination, by override_task (main
//      override asserted) and rfid_task (PN532 hardware fault) -- see the
//      race condition override_task.cpp's own comment acknowledges. A short,
//      bounded alert keeps the window in which one of those could silently
//      overwrite this small; it is not itself the durable record.
//   2. A small JSON report POSTed to /diag/report -- the same endpoint and
//      Pi already used for boot/crash diagnostics. app.py's
//      _ingest_one_diag_report() only requires a "machine" field and stores
//      whatever else is present in a per-machine JSONL log, so this reuses
//      that plumbing with its own shape (event/reason/target_version) rather
//      than shoehorning into diag_record_t, which is boot-report-specific
//      and wouldn't fit a failure that never actually reset the board.
//      Best-effort, not retried: if the Pi doesn't ACK, the failure is still
//      visible locally (Serial + the LED alert just shown), and if this
//      whole download attempt gets retried later (next periodic check, or
//      another fast-check trigger) and fails again, that attempt sends its
//      own report.
static void _report_ota_failure(const char* reason, const char* target_version) {
    Serial.print("[ota] UPGRADE FAILED: ");
    Serial.println(reason);
    Serial.flush();

    led_set(LED_RED,   LED_FAST);
    led_set(LED_WHITE, LED_FAST);
    vTaskDelay(pdMS_TO_TICKS(OTA_FAIL_LED_MS));
    led_set(LED_RED,   LED_OFF);
    led_set(LED_WHITE, LED_OFF);

    char json[256];
    snprintf(json, sizeof(json),
             "{\"machine\":%u,\"event\":\"ota_failed\",\"reason\":\"%s\","
             "\"target_version\":\"%s\",\"fw_version\":\"%s\"}",
             (unsigned)g_machine_number, reason,
             target_version ? target_version : "?", FW_VERSION);

    char base[40];
    _get_firmware_base_url(base, sizeof(base));

    HTTPClient report_http;
    report_http.setTimeout(OTA_HTTP_TIMEOUT_MS);
    report_http.begin(String(base) + "/diag/report");
    report_http.addHeader("Content-Type", "application/json");
    int code = report_http.POST((uint8_t*)json, strlen(json));
    report_http.end();

    if (code >= 200 && code < 300) {
        Serial.println("[ota] failure report sent");
    } else {
        Serial.printf("[ota] failure report send failed, HTTP %d (not retried)\r\n", code);
    }
}

// ── Block until no card session is open, then reboot ─────────────────────────
static void _wait_idle_then_reboot() {
    diag_mark(DIAG_MARK_OTA_WAIT_IDLE_REBOOT);
    Serial.println("[ota] update verified and flashed — waiting for shop to go idle before reboot");
    Serial.flush();

    for (;;) {
        bool active = true;   // fail safe: assume active (i.e. keep waiting) if the mutex is contended
        if (xSemaphoreTake(g_session_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            active = g_session.active;
            xSemaphoreGive(g_session_mutex);
        }
        if (!active) break;
        vTaskDelay(pdMS_TO_TICKS(OTA_IDLE_POLL_MS));
    }

    diag_mark(DIAG_MARK_OTA_REBOOT);
    Serial.println("[ota] shop idle — rebooting into new firmware now");
    Serial.flush();
    delay(200);
    ESP.restart();
}

// ── Fetch manifest.json, download + flash firmware.bin if it's newer ─────────
// Returns without doing anything (not an error) if we're already current.
// Blocks (via _wait_idle_then_reboot) and never returns if a flash succeeds.
static void _check_for_update() {
    diag_mark(DIAG_MARK_OTA_MANIFEST_FETCH);
    char base[40];
    _get_firmware_base_url(base, sizeof(base));

    // ── Manifest ──────────────────────────────────────────────────────────
    HTTPClient http;
    http.setTimeout(OTA_HTTP_TIMEOUT_MS);
    http.begin(String(base) + "/firmware/manifest.json");
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("[ota] manifest fetch failed, HTTP %d\r\n", code);
        http.end();
        return;
    }
    String body = http.getString();
    http.end();

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
        Serial.print("[ota] manifest JSON parse failed: ");
        Serial.println(err.c_str());
        return;
    }

    const char* file    = doc["file"];
    const char* version = doc["version"];
    const char* md5     = doc["md5"];
    long        size    = doc["size"] | 0L;

    if (!file || !version || !md5 || size <= 0 || strlen(md5) != 32) {
        Serial.println("[ota] manifest missing/invalid fields — nothing to do "
                        "(no firmware.bin staged on the server yet?)");
        return;
    }

    if (strcmp(version, FW_VERSION) == 0) {
        Serial.printf("[ota] up to date (version %s)\r\n", FW_VERSION);
        return;
    }

    Serial.printf("[ota] update available: %s -> %s (%s, %ld bytes)\r\n",
                  FW_VERSION, version, file, size);
    Serial.flush();

    // ── Download + flash ──────────────────────────────────────────────────
    // White blinks fast (LED_FAST) for exactly this block's scope, via
    // _DownloadLedGuard's constructor/destructor -- covers every return path
    // below plus the fall-through success case, so the indicator can't be
    // left on by a forgotten cleanup call. Deliberately scoped to end here,
    // before _wait_idle_then_reboot(): once the flash write itself is done,
    // "downloading" is over even if the actual reboot is still pending a
    // safe idle moment.
    {
    _DownloadLedGuard _download_led_guard;

    diag_mark(DIAG_MARK_OTA_DOWNLOAD);
    HTTPClient fw_http;
    fw_http.setTimeout(OTA_HTTP_TIMEOUT_MS);
    fw_http.begin(String(base) + "/firmware/" + file);
    int fw_code = fw_http.GET();
    if (fw_code != HTTP_CODE_OK) {
        char reason[48];
        snprintf(reason, sizeof(reason), "firmware fetch failed, HTTP %d", fw_code);
        fw_http.end();
        _report_ota_failure(reason, version);
        return;
    }

    int content_len = fw_http.getSize();
    if (content_len <= 0) {
        fw_http.end();
        _report_ota_failure("firmware response has no usable Content-Length", version);
        return;
    }

    if (!Update.begin(content_len)) {
        char reason[80];
        snprintf(reason, sizeof(reason), "Update.begin() failed: %s (check Partition Scheme)",
                 Update.errorString());
        fw_http.end();
        _report_ota_failure(reason, version);
        return;
    }

    if (!Update.setMD5(md5)) {
        Update.abort();
        fw_http.end();
        _report_ota_failure("Update.setMD5() rejected the manifest md5 (malformed?)", version);
        return;
    }

    diag_mark(DIAG_MARK_OTA_FLASH_WRITE);
    WiFiClient* stream = fw_http.getStreamPtr();
    size_t written = Update.writeStream(*stream);
    fw_http.end();

    if (written != (size_t)content_len) {
        char reason[64];
        snprintf(reason, sizeof(reason), "short write: %u of %d bytes",
                 (unsigned)written, content_len);
        Update.abort();
        _report_ota_failure(reason, version);
        return;
    }

    if (!Update.end(true) || !Update.isFinished()) {
        char reason[80];
        snprintf(reason, sizeof(reason), "Update.end() failed (bad MD5 / incomplete image?): %s",
                 Update.errorString());
        _report_ota_failure(reason, version);
        return;
    }
    } // _download_led_guard destructs here -- white LED off before the idle wait

    // Flash write succeeded and MD5 matched — this call never returns.
    _wait_idle_then_reboot();
}

// ── OTA task ──────────────────────────────────────────────────────────────────
void task_ota(void* arg) {
    Serial.println("[ota] task entry");
    Serial.flush();

    // Give task_wifi a head start connecting before our first check.
    vTaskDelay(pdMS_TO_TICKS(3000));

    uint32_t last_check_ms = 0;
    bool     first_pass    = true;   // force one check shortly after boot

    for (;;) {
        uint32_t now = millis();
        bool due = first_pass || s_fast_check_requested ||
                   (now - last_check_ms >= OTA_CHECK_INTERVAL_MS);

        if (due && WiFi.status() == WL_CONNECTED) {
            first_pass            = false;
            s_fast_check_requested = false;
            last_check_ms          = now;
            _check_for_update();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
