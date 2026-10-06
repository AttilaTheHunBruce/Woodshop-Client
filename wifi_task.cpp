/*
 * wifi_task.cpp
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * Maintains WiFi connection and sends binary messages to the Pi server
 * matching the existing 16-byte request / 8-byte response protocol.
 *
 * Startup:
 *   The task is created AFTER task_rfid has signaled g_rfid_ready (see
 *   woodshop_esp32.ino setup()), so by the time we run, the PN532 is
 *   already up. We still add a tiny delay here as belt-and-suspenders.
 *
 * Message format (16 bytes, big-endian):
 *   Byte 0    : machine_number (uint8)
 *   Bytes 1-4 : member_id      (uint32)
 *   Bytes 5-6 : duration       (uint16, seconds)
 *   Bytes 7-8 : current        (uint16, 0.01A units)
 *   Bytes 9-12: connect_time   (uint32, seconds)
 *   Byte 13   : override_flags (uint8)
 *   Byte 14   : starts         (uint8)
 *   Byte 15   : stops          (uint8)
 *
 * Response (8 bytes, big-endian):
 *   Byte 0    : machine_number (uint8)
 *   Bytes 1-4 : member_id      (uint32)
 *   Bytes 5-6 : status         (uint16)
 *   Byte 7    : update_available (uint8)
 */

#include "wifi_task.h"
#include "config.h"
#include "node_config.h"
#include "current_sense.h"
#include "led_ctrl.h"
#include "ota_task.h"
#include "diag.h"

#include <WiFi.h>
#include <HTTPClient.h>

// Defensive fallback: DIAG_HTTP_PATH is defined in config.h (Aug 2026 diag
// module). If it's ever missing -- e.g. an older config.h got left in place
// during an update -- fall back to the same default rather than failing the
// build on this file alone.
#ifndef DIAG_HTTP_PATH
#define DIAG_HTTP_PATH "/diag/report"
#endif

// ── Network status LED ────────────────────────────────────────────────────────
// YELLOW alone, medium blink (LED_MEDIUM, ~800ms period), means
// "no network" -- WiFi.status() != WL_CONNECTED, which on the ESP32 Arduino
// core covers both "never associated" and "associated but never got an IP
// via DHCP" (WL_CONNECTED is only set on the GOT_IP event).
//
// (Changed from RED+YELLOW to YELLOW only -- RED is left to task_override
// and rfid_task.) YELLOW is also driven by task_rfid (ON/FAST/OFF
// transiently while handling a card: card present, PN532 watchdog reset,
// config card result). That writer doesn't coordinate with this one, so a
// card tap can transiently stomp on LED_YELLOW while the network is
// genuinely down. _set_net_led(false) is called again
// on every pass through the disconnected branch below (~5s) specifically
// so the indicator reappears on its own instead of staying silently
// cleared after whatever else touched those LEDs.
static bool _net_led_active = false;   // true while we own the "no network" LEDs

static void _set_net_led(bool connected) {
    led_set(LED_YELLOW, connected ? LED_OFF : LED_MEDIUM);
}

// ── Message builder ───────────────────────────────────────────────────────────
static void _build_message(uint8_t* buf,
                            uint8_t  machine,
                            uint32_t member_id,
                            uint16_t duration,
                            uint16_t current_01A,
                            uint32_t connect_time,
                            uint8_t  override_flags,
                            uint8_t  starts,
                            uint8_t  stops) {
    buf[0]  = machine;
    buf[1]  = (member_id >> 24) & 0xFF;
    buf[2]  = (member_id >> 16) & 0xFF;
    buf[3]  = (member_id >>  8) & 0xFF;
    buf[4]  = (member_id      ) & 0xFF;
    buf[5]  = (duration  >>  8) & 0xFF;
    buf[6]  = (duration       ) & 0xFF;
    buf[7]  = (current_01A >> 8) & 0xFF;
    buf[8]  = (current_01A    ) & 0xFF;
    buf[9]  = (connect_time >> 24) & 0xFF;
    buf[10] = (connect_time >> 16) & 0xFF;
    buf[11] = (connect_time >>  8) & 0xFF;
    buf[12] = (connect_time      ) & 0xFF;
    buf[13] = override_flags;
    buf[14] = starts;
    buf[15] = stops;
}

// ── Send message, return status code or -1 on failure ────────────────────────
static int _send_message(const char* server_ip, uint16_t port,
                          const uint8_t* msg, uint8_t* resp_buf) {
    WiFiClient client;
    client.setTimeout(5000);

    if (!client.connect(server_ip, port)) {
        Serial.println("[wifi] connect failed");
        return -1;
    }

    client.write(msg, MSG_SEND_LEN);

    uint32_t deadline = millis() + 5000;
    while (client.available() < MSG_RECV_LEN && millis() < deadline)
        delay(10);

    if (client.available() < MSG_RECV_LEN) {
        Serial.println("[wifi] response timeout");
        client.stop();
        return -1;
    }

    client.readBytes(resp_buf, MSG_RECV_LEN);
    client.stop();

    uint16_t status = ((uint16_t)resp_buf[5] << 8) | resp_buf[6];
    return (int)status;
}

// ── Notice the update_available bit in a response ─────────────────────────────
// Response byte 7 has been part of this wire format since the MicroPython
// days (see master_server.py BinaryMessage.RESPONSE_FORMAT) but was never
// actually read on the C side until now. Wakes task_ota for an immediate
// check instead of waiting for its periodic poll.
static void _check_ota_flag(const uint8_t* resp_buf) {
    if (resp_buf[7] == 1) {
        ota_notify_update_flagged();
    }
}

// ── Derive server IP from our own subnet ─────────────────────────────────────
static void _get_server_ip(char* out, size_t len) {
    IPAddress my_ip = WiFi.localIP();
    snprintf(out, len, "%d.%d.%d.%d",
             my_ip[0], my_ip[1], my_ip[2], SERVER_HOST_BYTE);
}

// Same Pi, Flask HTTP port instead of the binary TCP port -- same trick
// ota_task.cpp uses for firmware manifest/download requests.
static void _get_http_base_url(char* out, size_t len) {
    IPAddress my_ip = WiFi.localIP();
    snprintf(out, len, "http://%d.%d.%d.%d:%d",
             my_ip[0], my_ip[1], my_ip[2], SERVER_HOST_BYTE, FIRMWARE_HTTP_PORT);
}

// ── Diagnostic report ─────────────────────────────────────────────────────────
// POSTs the pending boot's diag record (reset reason + the RTC breadcrumb
// trail from the run that just ended -- last_mark, heartbeats, free heap) to
// the Pi as JSON. This is what lets a reboot-storm in the field show up on
// the server without ever touching the unit again. See the expected Flask
// route in claude/diag-server-endpoint-spec.md.
//
// Retried on every call to this function until it succeeds --
// diag_mark_report_sent() only fires on a 2xx response, so if the Pi happens
// to be down when a unit boots, the report just stays queued in RAM and goes
// out next time this runs. The full ring buffer is safe in NVS and printed
// over Serial regardless of whether this ever succeeds.
static void _send_diag_report_if_pending() {
    if (!diag_has_pending_report()) return;

    char json[384];
    if (!diag_pending_report_json(json, sizeof(json), g_machine_number, FW_VERSION)) {
        Serial.println("[wifi] diag report JSON build failed (buffer too small?)");
        return;
    }

    char base[40];
    _get_http_base_url(base, sizeof(base));

    HTTPClient http;
    http.setTimeout(OTA_HTTP_TIMEOUT_MS);
    http.begin(String(base) + DIAG_HTTP_PATH);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST((uint8_t*)json, strlen(json));
    http.end();

    if (code >= 200 && code < 300) {
        Serial.println("[wifi] diag report sent");
        diag_mark_report_sent();
    } else {
        Serial.printf("[wifi] diag report send failed, HTTP %d — will retry\r\n", code);
    }
}

// ── WiFi task ─────────────────────────────────────────────────────────────────
void task_wifi(void* arg) {
    Serial.println("[wifi] task entry");
    Serial.flush();

    // Extra belt-and-suspenders: we're already created after RFID init, but
    // give the PN532 one more moment of quiet before we fire up the radio.
    vTaskDelay(pdMS_TO_TICKS(500));

    diag_mark(DIAG_MARK_WIFI_CONNECTING);
    Serial.println("[wifi] connecting to " WIFI_SSID "...");
    Serial.flush();

    // No IP yet -- flash the network-down indicator while we try.
    _set_net_led(false);
    _net_led_active = true;

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 20000)
        vTaskDelay(pdMS_TO_TICKS(500));

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[wifi] connection failed — task continuing without server");
        // Indicator left on -- the reconnect branch below keeps retrying
        // and reasserting it.
    } else {
        Serial.print("[wifi] connected, IP=");
        Serial.println(WiFi.localIP());
        _set_net_led(true);
        _net_led_active = false;
        _send_diag_report_if_pending();
    }
    Serial.flush();

    char server_ip[20] = {0};
    _get_server_ip(server_ip, sizeof(server_ip));
    Serial.printf("[wifi] server=%s:%d\n", server_ip, SERVER_PORT);
    Serial.flush();

    uint8_t  msg[MSG_SEND_LEN];
    uint8_t  resp[MSG_RECV_LEN];
    bool     last_active = false;
    bool     denied_session = false;   // server vetoed the current card session
    uint32_t last_heartbeat = 0;

    for (;;) {
        // Reconnect if dropped
        if (WiFi.status() != WL_CONNECTED) {
            diag_mark(DIAG_MARK_WIFI_RECONNECTING);
            Serial.println("[wifi] reconnecting...");
            // Reassert every pass through this branch (~5s), not just on
            // the first drop -- see the _set_net_led() comment above for
            // why this needs to self-heal rather than fire once.
            _set_net_led(false);
            _net_led_active = true;
            WiFi.reconnect();
            vTaskDelay(pdMS_TO_TICKS(5000));
            _get_server_ip(server_ip, sizeof(server_ip));
            continue;
        }

        if (_net_led_active) {
            _set_net_led(true);
            _net_led_active = false;
        }

        diag_mark(DIAG_MARK_WIFI_IDLE);

        // Read shared session state
        bool     active         = false;
        uint32_t member_id      = 0;
        uint32_t start_ms       = 0;
        float    avg_current    = 0.0f;
        uint8_t  starts         = 0;
        uint8_t  stops          = 0;
        uint8_t  override_flags = 0;

        if (xSemaphoreTake(g_session_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            active         = g_session.active;
            member_id      = g_session.member_id;
            start_ms       = g_session.start_time_ms;
            avg_current    = g_session.avg_current;
            starts         = g_session.starts;
            stops          = g_session.stops;
            override_flags = g_session.override_flags;
            xSemaphoreGive(g_session_mutex);
        }

        uint32_t now_ms = millis();

        // Card insert event
        if (active && !last_active) {
            diag_mark(DIAG_MARK_WIFI_SEND);
            Serial.println("[wifi] sending INSERT");
            denied_session = false;
            _build_message(msg, g_machine_number, member_id,
                           0, 0, 0, override_flags, 0, 0);
            int status = _send_message(server_ip, SERVER_PORT, msg, resp);
            if (status >= 0) {
                _check_ota_flag(resp);
                // Server veto: any non-zero status (0x0004 = not authorized,
                // i.e. member not signed in at the kiosk or no permission for
                // this machine; 0x0002 not found; 0x0003 machine disabled)
                // revokes the local grant. relay_ctrl.cpp drives the relay
                // from (active && authorized), so clearing `authorized` drops
                // power within one 20 ms relay tick. The card stays "active"
                // so the REMOVE event/report still goes out normally, and
                // re-presenting the card re-runs rfid_task's grant + this check.
                if (status != 0x0000) {
                    Serial.printf("[wifi] server DENIED member %u (status 0x%04X) -- relay off\n",
                                  (unsigned)member_id, (unsigned)status);
                    denied_session = true;
                    if (xSemaphoreTake(g_session_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        // Only revoke if the same card is still the open session
                        if (g_session.active && g_session.member_id == member_id) {
                            g_session.authorized = false;
                        }
                        xSemaphoreGive(g_session_mutex);
                    }
                }
            }
        }

        // Card remove event
        if (!active && last_active) {
            diag_mark(DIAG_MARK_WIFI_SEND);
            uint32_t duration_s   = (now_ms - start_ms) / 1000;
            uint16_t current_01A  = (uint16_t)(avg_current * 100.0f);  // 0.01A units
            Serial.printf("[wifi] sending REMOVE duration=%us cur=%.2fA starts=%u stops=%u ovr=0x%02X\n",
                          duration_s, avg_current, starts, stops, override_flags);
            // A card the server rejected never really ran the machine: report
            // machine-on time, current and start/stop counts as 0. connect_time
            // (how long the card was in) is still sent -- the server uses it to
            // recognise this message as a REMOVE.
            uint16_t on_time_field = denied_session ? 0 : (uint16_t)min(duration_s, (uint32_t)0xFFFF);
            uint16_t cur_field     = denied_session ? 0 : current_01A;
            uint8_t  starts_field  = denied_session ? 0 : starts;
            uint8_t  stops_field   = denied_session ? 0 : stops;
            if (denied_session) {
                Serial.println("[wifi] session was DENIED -- reporting machine-on time 0");
            }
            _build_message(msg, g_machine_number, member_id,
                           on_time_field,
                           cur_field,
                           (uint32_t)min(duration_s, (uint32_t)0xFFFFFFFF),
                           override_flags, starts_field, stops_field);
            if (_send_message(server_ip, SERVER_PORT, msg, resp) >= 0) {
                _check_ota_flag(resp);
            }
            denied_session = false;
        }

        last_active = active;

        // Periodic heartbeat
        if (now_ms - last_heartbeat > WIFI_HEARTBEAT_MS) {
            last_heartbeat = now_ms;
            // Optional: send keepalive with current sense data
            _send_diag_report_if_pending();   // no-op once already sent
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
