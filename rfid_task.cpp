/*
 * rfid_task.cpp
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * PN532 RFID reader task. Uses the Elechouse PN532/PN532_SPI library
 * (proven working on this hardware in the standalone test sketch —
 * returns firmware 1.6, reads 7-byte NTAG215 UIDs reliably).
 *
 * Wiring (this board; note MISO/MOSI are OPPOSITE the ESP32 VSPI defaults):
 *   SCK  -> GPIO18
 *   MISO -> GPIO23   (PN532 MISO out to ESP32 MISO in)
 *   MOSI -> GPIO19   (ESP32 MOSI out to PN532 MOSI in)
 *   SS   -> GPIO21   (10k pullup to 3V3)
 *   IRQ  -> GPIO4    (10k pullup to 3V3; falling-edge ISR)
 *   RSTO -> GPIO13   (ESP32 drives PN532 RSTPD_N; active LOW to reset)
 *
 * ── PN532 IRQ line — what fires it, and what we do with it ───────────────────
 *
 * In SPI mode the PN532 is a slave: it cannot initiate SPI transactions on
 * its own, and it cannot "push" data to the ESP32 spontaneously.  Instead it
 * uses the IRQ line to say "I have data ready — come read it over SPI."
 *
 * IRQ asserts LOW (open-drain) on EXACTLY TWO occasions per host command:
 *
 *   Edge 1 — ACK ready (~1-2 ms after the ESP32 sends a command).
 *             The PN532 has placed its ACK frame on the SPI bus.  The host
 *             must read the ACK before the PN532 will proceed.
 *
 *   Edge 2 — Response ready.  The PN532 has finished processing the command
 *             and placed the full response on the SPI bus:
 *               • readPassiveTargetID: response contains the card UID if a
 *                 card was found, or an error code if the PN532 timed out
 *                 after exhausting its PassiveActivationRetries.
 *               • mifareultralight_ReadPage: response contains 4 data bytes.
 *               • Any other command: response contains the result.
 *             IRQ goes HIGH again after the host reads the full response.
 *
 * So one call to readPassiveTargetID() produces TWO falling IRQ edges when
 * the PN532 is alive and well.  If NO edge arrives at all during the poll
 * window the PN532 did not respond to the command — a reliable indicator that
 * it is wedged and needs a hard reset.
 *
 * Current implementation:
 *   s_pn532_irq_flag  — volatile bool, set by the ISR on every falling IRQ
 *                       edge, cleared by the task before each poll.  After
 *                       the poll, the task checks this flag for the watchdog.
 *   s_rfid_task_handle — stored so the ISR can send a FreeRTOS task
 *                        notification, waking the task immediately if it
 *                        happens to be sleeping (e.g. in the retry-pause
 *                        path of the init failure loop).
 *
 * Future: once the polling approach is fully validated, this infrastructure
 * makes it straightforward to shift to a split send-then-wait model using
 * lower-level PN532 commands (sendCommandCheckAck / readResponse), where the
 * task blocks on the task notification instead of spinning inside the library
 * wait loop.
 *
 * OPERATIONAL NOTE — card-to-antenna distance:
 *   This PN532 board's RF frontend saturates when the card is too close.
 *   Optimal read distance is approximately 1.25 inches (32 mm) from the
 *   antenna coil. Reads become unreliable below ~0.5" (saturation) and
 *   above ~2" (field too weak). Keep this in mind when designing the
 *   enclosure / card-reader mounting: a small standoff or recess that
 *   prevents users from pressing the card flat against the board surface
 *   will improve read reliability significantly.
 */

#include "rfid_task.h"
#include "led_ctrl.h"
#include "current_sense.h"
#include "node_config.h"
#include "config.h"

#include <SPI.h>
#include <PN532_SPI.h>
#include <PN532.h>

// ── Elechouse library objects ────────────────────────────────────────────────
// SPI.begin() is called explicitly in _pn532_init() with our remapped pins
// BEFORE nfc.begin() to prevent the library from binding to ESP32 VSPI defaults.
static PN532_SPI _pn532spi(SPI, PIN_PN532_SS);
static PN532     nfc(_pn532spi);

static bool _pn532_ok = false;

// ── IRQ interrupt state ──────────────────────────────────────────────────────
// s_pn532_irq_flag:
//   Set by _pn532_irq_isr() on every falling edge of PIN_PN532_IRQ.
//   Cleared by task_rfid before each readPassiveTargetID() call.
//   Checked after each call: if still false, the PN532 sent no ACK or
//   response — increment the miss counter toward a hard-reset trigger.
//
// s_rfid_task_handle:
//   Saved at task entry so the ISR can send a FreeRTOS task notification,
//   which wakes the task out of any ulTaskNotifyTake() sleep instantly.
//   Must be set before attachInterrupt() is called.
static volatile bool  s_pn532_irq_flag      = false;
static volatile int   s_irq_edge_count      = 0;   // total edges since last clear
static TaskHandle_t   s_rfid_task_handle    = NULL;

// s_irq_in_use: true only when attachInterrupt() has been called for the IRQ
// pin.  The poll-loop watchdog is gated on this flag so that leaving the
// interrupt detached (the current operating mode) does not cause spurious
// "no IRQ" warnings and reinit triggers on every single poll cycle.
static bool           s_irq_in_use          = false;

// ISR — runs in IRAM so it works even while flash cache is busy.
// Minimal work: set flag, increment counter, notify task.
static void IRAM_ATTR _pn532_irq_isr() {
    s_pn532_irq_flag = true;
    s_irq_edge_count++;
    // Wake the rfid task if it is blocked in ulTaskNotifyTake().
    // portYIELD_FROM_ISR causes an immediate context switch to the notified
    // task if it has higher priority than whatever was running when IRQ fired.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_rfid_task_handle) {
        vTaskNotifyGiveFromISR(s_rfid_task_handle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ── PN532 hardware reset ─────────────────────────────────────────────────────
// Drives PIN_PN532_RSTO (→ PN532 RSTPD_N, active low) LOW for 150 ms then
// releases HIGH and waits 500 ms for the PN532 power-on self-test to finish.
//
// The pin is configured as OUTPUT here and left OUTPUT HIGH afterward.
// Driving it HIGH is safe: RSTPD_N is an input on the PN532, and HIGH means
// "normal operation."
//
// This is called:
//   • Once at task_rfid startup (after g_system_settled), before any SPI
//     traffic — ensures the PN532 always boots from a known state regardless
//     of what happened at power-up.
//   • Automatically inside task_rfid if the IRQ watchdog detects the PN532
//     has stopped responding.
static void _pn532_hard_reset() {
    Serial.println("[rfid] hard-resetting PN532 via RSTO (GPIO13 -> RSTPD_N)...");
    Serial.flush();
    pinMode(PIN_PN532_RSTO, OUTPUT);
    digitalWrite(PIN_PN532_RSTO, LOW);    // assert reset (active low)
    delay(150);                           // hold well beyond minimum pulse width
    digitalWrite(PIN_PN532_RSTO, HIGH);   // release — PN532 begins boot sequence
    delay(2000);                          // PN532 power-on self-test + supply settle.
                                          // The self-test briefly activates the RF
                                          // frontend (~150 mA spike).  On marginal
                                          // supplies 500 ms was not enough — the
                                          // self-test current was still dying down
                                          // when SPI.begin() added load and pushed
                                          // VDD below the POR threshold (~2.0 V).
                                          // 2000 ms is ample for any PN532 variant.
    Serial.println("[rfid] RSTO released — PN532 booting");
    Serial.flush();
}

// ── Small utilities ──────────────────────────────────────────────────────────
static void _hex_byte(uint8_t b) {
    if (b < 16) Serial.print('0');
    Serial.print(b, HEX);
}

// Note: relay control is centralized in task_relay (relay_ctrl.cpp).
// task_rfid only updates g_session.active/authorized and lets task_relay
// decide the actual pin states. This avoids two tasks racing on the same
// GPIO and makes override-driven relay-on behavior cleanly possible.

// ── NTAG page read with retries ──────────────────────────────────────────────
// Elechouse PN532::mifareultralight_ReadPage reads 4 bytes at a time into a
// 4-byte buffer. Keep the retry pattern from the C3 project but with longer
// gaps — 20ms was marginal in testing, 50ms is reliable.
static bool _read_page_4(uint8_t page, uint8_t* out4) {
    for (int attempt = 0; attempt < 5; attempt++) {
        if (nfc.mifareultralight_ReadPage(page, out4)) return true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

// Read CARD_TOTAL_LEN bytes starting at RFID_PAGE_START, 4 bytes per page.
static bool _read_card_bytes(uint8_t* card_buf) {
    const int CARD_LEN = CARD_TOTAL_LEN;
    const int pages    = (CARD_LEN + 3) / 4;
    uint8_t page_buf[4];

    // Let the tag settle in the RF field before the first read. 150ms was
    // used on the C3 project and proved reliable; shorter delays cause
    // intermittent first-page failures.
    vTaskDelay(pdMS_TO_TICKS(150));

    int filled = 0;
    for (int i = 0; i < pages; i++) {
        uint8_t page = RFID_PAGE_START + i;
        if (!_read_page_4(page, page_buf)) {
            Serial.print("[rfid] read failed page ");
            Serial.print(page);
            Serial.println(" (card may be too close/far — optimal ~1.25\" from antenna)");
            return false;
        }
        int needed = min(4, CARD_LEN - filled);
        memcpy(card_buf + filled, page_buf, needed);
        filled += needed;
        // Small gap between pages — the PN532 and NTAG both benefit
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return (filled == CARD_LEN);
}

// ── Crypto + permissions ─────────────────────────────────────────────────────
// NOTE: member cards (Lee's WriteNTAG215.py "Layout v2", Aug 2026) carry no
// Ed25519 signature, so there is no member-card verification step anymore --
// see the SECURITY NOTE in config.h. Config (admin) cards are also unsigned now and are
// still verified inline where they're handled below.

// Parse the 4-byte MemberID field as ASCII decimal digits (Layout v2 writes
// e.g. member 42 as the text "0042", not a binary value). Returns false —
// and leaves *out_id unset — if any of the 4 bytes isn't '0'..'9', which we
// treat as "this isn't a Layout v2 member card" since there is no other tag
// to check.
static bool _parse_member_id_ascii(const uint8_t* payload, uint32_t* out_id) {
    uint32_t id = 0;
    for (int i = 0; i < MEM_MEMBER_ID_LEN; i++) {
        uint8_t c = payload[MEM_OFF_MEMBER_ID + i];
        if (c < '0' || c > '9') return false;
        id = id * 10 + (uint32_t)(c - '0');
    }
    *out_id = id;
    return true;
}

// Permission bitmap lookup. Permission bytes live at MEM_OFF_PERMS (offset 4
// in Layout v2 — the bitmap starts right after the 4-byte ASCII member_id,
// with no header). 128 bits = 16 bytes = 4 uint32 words, one bit per machine.
static bool _has_permission(const uint8_t* payload, uint8_t machine_num) {
    if (machine_num < 1 || machine_num > 128) return false;
    uint8_t idx      = machine_num - 1;
    uint8_t word_idx = idx / 32;
    uint8_t bit_idx  = idx % 32;
    uint32_t perm;
    memcpy(&perm, payload + MEM_OFF_PERMS + word_idx * 4, 4);
    return (perm >> bit_idx) & 1;
}

// Extract a null-terminated name from a fixed-length ASCII field. Copies
// at most CARD_NAME_LEN bytes and guarantees a null terminator. Any
// non-printable bytes (including embedded nulls) end the string early so
// a corrupted card doesn't produce garbage serial output.
static void _extract_name(const uint8_t* payload, size_t offset, char* out) {
    size_t i = 0;
    for (; i < CARD_NAME_LEN; i++) {
        uint8_t c = payload[offset + i];
        if (c < 0x20 || c > 0x7E) break;   // stop at null or non-printable
        out[i] = (char)c;
    }
    out[i] = '\0';
}

// ── PN532 init (no reset — caller must call _pn532_hard_reset() first) ───────
// Assumes the PN532 has already completed its power-on self-test via a
// preceding _pn532_hard_reset() call.
//
// Returns true only if BOTH getFirmwareVersion AND SAMConfig succeed.
// Returning true with a failed SAMConfig was the previous bug: the RF
// frontend was never enabled so cards couldn't be read, yet the poll loop
// ran anyway — activating the RF on every InListPassiveTarget call, drawing
// the ~150 mA current spike, and collapsing a marginal supply.
//
// SAMConfig timing notes (no-USB / marginal-supply scenario):
//   getFirmwareVersion() queries a read-only register — low current, almost
//   always succeeds.  SAMConfig enables the RF frontend (high current) so a
//   weak supply can droop enough to corrupt the SPI response.  We now wait
//   substantially longer before each attempt:
//     delay(500 ms) after getFirmwareVersion
//     then 1 s / 2 s / 3 s before each of 3 SAMConfig tries
//   Total "quiet" time before the first RF activation attempt: ~3.5 s after
//   the hard reset is released.  Increase PN532_SAMCONFIG_BASE_DELAY_MS or
//   PN532_SAMCONFIG_TRIES in config.h if SAMConfig still fails reliably.
//
// Hardware fix (mandatory for no-USB reliability):
//   Add a 470 µF (or larger) electrolytic capacitor from 3V3 to GND as close
//   to the ESP32 module as possible.  The "flash read err, 1000" message on
//   cold boot is the ROM bootloader itself failing to read flash — no software
//   change can prevent that; only bulk decoupling capacitance can hold the
//   rail stable during the current inrush at power-on.
static bool _pn532_init() {
    Serial.println("[rfid] init");
    Serial.flush();

    // Pull IRQ high immediately, before any SPI traffic.
    // The PN532 IRQ is open-drain: it can only pull the line LOW.  If there is
    // no external pull-up resistor, or if the PN532 is asserting IRQ from a
    // previous unread transaction, we want the ESP32's internal ~45kΩ pull-up
    // fighting toward 3V3 so the pin settles to a known readable state.
    // Setting this here (not after a successful SAMConfig) means the pin is
    // defined even on a first-attempt failure.
    pinMode(PIN_PN532_IRQ, INPUT_PULLUP);

    // CRITICAL: remap VSPI to our wiring BEFORE the library touches SPI.
    // Without this explicit call, the ESP32 Arduino core binds VSPI to its
    // default pins (MOSI=23, MISO=19) which is the wrong way round for this
    // board and causes getFirmwareVersion() to time out silently.
    //
    // NOTE: SS is NOT passed to SPI.begin() here.  The PN532_SPI library
    // manages SS entirely through manual digitalWrite() calls (it stores the
    // pin number in its own _ss field).  Passing SS to SPI.begin() can hand
    // ownership of that pin to the SPI hardware CS controller, which then
    // conflicts with the library's manual toggling.
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    // Explicitly assert GPIO21 as OUTPUT HIGH after SPI.begin(), because the
    // library's internal SPI.begin() call (inside nfc.begin()) may reinitialize
    // VSPI with default pins and inadvertently release GPIO21.
    pinMode(PIN_PN532_SS, OUTPUT);
    digitalWrite(PIN_PN532_SS, HIGH);

    // Extra settle time after SPI bus init before the library takes over.
    delay(500);

    nfc.begin();

    // Re-assert SS as OUTPUT HIGH after nfc.begin(), which internally calls
    // SPI.begin() again (with default VSPI pins).  This ensures GPIO21 is still
    // in OUTPUT mode after that second SPI.begin() call.
    pinMode(PIN_PN532_SS, OUTPUT);
    digitalWrite(PIN_PN532_SS, HIGH);

    // Give the PN532 time to finish its internal startup before any command.
    delay(500);

    uint32_t versiondata = 0;
    for (int attempt = 1; attempt <= 5; attempt++) {
        Serial.print("[rfid] fw attempt ");  Serial.print(attempt);
        Serial.print(" ... ");
        Serial.flush();

        versiondata = nfc.getFirmwareVersion();
        if (versiondata) {
            Serial.print("OK chip=PN5");
            _hex_byte((versiondata >> 24) & 0xFF);
            Serial.print(" ver=");
            Serial.print((versiondata >> 16) & 0xFF);
            Serial.print('.');
            Serial.println((versiondata >> 8) & 0xFF);
            break;
        }
        Serial.println("no response");
        delay(500);
    }

    if (!versiondata) {
        Serial.println("[rfid] PN532 NOT DETECTED");
        Serial.println("[rfid]   check: SEL jumpers in SPI mode, 3.3V power,");
        Serial.println("[rfid]          MISO/MOSI not swapped, SS pullup, wiring continuity");
        led_set(LED_RED, LED_FAST);
        return false;
    }

    // SAMConfig: call promptly after getFirmwareVersion — the standalone test
    // sketch (which worked reliably) used no delay between these two calls.
    // Long delays between getFirmwareVersion and SAMConfig were added to help
    // the power supply but appear to CAUSE SAMConfig failures, probably because
    // the PN532 enters an unexpected internal state during the long silence.
    //
    // Retry strategy: up to PN532_SAMCONFIG_TRIES attempts.  On retry, do a
    // brief 100 ms pause (not seconds) then try again — we want to recover
    // from a transient, not starve the PN532 of commands.
    bool sam_ok = false;
    for (int i = 1; i <= PN532_SAMCONFIG_TRIES; i++) {
        if (i > 1) delay(100);   // brief pause on retry only; first try is immediate
        Serial.printf("[rfid] SAMConfig try %d/%d ... ",
                      i, PN532_SAMCONFIG_TRIES);
        Serial.flush();
        sam_ok = nfc.SAMConfig();
        Serial.println(sam_ok ? "OK" : "failed");
        Serial.flush();
        if (sam_ok) break;
    }


    if (!sam_ok) {
        // ALL SAMConfig attempts failed.  Return false so the caller's retry
        // loop issues another hard reset rather than entering the poll loop
        // with the RF frontend disabled.
        Serial.println("[rfid] SAMConfig FAILED — RF frontend not enabled, returning false");
        Serial.flush();
        led_set(LED_RED, LED_FAST);
        return false;
    }

    // Short passive-activation timeout so readPassiveTargetID returns quickly
    // when no card is present — keeps the poll loop responsive.
    nfc.setPassiveActivationRetries(0x05);

    Serial.println("[rfid] init complete — RF frontend active");
    led_set(LED_GREEN, LED_OFF);
    return true;
}

// ── PN532 full re-init (reset + software init) ───────────────────────────────
// Called by the watchdog path inside task_rfid.  Detaches the ISR, resets the
// hardware, re-runs software init, and re-attaches the ISR.  Returns true if
// the PN532 came back successfully.
static bool _pn532_reinit() {
    // Detach ISR so we don't get spurious edges during reset transitions.
    detachInterrupt(digitalPinToInterrupt(PIN_PN532_IRQ));
    s_pn532_irq_flag = false;
    s_irq_edge_count = 0;

    _pn532_hard_reset();
    bool ok = _pn532_init();

    // Re-attach regardless of outcome; if still broken the watchdog will
    // fire again and try once more.
    attachInterrupt(digitalPinToInterrupt(PIN_PN532_IRQ),
                    _pn532_irq_isr, FALLING);
    return ok;
}

// ── RFID task ────────────────────────────────────────────────────────────────
void task_rfid(void* arg) {
    Serial.println("[rfid] task entry — waiting for system to settle");
    Serial.flush();

    // ── Save task handle for ISR use ─────────────────────────────────────────
    // Must be stored BEFORE attachInterrupt() so the ISR can safely reference
    // it.  Storing it here (not in setup()) keeps the handle in rfid_task.cpp
    // and avoids exposing it as a global.
    s_rfid_task_handle = xTaskGetCurrentTaskHandle();

    // ── Wait for g_system_settled ─────────────────────────────────────────────
    // setup() gives this semaphore after creating all non-WiFi tasks (led,
    // override, relay, session).  Waiting here means the PN532 hard reset and
    // init happen only once the system is in a stable operating state — power
    // rails have settled, other tasks have started, and we are not racing with
    // the ESP32 boot transients.
    //
    // We wait up to 30 s; in normal operation setup() gives the semaphore
    // within a few hundred ms of creating this task.
    if (xSemaphoreTake(g_system_settled, pdMS_TO_TICKS(30000)) != pdTRUE) {
        Serial.println("[rfid] WARNING: g_system_settled timeout — proceeding anyway");
    }
    Serial.println("[rfid] system settled — beginning PN532 init");
    Serial.flush();

    // ── Hardware reset ────────────────────────────────────────────────────────
    // Drive GPIO13 (→ PN532 RSTPD_N) LOW then HIGH to guarantee the PN532
    // starts from a known state regardless of power-on transients or any
    // leftover state from a previous session.  The 2 s post-release wait in
    // _pn532_hard_reset() gives the PN532 self-test time to complete before
    // any SPI traffic.
    _pn532_hard_reset();

    _pn532_ok = _pn532_init();

    // ── IRQ interrupt (DISABLED) ──────────────────────────────────────────────
    // The ISR (_pn532_irq_isr) and all IRQ tracking infrastructure remain in
    // place, but attachInterrupt() is not called.  The poll loop's IRQ watchdog
    // is gated on s_irq_in_use (false) and will not fire.
    //
    // To re-enable: set s_irq_in_use = true and uncomment attachInterrupt below.
    //
    // if (_pn532_ok) {
    //     s_pn532_irq_flag = false;
    //     s_irq_edge_count = 0;
    //     s_irq_in_use     = true;
    //     attachInterrupt(digitalPinToInterrupt(PIN_PN532_IRQ),
    //                     _pn532_irq_isr, FALLING);
    //     Serial.printf("[rfid] IRQ interrupt attached (GPIO%d, FALLING edge)\r\n",
    //                   PIN_PN532_IRQ);
    //     Serial.flush();
    // }

    // ── Signal WiFi task ──────────────────────────────────────────────────────
    // Always signal ready (success or failure) so setup() can proceed to
    // create task_wifi.  WiFi startup is independent of RFID status.
    xSemaphoreGive(g_rfid_ready);
    Serial.printf("[rfid] signaled g_rfid_ready pn532_ok=%d\r\n", _pn532_ok ? 1 : 0);
    Serial.flush();

    // ── Init failure loop ─────────────────────────────────────────────────────
    if (!_pn532_ok) {
        // Init retry loop — software init only, no hard reset.
        // After PN532_INIT_RETRIES failures, reboot and let the PN532 power-on
        // reset itself from scratch.
        for (int attempt = 1; ; attempt++) {
            Serial.printf("[rfid] init retry %d/%d — waiting 5 s\r\n",
                          attempt, PN532_INIT_RETRIES);
            Serial.flush();

            vTaskDelay(pdMS_TO_TICKS(5000));

            _pn532_ok = _pn532_init();

            if (_pn532_ok) {
                Serial.println("[rfid] init succeeded on retry — entering poll loop");
                Serial.flush();
                break;
            }

            if (attempt >= PN532_INIT_RETRIES) {
                Serial.println("[rfid] all init retries exhausted — rebooting");
                Serial.flush();
                delay(200);
                ESP.restart();
            }
        }
    }

    // ── Main poll loop ────────────────────────────────────────────────────────
    Serial.println("[rfid] ready — waiting for card");
    Serial.flush();

    uint8_t  last_uid[7]    = {0};
    uint8_t  uid[7]         = {0};
    uint8_t  uid_len        = 0;
    bool     card_present   = false;
    uint32_t last_seen_ms   = 0;
    const uint32_t REMOVE_GRACE_MS = 500;

    uint8_t  card_buf[CARD_TOTAL_LEN];
    uint32_t last_detect_ms    = millis();
    uint32_t last_heartbeat_ms = millis();

    // Consecutive polls with no IRQ activity at all.
    // If this reaches PN532_IRQ_MISS_LIMIT the PN532 is considered wedged.
    int irq_miss_count = 0;

    for (;;) {
        uint32_t now = millis();

        // Heartbeat
//        if (now - last_heartbeat_ms > 5000) {
//            last_heartbeat_ms = now;
//            Serial.printf("[rfid] poll alive — card_present=%d irq_misses=%d\r\n",
//                          card_present, irq_miss_count);
//            Serial.flush();
//        }

        // ── IRQ flag management ───────────────────────────────────────────────
        // Clear flag and edge count before the poll.  After readPassiveTargetID
        // returns we inspect s_pn532_irq_flag to decide whether the PN532 is
        // alive:
        //
        //   flag = true  → at least one IRQ edge arrived (ACK or response) —
        //                  PN532 is alive; reset the miss counter.
        //   flag = false → no edge at all during the RFID_POLL_MS window —
        //                  PN532 did not respond; increment miss counter.
        //
        // Note: with setPassiveActivationRetries(0x05), the PN532 returns a
        // "no card" error response within ~25-30 ms, well inside RFID_POLL_MS.
        // The IRQ fires twice (ACK + error response) even when no card is
        // present.  So flag=false after a full RFID_POLL_MS timeout is a
        // strong sign the chip is not communicating at all.
        s_pn532_irq_flag = false;
        // (s_irq_edge_count is reset here too for clean per-poll diagnostics)
        s_irq_edge_count = 0;

        // ── Poll for a card ───────────────────────────────────────────────────
        bool detected = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A,
                                                uid, &uid_len,
                                                RFID_POLL_MS);

        // ── Watchdog check ────────────────────────────────────────────────────
        // Only active when the IRQ interrupt is in use (s_irq_in_use == true).
        // With the interrupt detached the flag never sets, so the watchdog
        // would fire on every poll — defeating its purpose.
        if (s_irq_in_use && !s_pn532_irq_flag) {
            // No IRQ edge during this poll — PN532 may be unresponsive.
            irq_miss_count++;
            Serial.printf("[rfid] WARNING: no IRQ during poll (miss %d/%d)\r\n",
                          irq_miss_count, PN532_IRQ_MISS_LIMIT);
            Serial.flush();

            if (irq_miss_count >= PN532_IRQ_MISS_LIMIT) {
                Serial.println("[rfid] WATCHDOG: PN532 not responding — hard reset + re-init");
                Serial.flush();
                led_set(LED_YELLOW, LED_FAST);
                bool ok = _pn532_reinit();
                led_set(LED_YELLOW, LED_OFF);
                irq_miss_count = 0;
                if (!ok) {
                    Serial.println("[rfid] re-init failed — retrying after delay");
                    vTaskDelay(pdMS_TO_TICKS(5000));
                } else {
                    Serial.println("[rfid] re-init OK — resuming poll loop");
                }
                // Skip the rest of this loop iteration; retry the poll fresh.
                continue;
            }
        } else {
            // IRQ fired — PN532 is alive.
            irq_miss_count = 0;
        }

        // ── Card present / absent logic (unchanged from original) ─────────────
        if (detected) {
            last_detect_ms = millis();
            last_seen_ms = now;  // refresh grace timer

            bool new_uid = (!card_present || uid_len == 0 ||
                            memcmp(uid, last_uid, uid_len) != 0);

            if (new_uid) {
                card_present = true;
                memcpy(last_uid, uid, uid_len);

                led_set(LED_YELLOW, LED_ON);

                if (!_read_card_bytes(card_buf)) {
                    Serial.println("[rfid] card read FAILED");
                    led_set(LED_GREEN,  LED_FAST);
                    led_set(LED_YELLOW, LED_OFF);
                    continue;
                }

                // ── Insert banner ─────────────────────────────────────────
                Serial.println();
                Serial.println("========================================");
                Serial.print  ("  CARD INSERTED  uid:");
                for (int i = 0; i < uid_len; i++) {
                    Serial.print(' '); _hex_byte(uid[i]);
                }
                Serial.println();
                Serial.println("========================================");

                // ── Full card hex dump (120 bytes, labeled by region) ─────
                for (int i = 0; i < CARD_TOTAL_LEN; i += 16) {
                    Serial.print("  ");
                    if (i < 0x10)  Serial.print('0');
                    if (i < 0x100) Serial.print('0');
                    Serial.print(i, HEX);

                    // Labeled by Layout v2 member-card field boundaries. On a
                    // config card these labels don't mean much past offset 5
                    // (its own fields overlap this region differently) — the
                    // dump is a raw byte view either way, useful for both.
                    const char* tag;
                    if      (i >= MEM_PAYLOAD_LEN)     tag = " x:";  // unused/zero (or config sig)
                    else if (i >= MEM_OFF_LAST_NAME)   tag = " L:";
                    else if (i >= MEM_OFF_FIRST_NAME)  tag = " F:";
                    else if (i >= MEM_OFF_PERMS)       tag = " P:";
                    else                                tag = " I:";  // member_id (ASCII digits)
                    Serial.print(tag);

                    for (int j = 0; j < 16 && (i + j) < CARD_TOTAL_LEN; j++) {
                        Serial.print(' ');
                        _hex_byte(card_buf[i + j]);
                    }
                    Serial.println();
                }
                Serial.println();

                const uint8_t* payload = card_buf;
                uint8_t card_type      = payload[CARD_OFF_TYPE];

                // ── Config card ───────────────────────────────────────────
                if (card_type == CARD_TYPE_CONFIG) {
                    // Admin (config) cards are NOT signed (Oct 2026 decision):
                    // anyone with a card writer can reconfigure a machine.
                    // Layout: 0x02, version, machine, blast units, reserved,
                    // then (version 2) a 16-byte machine name that the
                    // client only displays -- it is not stored or used.
                    uint8_t version       = payload[CARD_OFF_VERSION];
                    uint8_t new_machine   = payload[CFG_OFF_MACHINE];
                    uint8_t blast_raw     = payload[CFG_OFF_BLAST_RAW] & 0x0F;
                    uint32_t new_blast_ms = (uint32_t)blast_raw * CFG_BLAST_UNIT_MS;

                    Serial.println("========================================");
                    Serial.println("  CONFIG CARD");
                    Serial.println("========================================");
                    Serial.print  ("  Version       : "); Serial.println(version);
                    Serial.print  ("  Machine #     : "); Serial.println(new_machine);
                    if (version >= CFG_VERSION_NAMED) {
                        char cfg_name[CFG_NAME_LEN + 1];
                        memcpy(cfg_name, payload + CFG_OFF_NAME, CFG_NAME_LEN);
                        cfg_name[CFG_NAME_LEN] = '\0';
                        for (int k = 0; k < CFG_NAME_LEN; k++) {
                            if (cfg_name[k] != '\0' &&
                                (cfg_name[k] < 0x20 || cfg_name[k] > 0x7E)) cfg_name[k] = '?';
                        }
                        Serial.print  ("  Machine name  : "); Serial.println(cfg_name);
                    }
                    Serial.print  ("  Blast delay   : ");
                    Serial.print(blast_raw * 10); Serial.println(" s");
                    Serial.println();

                    if (version < CFG_VERSION_MIN || version > CFG_VERSION_NAMED) {
                        Serial.print("[rfid] config card version ");
                        Serial.print(version);
                        Serial.print(" not supported (accepted ");
                        Serial.print(CFG_VERSION_MIN);
                        Serial.print("..");
                        Serial.print(CFG_VERSION_NAMED);
                        Serial.println(") -- rejected");
                        led_set(LED_GREEN, LED_FAST);
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        led_set(LED_GREEN, LED_OFF);
                        continue;
                    }

                    bool ok = node_config_set(new_machine, new_blast_ms);
                    led_set(LED_YELLOW, LED_OFF);
                    if (ok) {
                        Serial.println("  Result        : ACCEPTED — config saved");
                        Serial.println();
                        led_set(LED_GREEN,  LED_FAST);
                        led_set(LED_YELLOW, LED_FAST);
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        led_set(LED_GREEN,  LED_OFF);
                        led_set(LED_YELLOW, LED_OFF);
                    } else {
                        Serial.println("  Result        : REJECTED — values out of range");
                        Serial.println();
                        led_set(LED_GREEN, LED_FAST);
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        led_set(LED_GREEN, LED_OFF);
                    }
                    continue;
                }

                // ── Member card (Layout v2 — no type tag, no signature) ────
                // Not a config card, and there's no member type byte to check
                // anymore, so we take this branch by exclusion and sanity-
                // check the MemberID field instead: Layout v2 always writes
                // it as 4 ASCII digits, so 4 non-digit bytes here means this
                // isn't a card we recognize at all (blank/foreign/corrupt).
                uint32_t member_id;
                if (!_parse_member_id_ascii(payload, &member_id)) {
                    Serial.print("[rfid] unrecognized card (not a config card, "
                                 "MemberID field is not ASCII digits: 0x");
                    for (int i = 0; i < MEM_MEMBER_ID_LEN; i++) _hex_byte(payload[MEM_OFF_MEMBER_ID + i]);
                    Serial.println(") -- rejected");
                    led_set(LED_GREEN,  LED_FAST);
                    led_set(LED_YELLOW, LED_OFF);
                    continue;
                }

                // NOTE: no signature check here — Layout v2 member cards are
                // unsigned. member_id and the permission bitmap below are
                // trusted as written on the card. See SECURITY NOTE in
                // config.h.
                bool auth = _has_permission(payload, g_machine_number);

                char first_name[CARD_NAME_LEN + 1];
                char last_name[CARD_NAME_LEN + 1];
                _extract_name(payload, MEM_OFF_FIRST_NAME, first_name);
                _extract_name(payload, MEM_OFF_LAST_NAME,  last_name);

                Serial.print  ("  Member ID : "); Serial.println(member_id);
                Serial.print  ("  Name      : ");
                Serial.print(first_name); Serial.print(' '); Serial.println(last_name);
                Serial.print  ("  Machine   : "); Serial.println(g_machine_number);
                Serial.print  ("  Access    : "); Serial.println(auth ? "GRANTED" : "DENIED");
                Serial.println();

                if (xSemaphoreTake(g_session_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                    g_session.active        = true;
                    g_session.member_id     = member_id;
                    g_session.authorized    = auth;
                    g_session.start_time_ms = millis();
                    g_session.duration_ms   = 0;
                    g_session.avg_current   = 0.0f;
                    g_session.starts        = 0;
                    g_session.stops         = 0;
                    xSemaphoreGive(g_session_mutex);
                }

                if (auth) {
                    led_set(LED_GREEN, LED_ON);
                    led_set(LED_RED,   LED_OFF);
                    // LED_YELLOW deliberately left solid (LED_ON), not
                    // cleared here. Aug 2026 design: this local grant is
                    // fail-open (keeps the shop running if the network/
                    // server is down) and now gets confirmed -- or vetoed --
                    // by the server asynchronously. wifi_task.cpp's
                    // _attempt_server_confirmation() owns LED_YELLOW from
                    // this point on: it resolves to OFF (confirmed) or
                    // FAST (no answer yet, still retrying every 30s). See
                    // wifi_task.cpp for the full state machine. Card removal
                    // below still unconditionally clears every LED including
                    // this one, regardless of where that confirmation stood.
                } else {
                    led_set(LED_GREEN,  LED_FAST);
                    led_set(LED_RED,    LED_OFF);
                    led_set(LED_YELLOW, LED_OFF);   // nothing running, nothing to confirm
                }
            }

        } else {
            // Not detected this poll. Only declare removal after grace period.
            if (card_present && (now - last_seen_ms) >= REMOVE_GRACE_MS) {
                card_present = false;
                memset(last_uid, 0, sizeof(last_uid));

                Serial.println();
                Serial.println("========================================");
                Serial.println("  CARD REMOVED");
                Serial.println("========================================");
                Serial.println();

                led_set(LED_GREEN,  LED_OFF);
                led_set(LED_RED,    LED_OFF);
                led_set(LED_YELLOW, LED_OFF);

                if (xSemaphoreTake(g_session_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                    g_session.active = false;
                    xSemaphoreGive(g_session_mutex);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}