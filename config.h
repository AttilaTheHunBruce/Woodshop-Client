/*
 * config.h — Pin assignments and system constants
 * Woodshop Access Control, ESP32 Arduino IDE
 */
#pragma once
#include <Arduino.h>

// ── OTA / firmware update (Aug 2026) ─────────────────────────────────────────
// Bump FW_VERSION every time you build and ship new firmware. task_ota
// compares this against GET /firmware/version on the Pi and only downloads +
// flashes when they differ. That self-check is what makes it safe for the
// server's UPDATE_AVAILABLE flag (master_server.py) to sit "True" for the
// whole rollout window while techs work through the shop machine-by-machine
// -- once a given node is running FW_VERSION == the server's version, it
// stops re-flashing itself even though the server keeps broadcasting
// update_available=1 in every response (byte 7) until someone remembers to
// flip the flag back off.
#define FW_VERSION              "1.0.5"

// ── Build mode ────────────────────────────────────────────────────────────────
// Uncomment ONLY for bench debugging with a laptop on Serial. It disables the
// brown-out detector and stretches the task watchdog to ~49 days (see
// Client.ino setup()) so a hang or a sagging supply is visible as a freeze
// you can poke at instead of a silent reset.
//
// LEAVE THIS COMMENTED OUT for any unit installed in the field. Both
// protections exist specifically to turn "reboots with no discernible
// pattern" into a clean, logged event: a real brown-out becomes a
// ESP_RST_BROWNOUT entry in the diag ring buffer instead of undefined
// behavior (including possible flash/NVS corruption if the rail sags mid-
// write), and a hung task becomes a ESP_RST_TASK_WDT entry instead of a
// machine that silently stops responding to cards with nobody watching.
// See diag.h / diag.cpp.
// #define DEV_BUILD

// ── Diagnostics (diag.h / diag.cpp) ──────────────────────────────────────────
#define DIAG_HTTP_PATH   "/diag/report"   // POSTed to the Pi, same host:port as OTA

// ── PN532 SPI ─────────────────────────────────────────────────────────────────
// NOTE: MISO/MOSI values below match the PHYSICAL WIRING on this board, which
// is OPPOSITE of the ESP32 VSPI defaults. The wiring was proven correct with
// the basic Elechouse-library test sketch (firmware 1.6, NTAG215 UIDs read
// cleanly). Do not "fix" these to match the VSPI defaults without re-checking
// the hardware — you'll break the RFID reader.
#define PIN_PN532_SS    21
#define PIN_SPI_MOSI    19      // ESP32 MOSI out -> PN532 MOSI in
#define PIN_SPI_MISO    23      // PN532 MISO out -> ESP32 MISO in
#define PIN_SPI_SCK     18

// IRQ: active-low open-drain output FROM the PN532.  Now physically wired to
// GPIO4 with a 10k pull-up to 3V3 on the line.
//
// When does IRQ fire?  In SPI mode the PN532 is a slave and cannot initiate
// SPI transactions on its own.  IRQ is the mechanism it uses to say "I have
// data ready for you to read."  It asserts LOW (open-drain) in exactly two
// situations per host command:
//   1. ACK edge  — ~1-2 ms after the ESP32 sends any command, the PN532
//                  asserts IRQ to say the ACK frame is ready over SPI.
//   2. Response edge — after the PN532 finishes processing the command it
//                  asserts IRQ again to say the full response is ready.
//                  For a card-scan command this second edge fires when a card
//                  is found, OR when the PN532 exhausts its retry count and
//                  sends an error response ("no card").
//
// So a single readPassiveTargetID() call produces TWO falling IRQ edges when
// it completes normally.  If the ISR flag is never set at all, the PN532 did
// not respond to the command — a reliable sign it is wedged.
#define PIN_PN532_IRQ   4

// RSTO (GPIO13) → PN532 RSTPD_N.  The "RSTO" or "RST" pad on the Elechouse
// breakout connects through a series resistor to the PN532's RSTPD_N input
// (active-low hardware reset).  Drive LOW to hold the chip in reset; release
// HIGH to let it boot.  GPIO13 is not a strapping pin and has no special
// boot-time role, so it is safe to drive as an output from the very start of
// setup().  The pin is left as INPUT (floating) until task_rfid is ready to
// issue a deliberate reset, so it does not interfere with the PN532's own
// power-on reset circuit.
#define PIN_PN532_RSTO  13

// Conservative SPI clock for PN532 init. PN532 tops out around 5 MHz; starting
// at 1 MHz gives the most reliable first-contact and works even with long
// breadboard jumpers.
#define PN532_SPI_CLOCK_HZ  1000000UL

// ── LEDs (direct GPIO, active HIGH) ──────────────────────────────────────────
#define PIN_LED_RED     12
#define PIN_LED_GREEN   14
#define PIN_LED_YELLOW  27
#define PIN_LED_WHITE   26

// ── Relays (active HIGH) ──────────────────────────────────────────────────────
#define PIN_RELAY_MAIN  32
#define PIN_RELAY_BLAST 33
#define PIN_RELAY_AUX   25

// ── Current sensor ────────────────────────────────────────────────────────────
#define PIN_CURRENT_ADC 36      // GPIO36 = ADC1_CH0, input-only, no pullup

// ── Override switches (active LOW, internal pullup) ───────────────────────────
#define PIN_SW_MAIN_OVERRIDE  17
#define PIN_SW_BLAST_OVERRIDE 16

// Override state bits — packed into session_t.override_flags and also sent as
// byte 13 of the WiFi protocol message so the server can log override usage.
#define OVR_FLAG_MAIN   0x01
#define OVR_FLAG_BLAST  0x02

// Override switch debounce — samples are taken every 20ms; a switch must
// hold its new state for this many consecutive samples to count.
#define OVERRIDE_DEBOUNCE_COUNT  3

// ── WiFi / Server ─────────────────────────────────────────────────────────────
// !! BEFORE BUILDING: replace both values below with the shop network's real
// !! WiFi name and password. The public repository deliberately contains
// !! placeholders only. The real values are kept in the private system
// !! document (see "Public repository and credentials"). Never commit them.
#define WIFI_SSID           "YOUR_SSID"
#define WIFI_PASSWORD       "YOUR_PASSWORD"
#define SERVER_PORT         35487
#define SERVER_HOST_BYTE    5       // server = 192.168.0.5 (derived from client subnet)

// ── Current sense hardware ────────────────────────────────────────────────────
#define CURRENT_SAMPLE_RATE     20000   // Hz — I2S ADC sample rate
#define CURRENT_DMA_BUF_LEN     512     // samples per DMA buffer
#define CURRENT_DMA_BUF_COUNT   4       // number of DMA buffers
// CT: 1000:1 ratio with 10-turn primary → effective turns ratio = 100:1
// Burden resistor: 220Ω on CT secondary
// At 14Vrms / 50Ω load: primary=0.280A, secondary=2.80mA rms,
// burden voltage=0.616Vrms = ±0.871V peak, ADC range 0.779–2.521V.
#define CURRENT_TURNS_RATIO     100.0f  // 1000:1 CT, 10-turn primary
#define CURRENT_BURDEN_OHMS     220.0f  // burden resistor (ohms)
#define CURRENT_VREF            3.3f
#define CURRENT_ADC_MAX         4095.0f
// Amps per ADC count (after bias removal):
// = (VREF / ADC_MAX / BURDEN_OHMS) * TURNS_RATIO
// = (3.3 / 4095 / 220) * 100 = 0.000366 A/count
#define CURRENT_AMPS_PER_COUNT  ((CURRENT_VREF / CURRENT_ADC_MAX / CURRENT_BURDEN_OHMS) \
                                  * CURRENT_TURNS_RATIO)
// Machine-on threshold. Idle ADC noise on GPIO36 reads ~77mA RMS even with
// CT disconnected — this is quantization noise and 60Hz pickup on the input.
// 200mA sits comfortably above the noise floor and well below any real load.
// Adjust downward if you need to detect very lightly loaded machines.
#define CURRENT_THRESHOLD_AMPS  0.20f   // machine considered ON above this

// ── RFID / NTAG215 ───────────────────────────────────────────────────────────
// As of Aug 2026, Lee Robertshaw writes member cards (WriteNTAG215.py,
// "Layout v2") and the ESP32 only ever reads them. Layout v2 has NO type/
// version header and NO signature — see MEMBER CARD LAYOUT below. Config
// cards are written by rfid_admin_card.py (server) and are unsigned too
// (Oct 2026); see the config card layout below. Because the two
// formats no longer share a type tag, discrimination works by exclusion:
// the reader checks for the config card's type byte first (CARD_TYPE_CONFIG
// at offset 0); anything else is treated as a Layout v2 member card and
// sanity-checked by requiring MEM_OFF_MEMBER_ID..+3 to be ASCII digits
// (CARD_TYPE_CONFIG's value, 0x02, can never be mistaken for an ASCII
// digit, so this can't collide with a config card). See rfid_task.cpp.
//
// MEMBER CARD LAYOUT v2 (52 bytes, UNSIGNED — Lee's WriteNTAG215.py):
//    bytes 0..3   : member_id, 4 ASCII digit characters, e.g. "0042"
//                   (NOT a binary uint32 — parse as decimal text)
//    bytes 4..19  : permission bitmap (128 bits = 16 bytes)
//    bytes 20..35 : first_name (16 bytes, ASCII, null-padded)
//    bytes 36..51 : last_name  (16 bytes, ASCII, null-padded)
//   Total: 52 bytes = 13 NTAG215 pages starting at page 4 (pages 4..16).
//   Everything from byte 52 onward (pages 17..129) is zero-filled by the
//   writer and unused.
//
// SECURITY NOTE: this layout carries no cryptographic signature, so the
// ESP32 can no longer verify a member card was issued by the server —
// it trusts whatever member_id/permissions are written on the card. This
// was an explicit decision (Aug 2026) to match Lee's writer as-is rather
// than block on adding signing to WriteNTAG215.py. Config cards are now
// unsigned too (Oct 2026), so the Ed25519 library and public key are gone.
//
// Config (admin) card payload layout -- UNSIGNED since Oct 2026 (written by
// rfid_admin_card.py / the server's Admin Card page):
//    byte 0     : CARD_TYPE_CONFIG (0x02)
//    byte 1     : version: 1 = machine + blast only (older cards, whatever
//                 follows byte 4 is ignored); 2 = also carries a machine name
//    byte 2     : machine number (1..128 from the writer; client allows 0..255)
//    byte 3     : blast gate delay (0..15, multiply by 10 for seconds)
//    byte 4     : reserved (0x00)
//    bytes 5..20: machine name, 16 bytes ASCII, null-padded (version 2 only;
//                 shown on the serial monitor, not stored or used)
//   Total: 21 bytes (version 2) = 6 NTAG215 pages from page 4.
//   There is no signature: anyone with a card writer can reconfigure a machine.
#define CARD_TYPE_CONFIG    0x02

// Header (config card only — member cards have no header anymore)
#define CARD_OFF_TYPE       0
#define CARD_OFF_VERSION    1

// Member card field offsets (within the 52-byte unsigned payload, Layout v2)
#define MEM_OFF_MEMBER_ID    0
#define MEM_OFF_PERMS        4
#define MEM_OFF_FIRST_NAME  20
#define MEM_OFF_LAST_NAME   36
#define MEM_PAYLOAD_LEN     52
#define MEM_MEMBER_ID_LEN    4   // ASCII digit characters, not bytes-as-value

// Config card field offsets (unsigned; see the config card layout above)
#define CFG_OFF_MACHINE     2
#define CFG_OFF_BLAST_RAW   3
#define CFG_OFF_RESERVED    4
#define CFG_OFF_NAME        5
#define CFG_NAME_LEN        16
#define CFG_VERSION_MIN     1      // oldest accepted (no name field)
#define CFG_VERSION_NAMED   2      // newest accepted (adds machine name)
#define CFG_BLAST_UNIT_MS   10000UL   // one blast-delay unit = 10 seconds

#define RFID_POLL_MS        250
#define RFID_PAGE_START     4
#define CARD_NAME_LEN       16
// Total bytes we read from the card. Padded to the next 4-byte NTAG page
// boundary so the page-read loop always reads complete pages. Sized for
// the LARGER of the two formats: config card (72 bytes) vs. member card
// (52 bytes) — kept at the old 120-byte size for now so a future config
// card format change has headroom; the extra pages are all zero on a
// Layout v2 member card and simply cost a bit of extra read time.
#define CARD_TOTAL_LEN      120

// ── PN532 reset / IRQ watchdog ────────────────────────────────────────────────
// After this many consecutive poll cycles with no IRQ activity at all, the
// task concludes the PN532 is wedged and issues another hard reset + re-init.
// One cycle ≈ RFID_POLL_MS (250 ms) + small overhead.  5 misses = ~1.3 s.
#define PN532_IRQ_MISS_LIMIT   5

// SAMConfig retry parameters.  SAMConfig enables the RF frontend (~150 mA);
// on marginal supplies it can fail if attempted too soon after getFirmwareVersion.
// We wait (attempt# × BASE_DELAY_MS) before each try: 1 s, 2 s, 3 s.
// Increase BASE_DELAY_MS or TRIES if SAMConfig still fails on your supply.
#define PN532_SAMCONFIG_TRIES         3
#define PN532_SAMCONFIG_BASE_DELAY_MS 1000

// How many full hard-reset + init cycles to attempt in the startup failure
// loop before giving up and calling ESP.restart().  Each cycle takes up to
// ~10 s (5 s pause + hard reset + SAMConfig attempts), so 3 retries = ~30 s
// maximum before a forced reboot.
#define PN532_INIT_RETRIES            3

// ── Machine config (compile-time defaults, used on first boot only) ─────────
// At runtime, use g_machine_number and g_blast_delay_ms from node_config.h —
// those reflect NVS-stored values which can be updated by a config card.
#define MACHINE_NUMBER      1       // 1-128, must match card permission bit
#define BLAST_DELAY_MS      30000   // ms to keep blast gate on after machine off

// ── Task timing ───────────────────────────────────────────────────────────────
#define LED_TICK_MS         100     // LED blink resolution
#define WIFI_HEARTBEAT_MS   10000   // WiFi keepalive interval

// ── Binary protocol message sizes ────────────────────────────────────────────
#define MSG_SEND_LEN        16
#define MSG_RECV_LEN        8

// Flask admin/firmware server runs on the same Pi as the binary-protocol
// server (SERVER_HOST_BYTE), just on port 80 instead of SERVER_PORT.
#define FIRMWARE_HTTP_PORT      80
#define OTA_HTTP_TIMEOUT_MS     8000UL
// Periodic self-check, independent of card activity -- this is what lets an
// idle machine (no one has badged in) still pick up an update instead of
// waiting indefinitely for a card event to carry the flag. A card event
// (see wifi_task.cpp / ota_notify_update_flagged()) can still trigger an
// earlier check on a machine that's actively being used.
#define OTA_CHECK_INTERVAL_MS   (10UL * 60UL * 1000UL)   // 10 min
// Once a downloaded image is verified, task_ota waits for the shop floor to
// go idle (no card session open) before rebooting into it, so a reboot never
// drops relay power on a machine mid-cut. This is how often it re-checks
// g_session.active while waiting.
#define OTA_IDLE_POLL_MS        2000UL
// How long red+white blink together (LED_FAST) after a failed download/flash
// attempt (an update was available and the node committed to fetching it --
// not a routine manifest-fetch hiccup or "nothing staged yet"). Timeboxed
// rather than persistent-until-cleared: red is already shared by
// override_task (main override asserted) and rfid_task (PN532 hardware
// fault), both of which can silently overwrite it with no coordination. A
// short, bounded alert keeps the collision window small; the POSTed
// /diag/report entry (see _report_ota_failure() in ota_task.cpp) is the
// actual durable record, not the LED.
#define OTA_FAIL_LED_MS         30000UL

// ── Session counting thresholds ──────────────────────────────────────────────
// Machine start/stop events are counted by watching current_machine_is_on()
// transitions. That helper uses its own hysteretic thresholds internally
// (CURRENT_THRESHOLD_AMPS with 80% drop-out), so a simple edge count is
// sufficient here — no debounce needed on top.
// avg_current is a running mean of current_get_rms_amps() sampled at this
// interval while g_session.active is true.
#define SESSION_SAMPLE_MS   50    // 50ms — fine enough for 100ms inrush window

// ── Shared session state ──────────────────────────────────────────────────────
typedef struct {
    bool     active;            // card is present
    uint32_t member_id;
    uint32_t start_time_ms;
    uint32_t duration_ms;       // machine run time (accumulated while on)
    float    avg_current;       // amps, running mean while machine is on
    uint8_t  starts;
    uint8_t  stops;
    bool     authorized;
    uint8_t  override_flags;    // OVR_FLAG_MAIN | OVR_FLAG_BLAST
} session_t;

// Declared in woodshop_esp32.ino, used by all tasks
extern volatile session_t g_session;
extern SemaphoreHandle_t  g_session_mutex;

// Startup synchronization:
//   g_system_settled — given by setup() after all non-WiFi tasks are created.
//                      task_rfid waits on this before touching the PN532, so
//                      the hard reset and init happen once the system is stable.
//   g_rfid_ready     — given by task_rfid once PN532 init is done (success or
//                      fail); setup() waits on this before starting task_wifi,
//                      so the WiFi radio does not come up while the PN532 SPI
//                      bus is being initialised (radio noise causes SAMConfig
//                      failures on marginal boards).
extern SemaphoreHandle_t  g_system_settled;
extern SemaphoreHandle_t  g_rfid_ready;
