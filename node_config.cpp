/*
 * node_config.cpp
 * Woodshop Access Control, ESP32 Arduino IDE
 *
 * Uses the Arduino Preferences library, which is a thin wrapper over the
 * ESP32 NVS partition. NVS handles wear leveling internally; writes go to
 * a log-structured region and are garbage-collected automatically.
 *
 * Namespace: "woodshop"
 * Keys:
 *   "machine"   — uint8_t, 1..128
 *   "blastms"   — uint32_t, blast gate drop-out delay in milliseconds
 *
 * On first boot (no keys present) the compile-time defaults from config.h
 * are written to NVS so subsequent reads are consistent.
 */

#include "node_config.h"
#include "config.h"
#include <Preferences.h>

// ── Runtime globals (tasks read these, not the #defines) ─────────────────────
volatile uint8_t  g_machine_number = MACHINE_NUMBER;   // seeded from #define
volatile uint32_t g_blast_delay_ms = BLAST_DELAY_MS;

static Preferences _prefs;
static const char* NS        = "woodshop";
static const char* K_MACHINE = "machine";
static const char* K_BLASTMS = "blastms";

void node_config_init() {
    // Open R/W so we can create defaults on first boot
    if (!_prefs.begin(NS, false)) {
        Serial.println("[cfg] NVS open FAILED — using compile-time defaults");
        return;
    }

    bool have_machine = _prefs.isKey(K_MACHINE);
    bool have_blast   = _prefs.isKey(K_BLASTMS);

    if (have_machine) {
        g_machine_number = _prefs.getUChar(K_MACHINE, MACHINE_NUMBER);
    } else {
        _prefs.putUChar(K_MACHINE, (uint8_t)MACHINE_NUMBER);
        g_machine_number = MACHINE_NUMBER;
        Serial.println("[cfg] first boot — seeded machine=default");
    }

    if (have_blast) {
        g_blast_delay_ms = _prefs.getUInt(K_BLASTMS, BLAST_DELAY_MS);
    } else {
        _prefs.putUInt(K_BLASTMS, (uint32_t)BLAST_DELAY_MS);
        g_blast_delay_ms = BLAST_DELAY_MS;
        Serial.println("[cfg] first boot — seeded blast_delay=default");
    }

    _prefs.end();

    Serial.print("[cfg] loaded: machine=");
    Serial.print(g_machine_number);
    Serial.print(" blast_delay_ms=");
    Serial.println(g_blast_delay_ms);
}

bool node_config_set(uint8_t machine_number, uint32_t blast_delay_ms) {
    // Sanity check — config card sig proves the server minted it, not that
    // the values are sensible. Cheap guard against a corrupted-but-signed
    // card or a forthcoming firmware/server version mismatch.
    //
    // Ranges match the server's config-card form (app.py /admin/config-card):
    //   machine_number : 0..255  (permission bitmap only uses 1..128, but
    //                             machine_number==0 is a legal "unassigned"
    //                             sentinel the server UI allows)
    //   blast_delay_ms : 0..150000 ms  (4-bit raw × 10s max)
    if (blast_delay_ms > 150000UL) {
        Serial.print("[cfg] rejected: blast_delay_ms too large: ");
        Serial.println(blast_delay_ms);
        return false;
    }

    // Read-before-write: NVS has wear leveling but flash isn't free. If the
    // admin card is presented repeatedly with the same values (e.g., someone
    // testing), we shouldn't burn flash cycles.
    if (machine_number == g_machine_number &&
        blast_delay_ms == g_blast_delay_ms) {
        Serial.println("[cfg] admin card values match current — no write");
        return true;  // still "accepted"
    }

    if (!_prefs.begin(NS, false)) {
        Serial.println("[cfg] NVS open FAILED on update");
        return false;
    }

    size_t w1 = _prefs.putUChar(K_MACHINE, machine_number);
    size_t w2 = _prefs.putUInt(K_BLASTMS, blast_delay_ms);
    _prefs.end();

    if (w1 == 0 || w2 == 0) {
        Serial.println("[cfg] NVS write FAILED");
        return false;
    }

    // Update live globals — tasks pick these up on their next tick.
    g_machine_number = machine_number;
    g_blast_delay_ms = blast_delay_ms;

    Serial.print("[cfg] updated: machine=");
    Serial.print(machine_number);
    Serial.print(" blast_delay_ms=");
    Serial.println(blast_delay_ms);
    return true;
}
