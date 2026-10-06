/*
 * node_config.h
 * NVS-backed runtime configuration: machine number and blast gate delay.
 *
 * At boot, node_config_init() loads these from NVS. If no values are stored
 * (first boot on a fresh flash), the compile-time defaults from config.h are
 * used and also written to NVS.
 *
 * Admin cards (member_id = 0xFFFFFFFF, signature valid) can update these
 * values at runtime via node_config_set(). New values take effect immediately
 * — task_relay and task_rfid read the globals on every tick, so no reboot
 * is needed.
 *
 * All accessors are safe to call from any task (NVS driver is thread-safe,
 * and the getters are single-word reads from volatile globals).
 */
#pragma once
#include <Arduino.h>

// Runtime values — read these instead of the config.h #defines.
// After node_config_init() they reflect NVS-stored values (or defaults).
extern volatile uint8_t  g_machine_number;
extern volatile uint32_t g_blast_delay_ms;

void node_config_init();   // call once from setup(), after Serial

// Update both values and persist to NVS.
// Returns true if values were written (or matched existing — both count as
// "accepted"). Caller should have verified the admin card signature first.
bool node_config_set(uint8_t machine_number, uint32_t blast_delay_ms);
