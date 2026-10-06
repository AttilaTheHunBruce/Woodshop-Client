/*
 * relay_ctrl.h
 * Single-owner relay driver task.
 *
 * Consolidates all writes to PIN_RELAY_MAIN and PIN_RELAY_BLAST into one
 * task so there's no contention between rfid, override, and session tasks
 * over who's driving what pin. All other tasks just update shared state
 * (g_session.active/authorized/override_flags); this task decides the
 * actual relay outputs.
 */
#pragma once
#include <Arduino.h>

void task_relay(void* arg);   // FreeRTOS task — pin to Core 0
