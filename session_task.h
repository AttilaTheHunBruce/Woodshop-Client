/*
 * session_task.h
 * Accumulates machine run statistics during an active session:
 *   - starts/stops (transitions through current-on threshold)
 *   - duration_ms  (wall-clock time machine was drawing current)
 *   - avg_current  (running mean RMS amps while machine was on)
 */
#pragma once
#include <Arduino.h>

void task_session(void* arg);   // FreeRTOS task — pin to Core 0
