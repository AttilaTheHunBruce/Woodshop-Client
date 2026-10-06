/*
 * override_task.h
 * Debounces the two override switches, updates g_session.override_flags,
 * and drives the WHITE LED as an override indicator.
 */
#pragma once
#include <Arduino.h>

void task_override(void* arg);   // FreeRTOS task — pin to Core 0
