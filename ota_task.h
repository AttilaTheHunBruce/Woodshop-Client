/*
 * ota_task.h
 * Over-the-air firmware update — HTTP fetch from the Pi's Flask server,
 * flashed into the inactive OTA partition, applied on next idle reboot.
 */
#pragma once
#include <Arduino.h>

void task_ota(void* arg);   // FreeRTOS task — pin to Core 0

// Called by wifi_task whenever a binary-protocol response comes back with
// update_available=1 (response byte 7). Wakes task_ota to check for a new
// version right away instead of waiting for its next periodic poll — lets a
// machine that's actively being used pick up an update within moments of it
// being flagged. Safe to call from any task; cheap; does not block.
void ota_notify_update_flagged();
