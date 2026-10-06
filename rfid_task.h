/*
 * rfid_task.h
 * PN532 RFID reader task — SPI, NTAG215, Ed25519 verify
 */
#pragma once
#include <Arduino.h>
#include "config.h"

void task_rfid(void* arg);   // FreeRTOS task — pin to Core 0
