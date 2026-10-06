/*
 * led_ctrl.h
 * Timer-based LED blink controller
 */
#pragma once
#include <Arduino.h>

// Blink speeds (ticks, 1 tick = LED_TICK_MS = 100ms)
#define LED_OFF     0
#define LED_ON      1
#define LED_FAST    2    // 400ms period
#define LED_MEDIUM  4    // 800ms period
#define LED_SLOW    10   // 2000ms period

// LED indices
#define LED_RED     0
#define LED_GREEN   1
#define LED_YELLOW  2
#define LED_WHITE   3

void led_init();
void led_set(int led, int mode);   // mode = LED_OFF / LED_ON / LED_FAST / etc.
void task_led(void* arg);          // FreeRTOS task — pin to Core 0
