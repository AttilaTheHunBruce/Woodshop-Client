/*
 * current_sense.h
 * I2S/DMA ADC current sensor — runs on Core 1
 */
#pragma once
#include <Arduino.h>

void  current_sense_init();         // call once from setup()
float current_get_rms_amps();       // thread-safe, callable from any task
float current_get_bias_volts();     // DC bias tracking (diagnostic)
bool  current_machine_is_on();      // true if above CURRENT_THRESHOLD_AMPS
