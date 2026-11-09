/* Minimal stand-ins so the Teensy sketches can be syntax-checked on the host. Not a simulator. */
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
inline uint32_t micros() { return 0; }
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline int digitalReadFast(uint8_t) { return 0; }
inline void analogWrite(uint8_t, int) {}
inline int analogRead(uint8_t) { return 0; }
inline void analogWriteFrequency(uint8_t, float) {}
inline void analogWriteResolution(int) {}
inline void analogReadResolution(int) {}
inline int digitalPinToInterrupt(uint8_t p) { return p; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void noInterrupts() {}
inline void interrupts() {}
