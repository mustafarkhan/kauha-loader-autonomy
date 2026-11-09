#pragma once
#include <stdint.h>
struct TwoWire { void begin() {} void setClock(uint32_t) {} void beginTransmission(uint8_t) {} void write(uint8_t) {}
  uint8_t endTransmission(bool = true) { return 0; } uint8_t requestFrom(uint8_t, uint8_t) { return 0; } int read() { return 0; } };
extern TwoWire Wire, Wire1;
inline TwoWire Wire, Wire1;
