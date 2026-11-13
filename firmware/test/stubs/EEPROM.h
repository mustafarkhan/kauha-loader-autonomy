#pragma once
#include <stdint.h>
struct EEPROMClass { uint8_t read(int) { return 0xFF; } void write(int, uint8_t) {} };
static EEPROMClass EEPROM;
