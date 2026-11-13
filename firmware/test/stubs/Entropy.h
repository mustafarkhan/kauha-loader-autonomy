#pragma once
#include <stdint.h>
struct EntropyClass { void Initialize() {} uint8_t randomByte() { return 0; } };
static EntropyClass Entropy;
