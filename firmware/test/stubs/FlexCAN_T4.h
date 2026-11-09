#pragma once
#include <stdint.h>
struct CAN_message_t { uint32_t id = 0; struct { bool extended = false; } flags; uint8_t len = 0; uint8_t buf[8] = {0}; };
enum CAN_DEV_TABLE { CAN1, CAN2, CAN3 };
enum RXQ_TABLE { RX_SIZE_16 = 16, RX_SIZE_64 = 64, RX_SIZE_256 = 256 };
enum TXQ_TABLE { TX_SIZE_16 = 16 };
template <CAN_DEV_TABLE B, RXQ_TABLE R, TXQ_TABLE T> struct FlexCAN_T4 {
  void begin() {} void setBaudRate(uint32_t) {} void enableFIFO() {} void enableFIFOInterrupt() {}
  void onReceive(void (*)(const CAN_message_t &)) {} void events() {} int write(const CAN_message_t &) { return 1; }
};
