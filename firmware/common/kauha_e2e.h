/* End-to-end protection for KAUHA machine-bus frames.
 * Port of kauha/can/e2e.py: CRC-8 SAE J1850 over (data ID, bytes 0..6), 4-bit alive
 * counter in byte 6. tests/firmware checks it against the Python implementation. */
#ifndef KAUHA_E2E_H
#define KAUHA_E2E_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum e2e_status { E2E_OK = 0, E2E_OK_SOME_LOST = 1, E2E_REPEATED = 2, E2E_WRONG_SEQUENCE = 3, E2E_CRC_ERROR = 4 };

typedef struct {
    uint8_t data_id;
    double timeout_s;
    int max_delta;
    int last_counter;          /* -1 before the first frame */
    double last_ok_time;
    bool have_ok;
} e2e_rx_t;

uint8_t kauha_crc8_j1850(uint8_t data_id, const uint8_t data[8]);
void kauha_e2e_protect(uint8_t data[8], uint8_t data_id, uint8_t counter);
void kauha_e2e_rx_init(e2e_rx_t *rx, uint8_t data_id, double timeout_s);
enum e2e_status kauha_e2e_check(e2e_rx_t *rx, const uint8_t data[8], double t);
bool kauha_e2e_fresh(const e2e_rx_t *rx, double t);

/* Data ID of a frame = low byte of its PGN (frame_id >> 8). */
static inline uint8_t kauha_data_id(uint32_t frame_id) { return (uint8_t)((frame_id >> 8) & 0xFFu); }

#ifdef __cplusplus
}
#endif
#endif
