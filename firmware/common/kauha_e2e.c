#include "kauha_e2e.h"

uint8_t kauha_crc8_j1850(uint8_t data_id, const uint8_t data[8]) {
    uint8_t crc = 0xFF;
    for (int i = -1; i < 7; i++) {
        crc ^= (i < 0) ? data_id : data[i];
        for (int k = 0; k < 8; k++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x1Du) : (uint8_t)(crc << 1);
        }
    }
    return crc ^ 0xFF;
}

void kauha_e2e_protect(uint8_t data[8], uint8_t data_id, uint8_t counter) {
    data[6] = (uint8_t)((data[6] & 0xF0u) | (counter & 0x0Fu));
    data[7] = kauha_crc8_j1850(data_id, data);
}

void kauha_e2e_rx_init(e2e_rx_t *rx, uint8_t data_id, double timeout_s) {
    rx->data_id = data_id;
    rx->timeout_s = timeout_s;
    rx->max_delta = 2;
    rx->last_counter = -1;
    rx->last_ok_time = 0.0;
    rx->have_ok = false;
}

enum e2e_status kauha_e2e_check(e2e_rx_t *rx, const uint8_t data[8], double t) {
    if (kauha_crc8_j1850(rx->data_id, data) != data[7]) return E2E_CRC_ERROR;
    int ctr = data[6] & 0x0F;
    enum e2e_status st = E2E_OK;
    if (rx->last_counter >= 0) {
        int delta = (ctr - rx->last_counter + 16) % 16;
        if (delta == 0) return E2E_REPEATED;
        st = delta > rx->max_delta ? E2E_WRONG_SEQUENCE : (delta == 1 ? E2E_OK : E2E_OK_SOME_LOST);
    }
    rx->last_counter = ctr;
    if (st == E2E_OK || st == E2E_OK_SOME_LOST) {
        rx->last_ok_time = t;
        rx->have_ok = true;
    }
    return st;
}

bool kauha_e2e_fresh(const e2e_rx_t *rx, double t) {
    return rx->have_ok && (t - rx->last_ok_time) <= rx->timeout_s;
}
