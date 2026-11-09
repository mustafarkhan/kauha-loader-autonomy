#include "gateway_core.h"

#include <math.h>
#include <string.h>

#include "generated/kauha_machine.h"

void gw_init(gw_t *gw) {
    memset(gw, 0, sizeof *gw);
    kauha_e2e_rx_init(&gw->rx_drive, kauha_data_id(KAUHA_MACHINE_DRIVE_CMD_FRAME_ID), GW_CMD_TIMEOUT_S);
    kauha_e2e_rx_init(&gw->rx_impl, kauha_data_id(KAUHA_MACHINE_IMPLEMENT_CMD_FRAME_ID), GW_CMD_TIMEOUT_S);
    kauha_e2e_rx_init(&gw->rx_safety, kauha_data_id(KAUHA_MACHINE_SAFETY_STATUS_FRAME_ID), GW_SAFETY_TIMEOUT_S);
    /* Python defaults when a message has never been decoded: Mode 0, State 3, Hold 1, BrakeReq 1 */
    gw->state = 3;
    gw->hold = 1;
    gw->brake_req = 1;
}

static bool accepted(gw_t *gw, e2e_rx_t *rx, const uint8_t data[8], double t) {
    enum e2e_status st = kauha_e2e_check(rx, data, t);
    if (st != E2E_OK && st != E2E_OK_SOME_LOST) {
        gw->e2e_errors++;
        return false;
    }
    return true;
}

void gw_receive(gw_t *gw, double t, uint32_t frame_id, const uint8_t data[8]) {
    switch (frame_id & 0x1FFFFFFFu) {
    case KAUHA_MACHINE_DRIVE_CMD_FRAME_ID: {
        if (!accepted(gw, &gw->rx_drive, data, t)) return;
        struct kauha_machine_drive_cmd_t m;
        kauha_machine_drive_cmd_unpack(&m, data, 8);
        gw->speed_req = kauha_machine_drive_cmd_speed_req_decode(m.speed_req);
        gw->articulation_req = kauha_machine_drive_cmd_articulation_req_decode(m.articulation_req);
        gw->mode = m.mode;
        gw->brake_req = m.brake_req;
        gw->have_drive = true;
        break;
    }
    case KAUHA_MACHINE_IMPLEMENT_CMD_FRAME_ID: {
        if (!accepted(gw, &gw->rx_impl, data, t)) return;
        struct kauha_machine_implement_cmd_t m;
        kauha_machine_implement_cmd_unpack(&m, data, 8);
        gw->boom_rate = kauha_machine_implement_cmd_boom_rate_decode(m.boom_rate);
        gw->bucket_rate = kauha_machine_implement_cmd_bucket_rate_decode(m.bucket_rate);
        gw->hold = m.hold;
        gw->have_impl = true;
        break;
    }
    case KAUHA_MACHINE_SAFETY_STATUS_FRAME_ID: {
        if (!accepted(gw, &gw->rx_safety, data, t)) return;
        struct kauha_machine_safety_status_t m;
        kauha_machine_safety_status_unpack(&m, data, 8);
        gw->state = m.state;
        gw->speed_limit = kauha_machine_safety_status_speed_limit_decode(m.speed_limit);
        gw->have_safety = true;
        break;
    }
    default:
        break;
    }
}

gw_command_t gw_command(gw_t *gw, double t) {
    gw_command_t c = {0.0, 0.0, 0.0, 0.0, true};
    bool fresh = kauha_e2e_fresh(&gw->rx_drive, t) && kauha_e2e_fresh(&gw->rx_impl, t) &&
                 kauha_e2e_fresh(&gw->rx_safety, t);
    if (!fresh) {
        if (!gw->timed_out) gw->timeouts++;
        gw->timed_out = true;
        return c;
    }
    gw->timed_out = false;
    c.articulation = gw->articulation_req;
    if (gw->mode == 0 || gw->state >= 2) return c;
    double boom = gw->hold ? 0.0 : gw->boom_rate;
    double bucket = gw->hold ? 0.0 : gw->bucket_rate;
    c.boom = boom;
    c.bucket = bucket;
    if (gw->brake_req) return c;
    double lim = gw->speed_limit, v = gw->speed_req;
    if (fabs(v) > lim) {
        gw->clamped++;
        v = v > lim ? lim : (v < -lim ? -lim : v);
    }
    c.speed = v;
    c.brake = false;
    return c;
}
