/* Drive-by-wire gateway logic, independent of the microcontroller.
 *
 * Port of kauha/can/gateway.py (MachineGateway): decode DriveCmd, ImplementCmd and
 * SafetyStatus, check them end to end, and produce the machine command:
 *   - any of the three stale or never received -> brake;
 *   - mode IDLE or safety state >= PROTECTIVE_STOP -> brake, keep articulation;
 *   - Hold -> no boom/bucket motion;
 *   - BrakeReq -> parked on the brake, implement may move (dumping);
 *   - otherwise the speed request clamped to the PLC's SpeedLimit.
 * tests/firmware/test_firmware.py runs it against the Python gateway on random frame
 * streams (including corrupted, repeated and missing frames).
 *
 * The gateway is a functional arbiter. The machine stops through the PLC's relays
 * whatever this code does. */
#ifndef KAUHA_GATEWAY_CORE_H
#define KAUHA_GATEWAY_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "kauha_e2e.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GW_CMD_TIMEOUT_S 0.15
#define GW_SAFETY_TIMEOUT_S 0.10

typedef struct {
    double speed;          /* m/s, signed */
    double articulation;   /* rad */
    double boom;           /* rate command -1..1 */
    double bucket;         /* rate command -1..1 */
    bool brake;
} gw_command_t;

typedef struct {
    e2e_rx_t rx_drive, rx_impl, rx_safety;
    bool have_drive, have_impl, have_safety;
    /* last accepted signal values */
    double speed_req, articulation_req;
    int mode, brake_req;
    double boom_rate, bucket_rate;
    int hold;
    int state;
    double speed_limit;
    /* statistics */
    uint32_t e2e_errors, timeouts, clamped;
    bool timed_out;
} gw_t;

void gw_init(gw_t *gw);
/* Feed one received frame (29-bit identifier without flags, 8 data bytes). */
void gw_receive(gw_t *gw, double t, uint32_t frame_id, const uint8_t data[8]);
gw_command_t gw_command(gw_t *gw, double t);

#ifdef __cplusplus
}
#endif
#endif
