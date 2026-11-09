/**
 * @file kauha_machine.h
 *
 *
 * @copyright Copyright (c) 2018-2019 Erik Moqvist
 *
 * @par License
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef KAUHA_MACHINE_H
#define KAUHA_MACHINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef EINVAL
#    define EINVAL 22
#endif

/* Frame ids. */
#define KAUHA_MACHINE_DRIVE_CMD_FRAME_ID (0xcff1080u)
#define KAUHA_MACHINE_IMPLEMENT_CMD_FRAME_ID (0xcff1180u)
#define KAUHA_MACHINE_SAFETY_STATUS_FRAME_ID (0x8ff2081u)
#define KAUHA_MACHINE_MACHINE_STATE1_FRAME_ID (0xcff3083u)
#define KAUHA_MACHINE_MACHINE_STATE2_FRAME_ID (0xcff3183u)
#define KAUHA_MACHINE_REMOTE_HEARTBEAT_FRAME_ID (0xcff4082u)
#define KAUHA_MACHINE_SAFE_SENSORS_FRAME_ID (0x8ff5084u)
#define KAUHA_MACHINE_SCANNER_STATUS_FRAME_ID (0x8ff5185u)
#define KAUHA_MACHINE_ZONE_STATUS_FRAME_ID (0xcff5286u)

/* Frame lengths in bytes. */
#define KAUHA_MACHINE_DRIVE_CMD_LENGTH (8u)
#define KAUHA_MACHINE_IMPLEMENT_CMD_LENGTH (8u)
#define KAUHA_MACHINE_SAFETY_STATUS_LENGTH (8u)
#define KAUHA_MACHINE_MACHINE_STATE1_LENGTH (8u)
#define KAUHA_MACHINE_MACHINE_STATE2_LENGTH (8u)
#define KAUHA_MACHINE_REMOTE_HEARTBEAT_LENGTH (8u)
#define KAUHA_MACHINE_SAFE_SENSORS_LENGTH (8u)
#define KAUHA_MACHINE_SCANNER_STATUS_LENGTH (8u)
#define KAUHA_MACHINE_ZONE_STATUS_LENGTH (8u)

/* Extended or standard frame types. */
#define KAUHA_MACHINE_DRIVE_CMD_IS_EXTENDED (1)
#define KAUHA_MACHINE_IMPLEMENT_CMD_IS_EXTENDED (1)
#define KAUHA_MACHINE_SAFETY_STATUS_IS_EXTENDED (1)
#define KAUHA_MACHINE_MACHINE_STATE1_IS_EXTENDED (1)
#define KAUHA_MACHINE_MACHINE_STATE2_IS_EXTENDED (1)
#define KAUHA_MACHINE_REMOTE_HEARTBEAT_IS_EXTENDED (1)
#define KAUHA_MACHINE_SAFE_SENSORS_IS_EXTENDED (1)
#define KAUHA_MACHINE_SCANNER_STATUS_IS_EXTENDED (1)
#define KAUHA_MACHINE_ZONE_STATUS_IS_EXTENDED (1)

/* Frame cycle times in milliseconds. */


/* Signal choices. */
#define KAUHA_MACHINE_DRIVE_CMD_MODE_IDLE_CHOICE (0u)
#define KAUHA_MACHINE_DRIVE_CMD_MODE_AUTO_CHOICE (1u)
#define KAUHA_MACHINE_DRIVE_CMD_MODE_TELEOP_CHOICE (2u)

#define KAUHA_MACHINE_SAFETY_STATUS_STATE_RUN_CHOICE (0u)
#define KAUHA_MACHINE_SAFETY_STATUS_STATE_WARNING_CHOICE (1u)
#define KAUHA_MACHINE_SAFETY_STATUS_STATE_PROTECTIVE_STOP_CHOICE (2u)
#define KAUHA_MACHINE_SAFETY_STATUS_STATE_EMERGENCY_STOP_CHOICE (3u)

/* Frame Names. */
#define KAUHA_MACHINE_DRIVE_CMD_NAME "DriveCmd"
#define KAUHA_MACHINE_IMPLEMENT_CMD_NAME "ImplementCmd"
#define KAUHA_MACHINE_SAFETY_STATUS_NAME "SafetyStatus"
#define KAUHA_MACHINE_MACHINE_STATE1_NAME "MachineState1"
#define KAUHA_MACHINE_MACHINE_STATE2_NAME "MachineState2"
#define KAUHA_MACHINE_REMOTE_HEARTBEAT_NAME "RemoteHeartbeat"
#define KAUHA_MACHINE_SAFE_SENSORS_NAME "SafeSensors"
#define KAUHA_MACHINE_SCANNER_STATUS_NAME "ScannerStatus"
#define KAUHA_MACHINE_ZONE_STATUS_NAME "ZoneStatus"

/* Signal Names. */
#define KAUHA_MACHINE_DRIVE_CMD_SPEED_REQ_NAME "SpeedReq"
#define KAUHA_MACHINE_DRIVE_CMD_ARTICULATION_REQ_NAME "ArticulationReq"
#define KAUHA_MACHINE_DRIVE_CMD_MODE_NAME "Mode"
#define KAUHA_MACHINE_DRIVE_CMD_BRAKE_REQ_NAME "BrakeReq"
#define KAUHA_MACHINE_DRIVE_CMD_DIG_FIELD_REQ_NAME "DigFieldReq"
#define KAUHA_MACHINE_DRIVE_CMD_DOCK_FIELD_REQ_NAME "DockFieldReq"
#define KAUHA_MACHINE_DRIVE_CMD_ALIVE_COUNTER_NAME "AliveCounter"
#define KAUHA_MACHINE_DRIVE_CMD_CRC_NAME "Crc"
#define KAUHA_MACHINE_IMPLEMENT_CMD_BOOM_RATE_NAME "BoomRate"
#define KAUHA_MACHINE_IMPLEMENT_CMD_BUCKET_RATE_NAME "BucketRate"
#define KAUHA_MACHINE_IMPLEMENT_CMD_HOLD_NAME "Hold"
#define KAUHA_MACHINE_IMPLEMENT_CMD_ALIVE_COUNTER_NAME "AliveCounter"
#define KAUHA_MACHINE_IMPLEMENT_CMD_CRC_NAME "Crc"
#define KAUHA_MACHINE_SAFETY_STATUS_STATE_NAME "State"
#define KAUHA_MACHINE_SAFETY_STATUS_RESET_PENDING_NAME "ResetPending"
#define KAUHA_MACHINE_SAFETY_STATUS_DOCK_BLOCKED_NAME "DockBlocked"
#define KAUHA_MACHINE_SAFETY_STATUS_ACTIVE_FIELD_SET_NAME "ActiveFieldSet"
#define KAUHA_MACHINE_SAFETY_STATUS_CAUSE_NAME "Cause"
#define KAUHA_MACHINE_SAFETY_STATUS_SPEED_LIMIT_NAME "SpeedLimit"
#define KAUHA_MACHINE_SAFETY_STATUS_ALIVE_COUNTER_NAME "AliveCounter"
#define KAUHA_MACHINE_SAFETY_STATUS_CRC_NAME "Crc"
#define KAUHA_MACHINE_MACHINE_STATE1_SPEED_NAME "Speed"
#define KAUHA_MACHINE_MACHINE_STATE1_ARTICULATION_NAME "Articulation"
#define KAUHA_MACHINE_MACHINE_STATE1_BOOM_NAME "Boom"
#define KAUHA_MACHINE_MACHINE_STATE1_ALIVE_COUNTER_NAME "AliveCounter"
#define KAUHA_MACHINE_MACHINE_STATE1_CRC_NAME "Crc"
#define KAUHA_MACHINE_MACHINE_STATE2_BUCKET_NAME "Bucket"
#define KAUHA_MACHINE_MACHINE_STATE2_PAYLOAD_NAME "Payload"
#define KAUHA_MACHINE_MACHINE_STATE2_OPERATOR_PRESENT_NAME "OperatorPresent"
#define KAUHA_MACHINE_MACHINE_STATE2_PARK_BRAKE_NAME "ParkBrake"
#define KAUHA_MACHINE_MACHINE_STATE2_ALIVE_COUNTER_NAME "AliveCounter"
#define KAUHA_MACHINE_MACHINE_STATE2_CRC_NAME "Crc"
#define KAUHA_MACHINE_REMOTE_HEARTBEAT_E_STOP_NAME "EStop"
#define KAUHA_MACHINE_REMOTE_HEARTBEAT_RESET_REQ_NAME "ResetReq"
#define KAUHA_MACHINE_REMOTE_HEARTBEAT_MODE_REQ_NAME "ModeReq"
#define KAUHA_MACHINE_REMOTE_HEARTBEAT_ALIVE_COUNTER_NAME "AliveCounter"
#define KAUHA_MACHINE_REMOTE_HEARTBEAT_CRC_NAME "Crc"
#define KAUHA_MACHINE_SAFE_SENSORS_SPEED_B_NAME "SpeedB"
#define KAUHA_MACHINE_SAFE_SENSORS_ARTICULATION_NAME "Articulation"
#define KAUHA_MACHINE_SAFE_SENSORS_BOOM_ANGLE_NAME "BoomAngle"
#define KAUHA_MACHINE_SAFE_SENSORS_ALIVE_COUNTER_NAME "AliveCounter"
#define KAUHA_MACHINE_SAFE_SENSORS_BOOM_RAISED_NAME "BoomRaised"
#define KAUHA_MACHINE_SAFE_SENSORS_SENSOR_OK_NAME "SensorOk"
#define KAUHA_MACHINE_SAFE_SENSORS_CRC_NAME "Crc"
#define KAUHA_MACHINE_SCANNER_STATUS_FRONT_PROT_NAME "FrontProt"
#define KAUHA_MACHINE_SCANNER_STATUS_FRONT_WARN_NAME "FrontWarn"
#define KAUHA_MACHINE_SCANNER_STATUS_REAR_PROT_NAME "RearProt"
#define KAUHA_MACHINE_SCANNER_STATUS_REAR_WARN_NAME "RearWarn"
#define KAUHA_MACHINE_SCANNER_STATUS_SIDE_PROT_NAME "SideProt"
#define KAUHA_MACHINE_SCANNER_STATUS_SIDE_CONTACT_NAME "SideContact"
#define KAUHA_MACHINE_SCANNER_STATUS_FRONT_SIDE_NAME "FrontSide"
#define KAUHA_MACHINE_SCANNER_STATUS_UNDER_NEAR_NAME "UnderNear"
#define KAUHA_MACHINE_SCANNER_STATUS_UNDER_MID_NAME "UnderMid"
#define KAUHA_MACHINE_SCANNER_STATUS_UNDER_FAR_NAME "UnderFar"
#define KAUHA_MACHINE_SCANNER_STATUS_SCANNERS_OK_NAME "ScannersOk"
#define KAUHA_MACHINE_SCANNER_STATUS_EVALUATED_SET_NAME "EvaluatedSet"
#define KAUHA_MACHINE_SCANNER_STATUS_ALIVE_COUNTER_NAME "AliveCounter"
#define KAUHA_MACHINE_SCANNER_STATUS_CRC_NAME "Crc"
#define KAUHA_MACHINE_ZONE_STATUS_IN_OPERATING_AREA_NAME "InOperatingArea"
#define KAUHA_MACHINE_ZONE_STATUS_IN_DIG_ZONE_NAME "InDigZone"
#define KAUHA_MACHINE_ZONE_STATUS_IN_DOCK_ZONE_NAME "InDockZone"
#define KAUHA_MACHINE_ZONE_STATUS_POSITION_VALID_NAME "PositionValid"
#define KAUHA_MACHINE_ZONE_STATUS_TRUCK_DOOR_OPEN_NAME "TruckDoorOpen"
#define KAUHA_MACHINE_ZONE_STATUS_ALIVE_COUNTER_NAME "AliveCounter"
#define KAUHA_MACHINE_ZONE_STATUS_CRC_NAME "Crc"

/**
 * Signals in message DriveCmd.
 *
 * PGN 0xFF10 prio 3. 20 Hz, receiver timeout 150 ms. E2E: AliveCounter + CRC8 (SAE J1850) over bytes 0-6 and data ID.
 *
 * All signal values are as on the CAN bus.
 */
struct kauha_machine_drive_cmd_t {
    /**
     * Range: -6000..6000 (-6..6 m/s)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t speed_req;

    /**
     * Range: -8000..8000 (-0.8..0.8 rad)
     * Scale: 0.0001
     * Offset: 0
     */
    int16_t articulation_req;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t mode;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t brake_req;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t dig_field_req;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t dock_field_req;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t alive_counter;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t crc;
};

/**
 * Signals in message ImplementCmd.
 *
 * PGN 0xFF11 prio 3. 20 Hz, receiver timeout 150 ms.
 *
 * All signal values are as on the CAN bus.
 */
struct kauha_machine_implement_cmd_t {
    /**
     * Range: -100..100 (-1..1 -)
     * Scale: 0.01
     * Offset: 0
     */
    int8_t boom_rate;

    /**
     * Range: -100..100 (-1..1 -)
     * Scale: 0.01
     * Offset: 0
     */
    int8_t bucket_rate;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t hold;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t alive_counter;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t crc;
};

/**
 * Signals in message SafetyStatus.
 *
 * PGN 0xFF20 prio 2. 40 Hz (PLC cycle 25 ms). Informational copy of safety state; the stop itself is hardwired.
 *
 * All signal values are as on the CAN bus.
 */
struct kauha_machine_safety_status_t {
    /**
     * Range: 0..3 (0..3 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t state;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t reset_pending;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t dock_blocked;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t active_field_set;

    /**
     * Bitfield: see kauha.safety.plc.Cause.
     *
     * Range: 0..65535 (0..65535 -)
     * Scale: 1
     * Offset: 0
     */
    uint16_t cause;

    /**
     * Range: 0..6000 (0..6 m/s)
     * Scale: 0.001
     * Offset: 0
     */
    uint16_t speed_limit;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t alive_counter;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t crc;
};

/**
 * Signals in message MachineState1.
 *
 * PGN 0xFF30 prio 3. 40 Hz.
 *
 * All signal values are as on the CAN bus.
 */
struct kauha_machine_machine_state1_t {
    /**
     * Range: -6000..6000 (-6..6 m/s)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t speed;

    /**
     * Range: -8000..8000 (-0.8..0.8 rad)
     * Scale: 0.0001
     * Offset: 0
     */
    int16_t articulation;

    /**
     * Range: -1000..1000 (-1..1 rad)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t boom;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t alive_counter;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t crc;
};

/**
 * Signals in message MachineState2.
 *
 * PGN 0xFF31 prio 3. 20 Hz.
 *
 * All signal values are as on the CAN bus.
 */
struct kauha_machine_machine_state2_t {
    /**
     * Range: -1000..1000 (-1..1 rad)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t bucket;

    /**
     * Range: 0..2000 (0..20000 kg)
     * Scale: 10
     * Offset: 0
     */
    uint16_t payload;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t operator_present;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t park_brake;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t alive_counter;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t crc;
};

/**
 * Signals in message RemoteHeartbeat.
 *
 * PGN 0xFF40 prio 3. 20 Hz. Loss > 500 ms forces a protective stop.
 *
 * All signal values are as on the CAN bus.
 */
struct kauha_machine_remote_heartbeat_t {
    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t e_stop;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t reset_req;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t mode_req;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t alive_counter;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t crc;
};

/**
 * Signals in message SafeSensors.
 *
 * PGN 0xFF50 prio 2. 40 Hz, receiver timeout 100 ms. Testbed only (the full machine wires these sensors to the PLC).
 *
 * All signal values are as on the CAN bus.
 */
struct kauha_machine_safe_sensors_t {
    /**
     * Range: -6000..6000 (-6..6 m/s)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t speed_b;

    /**
     * Range: -8000..8000 (-0.8..0.8 rad)
     * Scale: 0.0001
     * Offset: 0
     */
    int16_t articulation;

    /**
     * Range: -1500..1500 (-1.5..1.5 rad)
     * Scale: 0.001
     * Offset: 0
     */
    int16_t boom_angle;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t alive_counter;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t boom_raised;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t sensor_ok;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t crc;
};

/**
 * Signals in message ScannerStatus.
 *
 * PGN 0xFF51 prio 2. 40 Hz, receiver timeout 100 ms. Testbed only: the full machine and a rated testbed wire scanner outputs to the PLC.
 *
 * All signal values are as on the CAN bus.
 */
struct kauha_machine_scanner_status_t {
    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t front_prot;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t front_warn;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t rear_prot;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t rear_warn;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t side_prot;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t side_contact;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t front_side;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t under_near;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t under_mid;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t under_far;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t scanners_ok;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t evaluated_set;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t alive_counter;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t crc;
};

/**
 * Signals in message ZoneStatus.
 *
 * PGN 0xFF52 prio 3. 10 Hz, receiver timeout 500 ms. Testbed only.
 *
 * All signal values are as on the CAN bus.
 */
struct kauha_machine_zone_status_t {
    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t in_operating_area;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t in_dig_zone;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t in_dock_zone;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t position_valid;

    /**
     * Range: 0..1 (0..1 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t truck_door_open;

    /**
     * Range: 0..15 (0..15 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t alive_counter;

    /**
     * Range: 0..255 (0..255 -)
     * Scale: 1
     * Offset: 0
     */
    uint8_t crc;
};

/**
 * Pack message DriveCmd.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int kauha_machine_drive_cmd_pack(
    uint8_t *dst_p,
    const struct kauha_machine_drive_cmd_t *src_p,
    size_t size);

/**
 * Unpack message DriveCmd.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int kauha_machine_drive_cmd_unpack(
    struct kauha_machine_drive_cmd_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from DriveCmd.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int kauha_machine_drive_cmd_init(struct kauha_machine_drive_cmd_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t kauha_machine_drive_cmd_speed_req_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_drive_cmd_speed_req_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_speed_req_is_in_range(int16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_speed_req_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t kauha_machine_drive_cmd_articulation_req_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_drive_cmd_articulation_req_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_articulation_req_is_in_range(int16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_articulation_req_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_drive_cmd_mode_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_drive_cmd_mode_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_mode_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_mode_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_drive_cmd_brake_req_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_drive_cmd_brake_req_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_brake_req_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_brake_req_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_drive_cmd_dig_field_req_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_drive_cmd_dig_field_req_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_dig_field_req_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_dig_field_req_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_drive_cmd_dock_field_req_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_drive_cmd_dock_field_req_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_dock_field_req_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_dock_field_req_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_drive_cmd_alive_counter_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_drive_cmd_alive_counter_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_alive_counter_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_alive_counter_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_drive_cmd_crc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_drive_cmd_crc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_crc_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_drive_cmd_crc_is_in_phys_range(double value);

/**
 * Pack message ImplementCmd.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int kauha_machine_implement_cmd_pack(
    uint8_t *dst_p,
    const struct kauha_machine_implement_cmd_t *src_p,
    size_t size);

/**
 * Unpack message ImplementCmd.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int kauha_machine_implement_cmd_unpack(
    struct kauha_machine_implement_cmd_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from ImplementCmd.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int kauha_machine_implement_cmd_init(struct kauha_machine_implement_cmd_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int8_t kauha_machine_implement_cmd_boom_rate_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_implement_cmd_boom_rate_decode(int8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_implement_cmd_boom_rate_is_in_range(int8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_implement_cmd_boom_rate_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int8_t kauha_machine_implement_cmd_bucket_rate_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_implement_cmd_bucket_rate_decode(int8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_implement_cmd_bucket_rate_is_in_range(int8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_implement_cmd_bucket_rate_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_implement_cmd_hold_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_implement_cmd_hold_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_implement_cmd_hold_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_implement_cmd_hold_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_implement_cmd_alive_counter_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_implement_cmd_alive_counter_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_implement_cmd_alive_counter_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_implement_cmd_alive_counter_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_implement_cmd_crc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_implement_cmd_crc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_implement_cmd_crc_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_implement_cmd_crc_is_in_phys_range(double value);

/**
 * Pack message SafetyStatus.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int kauha_machine_safety_status_pack(
    uint8_t *dst_p,
    const struct kauha_machine_safety_status_t *src_p,
    size_t size);

/**
 * Unpack message SafetyStatus.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int kauha_machine_safety_status_unpack(
    struct kauha_machine_safety_status_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from SafetyStatus.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int kauha_machine_safety_status_init(struct kauha_machine_safety_status_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_safety_status_state_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safety_status_state_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_state_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_state_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_safety_status_reset_pending_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safety_status_reset_pending_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_reset_pending_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_reset_pending_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_safety_status_dock_blocked_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safety_status_dock_blocked_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_dock_blocked_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_dock_blocked_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_safety_status_active_field_set_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safety_status_active_field_set_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_active_field_set_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_active_field_set_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t kauha_machine_safety_status_cause_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safety_status_cause_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_cause_is_in_range(uint16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_cause_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t kauha_machine_safety_status_speed_limit_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safety_status_speed_limit_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_speed_limit_is_in_range(uint16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_speed_limit_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_safety_status_alive_counter_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safety_status_alive_counter_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_alive_counter_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_alive_counter_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_safety_status_crc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safety_status_crc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_crc_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safety_status_crc_is_in_phys_range(double value);

/**
 * Pack message MachineState1.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int kauha_machine_machine_state1_pack(
    uint8_t *dst_p,
    const struct kauha_machine_machine_state1_t *src_p,
    size_t size);

/**
 * Unpack message MachineState1.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int kauha_machine_machine_state1_unpack(
    struct kauha_machine_machine_state1_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from MachineState1.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int kauha_machine_machine_state1_init(struct kauha_machine_machine_state1_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t kauha_machine_machine_state1_speed_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state1_speed_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state1_speed_is_in_range(int16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state1_speed_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t kauha_machine_machine_state1_articulation_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state1_articulation_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state1_articulation_is_in_range(int16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state1_articulation_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t kauha_machine_machine_state1_boom_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state1_boom_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state1_boom_is_in_range(int16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state1_boom_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_machine_state1_alive_counter_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state1_alive_counter_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state1_alive_counter_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state1_alive_counter_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_machine_state1_crc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state1_crc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state1_crc_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state1_crc_is_in_phys_range(double value);

/**
 * Pack message MachineState2.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int kauha_machine_machine_state2_pack(
    uint8_t *dst_p,
    const struct kauha_machine_machine_state2_t *src_p,
    size_t size);

/**
 * Unpack message MachineState2.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int kauha_machine_machine_state2_unpack(
    struct kauha_machine_machine_state2_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from MachineState2.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int kauha_machine_machine_state2_init(struct kauha_machine_machine_state2_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t kauha_machine_machine_state2_bucket_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state2_bucket_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_bucket_is_in_range(int16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_bucket_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t kauha_machine_machine_state2_payload_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state2_payload_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_payload_is_in_range(uint16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_payload_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_machine_state2_operator_present_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state2_operator_present_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_operator_present_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_operator_present_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_machine_state2_park_brake_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state2_park_brake_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_park_brake_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_park_brake_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_machine_state2_alive_counter_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state2_alive_counter_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_alive_counter_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_alive_counter_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_machine_state2_crc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_machine_state2_crc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_crc_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_machine_state2_crc_is_in_phys_range(double value);

/**
 * Pack message RemoteHeartbeat.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int kauha_machine_remote_heartbeat_pack(
    uint8_t *dst_p,
    const struct kauha_machine_remote_heartbeat_t *src_p,
    size_t size);

/**
 * Unpack message RemoteHeartbeat.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int kauha_machine_remote_heartbeat_unpack(
    struct kauha_machine_remote_heartbeat_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from RemoteHeartbeat.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int kauha_machine_remote_heartbeat_init(struct kauha_machine_remote_heartbeat_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_remote_heartbeat_e_stop_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_remote_heartbeat_e_stop_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_remote_heartbeat_e_stop_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_remote_heartbeat_e_stop_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_remote_heartbeat_reset_req_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_remote_heartbeat_reset_req_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_remote_heartbeat_reset_req_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_remote_heartbeat_reset_req_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_remote_heartbeat_mode_req_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_remote_heartbeat_mode_req_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_remote_heartbeat_mode_req_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_remote_heartbeat_mode_req_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_remote_heartbeat_alive_counter_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_remote_heartbeat_alive_counter_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_remote_heartbeat_alive_counter_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_remote_heartbeat_alive_counter_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_remote_heartbeat_crc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_remote_heartbeat_crc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_remote_heartbeat_crc_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_remote_heartbeat_crc_is_in_phys_range(double value);

/**
 * Pack message SafeSensors.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int kauha_machine_safe_sensors_pack(
    uint8_t *dst_p,
    const struct kauha_machine_safe_sensors_t *src_p,
    size_t size);

/**
 * Unpack message SafeSensors.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int kauha_machine_safe_sensors_unpack(
    struct kauha_machine_safe_sensors_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from SafeSensors.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int kauha_machine_safe_sensors_init(struct kauha_machine_safe_sensors_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t kauha_machine_safe_sensors_speed_b_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safe_sensors_speed_b_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_speed_b_is_in_range(int16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_speed_b_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t kauha_machine_safe_sensors_articulation_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safe_sensors_articulation_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_articulation_is_in_range(int16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_articulation_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t kauha_machine_safe_sensors_boom_angle_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safe_sensors_boom_angle_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_boom_angle_is_in_range(int16_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_boom_angle_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_safe_sensors_alive_counter_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safe_sensors_alive_counter_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_alive_counter_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_alive_counter_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_safe_sensors_boom_raised_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safe_sensors_boom_raised_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_boom_raised_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_boom_raised_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_safe_sensors_sensor_ok_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safe_sensors_sensor_ok_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_sensor_ok_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_sensor_ok_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_safe_sensors_crc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_safe_sensors_crc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_crc_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_safe_sensors_crc_is_in_phys_range(double value);

/**
 * Pack message ScannerStatus.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int kauha_machine_scanner_status_pack(
    uint8_t *dst_p,
    const struct kauha_machine_scanner_status_t *src_p,
    size_t size);

/**
 * Unpack message ScannerStatus.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int kauha_machine_scanner_status_unpack(
    struct kauha_machine_scanner_status_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from ScannerStatus.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int kauha_machine_scanner_status_init(struct kauha_machine_scanner_status_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_front_prot_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_front_prot_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_front_prot_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_front_prot_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_front_warn_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_front_warn_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_front_warn_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_front_warn_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_rear_prot_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_rear_prot_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_rear_prot_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_rear_prot_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_rear_warn_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_rear_warn_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_rear_warn_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_rear_warn_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_side_prot_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_side_prot_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_side_prot_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_side_prot_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_side_contact_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_side_contact_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_side_contact_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_side_contact_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_front_side_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_front_side_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_front_side_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_front_side_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_under_near_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_under_near_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_under_near_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_under_near_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_under_mid_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_under_mid_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_under_mid_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_under_mid_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_under_far_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_under_far_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_under_far_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_under_far_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_scanners_ok_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_scanners_ok_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_scanners_ok_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_scanners_ok_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_evaluated_set_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_evaluated_set_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_evaluated_set_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_evaluated_set_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_alive_counter_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_alive_counter_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_alive_counter_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_alive_counter_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_scanner_status_crc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_scanner_status_crc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_crc_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_scanner_status_crc_is_in_phys_range(double value);

/**
 * Pack message ZoneStatus.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int kauha_machine_zone_status_pack(
    uint8_t *dst_p,
    const struct kauha_machine_zone_status_t *src_p,
    size_t size);

/**
 * Unpack message ZoneStatus.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int kauha_machine_zone_status_unpack(
    struct kauha_machine_zone_status_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from ZoneStatus.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int kauha_machine_zone_status_init(struct kauha_machine_zone_status_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_zone_status_in_operating_area_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_zone_status_in_operating_area_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_in_operating_area_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_in_operating_area_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_zone_status_in_dig_zone_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_zone_status_in_dig_zone_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_in_dig_zone_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_in_dig_zone_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_zone_status_in_dock_zone_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_zone_status_in_dock_zone_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_in_dock_zone_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_in_dock_zone_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_zone_status_position_valid_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_zone_status_position_valid_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_position_valid_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_position_valid_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_zone_status_truck_door_open_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_zone_status_truck_door_open_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_truck_door_open_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_truck_door_open_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_zone_status_alive_counter_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_zone_status_alive_counter_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_alive_counter_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_alive_counter_is_in_phys_range(double value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t kauha_machine_zone_status_crc_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double kauha_machine_zone_status_crc_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_crc_is_in_range(uint8_t value);

/**
 * Check that given physical value is in allowed range before encoding.
 *
 * Use this BEFORE calling _encode() to avoid silent integer overflow:
 *   encode(-1.0) on a [0,100] signal wraps to 255 and passes is_in_range().
 *   is_in_phys_range(-1.0) correctly returns false.
 *
 * @param[in] value Physical signal value to check.
 *
 * @return true if in range, false otherwise.
 */
bool kauha_machine_zone_status_crc_is_in_phys_range(double value);


#ifdef __cplusplus
}
#endif

#endif
