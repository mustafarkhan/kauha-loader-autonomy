/**
 * @file kauha_machine.c
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

#include <string.h>

#include "kauha_machine.h"

static inline uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u32(
    uint32_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u64(
    uint64_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint8_t pack_right_shift_u32(
    uint32_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint8_t pack_right_shift_u64(
    uint64_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint32_t unpack_left_shift_u32(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint32_t)((uint32_t)(value & mask) << shift);
}

static inline uint64_t unpack_left_shift_u64(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint64_t)((uint64_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

static inline uint32_t unpack_right_shift_u32(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint32_t)((uint32_t)(value & mask) >> shift);
}

static inline uint64_t unpack_right_shift_u64(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint64_t)((uint64_t)(value & mask) >> shift);
}

static uint64_t cantools_unsigned_from_double(double value)
{
    if (value < 0) {
        return 0;
    }

    return (uint64_t)value;
}

int kauha_machine_drive_cmd_pack(
    uint8_t *dst_p,
    const struct kauha_machine_drive_cmd_t *src_p,
    size_t size)
{
    uint16_t articulation_req;
    uint16_t speed_req;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    speed_req = (uint16_t)src_p->speed_req;
    dst_p[0] |= pack_left_shift_u16(speed_req, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(speed_req, 8u, 0xffu);
    articulation_req = (uint16_t)src_p->articulation_req;
    dst_p[2] |= pack_left_shift_u16(articulation_req, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(articulation_req, 8u, 0xffu);
    dst_p[4] |= pack_left_shift_u8(src_p->mode, 0u, 0x0fu);
    dst_p[4] |= pack_left_shift_u8(src_p->brake_req, 4u, 0x10u);
    dst_p[4] |= pack_left_shift_u8(src_p->dig_field_req, 5u, 0x20u);
    dst_p[4] |= pack_left_shift_u8(src_p->dock_field_req, 6u, 0x40u);
    dst_p[6] |= pack_left_shift_u8(src_p->alive_counter, 0u, 0x0fu);
    dst_p[7] |= pack_left_shift_u8(src_p->crc, 0u, 0xffu);

    return (8);
}

int kauha_machine_drive_cmd_unpack(
    struct kauha_machine_drive_cmd_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t articulation_req;
    uint16_t speed_req;

    if (size < 8u) {
        return (-EINVAL);
    }

    speed_req = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    speed_req |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->speed_req = (int16_t)speed_req;
    articulation_req = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    articulation_req |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->articulation_req = (int16_t)articulation_req;
    dst_p->mode = unpack_right_shift_u8(src_p[4], 0u, 0x0fu);
    dst_p->brake_req = unpack_right_shift_u8(src_p[4], 4u, 0x10u);
    dst_p->dig_field_req = unpack_right_shift_u8(src_p[4], 5u, 0x20u);
    dst_p->dock_field_req = unpack_right_shift_u8(src_p[4], 6u, 0x40u);
    dst_p->alive_counter = unpack_right_shift_u8(src_p[6], 0u, 0x0fu);
    dst_p->crc = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

int kauha_machine_drive_cmd_init(struct kauha_machine_drive_cmd_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_drive_cmd_t));

    return 0;
}

int16_t kauha_machine_drive_cmd_speed_req_encode(double value)
{
    return (int16_t)(value / 0.001);
}

double kauha_machine_drive_cmd_speed_req_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool kauha_machine_drive_cmd_speed_req_is_in_range(int16_t value)
{
    return ((value >= -6000) && (value <= 6000));
}

bool kauha_machine_drive_cmd_speed_req_is_in_phys_range(double value)
{
    return ((value >= -6.0) && (value <= 6.0));
}

int16_t kauha_machine_drive_cmd_articulation_req_encode(double value)
{
    return (int16_t)(value / 0.0001);
}

double kauha_machine_drive_cmd_articulation_req_decode(int16_t value)
{
    return ((double)value * 0.0001);
}

bool kauha_machine_drive_cmd_articulation_req_is_in_range(int16_t value)
{
    return ((value >= -8000) && (value <= 8000));
}

bool kauha_machine_drive_cmd_articulation_req_is_in_phys_range(double value)
{
    return ((value >= -0.8) && (value <= 0.8));
}

uint8_t kauha_machine_drive_cmd_mode_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_drive_cmd_mode_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_drive_cmd_mode_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_drive_cmd_mode_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_drive_cmd_brake_req_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_drive_cmd_brake_req_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_drive_cmd_brake_req_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_drive_cmd_brake_req_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_drive_cmd_dig_field_req_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_drive_cmd_dig_field_req_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_drive_cmd_dig_field_req_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_drive_cmd_dig_field_req_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_drive_cmd_dock_field_req_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_drive_cmd_dock_field_req_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_drive_cmd_dock_field_req_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_drive_cmd_dock_field_req_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_drive_cmd_alive_counter_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_drive_cmd_alive_counter_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_drive_cmd_alive_counter_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_drive_cmd_alive_counter_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_drive_cmd_crc_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_drive_cmd_crc_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_drive_cmd_crc_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_drive_cmd_crc_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

int kauha_machine_implement_cmd_pack(
    uint8_t *dst_p,
    const struct kauha_machine_implement_cmd_t *src_p,
    size_t size)
{
    uint8_t boom_rate;
    uint8_t bucket_rate;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    boom_rate = (uint8_t)src_p->boom_rate;
    dst_p[0] |= pack_left_shift_u8(boom_rate, 0u, 0xffu);
    bucket_rate = (uint8_t)src_p->bucket_rate;
    dst_p[1] |= pack_left_shift_u8(bucket_rate, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u8(src_p->hold, 0u, 0x01u);
    dst_p[6] |= pack_left_shift_u8(src_p->alive_counter, 0u, 0x0fu);
    dst_p[7] |= pack_left_shift_u8(src_p->crc, 0u, 0xffu);

    return (8);
}

int kauha_machine_implement_cmd_unpack(
    struct kauha_machine_implement_cmd_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint8_t boom_rate;
    uint8_t bucket_rate;

    if (size < 8u) {
        return (-EINVAL);
    }

    boom_rate = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->boom_rate = (int8_t)boom_rate;
    bucket_rate = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->bucket_rate = (int8_t)bucket_rate;
    dst_p->hold = unpack_right_shift_u8(src_p[2], 0u, 0x01u);
    dst_p->alive_counter = unpack_right_shift_u8(src_p[6], 0u, 0x0fu);
    dst_p->crc = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

int kauha_machine_implement_cmd_init(struct kauha_machine_implement_cmd_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_implement_cmd_t));

    return 0;
}

int8_t kauha_machine_implement_cmd_boom_rate_encode(double value)
{
    return (int8_t)(value / 0.01);
}

double kauha_machine_implement_cmd_boom_rate_decode(int8_t value)
{
    return ((double)value * 0.01);
}

bool kauha_machine_implement_cmd_boom_rate_is_in_range(int8_t value)
{
    return ((value >= -100) && (value <= 100));
}

bool kauha_machine_implement_cmd_boom_rate_is_in_phys_range(double value)
{
    return ((value >= -1.0) && (value <= 1.0));
}

int8_t kauha_machine_implement_cmd_bucket_rate_encode(double value)
{
    return (int8_t)(value / 0.01);
}

double kauha_machine_implement_cmd_bucket_rate_decode(int8_t value)
{
    return ((double)value * 0.01);
}

bool kauha_machine_implement_cmd_bucket_rate_is_in_range(int8_t value)
{
    return ((value >= -100) && (value <= 100));
}

bool kauha_machine_implement_cmd_bucket_rate_is_in_phys_range(double value)
{
    return ((value >= -1.0) && (value <= 1.0));
}

uint8_t kauha_machine_implement_cmd_hold_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_implement_cmd_hold_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_implement_cmd_hold_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_implement_cmd_hold_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_implement_cmd_alive_counter_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_implement_cmd_alive_counter_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_implement_cmd_alive_counter_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_implement_cmd_alive_counter_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_implement_cmd_crc_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_implement_cmd_crc_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_implement_cmd_crc_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_implement_cmd_crc_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

int kauha_machine_safety_status_pack(
    uint8_t *dst_p,
    const struct kauha_machine_safety_status_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->state, 0u, 0x0fu);
    dst_p[0] |= pack_left_shift_u8(src_p->reset_pending, 4u, 0x10u);
    dst_p[0] |= pack_left_shift_u8(src_p->dock_blocked, 5u, 0x20u);
    dst_p[1] |= pack_left_shift_u8(src_p->active_field_set, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u16(src_p->cause, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->cause, 8u, 0xffu);
    dst_p[4] |= pack_left_shift_u16(src_p->speed_limit, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(src_p->speed_limit, 8u, 0xffu);
    dst_p[6] |= pack_left_shift_u8(src_p->alive_counter, 0u, 0x0fu);
    dst_p[7] |= pack_left_shift_u8(src_p->crc, 0u, 0xffu);

    return (8);
}

int kauha_machine_safety_status_unpack(
    struct kauha_machine_safety_status_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->state = unpack_right_shift_u8(src_p[0], 0u, 0x0fu);
    dst_p->reset_pending = unpack_right_shift_u8(src_p[0], 4u, 0x10u);
    dst_p->dock_blocked = unpack_right_shift_u8(src_p[0], 5u, 0x20u);
    dst_p->active_field_set = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->cause = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->cause |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->speed_limit = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    dst_p->speed_limit |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);
    dst_p->alive_counter = unpack_right_shift_u8(src_p[6], 0u, 0x0fu);
    dst_p->crc = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

int kauha_machine_safety_status_init(struct kauha_machine_safety_status_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_safety_status_t));

    return 0;
}

uint8_t kauha_machine_safety_status_state_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safety_status_state_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_safety_status_state_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

bool kauha_machine_safety_status_state_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 3.0));
}

uint8_t kauha_machine_safety_status_reset_pending_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safety_status_reset_pending_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_safety_status_reset_pending_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_safety_status_reset_pending_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_safety_status_dock_blocked_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safety_status_dock_blocked_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_safety_status_dock_blocked_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_safety_status_dock_blocked_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_safety_status_active_field_set_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safety_status_active_field_set_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_safety_status_active_field_set_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_safety_status_active_field_set_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

uint16_t kauha_machine_safety_status_cause_encode(double value)
{
    return (uint16_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safety_status_cause_decode(uint16_t value)
{
    return ((double)value);
}

bool kauha_machine_safety_status_cause_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_safety_status_cause_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 65535.0));
}

uint16_t kauha_machine_safety_status_speed_limit_encode(double value)
{
    return (uint16_t)(cantools_unsigned_from_double(value / 0.001));
}

double kauha_machine_safety_status_speed_limit_decode(uint16_t value)
{
    return ((double)value * 0.001);
}

bool kauha_machine_safety_status_speed_limit_is_in_range(uint16_t value)
{
    return (value <= 6000u);
}

bool kauha_machine_safety_status_speed_limit_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 6.0));
}

uint8_t kauha_machine_safety_status_alive_counter_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safety_status_alive_counter_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_safety_status_alive_counter_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_safety_status_alive_counter_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_safety_status_crc_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safety_status_crc_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_safety_status_crc_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_safety_status_crc_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

int kauha_machine_machine_state1_pack(
    uint8_t *dst_p,
    const struct kauha_machine_machine_state1_t *src_p,
    size_t size)
{
    uint16_t articulation;
    uint16_t boom;
    uint16_t speed;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    speed = (uint16_t)src_p->speed;
    dst_p[0] |= pack_left_shift_u16(speed, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(speed, 8u, 0xffu);
    articulation = (uint16_t)src_p->articulation;
    dst_p[2] |= pack_left_shift_u16(articulation, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(articulation, 8u, 0xffu);
    boom = (uint16_t)src_p->boom;
    dst_p[4] |= pack_left_shift_u16(boom, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(boom, 8u, 0xffu);
    dst_p[6] |= pack_left_shift_u8(src_p->alive_counter, 0u, 0x0fu);
    dst_p[7] |= pack_left_shift_u8(src_p->crc, 0u, 0xffu);

    return (8);
}

int kauha_machine_machine_state1_unpack(
    struct kauha_machine_machine_state1_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t articulation;
    uint16_t boom;
    uint16_t speed;

    if (size < 8u) {
        return (-EINVAL);
    }

    speed = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    speed |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->speed = (int16_t)speed;
    articulation = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    articulation |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->articulation = (int16_t)articulation;
    boom = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    boom |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);
    dst_p->boom = (int16_t)boom;
    dst_p->alive_counter = unpack_right_shift_u8(src_p[6], 0u, 0x0fu);
    dst_p->crc = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

int kauha_machine_machine_state1_init(struct kauha_machine_machine_state1_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_machine_state1_t));

    return 0;
}

int16_t kauha_machine_machine_state1_speed_encode(double value)
{
    return (int16_t)(value / 0.001);
}

double kauha_machine_machine_state1_speed_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool kauha_machine_machine_state1_speed_is_in_range(int16_t value)
{
    return ((value >= -6000) && (value <= 6000));
}

bool kauha_machine_machine_state1_speed_is_in_phys_range(double value)
{
    return ((value >= -6.0) && (value <= 6.0));
}

int16_t kauha_machine_machine_state1_articulation_encode(double value)
{
    return (int16_t)(value / 0.0001);
}

double kauha_machine_machine_state1_articulation_decode(int16_t value)
{
    return ((double)value * 0.0001);
}

bool kauha_machine_machine_state1_articulation_is_in_range(int16_t value)
{
    return ((value >= -8000) && (value <= 8000));
}

bool kauha_machine_machine_state1_articulation_is_in_phys_range(double value)
{
    return ((value >= -0.8) && (value <= 0.8));
}

int16_t kauha_machine_machine_state1_boom_encode(double value)
{
    return (int16_t)(value / 0.001);
}

double kauha_machine_machine_state1_boom_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool kauha_machine_machine_state1_boom_is_in_range(int16_t value)
{
    return ((value >= -1000) && (value <= 1000));
}

bool kauha_machine_machine_state1_boom_is_in_phys_range(double value)
{
    return ((value >= -1.0) && (value <= 1.0));
}

uint8_t kauha_machine_machine_state1_alive_counter_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_machine_state1_alive_counter_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_machine_state1_alive_counter_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_machine_state1_alive_counter_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_machine_state1_crc_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_machine_state1_crc_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_machine_state1_crc_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_machine_state1_crc_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

int kauha_machine_machine_state2_pack(
    uint8_t *dst_p,
    const struct kauha_machine_machine_state2_t *src_p,
    size_t size)
{
    uint16_t bucket;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    bucket = (uint16_t)src_p->bucket;
    dst_p[0] |= pack_left_shift_u16(bucket, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(bucket, 8u, 0xffu);
    dst_p[2] |= pack_left_shift_u16(src_p->payload, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->payload, 8u, 0xffu);
    dst_p[4] |= pack_left_shift_u8(src_p->operator_present, 0u, 0x01u);
    dst_p[4] |= pack_left_shift_u8(src_p->park_brake, 1u, 0x02u);
    dst_p[6] |= pack_left_shift_u8(src_p->alive_counter, 0u, 0x0fu);
    dst_p[7] |= pack_left_shift_u8(src_p->crc, 0u, 0xffu);

    return (8);
}

int kauha_machine_machine_state2_unpack(
    struct kauha_machine_machine_state2_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t bucket;

    if (size < 8u) {
        return (-EINVAL);
    }

    bucket = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    bucket |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->bucket = (int16_t)bucket;
    dst_p->payload = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->payload |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->operator_present = unpack_right_shift_u8(src_p[4], 0u, 0x01u);
    dst_p->park_brake = unpack_right_shift_u8(src_p[4], 1u, 0x02u);
    dst_p->alive_counter = unpack_right_shift_u8(src_p[6], 0u, 0x0fu);
    dst_p->crc = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

int kauha_machine_machine_state2_init(struct kauha_machine_machine_state2_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_machine_state2_t));

    return 0;
}

int16_t kauha_machine_machine_state2_bucket_encode(double value)
{
    return (int16_t)(value / 0.001);
}

double kauha_machine_machine_state2_bucket_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool kauha_machine_machine_state2_bucket_is_in_range(int16_t value)
{
    return ((value >= -1000) && (value <= 1000));
}

bool kauha_machine_machine_state2_bucket_is_in_phys_range(double value)
{
    return ((value >= -1.0) && (value <= 1.0));
}

uint16_t kauha_machine_machine_state2_payload_encode(double value)
{
    return (uint16_t)(cantools_unsigned_from_double(value / 10.0));
}

double kauha_machine_machine_state2_payload_decode(uint16_t value)
{
    return ((double)value * 10.0);
}

bool kauha_machine_machine_state2_payload_is_in_range(uint16_t value)
{
    return (value <= 2000u);
}

bool kauha_machine_machine_state2_payload_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 20000.0));
}

uint8_t kauha_machine_machine_state2_operator_present_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_machine_state2_operator_present_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_machine_state2_operator_present_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_machine_state2_operator_present_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_machine_state2_park_brake_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_machine_state2_park_brake_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_machine_state2_park_brake_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_machine_state2_park_brake_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_machine_state2_alive_counter_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_machine_state2_alive_counter_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_machine_state2_alive_counter_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_machine_state2_alive_counter_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_machine_state2_crc_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_machine_state2_crc_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_machine_state2_crc_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_machine_state2_crc_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

int kauha_machine_remote_heartbeat_pack(
    uint8_t *dst_p,
    const struct kauha_machine_remote_heartbeat_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->e_stop, 0u, 0x01u);
    dst_p[0] |= pack_left_shift_u8(src_p->reset_req, 1u, 0x02u);
    dst_p[0] |= pack_left_shift_u8(src_p->mode_req, 4u, 0xf0u);
    dst_p[6] |= pack_left_shift_u8(src_p->alive_counter, 0u, 0x0fu);
    dst_p[7] |= pack_left_shift_u8(src_p->crc, 0u, 0xffu);

    return (8);
}

int kauha_machine_remote_heartbeat_unpack(
    struct kauha_machine_remote_heartbeat_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->e_stop = unpack_right_shift_u8(src_p[0], 0u, 0x01u);
    dst_p->reset_req = unpack_right_shift_u8(src_p[0], 1u, 0x02u);
    dst_p->mode_req = unpack_right_shift_u8(src_p[0], 4u, 0xf0u);
    dst_p->alive_counter = unpack_right_shift_u8(src_p[6], 0u, 0x0fu);
    dst_p->crc = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

int kauha_machine_remote_heartbeat_init(struct kauha_machine_remote_heartbeat_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_remote_heartbeat_t));

    return 0;
}

uint8_t kauha_machine_remote_heartbeat_e_stop_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_remote_heartbeat_e_stop_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_remote_heartbeat_e_stop_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_remote_heartbeat_e_stop_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_remote_heartbeat_reset_req_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_remote_heartbeat_reset_req_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_remote_heartbeat_reset_req_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_remote_heartbeat_reset_req_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_remote_heartbeat_mode_req_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_remote_heartbeat_mode_req_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_remote_heartbeat_mode_req_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_remote_heartbeat_mode_req_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_remote_heartbeat_alive_counter_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_remote_heartbeat_alive_counter_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_remote_heartbeat_alive_counter_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_remote_heartbeat_alive_counter_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_remote_heartbeat_crc_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_remote_heartbeat_crc_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_remote_heartbeat_crc_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_remote_heartbeat_crc_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

int kauha_machine_safe_sensors_pack(
    uint8_t *dst_p,
    const struct kauha_machine_safe_sensors_t *src_p,
    size_t size)
{
    uint16_t articulation;
    uint16_t boom_angle;
    uint16_t speed_b;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    speed_b = (uint16_t)src_p->speed_b;
    dst_p[0] |= pack_left_shift_u16(speed_b, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(speed_b, 8u, 0xffu);
    articulation = (uint16_t)src_p->articulation;
    dst_p[2] |= pack_left_shift_u16(articulation, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(articulation, 8u, 0xffu);
    boom_angle = (uint16_t)src_p->boom_angle;
    dst_p[4] |= pack_left_shift_u16(boom_angle, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(boom_angle, 8u, 0xffu);
    dst_p[6] |= pack_left_shift_u8(src_p->alive_counter, 0u, 0x0fu);
    dst_p[6] |= pack_left_shift_u8(src_p->boom_raised, 4u, 0x10u);
    dst_p[6] |= pack_left_shift_u8(src_p->sensor_ok, 5u, 0x20u);
    dst_p[7] |= pack_left_shift_u8(src_p->crc, 0u, 0xffu);

    return (8);
}

int kauha_machine_safe_sensors_unpack(
    struct kauha_machine_safe_sensors_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t articulation;
    uint16_t boom_angle;
    uint16_t speed_b;

    if (size < 8u) {
        return (-EINVAL);
    }

    speed_b = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    speed_b |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->speed_b = (int16_t)speed_b;
    articulation = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    articulation |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->articulation = (int16_t)articulation;
    boom_angle = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    boom_angle |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);
    dst_p->boom_angle = (int16_t)boom_angle;
    dst_p->alive_counter = unpack_right_shift_u8(src_p[6], 0u, 0x0fu);
    dst_p->boom_raised = unpack_right_shift_u8(src_p[6], 4u, 0x10u);
    dst_p->sensor_ok = unpack_right_shift_u8(src_p[6], 5u, 0x20u);
    dst_p->crc = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

int kauha_machine_safe_sensors_init(struct kauha_machine_safe_sensors_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_safe_sensors_t));

    return 0;
}

int16_t kauha_machine_safe_sensors_speed_b_encode(double value)
{
    return (int16_t)(value / 0.001);
}

double kauha_machine_safe_sensors_speed_b_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool kauha_machine_safe_sensors_speed_b_is_in_range(int16_t value)
{
    return ((value >= -6000) && (value <= 6000));
}

bool kauha_machine_safe_sensors_speed_b_is_in_phys_range(double value)
{
    return ((value >= -6.0) && (value <= 6.0));
}

int16_t kauha_machine_safe_sensors_articulation_encode(double value)
{
    return (int16_t)(value / 0.0001);
}

double kauha_machine_safe_sensors_articulation_decode(int16_t value)
{
    return ((double)value * 0.0001);
}

bool kauha_machine_safe_sensors_articulation_is_in_range(int16_t value)
{
    return ((value >= -8000) && (value <= 8000));
}

bool kauha_machine_safe_sensors_articulation_is_in_phys_range(double value)
{
    return ((value >= -0.8) && (value <= 0.8));
}

int16_t kauha_machine_safe_sensors_boom_angle_encode(double value)
{
    return (int16_t)(value / 0.001);
}

double kauha_machine_safe_sensors_boom_angle_decode(int16_t value)
{
    return ((double)value * 0.001);
}

bool kauha_machine_safe_sensors_boom_angle_is_in_range(int16_t value)
{
    return ((value >= -1500) && (value <= 1500));
}

bool kauha_machine_safe_sensors_boom_angle_is_in_phys_range(double value)
{
    return ((value >= -1.5) && (value <= 1.5));
}

uint8_t kauha_machine_safe_sensors_alive_counter_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safe_sensors_alive_counter_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_safe_sensors_alive_counter_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_safe_sensors_alive_counter_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_safe_sensors_boom_raised_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safe_sensors_boom_raised_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_safe_sensors_boom_raised_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_safe_sensors_boom_raised_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_safe_sensors_sensor_ok_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safe_sensors_sensor_ok_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_safe_sensors_sensor_ok_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_safe_sensors_sensor_ok_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_safe_sensors_crc_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_safe_sensors_crc_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_safe_sensors_crc_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_safe_sensors_crc_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

int kauha_machine_scanner_status_pack(
    uint8_t *dst_p,
    const struct kauha_machine_scanner_status_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->front_prot, 0u, 0x01u);
    dst_p[0] |= pack_left_shift_u8(src_p->front_warn, 1u, 0x02u);
    dst_p[0] |= pack_left_shift_u8(src_p->rear_prot, 2u, 0x04u);
    dst_p[0] |= pack_left_shift_u8(src_p->rear_warn, 3u, 0x08u);
    dst_p[0] |= pack_left_shift_u8(src_p->side_prot, 4u, 0x10u);
    dst_p[0] |= pack_left_shift_u8(src_p->side_contact, 5u, 0x20u);
    dst_p[0] |= pack_left_shift_u8(src_p->front_side, 6u, 0x40u);
    dst_p[0] |= pack_left_shift_u8(src_p->under_near, 7u, 0x80u);
    dst_p[1] |= pack_left_shift_u8(src_p->under_mid, 0u, 0x01u);
    dst_p[1] |= pack_left_shift_u8(src_p->under_far, 1u, 0x02u);
    dst_p[1] |= pack_left_shift_u8(src_p->scanners_ok, 2u, 0x04u);
    dst_p[2] |= pack_left_shift_u8(src_p->evaluated_set, 0u, 0xffu);
    dst_p[6] |= pack_left_shift_u8(src_p->alive_counter, 0u, 0x0fu);
    dst_p[7] |= pack_left_shift_u8(src_p->crc, 0u, 0xffu);

    return (8);
}

int kauha_machine_scanner_status_unpack(
    struct kauha_machine_scanner_status_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->front_prot = unpack_right_shift_u8(src_p[0], 0u, 0x01u);
    dst_p->front_warn = unpack_right_shift_u8(src_p[0], 1u, 0x02u);
    dst_p->rear_prot = unpack_right_shift_u8(src_p[0], 2u, 0x04u);
    dst_p->rear_warn = unpack_right_shift_u8(src_p[0], 3u, 0x08u);
    dst_p->side_prot = unpack_right_shift_u8(src_p[0], 4u, 0x10u);
    dst_p->side_contact = unpack_right_shift_u8(src_p[0], 5u, 0x20u);
    dst_p->front_side = unpack_right_shift_u8(src_p[0], 6u, 0x40u);
    dst_p->under_near = unpack_right_shift_u8(src_p[0], 7u, 0x80u);
    dst_p->under_mid = unpack_right_shift_u8(src_p[1], 0u, 0x01u);
    dst_p->under_far = unpack_right_shift_u8(src_p[1], 1u, 0x02u);
    dst_p->scanners_ok = unpack_right_shift_u8(src_p[1], 2u, 0x04u);
    dst_p->evaluated_set = unpack_right_shift_u8(src_p[2], 0u, 0xffu);
    dst_p->alive_counter = unpack_right_shift_u8(src_p[6], 0u, 0x0fu);
    dst_p->crc = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

int kauha_machine_scanner_status_init(struct kauha_machine_scanner_status_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_scanner_status_t));

    return 0;
}

uint8_t kauha_machine_scanner_status_front_prot_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_front_prot_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_front_prot_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_front_prot_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_front_warn_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_front_warn_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_front_warn_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_front_warn_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_rear_prot_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_rear_prot_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_rear_prot_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_rear_prot_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_rear_warn_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_rear_warn_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_rear_warn_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_rear_warn_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_side_prot_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_side_prot_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_side_prot_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_side_prot_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_side_contact_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_side_contact_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_side_contact_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_side_contact_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_front_side_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_front_side_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_front_side_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_front_side_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_under_near_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_under_near_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_under_near_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_under_near_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_under_mid_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_under_mid_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_under_mid_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_under_mid_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_under_far_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_under_far_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_under_far_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_under_far_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_scanners_ok_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_scanners_ok_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_scanners_ok_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_scanner_status_scanners_ok_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_scanner_status_evaluated_set_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_evaluated_set_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_evaluated_set_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_scanner_status_evaluated_set_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

uint8_t kauha_machine_scanner_status_alive_counter_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_alive_counter_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_alive_counter_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_scanner_status_alive_counter_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_scanner_status_crc_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_scanner_status_crc_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_scanner_status_crc_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_scanner_status_crc_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

int kauha_machine_zone_status_pack(
    uint8_t *dst_p,
    const struct kauha_machine_zone_status_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->in_operating_area, 0u, 0x01u);
    dst_p[0] |= pack_left_shift_u8(src_p->in_dig_zone, 1u, 0x02u);
    dst_p[0] |= pack_left_shift_u8(src_p->in_dock_zone, 2u, 0x04u);
    dst_p[0] |= pack_left_shift_u8(src_p->position_valid, 3u, 0x08u);
    dst_p[0] |= pack_left_shift_u8(src_p->truck_door_open, 4u, 0x10u);
    dst_p[6] |= pack_left_shift_u8(src_p->alive_counter, 0u, 0x0fu);
    dst_p[7] |= pack_left_shift_u8(src_p->crc, 0u, 0xffu);

    return (8);
}

int kauha_machine_zone_status_unpack(
    struct kauha_machine_zone_status_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->in_operating_area = unpack_right_shift_u8(src_p[0], 0u, 0x01u);
    dst_p->in_dig_zone = unpack_right_shift_u8(src_p[0], 1u, 0x02u);
    dst_p->in_dock_zone = unpack_right_shift_u8(src_p[0], 2u, 0x04u);
    dst_p->position_valid = unpack_right_shift_u8(src_p[0], 3u, 0x08u);
    dst_p->truck_door_open = unpack_right_shift_u8(src_p[0], 4u, 0x10u);
    dst_p->alive_counter = unpack_right_shift_u8(src_p[6], 0u, 0x0fu);
    dst_p->crc = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

int kauha_machine_zone_status_init(struct kauha_machine_zone_status_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_zone_status_t));

    return 0;
}

uint8_t kauha_machine_zone_status_in_operating_area_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_zone_status_in_operating_area_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_zone_status_in_operating_area_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_zone_status_in_operating_area_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_zone_status_in_dig_zone_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_zone_status_in_dig_zone_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_zone_status_in_dig_zone_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_zone_status_in_dig_zone_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_zone_status_in_dock_zone_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_zone_status_in_dock_zone_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_zone_status_in_dock_zone_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_zone_status_in_dock_zone_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_zone_status_position_valid_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_zone_status_position_valid_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_zone_status_position_valid_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_zone_status_position_valid_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_zone_status_truck_door_open_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_zone_status_truck_door_open_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_zone_status_truck_door_open_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

bool kauha_machine_zone_status_truck_door_open_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.0));
}

uint8_t kauha_machine_zone_status_alive_counter_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_zone_status_alive_counter_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_zone_status_alive_counter_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

bool kauha_machine_zone_status_alive_counter_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 15.0));
}

uint8_t kauha_machine_zone_status_crc_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_zone_status_crc_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_zone_status_crc_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_zone_status_crc_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

int kauha_machine_frame_auth_autonomy_pack(
    uint8_t *dst_p,
    const struct kauha_machine_frame_auth_autonomy_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->data_id, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u32(src_p->freshness, 0u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(src_p->freshness, 8u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->freshness, 16u, 0xffu);
    dst_p[4] |= pack_left_shift_u32(src_p->mac, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u32(src_p->mac, 8u, 0xffu);
    dst_p[6] |= pack_right_shift_u32(src_p->mac, 16u, 0xffu);
    dst_p[7] |= pack_right_shift_u32(src_p->mac, 24u, 0xffu);

    return (8);
}

int kauha_machine_frame_auth_autonomy_unpack(
    struct kauha_machine_frame_auth_autonomy_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->data_id = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->freshness = unpack_right_shift_u32(src_p[1], 0u, 0xffu);
    dst_p->freshness |= unpack_left_shift_u32(src_p[2], 8u, 0xffu);
    dst_p->freshness |= unpack_left_shift_u32(src_p[3], 16u, 0xffu);
    dst_p->mac = unpack_right_shift_u32(src_p[4], 0u, 0xffu);
    dst_p->mac |= unpack_left_shift_u32(src_p[5], 8u, 0xffu);
    dst_p->mac |= unpack_left_shift_u32(src_p[6], 16u, 0xffu);
    dst_p->mac |= unpack_left_shift_u32(src_p[7], 24u, 0xffu);

    return (0);
}

int kauha_machine_frame_auth_autonomy_init(struct kauha_machine_frame_auth_autonomy_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_frame_auth_autonomy_t));

    return 0;
}

uint8_t kauha_machine_frame_auth_autonomy_data_id_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_frame_auth_autonomy_data_id_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_frame_auth_autonomy_data_id_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_frame_auth_autonomy_data_id_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

uint32_t kauha_machine_frame_auth_autonomy_freshness_encode(double value)
{
    return (uint32_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_frame_auth_autonomy_freshness_decode(uint32_t value)
{
    return ((double)value);
}

bool kauha_machine_frame_auth_autonomy_freshness_is_in_range(uint32_t value)
{
    return (value <= 16777215u);
}

bool kauha_machine_frame_auth_autonomy_freshness_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 16777215.0));
}

uint32_t kauha_machine_frame_auth_autonomy_mac_encode(double value)
{
    return (uint32_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_frame_auth_autonomy_mac_decode(uint32_t value)
{
    return ((double)value);
}

bool kauha_machine_frame_auth_autonomy_mac_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_frame_auth_autonomy_mac_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 4294967295.0));
}

int kauha_machine_frame_auth_remote_pack(
    uint8_t *dst_p,
    const struct kauha_machine_frame_auth_remote_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->data_id, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u32(src_p->freshness, 0u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(src_p->freshness, 8u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->freshness, 16u, 0xffu);
    dst_p[4] |= pack_left_shift_u32(src_p->mac, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u32(src_p->mac, 8u, 0xffu);
    dst_p[6] |= pack_right_shift_u32(src_p->mac, 16u, 0xffu);
    dst_p[7] |= pack_right_shift_u32(src_p->mac, 24u, 0xffu);

    return (8);
}

int kauha_machine_frame_auth_remote_unpack(
    struct kauha_machine_frame_auth_remote_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->data_id = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->freshness = unpack_right_shift_u32(src_p[1], 0u, 0xffu);
    dst_p->freshness |= unpack_left_shift_u32(src_p[2], 8u, 0xffu);
    dst_p->freshness |= unpack_left_shift_u32(src_p[3], 16u, 0xffu);
    dst_p->mac = unpack_right_shift_u32(src_p[4], 0u, 0xffu);
    dst_p->mac |= unpack_left_shift_u32(src_p[5], 8u, 0xffu);
    dst_p->mac |= unpack_left_shift_u32(src_p[6], 16u, 0xffu);
    dst_p->mac |= unpack_left_shift_u32(src_p[7], 24u, 0xffu);

    return (0);
}

int kauha_machine_frame_auth_remote_init(struct kauha_machine_frame_auth_remote_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_frame_auth_remote_t));

    return 0;
}

uint8_t kauha_machine_frame_auth_remote_data_id_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_frame_auth_remote_data_id_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_frame_auth_remote_data_id_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_frame_auth_remote_data_id_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

uint32_t kauha_machine_frame_auth_remote_freshness_encode(double value)
{
    return (uint32_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_frame_auth_remote_freshness_decode(uint32_t value)
{
    return ((double)value);
}

bool kauha_machine_frame_auth_remote_freshness_is_in_range(uint32_t value)
{
    return (value <= 16777215u);
}

bool kauha_machine_frame_auth_remote_freshness_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 16777215.0));
}

uint32_t kauha_machine_frame_auth_remote_mac_encode(double value)
{
    return (uint32_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_frame_auth_remote_mac_decode(uint32_t value)
{
    return ((double)value);
}

bool kauha_machine_frame_auth_remote_mac_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_frame_auth_remote_mac_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 4294967295.0));
}

int kauha_machine_frame_auth_zone_pack(
    uint8_t *dst_p,
    const struct kauha_machine_frame_auth_zone_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->data_id, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u32(src_p->freshness, 0u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(src_p->freshness, 8u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->freshness, 16u, 0xffu);
    dst_p[4] |= pack_left_shift_u32(src_p->mac, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u32(src_p->mac, 8u, 0xffu);
    dst_p[6] |= pack_right_shift_u32(src_p->mac, 16u, 0xffu);
    dst_p[7] |= pack_right_shift_u32(src_p->mac, 24u, 0xffu);

    return (8);
}

int kauha_machine_frame_auth_zone_unpack(
    struct kauha_machine_frame_auth_zone_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->data_id = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->freshness = unpack_right_shift_u32(src_p[1], 0u, 0xffu);
    dst_p->freshness |= unpack_left_shift_u32(src_p[2], 8u, 0xffu);
    dst_p->freshness |= unpack_left_shift_u32(src_p[3], 16u, 0xffu);
    dst_p->mac = unpack_right_shift_u32(src_p[4], 0u, 0xffu);
    dst_p->mac |= unpack_left_shift_u32(src_p[5], 8u, 0xffu);
    dst_p->mac |= unpack_left_shift_u32(src_p[6], 16u, 0xffu);
    dst_p->mac |= unpack_left_shift_u32(src_p[7], 24u, 0xffu);

    return (0);
}

int kauha_machine_frame_auth_zone_init(struct kauha_machine_frame_auth_zone_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_frame_auth_zone_t));

    return 0;
}

uint8_t kauha_machine_frame_auth_zone_data_id_encode(double value)
{
    return (uint8_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_frame_auth_zone_data_id_decode(uint8_t value)
{
    return ((double)value);
}

bool kauha_machine_frame_auth_zone_data_id_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_frame_auth_zone_data_id_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 255.0));
}

uint32_t kauha_machine_frame_auth_zone_freshness_encode(double value)
{
    return (uint32_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_frame_auth_zone_freshness_decode(uint32_t value)
{
    return ((double)value);
}

bool kauha_machine_frame_auth_zone_freshness_is_in_range(uint32_t value)
{
    return (value <= 16777215u);
}

bool kauha_machine_frame_auth_zone_freshness_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 16777215.0));
}

uint32_t kauha_machine_frame_auth_zone_mac_encode(double value)
{
    return (uint32_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_frame_auth_zone_mac_decode(uint32_t value)
{
    return ((double)value);
}

bool kauha_machine_frame_auth_zone_mac_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_frame_auth_zone_mac_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 4294967295.0));
}

int kauha_machine_auth_challenge_pack(
    uint8_t *dst_p,
    const struct kauha_machine_auth_challenge_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u64(src_p->challenge, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u64(src_p->challenge, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u64(src_p->challenge, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u64(src_p->challenge, 24u, 0xffu);
    dst_p[4] |= pack_right_shift_u64(src_p->challenge, 32u, 0xffu);
    dst_p[5] |= pack_right_shift_u64(src_p->challenge, 40u, 0xffu);
    dst_p[6] |= pack_right_shift_u64(src_p->challenge, 48u, 0xffu);
    dst_p[7] |= pack_right_shift_u64(src_p->challenge, 56u, 0xffu);

    return (8);
}

int kauha_machine_auth_challenge_unpack(
    struct kauha_machine_auth_challenge_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->challenge = unpack_right_shift_u64(src_p[0], 0u, 0xffu);
    dst_p->challenge |= unpack_left_shift_u64(src_p[1], 8u, 0xffu);
    dst_p->challenge |= unpack_left_shift_u64(src_p[2], 16u, 0xffu);
    dst_p->challenge |= unpack_left_shift_u64(src_p[3], 24u, 0xffu);
    dst_p->challenge |= unpack_left_shift_u64(src_p[4], 32u, 0xffu);
    dst_p->challenge |= unpack_left_shift_u64(src_p[5], 40u, 0xffu);
    dst_p->challenge |= unpack_left_shift_u64(src_p[6], 48u, 0xffu);
    dst_p->challenge |= unpack_left_shift_u64(src_p[7], 56u, 0xffu);

    return (0);
}

int kauha_machine_auth_challenge_init(struct kauha_machine_auth_challenge_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kauha_machine_auth_challenge_t));

    return 0;
}

uint64_t kauha_machine_auth_challenge_challenge_encode(double value)
{
    return (uint64_t)(cantools_unsigned_from_double(value));
}

double kauha_machine_auth_challenge_challenge_decode(uint64_t value)
{
    return ((double)value);
}

bool kauha_machine_auth_challenge_challenge_is_in_range(uint64_t value)
{
    (void)value;

    return (true);
}

bool kauha_machine_auth_challenge_challenge_is_in_phys_range(double value)
{
    return ((value >= 0.0) && (value <= 1.8446744073709552e+19));
}
