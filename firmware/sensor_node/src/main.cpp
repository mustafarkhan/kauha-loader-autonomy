// KAUHA 1:8 testbed: the safety PLC's own sensors (Teensy 4.0).
//
// Independent of the gateway: own power supply branch, own microcontroller, own
// CAN transceiver. Measures
//   - articulation: AS5600 on the hinge pin (Wire),
//   - boom angle:   AS5600 on the boom pivot (Wire1); boom_raised when the bucket
//                   bottom is above the scaled 1.0 m (BOOM_RAISED_RAD, from the CAD),
//   - speed:        the front-left hub motor's Hall sensors, through their own
//                   buffer, counted in quadrature-like sequence for direction;
// and sends SafeSensors at 40 Hz with alive counter and CRC (E2E).
// SensorOk drops if a magnet is missing, an I2C read fails or the Hall sequence is
// invalid; the PLC then stops the machine (SCANNER_FAULT, see FB_SafetyNode).
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>

extern "C" {
#include "generated/kauha_machine.h"
#include "generated/kinematics.h"
#include "kauha_e2e.h"
}

FlexCAN_T4<CAN1, RX_SIZE_16, TX_SIZE_16> bus;

static const uint8_t AS5600 = 0x36;
static const uint8_t HALL_A = 2, HALL_B = 3, HALL_C = 4;
static const double HALL_STEPS_PER_REV = 90.0;           // 15 pole pairs x 6 states (typical 6.5 in hub motor)
static const double BOOM_RAISED_RAD = 0.10;              // bucket bottom above 125 mm: from the CAD, check in T1
static float art_zero = 0.0f, boom_zero = 0.0f;          // offsets set in commissioning T1 (EEPROM)

static volatile int32_t hall_steps = 0;
static volatile bool hall_bad = false;
static volatile uint8_t last_state = 0;
// valid Hall sequence (A,B,C bits): 1-3-2-6-4-5; index in sequence for each state, 255 = invalid
static const uint8_t SEQ[8] = {255, 0, 2, 1, 4, 5, 3, 255};

static void hall_isr() {
    uint8_t s = digitalReadFast(HALL_A) | (digitalReadFast(HALL_B) << 1) | (digitalReadFast(HALL_C) << 2);
    uint8_t a = SEQ[last_state], b = SEQ[s];
    if (b == 255) { hall_bad = true; return; }
    if (a != 255 && a != b) {
        int d = (b - a + 6) % 6;
        if (d == 1) hall_steps++;
        else if (d == 5) hall_steps--;
        else hall_bad = true;                    // skipped a state: too fast or a broken wire
    }
    last_state = s;
}

static bool read_angle(TwoWire &w, float &rad) {
    w.beginTransmission(AS5600);
    w.write(0x0B);                               // STATUS, then RAW ANGLE (0x0C, 0x0D)
    if (w.endTransmission(false) != 0) return false;
    if (w.requestFrom(AS5600, (uint8_t)3) != 3) return false;
    uint8_t status = w.read();
    uint16_t raw = ((uint16_t)(w.read() & 0x0F) << 8) | w.read();
    if (!(status & 0x20)) return false;          // MD: magnet detected
    rad = raw * (2.0f * (float)M_PI / 4096.0f);
    return true;
}

static float wrap(float a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

void setup() {
    Wire.begin();
    Wire.setClock(400000);
    Wire1.begin();
    Wire1.setClock(400000);
    const uint8_t halls[3] = {HALL_A, HALL_B, HALL_C};
    for (uint8_t p : halls) {
        pinMode(p, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(p), hall_isr, CHANGE);
    }
    bus.begin();
    bus.setBaudRate(500000);
}

void loop() {
    static uint32_t last = micros();
    static int32_t last_steps = 0;
    static uint8_t ctr = 0;
    static double v_filt = 0.0;
    uint32_t now = micros();
    if (now - last < 25000) return;               // 40 Hz
    double dt = (now - last) * 1e-6;
    last = now;
    noInterrupts();
    int32_t steps = hall_steps;
    bool bad = hall_bad;
    hall_bad = false;
    interrupts();
    double v = (steps - last_steps) / HALL_STEPS_PER_REV * 2.0 * M_PI * KIN_WHEEL_R / dt;
    last_steps = steps;
    v_filt += 0.5 * (v - v_filt);                // light filter: 2 samples
    float art = 0, boom = 0;
    bool ok_a = read_angle(Wire, art), ok_b = read_angle(Wire1, boom);
    art = wrap(art - art_zero);
    boom = wrap(boom - boom_zero);
    bool ok = ok_a && ok_b && !bad && fabsf(art) < KIN_MAX_ARTICULATION + 0.1f;

    struct kauha_machine_safe_sensors_t m = {};
    m.speed_b = kauha_machine_safe_sensors_speed_b_encode(v_filt);
    m.articulation = kauha_machine_safe_sensors_articulation_encode(art);
    m.boom_angle = kauha_machine_safe_sensors_boom_angle_encode(boom);
    m.boom_raised = boom > BOOM_RAISED_RAD;
    m.sensor_ok = ok;
    uint8_t d[8] = {0};
    kauha_machine_safe_sensors_pack(d, &m, 8);
    kauha_e2e_protect(d, kauha_data_id(KAUHA_MACHINE_SAFE_SENSORS_FRAME_ID), ctr++ & 0x0F);
    CAN_message_t f;
    f.id = KAUHA_MACHINE_SAFE_SENSORS_FRAME_ID;
    f.flags.extended = 1;
    f.len = 8;
    memcpy(f.buf, d, 8);
    bus.write(f);
}
