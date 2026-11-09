// KAUHA 1:8 testbed: drive-by-wire gateway (Teensy 4.1).
//
// CAN1 (500 kbit/s, 29-bit J1939 ids): the machine bus (DBC hardware/can/kauha_machine.dbc).
// CAN2 (500 kbit/s, 11-bit ids): two ODrive v3.6 controllers, CANSimple protocol,
//   node 0 = front axle (axis0 left, axis1 right as nodes 0/1), nodes 2/3 = rear axle.
// PWM + DIR to three Cytron MDD10A boards for the five PA-14P actuators,
// potentiometers on A0..A4 (via 1:1 RC filters, 3.3 V reference).
//
// All decisions live in firmware/common (tested on the host). This file is only
// hardware access. The machine stops through the safety PLC's relays whatever
// this code does: K-DRIVE removes the hub motors from the ODrives and shorts their
// phases; K-IMPL removes actuator power.
#include <Arduino.h>
#include <FlexCAN_T4.h>

extern "C" {
#include "drive_ctrl.h"
#include "gateway_core.h"
#include "generated/kauha_machine.h"
#include "generated/kinematics.h"
#include "kauha_e2e.h"
}

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> bus;     // machine bus
FlexCAN_T4<CAN2, RX_SIZE_64, TX_SIZE_16> motors;   // ODrives

// ------------------------------------------------------------------ pins
struct ActPins { uint8_t pwm, dir, pot; };
static const ActPins ACT[5] = {
    {2, 3, A0},    // lift L
    {4, 5, A1},    // lift R
    {6, 7, A2},    // tilt
    {8, 9, A3},    // steer L
    {10, 11, A4},  // steer R
};
static const uint8_t PIN_DRIVE_POWER_SENSE = 24;   // high when K-DRIVE is closed (opto)
static const uint8_t PIN_LED = 13;

// Potentiometer calibration (counts at retracted and extended), measured in commissioning T1.
static float pot_retracted[5] = {80, 80, 80, 80, 80};
static float pot_extended[5] = {3990, 3990, 3990, 3990, 3990};

static gw_t gw;
static const drive_geom_t GEOM = {KIN_LF, KIN_LR, KIN_TRACK, KIN_MAX_ACCEL, KIN_MAX_SPEED};
static const act_loop_t ACT_LOOP = {60.0, 0.0008, 1.0};      // duty per m, 0.8 mm deadband

static double v_ref = 0.0, gamma_now = 0.0, gamma_target = 0.0, boom = -0.2, boom_target = -0.2;
static double bucket_rel = 0.75, bucket_rel_target = 0.75;
static double wheel_speed[4] = {0, 0, 0, 0};       // m/s from ODrive encoder estimates
static uint8_t ctr_ms1 = 0, ctr_ms2 = 0;

static double now_s() { return micros() * 1e-6; }

static double act_length(int i) {
    float c = analogRead(ACT[i].pot);
    double f = (c - pot_retracted[i]) / (pot_extended[i] - pot_retracted[i]);
    double lo = (i < 2) ? KIN_LIFT_RETRACTED : (i == 2 ? KIN_TILT_RETRACTED : KIN_STEER_RETRACTED);
    double hi = (i < 2) ? KIN_LIFT_EXTENDED : (i == 2 ? KIN_TILT_EXTENDED : KIN_STEER_EXTENDED);
    return lo + f * (hi - lo);
}

static void act_out(int i, double duty) {
    digitalWrite(ACT[i].dir, duty >= 0 ? HIGH : LOW);
    analogWrite(ACT[i].pwm, (int)(fabs(duty) * 4095.0));
}

// ------------------------------------------------------------------ ODrive CANSimple
static void odrive_send(uint8_t node, uint8_t cmd, const uint8_t *data, uint8_t len) {
    CAN_message_t m;
    m.id = (node << 5) | cmd;
    m.flags.extended = 0;
    m.len = len;
    memcpy(m.buf, data, len);
    motors.write(m);
}

static void odrive_velocity(uint8_t node, double wheel_m_s) {
    float turns_s = (float)(wheel_m_s / (2.0 * M_PI * KIN_WHEEL_R));
    float ff = 0.0f;
    uint8_t d[8];
    memcpy(d, &turns_s, 4);
    memcpy(d + 4, &ff, 4);
    odrive_send(node, 0x0D, d, 8);             // Set_Input_Vel
}

static void odrive_state(uint8_t node, uint32_t state) {
    uint8_t d[4];
    memcpy(d, &state, 4);
    odrive_send(node, 0x07, d, 4);             // Set_Axis_State: 1 idle, 8 closed loop
}

static void on_motor_frame(const CAN_message_t &m) {
    uint8_t node = m.id >> 5, cmd = m.id & 0x1F;
    if (cmd == 0x09 && node < 4 && m.len == 8) {   // Get_Encoder_Estimates: pos, vel (turns, turns/s)
        float vel;
        memcpy(&vel, m.buf + 4, 4);
        double sign = (node % 2 == 0) ? 1.0 : -1.0;   // right-side motors are mounted mirrored
        wheel_speed[node] = sign * vel * 2.0 * M_PI * KIN_WHEEL_R;
    }
}

static void on_bus_frame(const CAN_message_t &m) {
    if (m.flags.extended && m.len == 8) gw_receive(&gw, now_s(), m.id, m.buf);
}

static void send_bus(uint32_t id, const uint8_t *payload) {
    CAN_message_t m;
    m.id = id;
    m.flags.extended = 1;
    m.len = 8;
    memcpy(m.buf, payload, 8);
    bus.write(m);
}

static void publish(double speed) {
    uint8_t d[8] = {0};
    struct kauha_machine_machine_state1_t s1 = {};
    s1.speed = kauha_machine_machine_state1_speed_encode(speed);
    s1.articulation = kauha_machine_machine_state1_articulation_encode(gamma_now);
    s1.boom = kauha_machine_machine_state1_boom_encode(boom);
    kauha_machine_machine_state1_pack(d, &s1, 8);
    kauha_e2e_protect(d, kauha_data_id(KAUHA_MACHINE_MACHINE_STATE1_FRAME_ID), ctr_ms1++ & 0x0F);
    send_bus(KAUHA_MACHINE_MACHINE_STATE1_FRAME_ID, d);
    static uint8_t div = 0;
    if (++div % 2 == 0) {
        uint8_t e[8] = {0};
        struct kauha_machine_machine_state2_t s2 = {};
        s2.bucket = kauha_machine_machine_state2_bucket_encode(bucket_rel + boom);   // world pitch, twin convention
        s2.park_brake = digitalRead(PIN_DRIVE_POWER_SENSE) ? 0 : 1;
        kauha_machine_machine_state2_pack(e, &s2, 8);
        kauha_e2e_protect(e, kauha_data_id(KAUHA_MACHINE_MACHINE_STATE2_FRAME_ID), ctr_ms2++ & 0x0F);
        send_bus(KAUHA_MACHINE_MACHINE_STATE2_FRAME_ID, e);
    }
}

void setup() {
    gw_init(&gw);
    for (auto &a : ACT) {
        pinMode(a.dir, OUTPUT);
        pinMode(a.pwm, OUTPUT);
        analogWriteFrequency(a.pwm, 20000);
    }
    analogWriteResolution(12);
    analogReadResolution(12);
    pinMode(PIN_DRIVE_POWER_SENSE, INPUT);
    pinMode(PIN_LED, OUTPUT);
    bus.begin();
    bus.setBaudRate(500000);
    bus.enableFIFO();
    bus.enableFIFOInterrupt();
    bus.onReceive(on_bus_frame);
    motors.begin();
    motors.setBaudRate(500000);
    motors.enableFIFO();
    motors.enableFIFOInterrupt();
    motors.onReceive(on_motor_frame);
}

void loop() {
    static uint32_t last = micros(), last_pub = 0;
    bus.events();
    motors.events();
    uint32_t nowu = micros();
    if (nowu - last < 5000) return;              // 200 Hz control loop
    double dt = (nowu - last) * 1e-6;
    last = nowu;
    double t = now_s();
    gw_command_t c = gw_command(&gw, t);
    bool power = digitalRead(PIN_DRIVE_POWER_SENSE);

    // articulation, boom and bucket targets (rate-limited), then actuator loops
    double g_cmd = fmax(-KIN_MAX_ARTICULATION, fmin(KIN_MAX_ARTICULATION, c.articulation));
    double g_step = KIN_ARTICULATION_RATE * dt;
    gamma_target += fmax(-g_step, fmin(g_step, g_cmd - gamma_target));
    boom_target = fmax(-0.62, fmin(0.95, boom_target + c.boom * KIN_BOOM_RATE * dt));
    bucket_rel_target = fmax(-1.55, fmin(0.95, bucket_rel_target + c.bucket * KIN_BUCKET_RATE * dt));

    double L[5];
    for (int i = 0; i < 5; i++) L[i] = act_length(i);
    boom = table_interp(KIN_LIFT_LENGTH, KIN_LIFT_ANGLE, KIN_LIFT_N, 0.5 * (L[0] + L[1]));
    bucket_rel = table_interp(KIN_TILT_LENGTH, KIN_TILT_ANGLE, KIN_TILT_N, L[2]);
    gamma_now = table_interp(KIN_STEER_LENGTH, KIN_STEER_ANGLE, KIN_STEER_N, L[3]);
    double lift_t = table_interp(KIN_LIFT_ANGLE, KIN_LIFT_LENGTH, KIN_LIFT_N, boom_target);
    double sync = 30.0 * (L[0] - L[1]);           // keep the two lift actuators together
    act_out(0, act_duty(&ACT_LOOP, lift_t, L[0]) - sync);
    act_out(1, act_duty(&ACT_LOOP, lift_t, L[1]) + sync);
    act_out(2, act_duty(&ACT_LOOP, table_interp(KIN_TILT_ANGLE, KIN_TILT_LENGTH, KIN_TILT_N, bucket_rel_target), L[2]));
    double steer_l = table_interp(KIN_STEER_ANGLE, KIN_STEER_LENGTH, KIN_STEER_N, gamma_target);
    double steer_r = table_interp(KIN_STEER_ANGLE, KIN_STEER_LENGTH, KIN_STEER_N, -gamma_target);   // mirrored
    act_out(3, act_duty(&ACT_LOOP, steer_l, L[3]));
    act_out(4, act_duty(&ACT_LOOP, steer_r, L[4]));

    // wheels
    v_ref = drive_ramp(&GEOM, v_ref, c.speed, c.brake || !power, dt);
    double w[4];
    drive_wheel_speeds(&GEOM, v_ref, gamma_now, 0.0, w);
    static bool closed_loop = false;
    if ((c.brake || !power) && closed_loop && fabs(v_ref) < 1e-6) {
        for (uint8_t n = 0; n < 4; n++) odrive_state(n, 1);
        closed_loop = false;
    } else if (!c.brake && power && !closed_loop) {
        for (uint8_t n = 0; n < 4; n++) odrive_state(n, 8);
        closed_loop = true;
    }
    if (closed_loop) {
        for (uint8_t n = 0; n < 4; n++) odrive_velocity(n, (n % 2 == 0 ? 1.0 : -1.0) * w[n]);
    }
    if (nowu - last_pub >= 25000) {               // 40 Hz MachineState1, 20 Hz MachineState2
        last_pub = nowu;
        publish(drive_speed_estimate(&GEOM, wheel_speed, gamma_now));
    }
    digitalWrite(PIN_LED, closed_loop);
}
