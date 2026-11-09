/* Low-level control of the 1:8 testbed, independent of the microcontroller.
 *
 *  - wheel speed targets for the four hub motors from (speed, articulation): the
 *    articulated-frame kinematics of the twin (kauha/sim/loader.py), with each
 *    wheel at its own speed so the tyres do not scrub in a turn;
 *  - acceleration limit on the speed command;
 *  - position control of the five PA-14P actuators from their potentiometers,
 *    using the length tables generated from the CAD linkage (kinematics.h). */
#ifndef KAUHA_DRIVE_CTRL_H
#define KAUHA_DRIVE_CTRL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double lf, lr;          /* front axle to joint, rear axle to joint, m */
    double track;           /* wheel centre spacing, m */
    double max_accel;       /* m/s^2 */
    double max_speed;       /* m/s at the wheel */
} drive_geom_t;

enum { WHEEL_FL = 0, WHEEL_FR = 1, WHEEL_RL = 2, WHEEL_RR = 3 };

/* Ramp the commanded speed toward v_cmd; returns the new reference. A brake command
 * drops the reference to zero at once (the motors' own braking does the rest). */
double drive_ramp(const drive_geom_t *g, double v_ref, double v_cmd, bool brake, double dt);

/* Wheel surface speeds (m/s) for front-axle speed v, articulation gamma, articulation rate. */
void drive_wheel_speeds(const drive_geom_t *g, double v, double gamma, double gamma_dot, double out[4]);

/* Machine speed from four wheel speeds: mean of the axle centres (front axle frame). */
double drive_speed_estimate(const drive_geom_t *g, const double wheel[4], double gamma);

/* Linear actuator position loop: returns a PWM duty in [-1, 1]. */
typedef struct {
    double kp;              /* duty per metre of error */
    double deadband;        /* m */
    double max_duty;
} act_loop_t;

double act_duty(const act_loop_t *c, double length_target, double length_now);

/* Table lookups (monotonic tables, linear interpolation, clamped). */
double table_interp(const double *xs, const double *ys, int n, double x);

#ifdef __cplusplus
}
#endif
#endif
