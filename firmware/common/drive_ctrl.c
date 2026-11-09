#include "drive_ctrl.h"

#include <math.h>

double drive_ramp(const drive_geom_t *g, double v_ref, double v_cmd, bool brake, double dt) {
    if (brake) return 0.0;
    if (v_cmd > g->max_speed) v_cmd = g->max_speed;
    if (v_cmd < -g->max_speed) v_cmd = -g->max_speed;
    double step = g->max_accel * dt;
    if (v_cmd > v_ref + step) return v_ref + step;
    if (v_cmd < v_ref - step) return v_ref - step;
    return v_cmd;
}

void drive_wheel_speeds(const drive_geom_t *g, double v, double gamma, double gamma_dot, double out[4]) {
    /* front frame yaw rate; no lateral slip at either axle (kauha/sim/loader.py) */
    double den = g->lf * cos(gamma) + g->lr;
    double yaw_f = (v * sin(gamma) + g->lr * gamma_dot) / den;
    double yaw_r = yaw_f - gamma_dot;
    double v_r = v * cos(gamma) + g->lf * yaw_f * sin(gamma);   /* rear axle centre speed */
    double h = g->track / 2.0;
    out[WHEEL_FL] = v - yaw_f * h;
    out[WHEEL_FR] = v + yaw_f * h;
    out[WHEEL_RL] = v_r - yaw_r * h;
    out[WHEEL_RR] = v_r + yaw_r * h;
}

double drive_speed_estimate(const drive_geom_t *g, const double w[4], double gamma) {
    double vf = 0.5 * (w[WHEEL_FL] + w[WHEEL_FR]);
    double vr = 0.5 * (w[WHEEL_RL] + w[WHEEL_RR]);
    /* steady-state ratio of rear to front axle centre speed */
    double ratio = (g->lf + g->lr * cos(gamma)) / (g->lf * cos(gamma) + g->lr);
    return 0.5 * (vf + vr / ratio);
}

double act_duty(const act_loop_t *c, double target, double now) {
    double e = target - now;
    if (fabs(e) < c->deadband) return 0.0;
    double d = c->kp * e;
    if (d > c->max_duty) d = c->max_duty;
    if (d < -c->max_duty) d = -c->max_duty;
    return d;
}

double table_interp(const double *xs, const double *ys, int n, double x) {
    if (n < 2) return n == 1 ? ys[0] : 0.0;
    bool up = xs[n - 1] > xs[0];
    if (up ? x <= xs[0] : x >= xs[0]) return ys[0];
    if (up ? x >= xs[n - 1] : x <= xs[n - 1]) return ys[n - 1];
    for (int i = 1; i < n; i++) {
        bool in = up ? x <= xs[i] : x >= xs[i];
        if (in) {
            double f = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return ys[i - 1] + f * (ys[i] - ys[i - 1]);
        }
    }
    return ys[n - 1];
}
