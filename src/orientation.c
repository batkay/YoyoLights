#include "orientation.h"


static double pitch;
static double roll;
static double yaw;

static double delta_pitch;
static double delta_roll;
static double delta_yaw;

void initialize(double initial_pitch, double initial_roll, double initial_yaw) {
    pitch = initial_pitch;
    roll = initial_roll;
    yaw = initial_yaw;
}

static void update_angle(double* angle, double* delta_angle, double acc, double gyro, double dt) {
    double prev_angle = *angle;
    *angle = 0.98 * (gyro*dt + (*angle)) + 0.02 * acc;

    *delta_angle = (*angle - prev_angle)/dt;

    while (*angle > 3.1415) {
        *angle -= 2 * 3.1415;
    }
    while (*angle < -3.1415) {
        *angle += 2 * 3.1415;
    }
}

void update_pitch(double acc, double gyro, double dt) {
    update_angle(&pitch, &delta_pitch, acc, gyro, dt);
}

void update_roll(double acc, double gyro, double dt) {
    update_angle(&roll, &delta_roll, acc, gyro, dt);
}

void update_yaw(double gyro, double dt) {
    yaw += gyro * dt;
    delta_yaw = gyro;

    while (yaw > 3.1415) {
        yaw -= 2 * 3.1415;
    }
    while (yaw < -3.1415) {
        yaw += 2 * 3.1415;
    }
}

double get_pitch() {
    return pitch;
}

double get_roll() {
    return roll;
}

double get_yaw() {
    return yaw;
}

double get_delta_pitch() {
    return delta_pitch;
}

double get_delta_roll() {
    return delta_roll;
}

double get_delta_yaw() {
    return delta_yaw;
}