#include "orientation.h"


static double pitch;
static double roll;
static double yaw;

void initialize(double initial_pitch, double initial_roll, double initial_yaw) {
    pitch = initial_pitch;
    roll = initial_roll;
    yaw = initial_yaw;
}

static void update_angle(double* angle, double acc, double gyro, double dt) {
    *angle = 0.98 * (gyro*dt + (*angle)) + 0.02 * acc;
}

void update_pitch(double acc, double gyro, double dt) {
    update_angle(&pitch, acc, gyro, dt);
}

void update_roll(double acc, double gyro, double dt) {
    update_angle(&roll, acc, gyro, dt);

}

void update_yaw(double acc, double gyro, double dt) {
    update_angle(&yaw, acc, gyro, dt);
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