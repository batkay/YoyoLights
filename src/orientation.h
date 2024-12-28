#pragma once

void initialize(double initial_pitch, double initial_roll, double initial_yaw);
void update_pitch(double acc, double gyro, double dt);
void update_roll(double acc, double gyro, double dt);
void update_yaw(double gyro, double dt);


double get_pitch();
double get_roll();
double get_yaw();

double get_delta_pitch();
double get_delta_roll();
double get_delta_yaw();