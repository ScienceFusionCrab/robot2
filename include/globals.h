#pragma once
#include "main.h"

namespace Robot{ namespace Globals{
extern pros::MotorGroup groupL;
extern pros::MotorGroup groupR;
extern pros::Motor chain;
extern pros::adi::DigitalOut hawk;
extern bool hawkDown;
extern pros::Imu imu;
extern pros::adi::DigitalOut tuah;
extern bool tuahDown;
extern pros::adi::DigitalOut bristol;
extern pros::Motor therizzler;
extern pros::adi::DigitalOut bristol;
extern bool bristolDown;
extern lemlib::Drivetrain drivetrain;
extern lemlib::ExpoDriveCurve steer_curve;
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::OdomSensors sensors;
extern lemlib::Chassis chassis;

}}