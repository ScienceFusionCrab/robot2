#include "globals.h"

namespace Robot{ namespace Globals{

	
pros::MotorGroup groupL({16, -17, 18});
	pros::MotorGroup groupR({13, -14, 15});
	pros::Motor chain(20);
	pros::adi::DigitalOut hawk('G', false);
    bool hawkDown = false;
	pros::Imu imu(12);
	pros::adi::DigitalOut tuah('H', false);
    bool tuahDown = false;
	pros::Motor therizzler(10);

	lemlib::Drivetrain drivetrain(&groupL, // left motor group
        &groupR, // right motor group
        10.5, // 10.5 inch track width
        lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
        450, // drivetrain rpm is 360
        2 // horizontal drift is 2 (for now)
	);

	lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
        10, // minimum output where drivetrain will move out of 127
        1 // expo curve gain
	);

	lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
    	10, // minimum output where drivetrain will move out of 127
    	1 // expo curve gain
	);

	lemlib::ControllerSettings lateral_controller(
		12, // proportional gain (kP)
        0, // integral gain (kI)
        8, // derivative gain (kD)
        3, // anti windup
        1, // small error range, in inches
        100, // small error range timeout, in milliseconds
        3, // large error range, in inches
        500, // large error range timeout, in milliseconds
        20 // maximum acceleration (slew)
	);

	lemlib::ControllerSettings angular_controller(
		6, // proportional gain (kP)
		0, // integral gain (kI)
		55, // derivative gain (kD)
		3, // anti windup
		1, // small error range, in inches
		100, // small error range timeout, in milliseconds
		3, // large error range, in inches
		500, // large error range timeout, in milliseconds
		0 // maximum acceleration (slew)
	);

	lemlib::OdomSensors sensors(
		nullptr, // vertical tracking wheel 1, set to null
		nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
		nullptr,
		nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
		&imu // inertial sensor
	);

	lemlib::Chassis chassis(drivetrain, // drivetrain settings
							lateral_controller, // lateral PID settings
							angular_controller, // angular PID settings
							sensors // odometry sensors
	);
	}}