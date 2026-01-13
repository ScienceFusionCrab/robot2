#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"


	pros::MotorGroup groupL({16, -17, 18});
	pros::MotorGroup groupR({13, -14, 15});
	pros::MotorGroup chain({20, 9});		 //intake
	pros::adi::DigitalOut hawk('G', false);	 //upsies downsies
    bool hawkDown = false;
	pros::Imu imu(12);
	pros::adi::DigitalOut tuah('H', false);	//match loader
    bool tuahDown = false;
	pros::Motor therizzler(10);				//wedge
	pros::adi::DigitalOut descore('F',false);
	bool descoreDown = false;

	lemlib::Drivetrain drivetrain(&groupL, 	// left motor group
        &groupR, 							// right motor group
        10.5, 								// 10.5 inch track width
        lemlib::Omniwheel::NEW_275, 		// using new 2.75" omnis
        450, 								// drivetrain rpm is 360
        2 									// horizontal drift is 2 (for now)
	);

	lemlib::ExpoDriveCurve steer_curve(3,	// joystick deadband out of 127
        10, 								// minimum output where drivetrain will move out of 127
        1 									// expo curve gain
	);

	lemlib::ExpoDriveCurve throttle_curve(3, 	// joystick deadband out of 127
    	10, 									// minimum output where drivetrain will move out of 127
    	1 										// expo curve gain
	);

	lemlib::ControllerSettings lateral_controller(
		12,		// proportional gain (kP)
        0, 		// integral gain (kI)
        8, 		// derivative gain (kD)
        3, 		// anti windup
        1, 		// small error range, in inches
        100, 	// small error range timeout, in milliseconds
        3, 		// large error range, in inches
        500, 	// large error range timeout, in milliseconds
        20 		// maximum acceleration (slew)
	);

	lemlib::ControllerSettings angular_controller(
		6, 		// proportional gain (kP)
		0, 		// integral gain (kI)
		55, 	// derivative gain (kD)
		3, 		// anti windup
		1, 		// small error range, in inches
		100, 	// small error range timeout, in milliseconds
		3, 		// large error range, in inches
		500,	// large error range timeout, in milliseconds
		0 		// maximum acceleration (slew)
	);

	lemlib::OdomSensors sensors(
		nullptr, 	// vertical tracking wheel 1, set to null
		nullptr, 	// vertical tracking wheel 2, set to nullptr as we are using IMEs
		nullptr,
		nullptr, 	// horizontal tracking wheel 2, set to nullptr as we don't have a second one
		&imu 		// inertial sensor
	);

	lemlib::Chassis chassis(drivetrain, // drivetrain settings
		lateral_controller, 			// lateral PID settings
		angular_controller, 			// angular PID settings
		sensors 						// odometry sensors
	);

void on_center_button() {}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	//initial
		chassis.setPose(0, 0, 0);
		hawkDown = !hawkDown;
		hawk.set_value(hawkDown);
	//move to blocks
		chassis.moveToPoint(0, 10, 1000, {.maxSpeed = 80});
		chassis.turnToHeading(-90, 500, {.maxSpeed = 80});
		chassis.moveToPoint(-24, 10, 4000, {.maxSpeed = 80});
	//suck blocks
		pros::delay(1000);
		tuahDown = !tuahDown;
		tuah.set_value(tuahDown);
		chain.move(-127);
		chassis.moveToPoint(-50, 10, 4000);
	//move to long goal
		chassis.moveToPoint(-15, 10, 4000, {.forwards = false, .maxSpeed = 100});
		chassis.turnToHeading(0, 1000);
		chassis.moveToPoint(-15, 35, 1000);
		chassis.turnToHeading(90, 2000);
		chassis.moveToPoint(-48, 35, 8000, {.forwards = false, .maxSpeed = 100});
	//deposit in long goal
		therizzler.move(-127);
		chain.move(127);
		pros::delay(2000);
	//push further in
		chassis.moveToPoint(-15, 25, 1000, {.maxSpeed = 100});
		chassis.turnToHeading(180, 1000);
		chassis.moveToPoint(-15, 12, 1000);
		chassis.turnToHeading(-90, 1000);
		chassis.moveToPoint(-48, 35, 1000);
		descoreDown = !descoreDown;
		descore.set_value(descoreDown);
}

void opcontrol() {
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	while (true) {
		int dir = controller.get_analog(ANALOG_LEFT_Y);
			int turn = controller.get_analog(ANALOG_RIGHT_X);
			groupL.move(dir + turn);
			groupR.move(dir - turn);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			therizzler.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			therizzler.move(-127);
		} else chain.move(0);

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			hawkDown = !hawkDown;
		}
		hawk.set_value(hawkDown);

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
			tuahDown = !tuahDown;
			pros::delay(50);
		}
		tuah.set_value(tuahDown);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			therizzler.move(-127);
			chain.move(-127);
		} else {
			chain.move(0);
		}

		pros::delay(20);
	}
}