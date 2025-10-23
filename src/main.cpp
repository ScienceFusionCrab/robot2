#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"


	pros::MotorGroup groupL({-1, -2, -3});
	pros::MotorGroup groupR({4, 5, 6});
	pros::Motor chain(10);
	pros::adi::DigitalOut hawk('B', false);
	pros::Imu imu(20);
	bool hawkDown = false;
	pros::adi::DigitalOut tuah('A', false);
	pros::Motor therizzler(9);
	bool tuahDown = false;
	pros::adi::DigitalOut bristol('C', true);
	bool bristolDown = true;

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
		10, // proportional gain (kP)
        0, // integral gain (kI)
        3, // derivative gain (kD)
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


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	//inital setup
		chassis.setPose(0, 0, 0);
		tuahDown = !tuahDown;
		tuah.set_value(tuahDown);
		bristolDown = !bristolDown;
		bristol.set_value(bristolDown);
	//drops the aligner
		chain.move(127);
		pros::delay(250);
		chain.move(0);
	//move to balls
		chassis.moveToPoint(0, 10, 1000, {.maxSpeed = 80});
		chassis.turnToHeading(-90, 500, {.maxSpeed = 80});
		//chassis.setPose(0,0,0);
		chassis.moveToPoint(-24, 10, 4000, {.maxSpeed = 80});
	//suck balls
		pros::delay(2000);
		hawkDown = !hawkDown;
		hawk.set_value(hawkDown);
		chain.move(-127);
		chassis.moveToPoint(-50, 10, 4000);
	//move to goal
		chassis.moveToPoint(-15, 10, 4000, {.forwards = false, .maxSpeed = 100});
		chassis.turnToHeading(0, 1000);
		chassis.moveToPoint(-15, 33, 1000);
		chassis.turnToHeading(90, 2000);
		chassis.moveToPoint(-48, 33, 6000, {.forwards = false, .maxSpeed = 100});
		therizzler.move(-127);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	while (true) {
		int dir = controller.get_analog(ANALOG_LEFT_Y);
			int turn = controller.get_analog(ANALOG_RIGHT_X);
			groupL.move(dir + turn);
			groupR.move(dir - turn);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			chain.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			chain.move(-127);
		} else chain.move(0);

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			hawkDown = !hawkDown;
		}
		hawk.set_value(hawkDown);

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
			tuahDown = !tuahDown;
			pros::delay(50);
			bristolDown = !bristolDown;
		}
		tuah.set_value(tuahDown);
		bristol.set_value(bristolDown);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			therizzler.move(-127);
			chain.move(-127);
		} else {
			therizzler.move(0);
		}

		pros::delay(20);
	}
}