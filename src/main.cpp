#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"


	pros::MotorGroup groupL({16, 17, 18});
	pros::MotorGroup groupR({-13, -14, -15});
	pros::Motor chain({8});		 //intake
	pros::adi::DigitalOut hawk('F', false);	 //match loader
    bool hawkDown = false;
	pros::Imu imu(12);
	pros::adi::DigitalOut tuah('H', false);	//upsies downsies
    bool tuahDown = false;
	pros::Motor therizzler(10);				//wedge
	pros::adi::DigitalOut bwomp('G',false); //descore
	bool bwompDown = false;

	// drivetrain settings
lemlib::Drivetrain drivetrain(&groupL, // left motor group
                              &groupR, // right motor group
                              10.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              6 // horizontal drift is 2 (for now)
);

lemlib::TrackingWheel leftDrivetrainTracking(
	&groupL,
	lemlib::Omniwheel::NEW_275,
	-5.25,
	450
);

lemlib::TrackingWheel rightDrivetrainTracking(
	&groupR,
	lemlib::Omniwheel::NEW_275,
	5.25,
	450
);
lemlib::OdomSensors sensors(&leftDrivetrainTracking, // vertical tracking wheel 1, set to null
                            &rightDrivetrainTracking, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	10, // proportional gain (kP)
    0, // integral gain (kI)
    0, // derivative gain (kD)
    0, // anti windup
    0, // small error range, in inches
    0, // small error range timeout, in milliseconds
    0, // large error range, in inches
    0, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
	-2, // proportional gain (kP)
    0, // integral gain (kI)
    -3.45, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void on_center_button() {}

// this runs at the start of the program
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
            pros::delay(200);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	/*initial
		chassis.setPose(0, 0, 0);
		chassis.moveToPoint(0, 30, 3000, {.maxSpeed = 80});
		chain.move(127);
		therizzler.move(-127);
		pros::delay(3000);
	*/
	chassis.setPose(0,0,0);
	groupL.move(127);
	groupR.move(127);
	pros::delay(1000);
	therizzler.move(127);
	
}

void opcontrol() {
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	while (true) {
		int dir = controller.get_analog(ANALOG_LEFT_Y);
			int turn = controller.get_analog(ANALOG_RIGHT_X);
			groupL.move(dir - turn);
			groupR.move(dir + turn);

		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			chain.move(127);
			therizzler.move(127);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			chain.move(-127);
			therizzler.move(-127);
		} else {
			chain.move(0);
			therizzler.move(0);
		}

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) && !tuahDown){
			hawkDown = !hawkDown;
		}
		hawk.set_value(hawkDown);

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) && !hawkDown) {
			tuahDown = !tuahDown;
			pros::delay(50);
			tuah.set_value(tuahDown);
		} 

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
			bwompDown = !bwompDown;
		} bwomp.set_value(bwompDown);

		pros::delay(20);

	}
}