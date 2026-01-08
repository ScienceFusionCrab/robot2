#include "autos.h"
#include "globals.h"
#include "pros/motors.h"

using namespace Robot;
using namespace Globals;

namespace Robot{ namespace Globals{

void meow() {
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
		chassis.moveToPoint(-15, 35, 1000);
		chassis.turnToHeading(90, 2000);
		chassis.moveToPoint(-48, 35, 8000, {.forwards = false, .maxSpeed = 100});
		therizzler.move(-127);
}

}}