#include "main.h"
#include "liblvgl/llemu.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


pros::Imu imu(2);

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	imu.reset();

	pros::lcd::register_btn1_cb(on_center_button);
}

// Chassis instantiation
std::shared_ptr<ChassisController> drive = 
	ChassisControllerBuilder()
		.withMotors(
			{-19, -5, -13},
			{11, 12, 16}
			)
		.withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 10.5_in}, imev5BlueTPR * (600.0 / 450.0)}) // 0.75 = 8_in, 1.0 = 11.5_in, 1.333 = 14_in (SHOULD BE 0.75)
		.build();

// Device instantiation
Motor intake1(-8); //right intake
Motor intake2(20); //left intake
pros::ADIDigitalOut piston('A');



//Intake management functions
void intakingAutoFwd() {
	intake1.moveVoltage(12000);
	intake2.moveVoltage(-12000);
}

void intakingAutoBack() {
	intake1.moveVoltage(-12000);
	intake2.moveVoltage(12000);
}

void driveIntakingForward() {
	intake1.moveVoltage(12000);
	intake2.moveVoltage(-12000);
}

void driveIntakingBackward() {
	intake1.moveVoltage(-12000);
	intake2.moveVoltage(12000);
}

void intakeStop() {
	intake1.moveVoltage(0);
	intake2.moveVoltage(0);
}

void disabled() {}

void competition_initialize() {}

//get the yaw from quaternion

/*	NOTES
imu.get_quaternion returns a STRUCT with w, x, y, z
yaw formula = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
in case ever needed: Qt's Yaw sin component = 2(wz + xy), qt's yaw cos component = 1 - 2(y^2 + z^2)
need to tune PID constants
*/
double getYawQuaternion() {
	pros::quaternion_s_t qt = imu.get_quaternion();

	//error fetching quat, retry
	if (qt.w == PROS_ERR_F) {
		qt = imu.get_quaternion();
		if (qt.w == PROS_ERR_F) {
			pros::lcd::set_text(1, "ERROR: IMU Quaternion Fetch Failed");
			return 360.0;
		}
	}

	//convert quat to yaw
	double yaw = atan2(2 * ((qt.w * qt.z) + (qt.x * qt.y)), 1 - (2 * ((qt.y * qt.y) + (qt.z * qt.z)))); //yaw formula = atan2(2(wz + xy), 1 - 2(y^2 + z^2))

	//returns yaw converted from rad to deg; angle is returned from -180 to 180
	return yaw * (180 / M_PI);
}

double calcAbsAngle(double targetAbsoluteAngle) {
    double currentYaw = getYawQuaternion(); // Get current heading
    pros::lcd::print(3, "Current Yaw: %lf", currentYaw);

    // Normalize target angle to -180 to 180 range
    if (targetAbsoluteAngle > 180) targetAbsoluteAngle -= 360;
    if (targetAbsoluteAngle < -180) targetAbsoluteAngle += 360;

    // Compute shortest turn direction
    double turnAmount = targetAbsoluteAngle - currentYaw;

    // Ensure shortest turn (-180 to 180)
    if (turnAmount > 180) turnAmount -= 360;
    if (turnAmount < -180) turnAmount += 360;

    pros::lcd::print(4, "Target Yaw: %lf | Turn Amt: %lf", targetAbsoluteAngle, turnAmount);
    
    return turnAmount; // Return the relative angle to turn
}


void turnAngleQuat(double angle) {
	double initialYaw = getYawQuaternion();
	if (initialYaw == 360.0) {
		pros::delay(2000);
		pros::lcd::set_text(2, "ERROR: Quaternion Cannot be Fetched");
		return;
	}
	double targetYaw = initialYaw + angle;

	//normalize angles to -180 - 180
	if (targetYaw > 180) {
		targetYaw -= 360;
	}
	if (targetYaw < -180) {
		targetYaw += 360;
	}

	//Custom PID
	//PID constants
    double kP = 0.8;   // Proportional gain (affects how aggressively it turns)
    double kI = 0.001;  // Integral gain (helps correct small errors)
    double kD = 0.15;   // Derivative gain (reduces overshoot)

    double integral = 0, derivative = 0, prevDiff = 0, turnSpeed = 0; //PID components
	double currentYaw = getYawQuaternion();
	double diff = targetYaw - currentYaw; // Difference between target and actual yaw
	int count = 0;

	while (count <= 10) { // Loop until within 0.5° of target
        currentYaw = getYawQuaternion();

        // Calculate shortest turn direction ; normalize -180 to 180
        diff = targetYaw - currentYaw;
		//pros::lcd::print(1, "Cur Diff %lf", diff);

		//normalizes to quaternion angle
        if (diff > 180) diff -= 360; 
        if (diff < -180) diff += 360;

		integral += diff * 0.001;

        derivative = (diff - prevDiff) / 0.3;

        turnSpeed = (kP * diff) + (kI * integral) + (kD * derivative);

        // Limit motor power to avoid excessive speed
        double limSpeed = fmax(0.009, fmin(0.19, fabs(diff) / 225.0)); // Limits speed between .0075 & 0.18

		turnSpeed = (kP * diff) + (kI * integral) + (kD * derivative);

		// Scale down as `diff` decreases
		turnSpeed = fmax(fmin(turnSpeed, limSpeed), -limSpeed);
		//pros::lcd::print(2, "Cur: turnspeed + %lf", turnSpeed);
		
		drive->getModel()->left(turnSpeed);
		drive->getModel()->right(-turnSpeed);

        prevDiff = diff; // Store previous error

		if (fabs(diff) < 0.1) {
			count++;
		} else {
			count = 0;
		}

		
        pros::delay(5); // Small delay for PID loop stability
    }

    // Stop motors when target is reached
	pros::lcd::set_text(1, "TURN COMPLETE!");
    drive->getModel()->stop();
}

void quaternion_testing() {
	pros::lcd::set_text(1, "Calibrating IMU");
	imu.reset();
	pros::delay(3000);
	pros::lcd::set_text(1, "Calibration done");

	//get first mg
	turnAngleQuat(calcAbsAngle(27.25));
	drive->setMaxVelocity(200);
	drive->moveRawAsync(-1300); //1200
	pros::delay(1200);
	piston.set_value(true);
	pros::delay(200);

	//turn and get ring 1
	turnAngleQuat(calcAbsAngle(180));
	pros::delay(200);
	drive->setMaxVelocity(250);
	intakingAutoFwd();
	drive->moveRaw(900);
	pros::delay(200);
	intakeStop();


	//turn to 45, collect 2 & 3
	turnAngleQuat(calcAbsAngle(43.5));
	pros::delay(200);
	intakingAutoFwd();
	drive->moveRaw(2750);
	pros::delay(500);

	//turn right 90, get ring 4
	turnAngleQuat(calcAbsAngle(135));
	pros::delay(200);
	drive->moveRaw(1300);
	pros::delay(200);

	//turn left 90, get ring 5 and 6
	turnAngleQuat(-90);
	pros::delay(200);
	drive->moveRaw(2400);

	//backup and turn 180
	drive->moveRaw(-600);
	pros::delay(200);
	turnAngleQuat(179.8);
	pros::delay(200);
	intakeStop();

	//deposit MG1
	drive->setMaxVelocity(100);
	drive->moveRaw(-650);
	piston.set_value(false);
	intakingAutoBack();
	pros::delay(100);
	intakeStop();
	drive->setMaxVelocity(250);
	drive->moveRaw(600);
	pros::delay(150);

	//move up to next MG
	turnAngleQuat(calcAbsAngle(0.5));
	pros::delay(300);
	drive->moveRaw(-2700);
	pros::delay(200);

	//turn to and clamp MG2
	turnAngleQuat(calcAbsAngle(90));
	drive->moveRawAsync(-1200);
	pros::delay(1050);
	piston.set_value(true);
	pros::delay(100);

	//turn drive straight get 2nd MG 1-4
	turnAngleQuat(calcAbsAngle(-46.5));
	intakingAutoFwd();
	pros::delay(100);
	drive->moveRaw(4850);
	pros::delay(100);

	//turn around and deposit MG2
	drive->moveRaw(-600);
	pros::delay(100);
	turnAngleQuat(179.3);
	pros::delay(100);
	drive->moveRawAsync(-1000);
	pros::delay(1050);
	piston.set_value(false);
	pros::delay(100);
	drive->moveRaw(600);
	pros::delay(100);

	pros::delay(2000);
}

void PID_tuning() {
	turnAngleQuat(-90);
	pros::delay(500);
	turnAngleQuat(179.99);
	pros::delay(500);
	turnAngleQuat(90);
	pros::delay(500);
	turnAngleQuat(45);
	pros::delay(500);
	turnAngleQuat(-45);
	pros::delay(500);
	turnAngleQuat(-179.99);
}

void autonomous() {
	//basic_autonomous();
	if (pros::millis() > 3000) {
		PID_tuning();
	}
}

void opcontrol() {

	bool toggle = false, latch = false;
	float leftY, rightY;
	Controller controller;

	while (true) {
		// may remove this later
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// gathering joystick input for drive
		leftY = controller.getAnalog(ControllerAnalog::leftY);
		rightY = controller.getAnalog(ControllerAnalog::rightY);

		// auton testing
		if(controller.getDigital(ControllerDigital::B)) 
			autonomous();

		// added dampened zone for more precise small movements
		/*
		if((abs(leftY) <= 0.3) || (abs(rightY) <= 0.3)) {
			drive->getModel()->tank(
				leftY * 0.5,
				rightY * 0.5,
				0.05);
		}
		else {
			drive->getModel()->tank(
				leftY,
				rightY);
		}
		*/

		drive->getModel()->tank(
				leftY,
				rightY);

		if (controller.getDigital(ControllerDigital::A)) {
			drive->getModel()->tank(.12, .12);
		}
		if (controller.getDigital(ControllerDigital::Y)) {
			drive->getModel()->tank(.13, .13);
		}
		// intake/indexer
		if(controller.getDigital(ControllerDigital::R1))
			driveIntakingForward();
		else if(controller.getDigital(ControllerDigital::R2)) // reverse (in the event the hook gets caught)
			driveIntakingBackward();
		else 
			intakeStop();

		// clamp toggle
		if (toggle)
			piston.set_value(true); // turns clamp solenoid on
		else
			piston.set_value(false); // turns clamp solenoid off

		if (controller.getDigital(ControllerDigital::L1)) {
			if(!latch){ // if latch is false, flip toggle one time and set latch to true
				toggle = !toggle;
				latch = true;
			}
		} else {
			latch = false; //once button is released then release the latch too
		}

		pros::delay(20); // Run for 20 ms then update
	}
}