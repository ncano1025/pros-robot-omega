#include "main.h"
#include "pros/rtos.hpp"

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


void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

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



//begin fucking around with quaternions

pros::Imu imu(1);

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
			return -1.0;
		}
	}

	//convert quat to yaw
	double yaw = atan2(2 * ((qt.w * qt.z) + (qt.x * qt.y)), 1 - (2 * ((qt.y * qt.y) + (qt.z * qt.z)))); //yaw formula = atan2(2(wz + xy), 1 - 2(y^2 + z^2))

	//returns yaw converted from rad to deg; angle is returned from -180 to 180
	return yaw * (180 / M_PI);
}

void turnAngleQuat(double angle) {
	double initialYaw = getYawQuaternion();
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
    double kP = 1.2;   // Proportional gain (affects how aggressively it turns)
    double kI = 0.01;  // Integral gain (helps correct small errors)
    double kD = 0.4;   // Derivative gain (reduces overshoot)

    double diff = 0; // Difference between target and actual yaw
    double integral = 0, derivative = 0, prevDiff = 0, turnSpeed = 0; //PID components


	while (fabs(diff) > 0.75) { // Loop until within 0.5° of target
        double currentYaw = getYawQuaternion();

        // Calculate shortest turn direction ; normalize -180 to 180
        diff = targetYaw - currentYaw;
        if (diff > 180) diff -= 360; 
        if (diff < -180) diff += 360;

        integral += diff * 0.02;
        derivative = (diff - prevDiff) / 0.02;

        turnSpeed = (kP * diff) + (kI * integral) + (kD * derivative);

        // Limit motor power to avoid excessive speed
        turnSpeed = fmax(fmin(turnSpeed, 0.8), -0.8);

        drive->getModel()->tank(-turnSpeed, turnSpeed);

        prevDiff = diff; // Store previous error
        pros::delay(20); // Small delay for PID loop stability
    }

    // Stop motors when target is reached
    drive->getModel()->stop();
}





void programming_skills() {
	//front right railing, aligned w/ end closest wall 4th triangle

	//backup, turn, get MG
	drive->setMaxVelocity(100);
	drive->moveRaw(-275); //prev 225, 235
	pros::delay(50);
	drive->setMaxVelocity(100);
	drive->turnRaw(130); //162, 167, 172, 149
	pros::delay(50);
	drive->setMaxVelocity(200);
	drive->moveRawAsync(-1200); //1200
	pros::delay(1200);
	piston.set_value(true);
	pros::delay(200);


	//get first ring
	drive->setMaxVelocity(120);
	drive->turnRaw(547); //605
	pros::delay(50);
	drive->setMaxVelocity(250);
	drive->moveRawAsync(900);
	bool intake_running = true;
	while (intake_running) {
		intakingAutoFwd();

		if (intake2.getActualVelocity() == 0) {
			intake2.moveVelocity(-10000);
			pros::delay(100);
			intake2.moveVelocity(10000);
		}

		if (drive->isSettled()) {
			intake_running = false;
		}
	}
	intakingAutoFwd();
	pros::delay(850);

	//turn around and go straight
	drive->setMaxVelocity(100);
	intake1.moveVoltage(0);
	drive->turnRaw(880); //if stops working go 880, 895
	intakingAutoFwd();
	drive->setMaxVelocity(150);
	pros::delay(350);
	drive->moveRawAsync(2750); //return to 2450, 2650-2750
	intake_running = true;
	while (intake_running) {
		intakingAutoFwd();

		if (intake2.getActualVelocity() == 0) {
			intakingAutoBack();
			pros::delay(100);
			intake2.moveVelocity(10000);
		}

		if (drive->isSettled()) {
			intake_running = false;
		}
	}
	pros::delay(250);

	//turn right 90, move forward
	drive->setMaxVelocity(150);
	drive->turnRaw(350); //350
	pros::delay(350);
	drive->setMaxVelocity(250);
	drive->moveRawAsync(1300);
	while (intake_running) {
		intakingAutoFwd();

		if (intake2.getActualVelocity() == 0) {
			intake2.moveVelocity(-10000);
			pros::delay(100);
			intake2.moveVelocity(10000);
		}

		if (drive->isSettled()) {
			intake_running = false;
		}
	}

	//turn left 90, move forward
	pros::delay(2000);
	drive->setMaxVelocity(150);
	drive->turnRaw(-355); //382 or 342
	pros::delay(200);
	drive->moveRawAsync(1600);
	intake_running = true;
	while (intake_running) {
		intakingAutoFwd();

		if (intake2.getActualVelocity() == 0) {
			intake2.moveVelocity(-10000);
			pros::delay(100);
			intake2.moveVelocity(10000);
		}

		if (drive->isSettled()) {
			intake_running = false;
		}
	}

	//ram into wall
	drive->moveRaw(800);
	pros::delay(300);
	intakingAutoBack();
	pros::delay(200);
	intakingAutoFwd();
	drive->moveRaw(-800);

	//deposit MG at corner 1
	pros::delay(300);
	intakingAutoFwd();
	pros::delay(300);
	drive->turnRaw(-680);
	for (int i = 0; i < 5; i++) {
		intakingAutoBack();
		pros::delay(100);
		intakingAutoFwd();
	}
	drive->moveRaw(-1000); //550 or 350
	piston.set_value(false);

	pros::delay(250);

	//turn left, aimed for next
	intakeStop();
	drive->moveRaw(500);
	drive->turnRaw(-165);

	//move to mg2
	drive->setMaxVelocity(300);
	drive->moveRaw(2850);

	//turn and clamp
	pros::delay(250);
	drive->setMaxVelocity(150);
	drive->turnRaw(-360);
	pros::delay(100);
	drive->moveRawAsync(-950);
	pros::delay(1150);
	piston.set_value(true);


	//turn and go forward
	pros::delay(100);
	intakingAutoFwd();
	drive->turnRaw(-588);
	pros::delay(100);
	drive->setMaxVelocity(250);
	drive->moveRaw(900);
	pros::delay(100);
	drive->setMaxVelocity(150);
	pros::delay(100);
	drive->moveRaw(3750);
	pros::delay(100);

	//back up turn around, deposit mg, back up, move forward
	intakeStop();
	drive->moveRaw(-500);
	drive->turnRaw(680);
	drive->moveRaw(-800);
	pros::delay(500);
	piston.set_value(false);
	drive->moveRaw(500);


	//get last mg and try to get the 2 rings under ladder
	pros::delay(100);
	drive->turnRaw(220);
	pros::delay(100);
	drive->setMaxVelocity(300);
	drive->moveRaw(2950);
	pros::delay(100);
	drive->setMaxVelocity(150);
	drive->turnRaw(390);
	pros::delay(100);
	drive->moveRawAsync(-1150);
	pros::delay(1300);
	piston.set_value(true);
	drive->turnRaw(545);
	pros::delay(100);
	drive->moveRaw(1800);
}

void basic_autonomous() {
	bool outtaking;

	// move out
	drive->setMaxVelocity(150);
	drive->moveRaw(610);
	drive->setMaxVelocity(100);
	
	// turn to MG
	drive->setTurnsMirrored(true);
	drive->turnRaw(-335);

	// move to MG
	drive->setMaxVelocity(75);
	drive->moveRawAsync(-395);

	// clamp MG
	pros::delay(1250);
	piston.set_value(true);
	pros::delay(1000);
	
	intakingAutoFwd();

	// move back while running intake
	drive->setMaxVelocity(175);
	drive->moveRaw(1300);

	
	pros::delay(2000);
	intakeStop();

	// turn towards ladder
	drive->turnRaw(575);

	// move toward ladder, stop when intake is settled
	drive->moveRaw(1450);
	
	//outtaking = true;

	//intake.moveVoltage(-12000);
	//pros::delay(5000);
	
	//intake.moveVoltage(0);

	
}

void rush_autonomous() {
	//move forward a bit
	drive->moveRaw(200);

	//turn towards mobile goal
	drive->turnRaw(265);
	//move to corner before MG
	drive->moveRaw(1350);

	//turn backwards
	drive->turnRaw(1535);
	//move back to MG
	drive->moveRaw(-600);

	//grab MG
	pros::delay(500);
	piston.set_value(true);

	//move to red donut
	drive->moveRaw(1150);


}

void autonomous() {
	//basic_autonomous();
	programming_skills();
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
		}
		else
			latch = false; //once button is released then release the latch too

		pros::delay(20); // Run for 20 ms then update
	}
}