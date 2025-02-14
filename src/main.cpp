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
			{-21, -15, -14},
			{13, 12, 11}
			)
		.withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 10.5_in}, imev5BlueTPR * (600.0 / 450.0)}) // 0.75 = 8_in, 1.0 = 11.5_in, 1.333 = 14_in (SHOULD BE 0.75)
		.build();

// Device instantiation
Motor intake(16);
pros::ADIDigitalOut piston('A');

void disabled() {}

void competition_initialize() {}

int port = 6;
double left_turn_deg = -90, right_turn_deg = 90, turn_speed = 100, target_yaw;
bool turning = false;
pros::Imu imu(port);

void programming_skills() {

	bool intaking = true;

	//move out
	drive->setMaxVelocity(100);
	drive->moveRaw(-335);

	// turn to MG
	drive->setTurnsMirrored(true);
	drive->turnRaw(-120);

	drive->moveRawAsync(-1275);

	// clamp MG
	pros::delay(2500);
	piston.set_value(true);
	pros::delay(1000);

	// turn to get the 3-ring
	drive->turnRaw(-560);

	// start intake and move
	intake.moveVoltage(12000);

	// move until red ring
	drive->setMaxVelocity(150);
	drive->moveRaw(1550);
	
	pros::delay(1500);
	intake.moveVoltage(0);

	// grab blue ring
	intake.moveVoltage(6000);

	drive->moveRaw(300);
	intake.moveVoltage(0);

	// turn 90
	drive->setMaxVelocity(100);
	drive->turnRaw(-345);

	// outtake blue ring
	intake.moveVoltage(-12000);

	pros::delay(2000);
	intake.moveVoltage(0);

	// turn back
	drive->turnRaw(330);

	// start intake and move
	intake.moveVoltage(12000);

	// move until 3rd red ring
	drive->setMaxVelocity(150);
	drive->moveRaw(500);
	
	pros::delay(1500);
	intake.moveVoltage(0);

	// turn around
	drive->setMaxVelocity(100);
	drive->turnRaw(675);

	// move back to old pos of MG
	drive->setMaxVelocity(150);
	drive->moveRaw(2400);

	// turn toward 4th ring
	drive->setMaxVelocity(100);
	drive->turnRaw(-335);

	// move to and grab 4th ring
	drive->setMaxVelocity(150);
	drive->moveRawAsync(800);

	intaking = true;

	while (intaking) {
		intake.moveVoltage(12000);

		if (intake.getActualVelocity() == 0){
			intake.moveVoltage(-6000);
			pros::delay(500);
		}

		if (drive->isSettled()){
			intaking = false;
		}
	}

	pros::delay(2000);
	intake.moveVoltage(0);

	// turn towards corner
	drive->setMaxVelocity(100);
	drive->turnRaw(500); // tune num

	// move and intake last two rings
	drive->moveRawAsync(2000);
	
	intaking = true;

	while (intaking) {
		intake.moveVoltage(12000);

		if (intake.getActualVelocity() == 0){
			intake.moveVoltage(-12000);
			pros::delay(500);
		}

		if (drive->isSettled()){
			intaking = false;
		}
	} 

	pros::delay(2000);
	intake.moveVoltage(0);

	// back up
	drive->moveRaw(-300);

	// turn MG towards corner
	drive->setMaxVelocity(100);
	drive->turnRaw(-670); // tune num

	// move MG into corner
	drive->moveRaw(-300); // tune num

	// release MG
	piston.set_value(false);

	
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
	
	intake.moveVoltage(12000);

	// move back while running intake
	drive->setMaxVelocity(175);
	drive->moveRaw(1300);

	
	pros::delay(2000);
	intake.moveVoltage(0);

	// turn towards ladder
	drive->turnRaw(575);

	// move toward ladder, stop when intake is settled
	drive->moveRaw(1450);
	
	//outtaking = true;

	//intake.moveVoltage(-12000);
	//pros::delay(5000);
	
	//intake.moveVoltage(0);
}

void basic_3p_autonomous() {
	bool outtaking, intaking;

	// move out
	drive->setMaxVelocity(150);
	drive->moveRaw(610);
	
	
	// turn to MG
	drive->setTurnsMirrored(true);
	drive->setMaxVelocity(100);
	drive->turnRaw(-340);

	// move to MG
	drive->setMaxVelocity(75);
	drive->moveRawAsync(-395);

	// clamp MG
	pros::delay(1250);
	piston.set_value(true);
	pros::delay(1000); 

	drive->moveRaw(-725);

	intake.moveVoltage(0);

	// INSERT EXTRA RING HERE
	drive->setMaxVelocity(100);
	drive->turnRaw(-150);

	drive->setMaxVelocity(150);

	intake.moveVoltage(12000);
	drive->moveRaw(300);

	pros::delay(2000);
	intake.moveVoltage(0);

	drive->setMaxVelocity(100);
	drive->turnRaw(160);
	
	
	intake.moveVoltage(12000);

	// move back while running intake
	drive->setMaxVelocity(175);
	drive->moveRaw(2000);

	
	pros::delay(2000);
	intake.moveVoltage(0);

	// turn towards ladder
	drive->turnRaw(550);

	// move toward ladder, stop when intake is settled
	drive->moveRaw(1525);
	
	//outtaking = true;

	//intake.moveVoltage(-12000);
	//pros::delay(5000);
	
	//intake.moveVoltage(0);
	
}

void rush_autonomous() {
	drive->setMaxVelocity(250);

	//move forward a bit
	drive->moveRaw(200);

	drive->setMaxVelocity(100);

	//turn towards mobile goal
	drive->turnRaw(265);

	drive->setMaxVelocity(250);

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
	//programming_skills();
	basic_autonomous();
	//basic_3p_autonomous();
	//rush_autonomous();
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
			intake.moveVoltage(12000);
		else if(controller.getDigital(ControllerDigital::R2)) // reverse (in the event the hook gets caught)
			intake.moveVoltage(-12000);
		else 
			intake.moveVelocity(0);

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