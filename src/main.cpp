#include "main.h"

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
			{-7, -8, -9},
			{4, 5, 6}
			)
		.withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 10.5_in}, imev5BlueTPR * (600.0 / 450.0)}) // 0.75 = 8_in, 1.0 = 11.5_in, 1.333 = 14_in (SHOULD BE 0.75)
		.build();

// Device instantiation
Motor intake(19);
pros::ADIDigitalOut piston('A');

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	bool toggle = false, latch = false;
	float leftY, rightY;
	Controller controller;

	while (true) {
		// may remove this later
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		leftY = controller.getAnalog(ControllerAnalog::leftY);
		rightY = controller.getAnalog(ControllerAnalog::rightY);

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

		// intake/indexer
		if(controller.getDigital(ControllerDigital::R1))
			intake.moveVelocity(170);
		else if(controller.getDigital(ControllerDigital::R2)) // reverse (in the event the hook gets caught)
			intake.moveVelocity(-170);
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