#include "main.h"

// Chassis instantiation
std::shared_ptr<ChassisController> drive = 
	ChassisControllerBuilder()
		.withMotors(
			{-7, -8, -9},
			{4, 5, 6}
			)
		.withDimensions(AbstractMotor::gearset::blue, {{3.25, 10.5}, imev5BlueTPR})
		.build();

// Device instantiation
Motor intake(19);
pros::ADIDigitalOut piston = pros::ADIDigitalOut('A');

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

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

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

	bool toggle = false, latch = false;

	Controller controller;

	while (true) {
		// may remove this later
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		drive->getModel()->tank(
			controller.getAnalog(ControllerAnalog::leftY),
			controller.getAnalog(ControllerAnalog::rightY));

		// intake/indexer
		if(controller.getDigital(ControllerDigital::R1))
			intake.moveVoltage(12000);
		else if(controller.getDigital(ControllerDigital::R2)) // reverse (in the event the hook gets caught)
			intake.moveVoltage(-12000);
		else 
			intake.moveVelocity(0);

		// clamp toggle
		if (toggle){
			piston.set_value(true); // turns clamp solenoid on
		}
		else {
			piston.set_value(false); // turns clamp solenoid off
		}

		if (controller.getDigital(ControllerDigital::L1)) {
			if(!latch){ // if latch is false, flip toggle one time and set latch to true
				toggle = !toggle;
				latch = true;
			}
		}
		else {
			latch = false; //once button is released then release the latch too
		}

		pros::delay(20); // Run for 20 ms then update
	}
}