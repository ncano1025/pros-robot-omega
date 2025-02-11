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


pros::ADIAnalogIn sensor('B');
int initSensor;

void initialize() {
	pros::lcd::initialize();
	//pros::lcd::set_text(1, "Hello PROS User!");

	//pros::lcd::register_btn1_cb(on_center_button);

	//dunker.set_brake_mode(MOTOR_BRAKE_HOLD);

	

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
Motor dunker(8);

pros::ADIDigitalOut piston('A');


void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	bool togglePiston = false, latchPiston = false;
	float leftY, rightY;
	// update targetPos based on potentiometer readings
	//double currentPos, targetPos = 1800, inactive = 0, active = 1000;
	int inactive = 672, active = 861, scoring = 1624, targetPos = inactive, currentPos;
	Controller controller;

	//dunker.set_brake_mode(MOTOR_BRAKE_HOLD);

	set_angles(756, 4528, 750);
	
	pros::delay(1000);
	initSensor = sensor.get_value();

	while (true) {

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
		if (togglePiston)
			piston.set_value(true); // turns clamp solenoid on
		else
			piston.set_value(false); // turns clamp solenoid off

		if (controller.getDigital(ControllerDigital::L1)) {
			if(!latchPiston){ // if latch is false, flip toggle one time and set latch to true
				togglePiston = !togglePiston;
				latchPiston = true;
			}
		}
		else
			latchPiston = false; //once button is released then release the latch too

		//currentPos = sensor.get_value() - initSensor;

		// dunker
		/*
		if(controller.getDigital(ControllerDigital::A) && abs(currentPos - active) > 5) {
			if(currentPos < active)
				dunker.move_voltage(10 * abs(currentPos - active) + 1500);
			else
				dunker.move_voltage(0);
		}
		else if(controller.getDigital(ControllerDigital::B) && abs(currentPos - targetPos) > 5) {
			if(currentPos < targetPos)
				dunker.move_voltage(10 * abs(currentPos - targetPos) + 1000);
			else
				dunker.move_voltage(-100);
		}
		else if(controller.getDigital(ControllerDigital::up))
			dunker.move_voltage(500);
		else if(controller.getDigital(ControllerDigital::down))
			dunker.move_voltage(-1000);
		else
			dunker.brake();
		*/

		currentPos = sensor.get_value() - initSensor;

		if(controller.getDigital(ControllerDigital::B))
			targetPos = inactive;
		else if(controller.getDigital(ControllerDigital::Y))
			targetPos = active;
		else if(controller.getDigital(ControllerDigital::X))
			targetPos = scoring;

		dunker.moveVoltage(feedforward_kV(targetPos, currentPos) + feedforward_kG(currentPos));

		pros::delay(20); // Run for 20 ms then update

		// print out current and target values for dunker
		pros::lcd::print(0, "Initial - %d", initSensor);
		pros::lcd::print(1, "Sensor Val - %d", sensor.get_value());
		
		pros::lcd::print(2, "kG - %d", feedforward_kG(sensor.get_value()));
		pros::lcd::print(3, "kV - %d", feedforward_kV(targetPos, sensor.get_value()));
		pros::lcd::print(4, "target Position - %d", targetPos);
	}
}