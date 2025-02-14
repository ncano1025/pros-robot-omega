/*
#include "api.h"

int port = 0, left_turn_deg = -90, right_turn_deg = 90, turn_speed = 100, target_yaw;
bool turning = false;
pros::Imu imu(port);

double get_turn_speed(int start, int end, int velocity){
    return (end - start) * velocity;
}

void basic_autonomous(){
    // move out
	drive->setMaxVelocity(75);
    drive->moveRaw(600);

    // turn towards MG w "PID"
    turning = true;
    target_yaw = imu.get_yaw() + left_turn_deg;

    while(turning == true){
        drive->getModel()->tank(
            -get_turn_speed(imu.get_yaw(), target_yaw, turn_speed), 
            get_turn_speed(imu.get_yaw(), target_yaw, turn_speed)
        );

        if(get_turn_speed(imu.get_yaw(), target_yaw, turn_speed) == 0)
            turning = false;
    }
}
*/