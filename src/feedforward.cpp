#include "api.h"

int startAngle, endAngle, baseVolt;
double radianConversion, unitPos;

void set_angles(int start, int end, int voltage){
    startAngle = start;
    endAngle = end;
    baseVolt = voltage;
    radianConversion = (end - start) / (2 * 3.14159);
}

int feedforward_kV(int targetAngle, int currentAngle){
    //10 * abs(currentPos - active) + 1500;

    return 2 * (targetAngle - currentAngle);
}


int feedforward_kG(int angle){
    unitPos = cos((angle - startAngle)/radianConversion);
    
    return (int)(unitPos * baseVolt);
}