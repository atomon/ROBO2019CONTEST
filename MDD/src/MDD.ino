#include <Teensy_CAN.h>
#include "Cytron_for_Teensy.h"

TEENSY_CAN can(1000000);
Cytron_T motor1(6, 1, 20000, 11); //(PWM_PIN,invert_PIN)
Cytron_T motor2(5, 2, 20000, 11);
Cytron_T motor3(9, 8, 20000, 11);
Cytron_T motor4(10, 11, 20000, 11);
Cytron_T motor5(23, 18, 20000, 11);
Cytron_T motor6(22, 15, 20000, 11);
Cytron_T motor7(21, 16, 20000, 11);
Cytron_T motor8(20, 17, 20000, 11);
//declare array
int val[4] {};

void setup() 
{
    // setup can_read and can_send
    can.set();
    motor1.set();
    motor2.set();
    motor3.set();
    motor4.set();
    motor5.set();
    motor6.set();
    motor7.set();
    motor8.set();
    Serial.begin(9600);
}

void loop() 
{
    //Read together two data with CAN
    tie(val[0], val[1]) = can.read(31);  // tie(Value, Value2) = object name.read(ID);
    tie(val[2], val[3]) = can.read(32);
    tie(val[4], val[5]) = can.read(33);  // tie(Value, Value2) = object name.read(ID);
    tie(val[6], val[7]) = can.read(34);
    tie(val[8], val[9]) = can.read(35);
    
    motor1.writeMicroseconds(val[0]);
    motor2.writeMicroseconds(val[1]);
    motor6.writeMicroseconds(val[2]);
    motor5.writeMicroseconds(val[3]);
    //motor5.writeMicroseconds(val[4]);
    //motor6.writeMicroseconds(val[5]);
    //motor7.writeMicroseconds(val[6]);
    //motor8.writeMicroseconds(val[7]);

    for(int x : val){
        Serial.print(x);
        Serial.print(" ");
    }
    Serial.println();
    
}
