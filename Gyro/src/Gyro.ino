#include <Teensy_CAN.h>
TEENSY_CAN can(1000000);

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Gyro.h"

IntervalTimer Interrupt_5ms;
GYRO gyro;

int id[4] {21};      // set ID
int data[4] {};  // initial setting

void gyro_update(){
    gyro.update();
}

void setup(void)
{
    //Serial.begin(250000);

    gyro.set();
    can.set_send();
    delay(100);

    Interrupt_5ms.begin(gyro_update, 5000);  // おかしくなったら4msに変えて
    bno.setExtCrystalUse(true);
}

void loop(void)
{
    
}
