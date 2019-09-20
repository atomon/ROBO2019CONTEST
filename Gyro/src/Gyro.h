#ifndef GYRO_H
#define GYRO_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

class GYRO
{
    private:

        double w, x, y, z;
        
        double roll, pitch, yaw;

        double ysqr;

        double t3, t4;

        int id[1] {21};      // set ID

        int data[1] {};  // initial setting

        int pre=0;

    public:

        GYRO();

        void set();

        void update();
};

GYRO::GYRO(){}

void GYRO::set()
{
    if(!bno.begin(bno.OPERATION_MODE_IMUPLUS)){
        /* There was a problem detecting the BNO055 ... check your connections */
        //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
}

void GYRO::update()
{
    imu::Quaternion quat = bno.getQuat();
 
    /* read the quat data */
    w = quat.w();
    x = quat.x();
    y = quat.y();
    z = quat.z();
    
    ysqr = y * y;
    
    /*
    // roll (x-axis rotation)
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + ysqr);
    roll = atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = asin(t2);*/

    // yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y);
    t4 = +1.0 - 2.0 * (ysqr + z * z);  
    yaw = atan2(t3, t4);

    //定数倍
    //roll *= 57.2957795131;
    //pitch *= 57.2957795131;
    yaw *= 57.2957795131;
    data[0] = yaw * 100000;

    //Send data with CAN
    can.send(id, data, 1);
    
    /*
    Serial.print("qW: ");
    Serial.print(roll, 4);
    Serial.print(" qX: ");
    Serial.print(pitch, 4);
    Serial.print(" qY: ");*/
    /*Serial.print(yaw, 4);
    Serial.print(" ");
    Serial.println(millis()-pre);
    pre = millis();*/
}

#endif
