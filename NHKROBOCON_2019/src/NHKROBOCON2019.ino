/*
 * 
 * 
 */ 

#include <Teensy_CAN.h>
TEENSY_CAN can(1000000); // baud setting
#include "config.h"
#include <vector>
#include <Eigen.h>
#include <Eigen/Core>                                                                                                                                       
#include <Eigen/geometry>
#include "Eigen_print.h"
#include "Eigen_calc.h"
#include "PID_Vector4f.h"
#include "Type_Change.h"
#include "move.h"
#include "Nose.h"

using namespace std;
using namespace Eigen;

MOVE moving;
NOSE nose;

#include "Timer_interrupt.h"


int pre_millis=0;
int time_lapse = 0;
int process = 0;
int wheel_process = 0;
int wheel_preprocess = 0;
bool wheel_sign = false;
bool zou_sign = false; // 象のステータス　true : ON , false : OFF
bool go = false;
int zone = -1;
int mode = 0b11;

void setup() 
{
    // setup can_read and can_send
    can.set();
    //while(millis() < 5000){}
    Serial.begin(9600);
    pinMode(13, OUTPUT);
    pinMode(14, INPUT_PULLUP);
    pinMode(15, INPUT_PULLUP);
    pinMode(16, INPUT_PULLUP);
}

void loop() 
{
    
    //next target location
    if(digitalRead(14)) go = true;
    //if(digitalRead(13)) zone = 1;
    //else zone = -1;  // push
    zone=-1;
    if(digitalRead(15)) mode |= 0b01; //push
    else mode &= 0b10;
    if(digitalRead(16) == false) mode |= 0b10; //push
    else mode &= 0b01;
    Serial.println();

    if(go == true && mode == 0b10){
        if(process == 0 && wheel_sign == false) {
            moving.set_target(X11, Y11, 0.0, zone);
            pid_wheel.reset_i();
            wheel_sign = true;
            begin_interval_wheel();
        }
        if(process == 1 && zou_sign == false){
            nose.set_target(Hung_target);
            pid_zou.reset_i();
            zou_sign = true;
            begin_interval_zou();
        }
        if(process == 2 && zou_sign == false && millis() - time_lapse > 500){
            nose.set_target(Back_target);
            pid_zou.reset_i();
            zou_sign = true;
            begin_interval_zou();
        }
        if(process == 3 && wheel_sign == false && millis() - time_lapse > 1000){
            moving.set_target(bX11, bY11, 0.0, zone);
            pid_wheel.reset_i();
            wheel_sign = true;
            begin_interval_wheel();
        }


        // update sign check
        // wheel process status
        if(wheel_sign == true && moving.process_end() == true && millis() - pre_millis >200) {
            pre_millis = millis();
            if(++wheel_process - wheel_preprocess > 4){
                end_interval_wheel();
                moving.stop();
                wheel_sign = false;
                wheel_process = 0;
                wheel_preprocess = 0;
                time_lapse = millis();
                process++;
            }
        }
        
        // zou process status
        if(zou_sign == true && nose.process_end() == true){
            end_interval_zou();
            nose.stop();
            zou_sign = false;
            time_lapse = millis();
            process++;
        }
    }


    if(go == true && mode == 0b01){
        if(process == 0 && wheel_sign == false) {
            moving.set_target(X12, Y12, 0.0, zone);
            pid_wheel.reset_i();
            wheel_sign = true;
            begin_interval_wheel();
        }if(process == 1 && zou_sign == false){
            nose.set_target(Hung_target);
            pid_zou.reset_i();
            zou_sign = true;
            begin_interval_zou();
        }
        if(process == 2 && zou_sign == false && millis() - time_lapse > 500){
            nose.set_target(Back_target);
            pid_zou.reset_i();
            zou_sign = true;
            begin_interval_zou();
        }
        if(process == 3 && wheel_sign == false && millis() - time_lapse > 3000){
            moving.set_target(bX12, bY12, 0.0, zone);
            pid_wheel.reset_i();
            wheel_sign = true;
            begin_interval_wheel();
        }



        // wheel process status
        if(wheel_sign == true && moving.process_end() == true && millis() - pre_millis >200) {
            pre_millis = millis();
            if(++wheel_process - wheel_preprocess > 4){
                end_interval_wheel();
                moving.stop();
                wheel_sign = false;
                wheel_process = 0;
                wheel_preprocess = 0;
                time_lapse = millis();
                process++;
            }
        }

        // zou process status
        if(zou_sign == true && nose.process_end() == true){
            end_interval_zou();
            nose.stop();
            zou_sign = false;
            time_lapse = millis();
            process++;
        }
    }

    if(go == true && mode == 0b11){
        if(process == 0 && wheel_sign == false) {
            moving.set_target(X21, Y21, 0.0, zone);
            pid_wheel.reset_i();
            wheel_sign = true;
            begin_interval_wheel();
        }

        if(process == 1 && wheel_sign == false && millis() - time_lapse > 3000){
            moving.set_target(X22, Y22, 0.0, zone);
            pid_wheel.reset_i();
            wheel_sign = true;
            begin_interval_wheel();
        }

        if(process == 2 && wheel_sign == false && millis() - time_lapse > 3000){
            reverse(X22.begin(), X22.end()); reverse(Y22.begin(), Y22.end());
            moving.set_target(X22, Y22, 0.0, zone);
            pid_wheel.reset_i();
            wheel_sign = true;
            begin_interval_wheel();
        }

        if(process == 3 && wheel_sign == false && millis() - time_lapse > 3000){
            moving.set_target(bX21, bY21, 0.0, zone);
            pid_wheel.reset_i();
            wheel_sign = true;
            begin_interval_wheel();
        }

        // wheel process status
        if(wheel_sign == true && moving.process_end() == true && millis() - pre_millis >200) {
            pre_millis = millis();
            if(++wheel_process - wheel_preprocess > 4){
                end_interval_wheel();
                moving.stop();
                wheel_sign = false;
                wheel_process = 0;
                wheel_preprocess = 0;
                time_lapse = millis();
                process++;
            }
        }

    }

    digitalWrite(13, wheel_sign);

    //Show on serial monitor
    //moving.print(2);
    //moving.print(5);
    moving.print(1);Serial.println(process);
    //Serial.println();
    
    delayMicroseconds(500);
}
