#include "config.h"
#include <Teensy_CAN.h>
#include <vector>
#include <Eigen.h>
#include <Eigen/Core>                                                                                                                                       
#include <Eigen/geometry>
#include "Eigen_print.h"
#include "Eigen_calc.h"
#include "PID_Vector4f.h"
#include "Type_Change.h"
#include "move.h"

using namespace std;
using namespace Eigen;

MOVE moving;


int pre_millis=0;
int time_lapse = 0;
int process = 0;
int preprocess = 0;
bool update_sign = false;

void setup() 
{
    // setup can_read and can_send
    can.set();
    Serial.begin(9600);
    
}

void loop() 
{
    //next target location
    if(millis() > 6000){
        if(process == 0 && update_sign == false) {
            moving.set_target(x, y5, 0.0);
            update_sign = true;
        }
        if(process == 1 && update_sign == false && millis() - time_lapse > 3000){
            preprocess = process;
            //reverse(y4.begin(), y4.end());
            //moving.set_target(x, y4, 0.0);
            //update_sign = true;
        }

        if(millis()-pre_millis > (timer * 1000) && update_sign == true){
            pre_millis = millis();
            time_lapse = millis();
            if(moving.update() == true) {
                
                if(process - preprocess > 4){
                    process++;update_sign = false;
                }
            }
        }

    }
   
    //足回りの出力値をCAN-busに送信
    if(millis() > 12000){
        //moving.stop();
    }

    //Show on serial monitor
    //moving.print(2);
    //moving.print(5);
    moving.print(6);Serial.println(process);
    //Serial.println();
    
    delayMicroseconds(500);
}