#ifndef MOVE_H
#define MOVE_H

#include "config.h"
#include <Teensy_CAN.h>
#include <vector>
#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Eigen_print.h"
#include "Eigen_calc.h"
#include "PID_Vector4f.h"
#include "Type_Change.h"

using namespace std;
using namespace Eigen;

//declare object
TEENSY_CAN can(1000000); // baud setting
PID_VECTOR4F pid_vec4f(matrin(arrays));
EIGEN_CALC eigen_calc;

class MOVE
{
    private:

        vector<int> location_x, location_y;

        //declare array
        int wheel_outrpm[4];

        float wheel_rpm[4];
        
        float odometry[4];
        
        Vector4f vec_wheel_rpm;
        
        Vector4f vec_odometry, pre_vec_odometry;
        
        Vector3f vec_location;
        
        Vector3f vec_location_target;
        
        Vector4f vec_target_rpm, vec_wheel_outrpm;

        float odometry_R = 0.159592907;

        float theta = 0.0;

        float target_theta = 0.0;

        float pre_theta = 0.0;

        int i = 1;

    public:

        MOVE();

        void set_target(vector<int> _x, vector<int> _y, float _theta);

        bool update();

        bool process_end();

        void stop();

        void print(int num);
        
};

MOVE::MOVE(){}

void MOVE::set_target(vector<int> _x, vector<int> _y, float _theta)
{
    location_x = _x;
    location_y = _y;
    target_theta = _theta;
    i = 1;
}

bool MOVE::update()
{
    //Read together two data with CAN
    tie(wheel_rpm[0], wheel_rpm[1]) = can.read(1);  // tie(Value, Value2) = object name.read(ID);
    tie(wheel_rpm[2], wheel_rpm[3]) = can.read(2);
    tie(odometry[1], odometry[0]) = can.read(3);
    tie(odometry[3], odometry[2]) = can.read(4);
    tie(theta, ignore) = can.read(21);
    //tie(analog_r[0], analog_r[1]) = can.read(22);
    //tie(analog_r[2], analog_r[3]) = can.read(23);

    //Vector4f conversion
    Map<Vector4f> vec_wheel_rpm(wheel_rpm);
    Map<Vector4f> vec_odometry(odometry);

    //current location
    theta /= 100000;
    theta *= PI/180;
    //vec_location << odometry[0] * (odometry_R / 8192.0), odometry[1] * (odometry_R / 8192.0), theta;
    vec_location += eigen_calc.location((vec_odometry - pre_vec_odometry) * (odometry_R / 8192.0), theta); //(vec_odometry - pre_vec_odometry) * (odometry_R / 8192.0), theta
    vec_location[2] = theta;

    //target location
    if(i > (int)location_y.size()-1) i = (int)location_y.size()-1;
    vec_location_target << location_x[i] * (odometry_R / 8192.0), location_y[i] * (odometry_R / 8192.0), target_theta;

    //目標値についたか？
    if(MOVE::process_end() == true){
        MOVE::stop();
        pid_vec4f.reset_i();
        return true;
    } 

    //各ホイールの回転数を計算
    vec_target_rpm = eigen_calc.wheel(vec_location, vec_location_target);
    pid_vec4f.update(vec_wheel_rpm, vec_target_rpm);
    vec_wheel_outrpm = pid_vec4f.result_val();

    //Vector4f型をint型に変換
    //output_wheel = Vector4f_int(vec_output_wheel); // pointa
    memcpy(wheel_outrpm, Vector4f_int(vec_wheel_outrpm), sizeof(int) * 4);

    //send data for motor
    can.dual_send(ID_MDD, wheel_outrpm, 4);

    i++;
    pre_vec_odometry = vec_odometry;
    pre_theta = theta;
    
    return 0;
}

bool MOVE::process_end()
{
    int val = 0b000;
    val |= ((odometry[0] < location_x.back() + error_range) && (odometry[0] > location_x.back() - error_range))? 0b001 : 0b000;
    val |= ((odometry[1] < location_y.back() + error_range) && (odometry[1] > location_y.back() - error_range))? 0b010 : 0b000;
    val |= ((theta < target_theta + error_range_angle) && (theta > target_theta - error_range_angle))? 0b100 : 0b000;
    
    return val == 0b111 ? true : false;

}

void MOVE::stop()
{
    vec_target_rpm = Vector4f::Zero();
    wheel_outrpm[0] = 0;
    wheel_outrpm[1] = 0;
    wheel_outrpm[2] = 0;
    wheel_outrpm[3] = 0;
    can.dual_send(ID_MDD, wheel_outrpm, 4);
}

void MOVE::print(int num)
{
    switch (num)
    {
    case 1: for(int kari : wheel_rpm){
                Serial.print(kari);
                Serial.print(" ");
            }
            break;

    case 2: for(int kari : odometry){
                Serial.print(kari);
                Serial.print(" ");
            }
            break;

    case 3: print_mtxf(vec_location); break;

    case 4: print_mtxf(vec_location_target); break;

    case 5: print_mtxf(vec_target_rpm); break;

    case 6: for(int kari : wheel_outrpm){
                Serial.print(kari);
                Serial.print(" ");
            }
            break;

    case 7: Serial.print(vec_location[0],5);
            Serial.print(" ");
            Serial.print(vec_location[1],5);
            Serial.print(" ");
            Serial.print(odometry[1] * (odometry_R / 8192.0),5);
            Serial.print(" ");
            Serial.print(odometry[0] * (odometry_R / 8192.0),5);
            Serial.print(" ");
    }
    //Serial.println();
}

#endif
