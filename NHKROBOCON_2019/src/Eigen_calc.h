/*
 * 
 * 
 */ 


#ifndef EIGEN_CALC_H
#define EIGEN_CALC_H

#include "config.h"
#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class EIGEN_CALC
{
    private:
    
        float wheel_R = 0.1016;
        
        float body_R = 0.59459; //中心から移動輪までの距離

        float body_L = 0.303; //中心から計測林までの距離
        
        float fai = M_PI/4;
        
        float theta; // Angle in robot coordinates system

        Vector3f vec_velocity;

    public:

        EIGEN_CALC();

        Vector3f location(Vector4f vec_odometrys, float previons_angle);

        Vector4f wheel(Vector3f vec_loocal, Vector3f vec_target);
        
};

EIGEN_CALC::EIGEN_CALC(){}

Vector3f EIGEN_CALC::location(Vector4f vec_odometrys, float previons_angle)
{
    MatrixXf mat3(3,4);
    MatrixXf mat4(3,3);

    mat3 <<        0,       -1,        0,        1,
                   1,        0,       -1,        0,
            body_L/2, body_L/2, body_L/2, body_L/2;

    mat4 <<  cos(previons_angle), -sin(previons_angle), 0,
             sin(previons_angle),  cos(previons_angle), 0,
                               0,                    0, 1;
    
    Vector3f moving_speed = (mat3 * vec_odometrys) / 2;
    Vector3f D_location = mat4 * moving_speed; // 移動速度を時間微分

    return D_location;

}

Vector4f EIGEN_CALC::wheel(Vector3f vec_loocal, Vector3f vec_target)
{
    theta = vec_loocal[2]; // gyro時10倍
    vec_velocity = vec_target - vec_loocal;
    vec_velocity /= (location_interrupt_timer / 1000);

    MatrixXf mat1(4,3);        
    MatrixXf mat2(3,3);

    mat1 << -sin(fai),  cos(fai), body_R,
             sin(fai),  cos(fai), body_R,
             sin(fai), -cos(fai), body_R,
             -sin(fai), -cos(fai), body_R;
    mat2 <<  cos(theta), sin(theta),   0,
            -sin(theta), cos(theta),   0,
                      0,          0,   1;
    
    Vector4f t = mat1 * mat2 * vec_velocity;
    //print_mtxf(t);

    //周速度からrpmに変換
    t *= 60/(PI*wheel_R);

    return t;
}

#endif
