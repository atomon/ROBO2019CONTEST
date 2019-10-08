#ifndef NOSE_CONFIG_H
#define NOSE_CONFIG_H

#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;
using namespace Eigen;

#define Max_speed 150
#define Min_speed -500
#define Hung_target 1810
#define Back_target 2000
#define zou_range 10 //±10の誤差
#define zou_interrupt_time 10

int ID_ZOU_MDD[1] {35};  //

float zou_gain[12] {3, 2, 0.0,
                  0.5, 0.3, 0.0,
                  0.7, 0.5, 0.0,
                  0.5, 0.3, 0.0};

#endif