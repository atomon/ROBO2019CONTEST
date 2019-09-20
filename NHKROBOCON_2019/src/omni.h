#ifndef OMNI_H
#define OMNI_H

#include "config.h"
#include <Servo.h>
#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class OMNI
{
    private:

        int pin1, pin2, pin3, pin4;
    
        Servo md1, md2, md3, md4;

    public:

        OMNI();

        OMNI(const int _pin[]);

        void set();

        void set(const int _pin[]);

        void update(Vector4f w);

        void stop();
        
};

OMNI::OMNI(const int _pin[])
{
    pin1 = _pin[0];
    pin2 = _pin[1];
    pin3 = _pin[2];
    pin4 = _pin[3];
}

void OMNI::set()
{
    md1.attach(pin1); md1.writeMicroseconds(1500);
    md2.attach(pin2); md2.writeMicroseconds(1500);
    md3.attach(pin3); md3.writeMicroseconds(1500);
    md4.attach(pin4); md4.writeMicroseconds(1500);
}

void OMNI::set(const int _pin[])
{
    pin1 = _pin[0];
    pin2 = _pin[1];
    pin3 = _pin[2];
    pin4 = _pin[3];
    md1.attach(pin1); md1.writeMicroseconds(1500);
    md2.attach(pin2); md2.writeMicroseconds(1500);
    md3.attach(pin3); md3.writeMicroseconds(1500);
    md4.attach(pin4); md4.writeMicroseconds(1500);
}

void OMNI::update(Vector4f w)
{
    md1.writeMicroseconds(w[0] + 1500);
    md2.writeMicroseconds(w[1] + 1500);
    md3.writeMicroseconds(w[2] + 1500);
    md4.writeMicroseconds(w[3] + 1500);
}

void OMNI::stop()
{
    md1.writeMicroseconds(1500);
    md2.writeMicroseconds(1500);
    md3.writeMicroseconds(1500);
    md4.writeMicroseconds(1500);
}

#endif
