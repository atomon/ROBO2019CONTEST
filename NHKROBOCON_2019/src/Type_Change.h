#ifndef TYPE_CHANGE_H
#define TYPE_CHANGE_H

#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int Type_change_Vector4f_int[4];
int* Vector4f_int(Vector4f vector4f)
{
    for(int lp = 0; lp <4; lp++){
        Type_change_Vector4f_int[lp] = vector4f[lp];
    }
    return Type_change_Vector4f_int;
}

#endif
