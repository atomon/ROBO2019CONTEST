//PID制御クラス
//2018.12.27
//大西　凌平
#ifndef PID_VECTOR4F_H
#define PID_VECTOR4F_H

#include <Eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class PID_VECTOR4F
{
    public:

        //空コンストラクタ
        //PID_VECTOR4F(){}
        
        //コンストラクタ
        //kp:比例定数
        //ki:積分定数
        //kd:微分定数
        PID_VECTOR4F(const MatrixXf _pid_gain);        

        //初期化関数
        //コンストラクタで定数を設定しなかった場合や
        //プログラムの途中で定数を変更する場合に使う。
        //kp:比例定数
        //ki:積分定数
        //kd:微分定数
        void set(const MatrixXf _pid_gain);

        //更新関数
        //1ループに一つ入れる。
        //制御周期を固定しない場合。
        //local_val:現在地
        //target_val:目標値
        //戻り値:PID計算結果
        Vector4f update(Vector4f local_val, Vector4f target_val);
        
        //更新関数
        //1ループに一つ入れる。
        //制御周期を固定する場合。
        //local_val:現在地
        //target_val:目標値
        //loop_cycle_ms:制御周期(ミリ秒)
        //戻り値:PID計算結果
        Vector4f update(Vector4f local_val, Vector4f target_val, unsigned long loop_cycle_ms);

        //計算結果を返す関数
        //戻り値:PID計算結果
        Vector4f result_val();

        //積分値をリセットする関数。
        void reset_i();

    private:

        float dt;
    
        unsigned long pretime;

        MatrixXf pid_gain;

        Matrix<float, 3, 4> pid_res, res_prep;
    
        Matrix<float, 4, 4> pid_res2;

        Vector4f result_Value;
    
};


/*
*
*以下関数の実装部
*
*/

PID_VECTOR4F::PID_VECTOR4F(const MatrixXf _pid_gain){
    pid_gain = _pid_gain;
}

void PID_VECTOR4F::set(const MatrixXf _pid_gain){
    pid_gain = _pid_gain;
}

Vector4f PID_VECTOR4F::update(Vector4f local_val, Vector4f target_val){
    dt = (micros() - pretime) / 1000000.0;
    pretime = micros();

    pid_res.row(0)  = target_val - local_val;
    pid_res.row(1) += pid_res.row(0) * dt;
    pid_res.row(2)  = (pid_res.row(0)- res_prep.row(0)) / dt;

    res_prep = pid_res;
    pid_res2 = pid_gain * pid_res;
    result_Value << pid_res2(0,0), pid_res2(1,1), pid_res2(2,2), pid_res2(3,3);
    //print_mtxf(result_Value);
    return result_Value;
}

Vector4f PID_VECTOR4F::update(Vector4f local_val, Vector4f target_val, unsigned long loop_cycle_ms){
    dt = loop_cycle_ms / 1000.0;

    pid_res.row(0)  = target_val - local_val;
    pid_res.row(1) += pid_res.row(0) * dt;
    pid_res.row(2)  = (pid_res.row(0) - res_prep.row(0)) / dt;

    res_prep=pid_res;

    result_Value = pid_res.row(0) + pid_res.row(1) + pid_res.row(2);
    return result_Value;
}

Vector4f PID_VECTOR4F::result_val(){
    return result_Value;
}

void PID_VECTOR4F::reset_i(){
    pid_res.row(1) = Vector4f::Zero();
}

#endif
