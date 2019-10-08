#ifndef NOSE_H
#define NOSE_H

#include "PID_Vector4f.h"
#include "Type_Change.h"
#include "Nose_config.h"

PID_VECTOR4F pid_zou(matrin(zou_gain));

class NOSE{
    public:
        
        void set_target(int _target);
        
        float process_end();
        
        void feedback();

        void stop();
        
        void print();

    private:
int i=0;
        bool hung;
        
        bool back;
        
        float zou_position;
        
        int zou_speed[4];

        Vector4f vec_zou_position;
        
        Vector4f vec_zou_target;
        
        Vector4f vec_zou_speed;
};

void NOSE::set_target(int _target){
  vec_zou_target << _target, 0.0, 0.0, 0.0;
}

float NOSE::process_end(){
  if(abs(vec_zou_target[0] - vec_zou_position[0]) < zou_range){
    return true;
  }else{
    return false;
  }
}

void NOSE::feedback()
{
    //Read together two data with CAN
    tie(zou_position, ignore) = can.read(41);  // zou_position ID[41]

    //Vector4f conversion
    vec_zou_position << zou_position, 0.0, 0.0, 0.0;

    //calculation PID
    pid_zou.update(vec_zou_position, vec_zou_target, zou_interrupt_time);
    vec_zou_speed = pid_zou.result_val();

    for(i=0; i<4; i++){
      vec_zou_speed[i] =  constrain(vec_zou_speed[i], Min_speed, Max_speed);
    }

    //Vector4f型をint型に変換
    //output_wheel = Vector4f_int(vec_output_wheel); // pointa
    memcpy(zou_speed, Vector4f_int(vec_zou_speed), sizeof(int) * 4);
    
    //send data for motor
    can.dual_send(ID_ZOU_MDD, zou_speed, 2);
}

void NOSE::stop()
{
    vec_zou_speed = Vector4f::Zero();
    zou_speed[0] = 0;
    zou_speed[1] = 0;
    zou_speed[2] = 0;
    zou_speed[3] = 0;
    //send data for motor
    can.dual_send(ID_ZOU_MDD, zou_speed, 2);
}

void NOSE::print()
{
    for(i=0; i<4; i++){
      Serial.print(zou_speed[0]);
      Serial.print(" ");
    }
    Serial.print(zou_position);
      Serial.print(" ");
}

#endif
