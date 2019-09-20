/*
 * Cytron_for_Teensy.h
 * 2019-04-27
 */


#ifndef CYTRON_for_Teensy_H
#define CYTRON_for_Teensy_H
#include <Arduino.h>

class Cytron_T {
   public:

      Cytron_T(int _PWM_PIN, int _invert_PIN, int _PWM_frequency = 488.28, int _PWM_resolution = 8);
   
      void set();
   
      void writeMicroseconds(int _pulse);
      
   private:
      int PWM_PIN, invert_PIN;
      int PWM_frequency, PWM_resolution;
      int pulse;
};

#endif
