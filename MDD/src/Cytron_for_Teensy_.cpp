/*
  Copyright (c) 2012 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <Arduino.h>
#include "Cytron_for_Teensy.h"

Cytron_T::Cytron_T(int _PWM_PIN, int _invert_PIN, int _PWM_frequency, int _PWM_resolution){
    PWM_PIN = _PWM_PIN;
    invert_PIN = _invert_PIN;
    PWM_frequency = _PWM_frequency;
    PWM_resolution = _PWM_resolution;
}

// Teensy用に周波数変更の関数追加 2019.08.28
void Cytron_T::set(){
    
    #if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    #else
    #error serect Teensy3.x
    #endif
    
    pinMode(PWM_PIN, OUTPUT);
    pinMode(invert_PIN, OUTPUT);
    analogWriteFrequency(PWM_PIN, PWM_frequency);
    analogWriteResolution(PWM_resolution);
    Serial.println(PWM_frequency);
    
}

void Cytron_T::writeMicroseconds(int _pulse){
    pulse = _pulse;
    if(pulse < -25){
        pulse = pulse < -2000 ? -2000 : pulse;
        digitalWrite(invert_PIN, LOW);
        analogWrite(PWM_PIN, map(pulse, -25, -2000, 1, 2047));
    }
    else if(pulse > 25){
        pulse = pulse > 2000 ? 2000 : pulse;
        digitalWrite(invert_PIN, HIGH);
        analogWrite(PWM_PIN, map(pulse, 25, 2000, 1, 2047));
    }
    else{
        analogWrite(PWM_PIN, 0);
    }
}
