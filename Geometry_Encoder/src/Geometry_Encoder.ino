#include <Encoders.h>
#include <Teensy_CAN.h>

//declare object
ENCODERS myEncoder;
TEENSY_CAN can(1000000); //set baud

//declare array
int id[1] {4}; // set ID
int geometry[2] {};

void setup() {

    can.set_send();

    // setup only can_send
    //Encoder resolution settings
    myEncoder.Encoder3.set(8192);
    myEncoder.Encoder1.set(8192);
}

void loop() {


    //Read rpm
    geometry[0] = myEncoder.Encoder3.read_pulse(); //6 3
    geometry[1] = myEncoder.Encoder1.read_pulse(); //4 1
    

    //Send together two data with CAN
    can.dual_send(id, geometry, 2);  //(ID[array], value[array], number of data)
  
    //Serial monitoring
    /*for(int x : geometry){
        Serial.print(x);
        Serial.print(" ");
    }
    Serial.println();*/

    delayMicroseconds(500);

}
