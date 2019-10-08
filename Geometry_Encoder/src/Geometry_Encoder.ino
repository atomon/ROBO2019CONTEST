#include <Encoders.h>
#include <Teensy_CAN.h>

//declare object
ENCODERS myEncoder;
TEENSY_CAN can(1000000); //set baud
IntervalTimer Interrupt_5ms;

//declare array
int id[1] {3}; // set ID
int geometry[2] {};

void update(){

    //Read rpm
    geometry[0] = myEncoder.Encoder3.read_pulse(); //3 6
    geometry[1] = myEncoder.Encoder5.read_pulse(); //5 4

    //Send together two data with CAN
    can.dual_send(id, geometry, 2);  //(ID[array], value[array], number of data)
  
    

}

void setup() {

    // setup only can_send
    can.set_send();

    //Encoder resolution settings
    myEncoder.Encoder3.set(8192);
    myEncoder.Encoder5.set(8192);

    Interrupt_5ms.begin(update, 5000);

}

void loop() {
    //Serial monitoring
    /*for(int x : geometry){
        Serial.print(x);
        Serial.print(" ");
    }
    Serial.println();*/
}
