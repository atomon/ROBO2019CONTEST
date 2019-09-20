#include <Encoders.h>
#include <Teensy_CAN.h>

//declare object
ENCODERS myEncoder;
TEENSY_CAN can(1000000); //set baud

//declare array
int id[2] = {1, 2}; // set ID
int w[4];

void setup() {

    // setup only can_send
    can.set_send();
    
    //Encoder resolution settings
    myEncoder.Encoder1.set(3200);
    myEncoder.Encoder3.set(3200);
    myEncoder.Encoder4.set(3200);
    myEncoder.Encoder6.set(3200);
  
}
int l=0;
void loop() {

    //Read rpm
    w[0] = -myEncoder.Encoder3.read_rpm();
    w[1] = -myEncoder.Encoder1.read_rpm();
    w[2] = -myEncoder.Encoder4.read_rpm();
    w[3] = -myEncoder.Encoder6.read_rpm();

    //Send together two data with CAN
    can.dual_send(id, w, 4);  //(ID[array], value[array], number of data)
  
    //Serial monitoring
    for(int x : w){
        Serial.print(x);
        Serial.print(" ");
    }
    Serial.println();
    delayMicroseconds(1000);

}

