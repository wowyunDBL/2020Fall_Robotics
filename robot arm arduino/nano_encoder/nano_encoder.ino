// Visual Micro is in vMicro>General>Tutorial Mode
//
/*
    Name:       nano_encoder_v1.ino
    Created:	6/12/2020 1:15:11 AM
    Author:     DESKTOP-EU6B2UN\leo19
*/

// Define User Types below here or use a .h file
//

// Define Function Prototypes that use User Types below here or use a .h file
//
#include <Wire.h>
#define SLAVE_ADDRESS 0x03
#define length_of_a_data 
#define Serial_Baud 57600
#define enc_Al_pin motorL_Y
#define enc_Bl_pin motorL_G

// pin setting(need change)
const uint8_t motorL_G = 10;
const uint8_t motorL_Y = 9;

// encoder variables
uint8_t last_LL = 0;
long counter_L = 0;
const int outcome[] = {0, -1, 1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
long last_counter_L = 0;

// Define Functions below here or use other .ino or cpp files
//

// The setup() function runs once each time the micro-controller starts
void setup()
{
    // encoder pins //
    pinMode(motorL_G, INPUT);
    pinMode(motorL_Y, INPUT);
    //////////////////

    Wire.begin(SLAVE_ADDRESS);
    Wire.onRequest(sendData);
    Serial.begin(Serial_Baud);
    Serial.println("Slave is ready");
    
}

// Add the main program code into the continuous loop() function
void loop()
{
    encoderCounter();
//    Serial.println(counter_L);
    
}

void sendData()
{
    int deltaCounterL = counter_L - last_counter_L;
    last_counter_L = counter_L;
    char deltaCounterL_array[6];
    sprintf(deltaCounterL_array, "%06d", deltaCounterL);

    Serial.print("sending: deltaCounterL: ");
    Serial.println(deltaCounterL_array);
    Wire.write(deltaCounterL_array);
}


void encoderCounter()
{

    uint8_t left_A = digitalRead(enc_Al_pin);
    uint8_t left_B = digitalRead(enc_Bl_pin);
    uint8_t current_ll = (left_A << 1) | left_B;
    uint8_t positionL = (last_LL << 2) | current_ll;
    //    Serial.print(left_A);
    //    Serial.print(",");
    //    Serial.println(left_B);

    counter_L += outcome[positionL];
    last_LL = current_ll;
}
