
#define SLAVE_ADDRESS 0x03
#define length_of_a_data 6
#define Serial_Baud 57600
const double ratio_phi_counter = 3000.0 / 23656.0;

void SetEncoder() {
	Wire.begin();
	Serial.println("Encoder Master is ready");
}

void getEncoder(long *counter_L, DCMotor *motor1) {
	char deltaCounter_array[length_of_a_data];
	int Counter = 0;
	int count = 0;
	Wire.beginTransmission(SLAVE_ADDRESS);
	Wire.requestFrom(SLAVE_ADDRESS, length_of_a_data);
  // Serial.println(Wire.available());
	if (Wire.available()) {
    // Serial.println("GET NANO DATA!!");
		while ((Wire.available()))
		{
			deltaCounter_array[count] = (char) Wire.read();
			count++; 
		}
    Counter = atoi(deltaCounter_array);
    Serial.println(Counter);
	}
	Wire.endTransmission();
    
	*counter_L += Counter;
	motor1->phi = *counter_L * ratio_phi_counter;
}
