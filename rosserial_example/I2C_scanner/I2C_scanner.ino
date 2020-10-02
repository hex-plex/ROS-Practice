#include <Wire.h>

void setup(){
#ifdef ESP8266
 	Wire.begin(D1,D2);
#else
 	Wire.begin();
#endif
 	Serial.begin(115200);
 	//while(!Serial.available());
 	Serial.println("I2C Scanner");
}

void loop(){
	byte error, address;
	int nDevices;

	Serial.println("Scanning....");

	nDevices=0;
	for(address=1;address<127;address++){
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if(error==0){
			Serial.print("I2C device found at address 0x");
			if(address<16)
				Serial.print("0");
			Serial.print(address,HEX);
			Serial.println(" !");
			nDevices++;
		} else if(error == 4) {
			Serial.print("Unknown error at address 0x");
			if(address<16)
				Serial.print("0");
			Serial.println(address,HEX);
		}
	}
	if(nDevices==0)Serial.println("No I2C devices found");
	else Serial.println("done");

	delay(5000);
}
