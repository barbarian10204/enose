
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SIM800L.h>
#include "Multichannel_Gas_GMXXX.h" // Grove Multichannel Gas Sensor v2
#include "seeed_bme680.h"           // Seeed BME680
#include "SensirionI2CSgp41.h"      // Seeed SGP41
// Simple USART communication example for Arduino Nano ESP32

// Constants

#define HOST_NAME "Name"  // The server address
#define PORT 1234 		  // The port number for sending data with gsm

// Constructors
// Constructors: GSM objects
HardwareSerial GSMserial(1); // RX, TX
SIM800L gsm;

// Constructors: Sensor objects
GAS_GMXXX<TwoWire> gasSensor;
Seeed_BME680 bme680(uint8_t(0x76));
SensirionI2CSgp41 sgp41;

typedef enum {
	Sensor_failed_to_start,
	Emmiter_not_found,
	Enose_power_critical,
	Emmiter_power_critical
} error_codes_t;

// Function prototypes
// Function prototypes: Init functions
void init_GSM();		// innitialises the communication with server
bool init_bluetooth();		// innitialises the bluetooth module
bool init_sensors();		// starts the sensors
bool init_emmi();		// starts the emmiter
bool send_error(error_codes_t); // sends the error code to server

// Function prototypes: Main functions
void sensor_readings(); 	// reads data from sensors

void setup() {
	init_sensors();
	init_bluetooth();
	init_GSM();
}

// Main loop
void loop() {

	// do stuff
	sensor_readings();
	// make sure delaye before next reading is at least 1 second

}

// Functions Definitions
// Functions Definitions: Main functions
void sensor_readings() {
	// read data from sensors
}

// Functions Definitions: Init functions
void init_GSM() {}

bool init_bluetooth() {

	return true;
}
bool init_emmi() {}

bool init_sensors() {
	bool errorless = 1;
	// if (i2c_init() != 0) { // if i2c failed to start
	//	send_error(Sensor_failed_to_start);
	//	errorless = 0;
	// }
	// if (init_sensor1() != 0) { // if i2c failed to start
	//	send_error(Sensor_failed_to_start);
	//	errorless = 0;
	// }
	// if (init_sensor2() != 0) { // if i2c failed to start
	//	send_error(Sensor_failed_to_start);
	//	errorless = 0;
	// }
	// if (init_sensor3() != 0) { // if i2c failed to start
	//	send_error(Sensor_failed_to_start);
	//	errorless = 0;
	// }
	return errorless;
}
bool send_error(error_codes_t) {
	// GSM_send(the error);
}
