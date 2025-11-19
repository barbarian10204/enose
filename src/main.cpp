
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SIM800L.h>
// Simple USART communication example for Arduino Nano ESP32

HardwareSerial GSMserial(1); // RX, TX
SIM800L gsm;

typedef enum {
	Sensor_failed_to_start,
	Emmiter_not_found,
	Enose_power_critical,
	Emmiter_power_critical
} error_codes_t;

void init_GSM();		// innitialises the communication with server
bool init_sensors();		// starts the sensors
bool init_emmi();		// starts the emmiter
bool send_error(error_codes_t); // sends the error code to server

void setup() {
	init_sensors();
	init_GSM();
}

void loop() {}

// functions definitions
void init_GSM() {}
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
