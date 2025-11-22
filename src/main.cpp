
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SIM800L.h>
#include "Multichannel_Gas_GMXXX.h" // Grove Multichannel Gas Sensor v2
#include "seeed_bme680.h"           // Seeed BME680
#include "SensirionI2CSgp41.h"      // Seeed SGP41
#include <ArduinoBLE.h>        	    // Arduino BLE library
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

// Global variables
uint8_t emitterBytes[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // <-- control bytes for emitters (0..255)

// Function prototypes
// Function prototypes: Init functions
void init_GSM();		// innitialises the communication with server
bool init_bluetooth();		// innitialises the bluetooth module
bool init_sensors();		// starts the sensors
bool init_emmi();		// starts the emmiter (why ??)
bool send_error(error_codes_t); // sends the error code to server

// Function prototypes: Main functions
void sensor_readings(); 	// reads data from sensors

void setup() {
	// set up leds before inits so we can indicate if any of them failed
	init_sensors();
	init_bluetooth();
	init_GSM();
}

// Main loop
void loop() {
	// make sure time between each reading is at least 1 second
	sensor_readings();

	// wrap every other action in a timer-activated block (active after 30 minutes of collecting readings to preheat gas sensors)
	
}

// Functions Definitions
// Functions Definitions: Main functions
void sensor_readings() {
	// read data from sensors
}

// Functions Definitions: Init functions
void init_GSM() {}

bool init_bluetooth() {
	
	// initialize the Bluetooth® Low Energy hardware
	if (!BLE.begin()) {
		Serial.println("Failed to initialize BLE!");
		return false;
	}

	// start scanning for peripherals
	BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
	return true;
}
bool init_emmi() {} // why ??

bool init_sensors() {

	if (bme680.init() != 0) { // if BME680 failed to start
		send_error(Sensor_failed_to_start);
		Serial.println("BME680 not found...");
		return false;
	}
	// if (init_sensor2() != 0) { // if Multichannel Gas Sensor failed to start
	//	send_error(Sensor_failed_to_start);
	//	return false;
	// }
	// if (init_sensor3() != 0) { // if SGP41 failed to start
	//	send_error(Sensor_failed_to_start);
	//	return false;
	// }
	return true;
}
bool send_error(error_codes_t) {
	// GSM_send(the error);
}
