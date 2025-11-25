
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SIM800L.h>
#include <ArduinoBLE.h>        	    // Arduino BLE library
#include "Multichannel_Gas_GMXXX.h" // Grove Multichannel Gas Sensor v2
#include "seeed_bme680.h"           // Seeed BME680
#include "SensirionI2CSgp41.h"      // Seeed SGP41
// Simple USART communication example for Arduino Nano ESP32

// Constants
#define HOST_NAME "Name"  // The server address
#define PORT 1234 		  // The port number for sending data with gsm
#define CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214" // The characteristic UUID for the emitter device
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"    // The service UUID for the emitter control

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
uint8_t emitterBytes[8] = {255, 0, 0, 0, 0, 0, 0, 0}; // <-- control bytes for emitters (0..255)
bool peripheral_Flag = false; // flag to indicate if peripheral is connected
BLECharacteristic emittersCharacteristic;
BLEDevice peripheral;

// Function prototypes
// Function prototypes: Init functions
void init_GSM();				// innitialises the communication between GSM and server
bool init_bluetooth();			// innitialises the bluetooth module
bool init_sensors();			// starts the sensors
bool send_error(error_codes_t); // sends the error code to server

// Function prototypes: Main functions
void sensor_readings(); 						// reads data from sensors
bool bluetooth_to_emitter();					// Attempts to discover emitter device over bluetooth. If it finds any, it calls controlEmitters to send control bytes. If a device is not found or something fails, it returns false.
bool controlEmitters(BLEDevice peripheral);		// connects to device and sends control bytes to emitter over bluetooth

void setup() {
	// set up leds before inits so we can indicate if any of them failed
	init_sensors();
	init_bluetooth();
	init_GSM();
}

// Main loop
void loop() {
	// make sure time between each reading is at least 1 second
	// sensor_readings();

	// wrap every other action in a timer-activated block (active after 30 minutes of collecting readings to preheat gas sensors)

	if (bluetooth_to_emitter()) {
		Serial.println("good");
	} else {
		Serial.println("bad");
	}
	delay(100); // Delay for testing purposes, will add a dynamic timer for sensors later
}

// Functions Definitions
// Functions Definitions: Main functions
void sensor_readings() {
	// read data from sensors
}

bool bluetooth_to_emitter() {
	// check if a peripheral has been discovered
	
	if (!peripheral_Flag){
		peripheral = BLE.available();
	}

	if (peripheral || peripheral_Flag) {
		if (!peripheral_Flag) {
			// discovered a peripheral, print out address, local name, and advertised service
			Serial.print("Found ");
			Serial.print(peripheral.address());
			Serial.print(" '");
			Serial.print(peripheral.localName());
			Serial.print("' ");
			Serial.print(peripheral.advertisedServiceUuid());
			Serial.println();

			if (peripheral.localName() != "EmitterDevice") {
			return false;
			}

			// stop scanning
			BLE.stopScan();
		}
		
		if (controlEmitters(peripheral)) {
			Serial.println("Emitter control successful");
			peripheral_Flag = true;
			return true;
		} else {
			Serial.println("Peripheral disconnected");
			BLE.scanForUuid(SERVICE_UUID); // restart scanning
			peripheral_Flag = false;
			return false;
		}
	}
	return false;
}

bool controlEmitters(BLEDevice peripheral){

	if (!peripheral_Flag) {
		// connect to the peripheral
		Serial.println("Connecting ...");

		if (peripheral.connect()) {
			Serial.println("Connected");
		} else {
			Serial.println("Failed to connect!");
			return false;
		}

		// discover peripheral attributes
		Serial.println("Discovering attributes ...");
		if (peripheral.discoverAttributes()) {
			Serial.println("Attributes discovered");
		} else {
			Serial.println("Attribute discovery failed!");
			peripheral.disconnect();
			return false;
		}

		// retrieve the Emitter characteristic
		emittersCharacteristic = peripheral.characteristic(CHARACTERISTIC_UUID);

		if (!emittersCharacteristic) {
			Serial.println("Peripheral does not have Emitter characteristic!");
			peripheral.disconnect();
			return false;
		} else if (!emittersCharacteristic.canWrite()) {
			Serial.println("Peripheral does not have a writable Emitter characteristic!");
			peripheral.disconnect();
			return false;
		}
	}

  	if (peripheral.connected()) {
		// while the peripheral is connected

		// Print what we're about to send
		Serial.print("Bytes: ");
		for (int i = 0; i < 8; i++) {
		Serial.print(emitterBytes[i]);
		if (i < 7) Serial.print(", ");
		}
		Serial.println();

		// write the 8-byte array to the characteristic
		// writeValue accepts (const uint8_t* data, unsigned int length)
		bool ok = emittersCharacteristic.writeValue(emitterBytes, 8);
		if (ok) {
		Serial.println("Write successful");
		} else {
		Serial.println("Write failed");
		return false;
		}
		return true;
	}
	else {
		peripheral_Flag = false;
		return false;
	}

	return false;
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
	BLE.scanForUuid(SERVICE_UUID);
	return true;
}

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
