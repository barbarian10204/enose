#include "main.h"
// Simple USART communication example for Arduino Nano ESP32

// Constructors
// Constructors: GSM objects
HardwareSerial GSMserial(1); // RX, TX
SIM800L gsm;

// Constructors: Sensor objects
GAS_GMXXX<TwoWire> gasSensor;
Seeed_BME680 bme680(uint8_t(0x76));
SensirionI2CSgp41 sgp41;
// Global variables
uint8_t emitterBytes[8] = {
    255, 0, 0, 0, 0, 0, 0, 0}; // <-- control bytes for emitters (0..255)
bool peripheral_Flag = false;  // flag to indicate if peripheral is connected
BLECharacteristic emittersCharacteristic;
BLEDevice peripheral;

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

	// wrap every other action in a timer-activated block (active after 30
	// minutes of collecting readings to preheat gas sensors)

	// 	char sensors_data[100];
	// 	char *time;
	// 	sprintf(time, "%lu", millis());
	// 	all_sensors_data_t sens_struct;
	// 	sensor_readings(&sens_struct);
	// 	sprintf(sensors_data,
	// "%s,%f.2,%f.2,%f.2,%f.2,%d,%d,%d,%d,%d,%d,", time,
	// sens_struct.temp, sens_struct.preasure, sens_struct.c,
	// sens_struct.d, sens_struct.e, sens_struct.f, sens_struct.g,
	// sens_struct.h, sens_struct.i, sens_struct.j); 	if
	// (gsm.tcpAvailable()) { 		gsm.tcpSend(sensors_data);
	// 		loops_without_sending = 0;
	// 	} else {
	// 		sensor_lines[loops_without_sending] = sensors_data;
	// 		loops_without_sending++;
	// 	}

	if (bluetooth_to_emitter()) { // not sure where in the loop this should
				      // be placed
		Serial.println("good");
	} else {
		Serial.println("bad");
	}
	delay(100); // Delay for testing purposes, will add a dynamic timer for
		    // sensors later
}

// Functions Definitions
// Functions Definitions: Main functions
void sensor_readings() {
	// read data from sensors into the provided structure
}

bool bluetooth_to_emitter() {
	// check if a peripheral has been discovered

	if (!peripheral_Flag) {
		peripheral = BLE.available();
	}

	if (peripheral || peripheral_Flag) {
		if (!peripheral_Flag) {
			// discovered a peripheral, print out address, local
			// name, and advertised service
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

		if (control_emitters(peripheral)) {
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
	return false; // shouldn't this be true?
}

bool control_emitters(BLEDevice peripheral) {

	if (!peripheral_Flag) {
		// connect to the peripheral
		Serial.println("Connecting ...");

		if (peripheral.connect()) {
			Serial.println("Connected");
		} else {
			Serial.println("Failed to connect!");
			send_error(BLE_connection_failed);
			return false;
		}

		// discover peripheral attributes
		Serial.println("Discovering attributes ...");
		if (peripheral.discoverAttributes()) {
			Serial.println("Attributes discovered");
		} else {
			Serial.println("Attribute discovery failed!");
			send_error(BLE_connection_failed);
			peripheral.disconnect();
			return false;
		}

		// retrieve the Emitter characteristic
		emittersCharacteristic =
		    peripheral.characteristic(CHARACTERISTIC_UUID);

		if (!emittersCharacteristic) {
			Serial.println(
			    "Peripheral does not have Emitter characteristic!");
			send_error(BLE_connection_failed);
			peripheral.disconnect();
			return false;
		} else if (!emittersCharacteristic.canWrite()) {
			Serial.println("Peripheral does not have a writable "
				       "Emitter characteristic!");
			send_error(BLE_connection_failed);
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
			if (i < 7)
				Serial.print(", ");
		}
		Serial.println();

		// write the 8-byte array to the characteristic
		// writeValue accepts (const uint8_t* data, unsigned int length)
		bool ok = emittersCharacteristic.writeValue(emitterBytes, 8);
		if (ok) {
			Serial.println("Write successful");
		} else {
			Serial.println("Write failed");
			send_error(BLE_send_failed);
			return false;
		}
		return true;
	} else {
		peripheral_Flag = false;
		send_error(BLE_connection_failed);
		return false;
	}

	return false; // shouldn't this be true?
}

// Functions Definitions: Init functions
bool init_GSM() {
	GSMserial.begin(9600, SERIAL_8N1, 9, 8);
	if (!gsm.begin(GSMserial)) {
		Serial.println("Couldn't start the GSM");
		return false;
	}
	if (!gsm.startGPRS()) {
		Serial.println("couldn't connect to internet");
		return false;
	}
	gsm.tcpConnect(HOST_NAME, PORT);
	if (!gsm.tcpStatus()) {
		Serial.println("couldn't connect to internet");
		return false;
	}
	return true;
}

bool init_bluetooth() {

	// initialize the Bluetooth® Low Energy hardware
	if (!BLE.begin()) {
		Serial.println("Failed to initialize BLE!");
		send_error(BLE_connection_failed);
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
	// if (init_sensor2() != 0) { // if Multichannel Gas Sensor failed to
	// start
	//	send_error(Sensor_failed_to_start);
	//	return false;
	// }
	// if (init_sensor3() != 0) { // if SGP41 failed to start
	//	send_error(Sensor_failed_to_start);
	//	return false;
	// }
	return true;
}
bool send_error(error_codes_t error) {
	char *error_message = (char *)error; // this might not work
	gsm.tcpSend(error_message);
	Serial.println(error_message);
	return true;
}
