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
long unsigned int start = 0; // timer for sensor readings

void setup() {
	// =======FOR TESTING REMOVRE LATER =======
	Serial.begin(9600);
	Wire.begin();

	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB
	}
	// =======FOR TESTING REMOVRE LATER =======

	start = millis();
	// set up leds before inits so we can indicate if any of them failed
	init_sensors();
	// init_bluetooth();
	// init_GSM();
}

// Main loop
void loop() {
	// make sure time between each reading is at least 1 second
	sensor_readings();

	// wrap every other action in a timer-activated block (active after 30
	// minutes of collecting readings to preheat gas sensors)

	// if (bluetooth_to_emitter()) {
	// 	Serial.println("good");
	// } else {
	// 	Serial.println("bad");
	// }
	delay(1000); // Delay for testing purposes, will add a dynamic timer for
		     // sensors later
}

// Functions Definitions
// Functions Definitions: Main functions
void sensor_readings() {
	// == BME680 readings ==
	float temp = 0;
	float hum = 0;
	float pres = 0;
	float gas = 0;

	if (!bme680.read_sensor_data()) {
		temp = bme680.sensor_result_value.temperature;
		hum = bme680.sensor_result_value.humidity;
		pres = bme680.sensor_result_value.pressure / 1000.0;
		gas = bme680.sensor_result_value.gas / 1000.0;
	} else {
		Serial.println("BME680 read failed!");
		send_error(SensorBME680_failed_to_read);
	}

	// == SGP41 readings ==
	uint16_t error;
	char errorMessage[256];
	uint16_t defaultRh = 0x8000;
	uint16_t defaultT = 0x6666;
	uint16_t srawVoc = 0;
	uint16_t srawNox = 0;

	// convert BME680 humidity/temperature to sensor ticks (for humidity and
	// temp compensation)
	uint16_t rhTicks = (uint16_t)(hum * 65535.0f / 100.0f + 0.5f);
	uint16_t tTicks =
	    (uint16_t)(((temp) + 45.0f) * 65535.0f / 175.0f + 0.5f);

	error = sgp41.measureRawSignals(rhTicks, tTicks, srawVoc, srawNox);

	if (error) {
		Serial.print("Error trying to execute measureRawSignals(): ");
		errorToString(error, errorMessage, 256);
		send_error(SensorSGP41_failed_to_read);
	}

	// == Multichannel Gas Sensor readings ==
	uint16_t NO2 = gasSensor.getGM102B(); // NO2 (Nitrogen Dioxide)
	uint16_t eth = gasSensor.getGM302B(); // Ethanol
	uint16_t VOC = gasSensor.getGM502B(); // VOC (Alcohol, Nitrogen, Formaldehyde)
	uint16_t COandH2 = gasSensor.getGM702B(); // CO and H2 (Carbon Monoxide and Hydrogen)

	// append readings to string for GSM transmission (TODO)
	unsigned long elapsed = millis() - start;
	// Timestamp
	Serial.print(elapsed);
	Serial.print(",");
	// BME680 readings
	Serial.print(temp, 2);
	Serial.print(",");
	Serial.print(hum, 2);
	Serial.print(",");
	Serial.print(pres, 2);
	Serial.print(",");
	Serial.print(gas, 2);
	Serial.print(",");
	// SGP41 readings
	Serial.print(srawVoc);
	Serial.print(",");
	Serial.print(srawNox);
	Serial.print(",");
	// Multichannel gas sensor readings
	Serial.print(NO2);
	Serial.print(",");
	Serial.print(eth);
	Serial.print(",");
	Serial.print(VOC);
	Serial.print(",");
	Serial.println(COandH2);
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
	return false;
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

	return false;
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
	// BME680 init
	float temp = 0;
	float hum = 0;
	float pres = 0;
	float gas = 0;

	if (!bme680.init()) { // if BME680 failed to start
		send_error(SensorBME680_failed_to_start);
		Serial.println("BME680 not found...");
		return false;
	}

	if (!bme680.read_sensor_data()) {
		temp = bme680.sensor_result_value.temperature;
		hum = bme680.sensor_result_value.humidity;
		pres = bme680.sensor_result_value.pressure / 1000.0;
		gas = bme680.sensor_result_value.gas / 1000.0;
	} else {
		send_error(SensorBME680_failed_to_read);
		Serial.println("BME680 read failed!");
		return false;
	}

	// SGP41 init
	int cond_s = 10;
	uint16_t error;
	char errorMessage[256];
	uint16_t defaultRh = 0x8000;
	uint16_t defaultT = 0x6666;
	uint16_t srawVoc = 0;
	uint16_t srawNox = 0;

	sgp41.begin(Wire);

	// convert BME680 humidity/temperature to sensor ticks (for humidity and
	// temp compensation)
	uint16_t rhTicks = (uint16_t)(hum * 65535.0f / 100.0f + 0.5f);
	uint16_t tTicks =
	    (uint16_t)(((temp) + 45.0f) * 65535.0f / 175.0f + 0.5f);

	Serial.println("Starting SGP41 conditioning for 10s...");
	while (cond_s > 0) {
		// During NOx conditioning (max. 10s) SRAW NOx will remain 0
		error = sgp41.executeConditioning(rhTicks, tTicks, srawVoc);
		cond_s--;
		delay(1000);
	} // after 10 seconds of conditioning

	// Read Measurement
	error = sgp41.measureRawSignals(rhTicks, tTicks, srawVoc, srawNox);

	if (error) {
		Serial.print("Error trying to execute measureRawSignals(): ");
		errorToString(error, errorMessage, 256);
		send_error(SensorSGP41_failed_to_start);
		return false;
	}

	// Multichannel Gas Sensor v2 init

	gasSensor.begin(Wire, 0x08); // Default I2C address is 0x08
	// Doesn't have init function so we have to just make sure the readings
	// look okay
	uint16_t NO2 = gasSensor.getGM102B(); // NO2 (Nitrogen Dioxide)
	uint16_t eth = gasSensor.getGM302B(); // Ethanol
	uint16_t VOC =
	    gasSensor.getGM502B(); // VOC (Alcohol, Nitrogen, Formaldehyde)
	uint16_t COandH2 =
	    gasSensor.getGM702B(); // CO and H2 (Carbon Monoxide and Hydrogen)

	// if any of the readings are zero, sensor likely failed to start
	if (NO2 == 0 || eth == 0 || VOC == 0 || COandH2 == 0) {
		send_error(SensorMultichannelGas_failed_to_start);
		Serial.println("Multichannel Gas Sensor failed to start");
		return false;
	}

	return true;
}
bool send_error(error_codes_t error) {
	char *error_message = (char *)error; // this might not work
	gsm.tcpSend(error_message);
	Serial.println(error_message);
	return true;
}
