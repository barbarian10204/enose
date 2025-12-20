#include "main.h"

void setup() {
	// =======FOR TESTING REMOVRE LATER =======
	Wire.begin();

	start = millis();

	init_ADC();
	init_LEDs();
    init_sensors();
	// sensors_warmup();
	init_bluetooth();
	init_GSM();
}

// Main loop
void loop() {
	unsigned long cycle_start = millis(); // starting timer for consistent interval between readings
	// === TIMER START ===

	Serial.println();

	digitalWrite(LED_WARMUPSENSORS_GSM, HIGH); // indicate GSM activity

	GasDataPOST();

	digitalWrite(LED_WARMUPSENSORS_GSM, LOW); // indicate GSM activity

	if (!ControlBytesGET()){
		// Clear emitterBytes if GET fails
		for (uint8_t i = 0; i < 8; i++) {
			emitterBytes[i] = 0;
		}
	}

	digitalWrite(LED_WARMUPSENSORS_GSM, HIGH); // indicate GSM activity

	// // Manually testing emitters
	// emitterBytes[0] = 0;
	// emitterBytes[1] = 0;
	// emitterBytes[2] = 0; 
	// emitterBytes[3] = 200;
	// emitterBytes[4] = 0;
	// emitterBytes[5] = 0;
	// emitterBytes[6] = 0;
	// emitterBytes[7] = 0; 

	BluetoothToEmitters();

	CheckBattery();

	// === TIMER END ===
	unsigned long elapsed = millis() - cycle_start;
	Serial.print("Cycle time (ms): "); // for testing , remove later
	Serial.println(elapsed); // for testing , remove later
	if (elapsed < SENSOR_INTERVAL_MS) {
		delay(SENSOR_INTERVAL_MS - elapsed);
	}
}

// Functions Definitions
// Functions Definitions: Main functions
bool GasDataPOST() {
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
		return false;
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
		Serial.println(errorMessage);
	}

	// == Multichannel Gas Sensor readings ==
	uint16_t NO2 = gasSensor.getGM102B(); // NO2 (Nitrogen Dioxide)
	uint16_t eth = gasSensor.getGM302B(); // Ethanol
	uint16_t VOC = gasSensor.getGM502B(); // VOC (Alcohol, Nitrogen, Formaldehyde)
	uint16_t COandH2 = gasSensor.getGM702B(); // CO and H2 (Carbon Monoxide and Hydrogen)

	// Now appending readings to string for GSM transmission
	unsigned long elapsed = millis() - start;

	// Build JSON payload string
	String payload = "{";
	payload += "\"device_id\":\"" + String(DEVICE_ID) + "\",";
	payload += "\"timestamp\":" + String(elapsed) + ",";
	payload += "\"temperature\":" + String(temp, 2) + ",";
	payload += "\"humidity\":" + String(hum, 2) + ",";
	payload += "\"gas\":" + String(gas, 2) + ",";
	payload += "\"voc_raw\":" + String(srawVoc) + ",";
	payload += "\"nox_raw\":" + String(srawNox) + ",";
	payload += "\"no2\":" + String(NO2) + ",";
	payload += "\"ethanol\":" + String(eth) + ",";
	payload += "\"voc\":" + String(VOC) + ",";
	payload += "\"co_h2\":" + String(COandH2);
	payload += "}";

	// Send using GSM
	if (!send_gsm_payload(payload)) {
	return false;
	}
	return true;
}

bool send_gsm_payload(String jsonPayload) {
    // Convert String to char buffer
    int len = jsonPayload.length();
    char payloadChar[len + 1];
    jsonPayload.toCharArray(payloadChar, len + 1);

    // Ensure module is ready
    if (!sim800l->isReady()) {
        Serial.println("SIM800 not ready!");
        return false;
    }

	// Check if GPRS is already connected
	if (!sim800l->isConnectedGPRS()) {
		// Try connecting GPRS
		bool connected = false;
		for (uint8_t i = 0; i < 5 && !connected; i++) {
			connected = sim800l->connectGPRS();
			delay(500);
		}

		if (!connected) {
			Serial.println("GPRS connection failed!");
			sim800l->reset();
    		init_GSM();
			return false;
		}
	}

    Serial.println("HTTP POST sending:");
    Serial.println(jsonPayload);

    // Send HTTP POST
    uint16_t rc = sim800l->doPost(URLPOST, CONTENT_TYPE, payloadChar, 10000, 10000);

    if (rc == 200) {
        Serial.println("HTTP POST OK");
        Serial.println(sim800l->getDataReceived());
    } else {
        Serial.print("HTTP POST error: ");
        Serial.println(rc);
        return false;
    }

	return true;
}

bool BluetoothToEmitters() {
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
			peripheral_Flag = false;
			return false;
		}

		// discover peripheral attributes
		Serial.println("Discovering attributes ...");
		if (peripheral.discoverAttributes()) {
			Serial.println("Attributes discovered");
		} else {
			Serial.println("Attribute discovery failed!");
			peripheral_Flag = false;
			peripheral.disconnect();
			return false;
		}

		// retrieve the Emitter characteristic
		emittersCharacteristic = peripheral.characteristic(CHARACTERISTIC_UUID);

		if (!emittersCharacteristic) {
			Serial.println("Peripheral does not have Emitter characteristic!");
			peripheral_Flag = false;
			peripheral.disconnect();
			return false;
		} else if (!emittersCharacteristic.canWrite()) {
			Serial.println("Peripheral does not have a writable ""Emitter characteristic!");
			peripheral_Flag = false;
			peripheral.disconnect();
			return false;
		}
	}

	// Double-check connection before attempting write
	if (!peripheral.connected()) {
		Serial.println("Peripheral not connected");
		peripheral_Flag = false;
		return false;
	}

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
	if (!ok) {
		Serial.println("Write failed - peripheral likely disconnected");
		peripheral_Flag = false;
		peripheral.disconnect();
		return false;
	}

	// Verify connection is still active after write
	if (!peripheral.connected()) {
		Serial.println("Peripheral disconnected after write");
		peripheral_Flag = false;
		return false;
	}

	Serial.println("Write successful");
	return true;
}

bool ControlBytesGET() {
	// Ensure module is ready
    if (!sim800l->isReady()) {
        Serial.println("SIM800 not ready!");
        return false;
    }

	// Check if GPRS is already connected
	if (!sim800l->isConnectedGPRS()) {
		// Try connecting GPRS
		bool connected = false;
		for (uint8_t i = 0; i < 5 && !connected; i++) {
			connected = sim800l->connectGPRS();
			delay(500);
		}

		if (!connected) {
			Serial.println("GPRS connection failed!");
			sim800l->reset();
    		init_GSM();
			return false;
		}
	}
		
	Serial.println(F("Start HTTP GET..."));

	// Do HTTP GET communication with 10s for the timeout (read)
	uint16_t rc = sim800l->doGet(URLGET, 10000);
	if(rc == 200) {
		// Success, output the data received on the serial
		Serial.print(F("HTTP GET successful ("));
		Serial.print(sim800l->getDataSizeReceived());
		Serial.println(F(" bytes)"));
		Serial.print(F("Received : "));
		Serial.println(sim800l->getDataReceived());
	} else {
		// Failed...
		Serial.print(F("HTTP GET error "));
		Serial.println(rc);
		if (rc > 700){
			sim800l->reset();
			Serial.println(F("SIM800 reset due to GET error"));
		}
		return false;
	}

	// Now setting control bytes from received data
	String received = sim800l->getDataReceived();
	
	// Parse JSON and extract the 8 values into emitterBytes array
	// Expected format: {"0":value0,"1":value1,...,"7":value7}
	// Where each value is an integer between 0 and 255
	uint8_t tempEmitterBytes[8]; // temporary storage for received values
	
	for (uint8_t i = 0; i < 8; i++) {
		String key = "\"" + String(i) + "\":";
		int keyIndex = received.indexOf(key);
		
		if (keyIndex >= 0) {
			// Move past the key to find the value
			int valueStart = keyIndex + key.length();
			
			// Skip whitespace
			while (valueStart < received.length() && 
			       (received[valueStart] == ' ' || received[valueStart] == '\t')) {
				valueStart++;
			}
			
			// Extract the numeric value
			int valueEnd = valueStart;
			while (valueEnd < received.length() && 
			       received[valueEnd] >= '0' && received[valueEnd] <= '9') {
				valueEnd++;
			}
			
			// Convert substring to integer
			String valueStr = received.substring(valueStart, valueEnd);
			int value = valueStr.toInt();
			
			// Clamp value to 0-255 range and assign to temp array
			if (value < 0) value = 0;
			if (value > 255) value = 255;
			tempEmitterBytes[i] = (uint8_t)value;
		} else {
			// Key not found, set to 0 as default
			tempEmitterBytes[i] = 0;
			Serial.println("Key " + String(i) + " not found in JSON, setting to 0");
		}
	}
	
	// Increment counter
	emitterCounter++;
	
	// Only apply emitter bytes every 3rd cycle
	if (emitterCounter >= 3) {
		// Apply the received values to emitterBytes
		for (uint8_t i = 0; i < 8; i++) {
			emitterBytes[i] = tempEmitterBytes[i];
		}
		emitterCounter = 0; // Reset counter
	} else {
		// Clear emitterBytes on cycles 1 and 2
		for (uint8_t i = 0; i < 8; i++) {
			emitterBytes[i] = 0;
		}
	}
	
	Serial.println("Control bytes updated:");
	for (int i = 0; i < 8; i++) {
		Serial.print("emitterBytes[");
		Serial.print(i);
		Serial.print("] = ");
		Serial.println(emitterBytes[i]);
	}
	
	return true;
}

bool sensors_warmup() {
	digitalWrite(LED_WARMUPSENSORS_GSM, HIGH); // indicate warmup period with LED

	for(int i = 0; i < 900; i++) { // 15 minutes of warmup at 1 reading per second
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
			return false;
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
			Serial.println(errorMessage);
			return false;
		}

		// == Multichannel Gas Sensor readings ==
		uint16_t NO2 = gasSensor.getGM102B(); // NO2 (Nitrogen Dioxide)
		uint16_t eth = gasSensor.getGM302B(); // Ethanol
		uint16_t VOC = gasSensor.getGM502B(); // VOC (Alcohol, Nitrogen, Formaldehyde)
		uint16_t COandH2 = gasSensor.getGM702B(); // CO and H2 (Carbon Monoxide and Hydrogen)

		// Now printing all readings for debugging
		Serial.print("BME680 - Temp: "); Serial.print(temp); Serial.print(" °C, Hum: "); 
		Serial.print(hum); Serial.print(pres); Serial.print(" kPa, Gas: "); 
		Serial.print(gas); Serial.println(" KOhms");
		Serial.print("SGP41 - SRAW VOC: "); Serial.print(srawVoc); Serial.print(", SRAW NOx: "); 
		Serial.println(srawNox);
		Serial.print("Gas Sensor - NO2: "); Serial.print(NO2); Serial.print(" ppb, Ethanol: "); 
		Serial.print(eth); Serial.print(" ppb, VOC: "); Serial.print(VOC); Serial.print(" ppb, CO+H2: "); 
		Serial.print(COandH2); Serial.println(" ppb");
		Serial.print("Warmup reading "); Serial.print(i + 1); Serial.println(" complete.");
		Serial.println();

		delay(1000); // 1 second between warmup readings
	}

	digitalWrite(LED_WARMUPSENSORS_GSM, LOW); // indicate end of warmup period with LED

	Serial.print("Warmup complete.");
	return true;
}

bool CheckBattery() {
	int battery_voltage = analogReadMilliVolts(ADC_PIN)*2; // multiply by 2 because of voltage divider
	Serial.print("Battery voltage (mV): ");
	Serial.println(battery_voltage);
	if (battery_voltage < LOW_BATTERY_THRESHOLD) {
		digitalWrite(LED_LOWBAT_ERRORS, HIGH); // turn on low battery LED
		return false;
	} else if (battery_voltage >= (LOW_BATTERY_THRESHOLD + 150)){
		digitalWrite(LED_LOWBAT_ERRORS, LOW); // turn off low battery LED
	}
	return true;
}

// Functions Definitions: Init functions
bool init_GSM() {

	// Initialize the hardware GSMserial
	GSMserial.begin(9600, SERIAL_8N1, 0, 1);
	delay(1000);
	
	// Initialize SIM800L driver with an internal buffer of 200 bytes and a reception buffer of 512 bytes, debug disabled
	sim800l = new SIM800L((Stream *)&GSMserial, SIM800_RST_PIN, 200, 512);

	sim800l->reset();
	delay(3000);

	// Wait until the module is ready to accept AT commands
	while(!sim800l->isReady()) {
		Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
		delay(1000);
	}
	Serial.println(F("Setup Complete!"));

	// Wait for the GSM signal
	uint8_t signal = sim800l->getSignal();
	while(signal <= 0) {
		delay(1000);
		signal = sim800l->getSignal();
	}
	Serial.print(F("Signal OK (strenght: "));
	Serial.print(signal);
	Serial.println(F(")"));
	delay(1000);

	// Wait for operator network registration (national or roaming network)
	NetworkRegistration network = sim800l->getRegistrationStatus();
	while(network != REGISTERED_HOME && network != REGISTERED_ROAMING) {
		delay(1000);
		network = sim800l->getRegistrationStatus();
	}
	Serial.println(F("Network registration OK"));
	delay(1000);

	// Setup APN for GPRS configuration
	bool success = sim800l->setupGPRS(APN);
	while(!success) {
		success = sim800l->setupGPRS(APN);
		delay(5000);
	}
	Serial.println(F("GPRS config OK"));
	return true;
}

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
	// BME680 init
	float temp = 0;
	float hum = 0;
	float pres = 0;
	float gas = 0;

	if (!bme680.init()) { // if BME680 failed to start
		Serial.println("BME680 not found...");
		return false;
	}

	if (!bme680.read_sensor_data()) {
		temp = bme680.sensor_result_value.temperature;
		hum = bme680.sensor_result_value.humidity;
		pres = bme680.sensor_result_value.pressure / 1000.0;
		gas = bme680.sensor_result_value.gas / 1000.0;
	} else {
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
		Serial.println(errorMessage);
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
		Serial.println("Multichannel Gas Sensor failed to start");
		return false;
	}

	return true;
}

void init_LEDs() {
	// Setting the pins connected to the LEDs as output pins
	pinMode(LED_LOWBAT_ERRORS, OUTPUT);
	pinMode(LED_WARMUPSENSORS_GSM, OUTPUT);
}

void init_ADC() {
	adcAttachPin(ADC_PIN);
	analogSetPinAttenuation(ADC_PIN, ADC_ATTENUATION);
}