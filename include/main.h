
#include "Multichannel_Gas_GMXXX.h" // Grove Multichannel Gas Sensor v2
#include "SensirionI2CSgp41.h"	    // Seeed SGP41
#include "seeed_bme680.h"	    // Seeed BME680
#include <Arduino.h>
#include <ArduinoBLE.h> // Arduino BLE library
#include <HardwareSerial.h>
#include <SIM800L.h>
#include <stdio.h>

// Constants
// Constants: Bluetooth UUIDs and Device ID
#define CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214" // (should be different for each device)
/* The characteristic UUID for the emitter device.

	Note: Note: Should be different for each device.
*/
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
/* The service UUID for the emitter control.

	Note: Should be different for each device.
*/
#define DEVICE_ID "EnoseDevice001"
/* Unique device ID for this Enose device.

	Note: Should be different for each device.
*/

// Constants: Pin definitions
//SIM800L rest pin
#define SIM800_RST_PIN 19 // GPIO pin connected to SIM800L reset pin
// LED Indicator Pins
#define LED_WARMUPSENSORS_GSM 23 // pin for blue LED
/*	Indicates sensor warmup period and GSM activity for POST and GET requests
*/
#define LED_LOWBAT_ERRORS 24 // pin for red LED
/*	Low battery status and error indicator LED pin
*/
#define ADC_PIN 18  // ADC pin for battery voltage reading
#define ADC_ATTENUATION ADC_11db // Attenuation for ADC reading (11db for full range 3.3V)
#define LOW_BATTERY_THRESHOLD 3100 // Low battery threshold in millivolts

// The PCB has two additional LEDs not controlled by the program, but by the charger IC:
// LED1: Battery charger STATUS LED pin (turns on during charging)
// LED2: Battery charger FAULT LED pin (turns on if anything goes wrong with the charger IC)

// Constants: GSM and Server parameters
const char APN[] = "www.internet.mtelia.dk";
const char URLPOST[] = "http://outdated-acclimatable-leoma.ngrok-free.dev/api/sensor-data";
const char URLGET[] = "http://outdated-acclimatable-leoma.ngrok-free.dev/api/sensor-data/emitter";
const char CONTENT_TYPE[] = "application/json";

// Constructors
// Constructors: GSM objects
SIM800L* sim800l;
HardwareSerial GSMserial(1); // RX, TX

// Constructors: Sensor objects
GAS_GMXXX<TwoWire> gasSensor;
Seeed_BME680 bme680(uint8_t(0x76));
SensirionI2CSgp41 sgp41;

/* == Function prototypes == */

/* Function prototypes: Init functions
*/

bool init_GSM();       			// initialises the GSM module
bool init_bluetooth(); 			// initialises the bluetooth module
bool init_sensors();   			// initialises the sensors
void init_LEDs();   			// Sets the LED indicator pins to output
void init_ADC();				// initializes ADC for battery voltage reading

/* Function prototypes: Main functions

	Note: Primary functions are called directly in the main loop or setup
	Secondary functions are called by primary functions
*/

bool GasDataPOST(); // PRIMARY FUNCTION
/* Function providing readings from SGP41, Multichannel gas sensor v2, 
	and BME680. It appends them to a JSON payload string for GSM transmission.
	It calls send_gsm_payload() and passes the payload with sensor readings 
	in the end to send the data to the server.

	Note: Make sure there is a consistent delay between readings that is at least 1 second. We can
		implement a dynamic timer later to optimize the time between readings based on how long other 
		processes are between each call. 
*/

bool BluetoothToEmitters(); // PRIMARY FUNCTION
/* Attempts to discover emitter device over bluetooth. If it finds any, it calls
	controlEmitters to send control bytes. If a device is not found or something fails, it
	returns false. 
*/

bool control_emitters(BLEDevice peripheral); // SECONDARY FUNCTION (called by BluetoothToEmitters)
/* Connects to multi-emitterdevice and sends control bytes to
	emitter over bluetooth
*/

bool send_gsm_payload(String jsonPayload); // SECONDARY FUNCTION (called by GasDataPOST)
/* Sends a payload containing the sensor readings and the device ID to the web server using GSM.
	The payload is in JSON format.
*/

bool ControlBytesGET(); // PRIMARY FUNCTION
/* Retrieves control bytes from server using GSM to control the emitters.
	The control bytes are stored in the global variable emitterBytes[8].
*/

bool CheckBattery(); // PRIMARY FUNCTION
/* Reads battery voltage using ADC and sets low battery LED if voltage is 
	below threshold.
*/

bool sensors_warmup(); // SECONDARY FUNCTION (called in setup)
/* Requests readings from the sensor at 1 Hz rate to allow sensors to warm up. Runs
	continuously for 15 minutes.
*/
