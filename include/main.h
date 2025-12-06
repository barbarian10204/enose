
#include "Multichannel_Gas_GMXXX.h" // Grove Multichannel Gas Sensor v2
#include "SensirionI2CSgp41.h"	    // Seeed SGP41
#include "seeed_bme680.h"	    // Seeed BME680
#include <Arduino.h>
#include <ArduinoBLE.h> // Arduino BLE library
#include <HardwareSerial.h>
#include <SIM800L.h>
#include <stdio.h>

// Constants
#define CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214" 
/* The characteristic UUID for
	the emitter device
*/
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214" 
/* The service UUID for the emitter control
*/
#define DEVICE_ID "EnoseDevice001" // Unique device ID for this Enose device

#define SIM800_RST_PIN 19 // GPIO pin connected to SIM800L reset pin

typedef enum {
	SensorBME680_failed_to_start,
	SensorBME680_failed_to_read,
	SensorSGP41_failed_to_start,
	SensorSGP41_failed_to_read,
	SensorMultichannelGas_failed_to_start,
	// SensorMultichannelGas_failed_to_read, // We cannot really tell if this sensors fails to read
	Emmiter_not_found,
	Enose_power_critical,
	Emmiter_power_critical,
	BLE_connection_failed,
	BLE_send_failed
} error_codes_t;

typedef struct {
	float temp;
	float preasure;
	float c;
	float d;
	int e;
	int f;
	int g;
	int h;
	int i;
	int j;
} all_sensors_data_t;

const char APN[] = "www.internet.mtelia.dk";
const char URL[] = "http://outdated-acclimatable-leoma.ngrok-free.dev/api/sensor-data"; // perhaps remove endpoint specification and add it while POST-ing or GET-ing data
const char CONTENT_TYPE[] = "application/json";

/* == Function prototypes == */

/* Function prototypes: Init functions
*/

bool init_GSM();       			// initialises the GSM module
bool init_bluetooth(); 			// initialises the bluetooth module
bool init_sensors();   			// initialises the sensors
bool send_error(error_codes_t); // sends the error code to server

/* Function prototypes: Main functions

	Note: Primary functions are called directly in the main loop
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
