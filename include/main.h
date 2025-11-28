
#include "Multichannel_Gas_GMXXX.h" // Grove Multichannel Gas Sensor v2
#include "SensirionI2CSgp41.h"	    // Seeed SGP41
#include "seeed_bme680.h"	    // Seeed BME680
#include <Arduino.h>
#include <ArduinoBLE.h> // Arduino BLE library
#include <HardwareSerial.h>
#include <SIM800L.h>
#include <stdio.h>

// Constants
#define HOST_NAME "Name" // The server address
#define PORT 1234	 // The port number for sending data with gsm
#define CHARACTERISTIC_UUID                                                    \
	"19b10001-e8f2-537e-4f6c-d104768a1214" // The characteristic UUID for
					       // the emitter device
#define SERVICE_UUID                                                           \
	"19b10000-e8f2-537e-4f6c-d104768a1214" // The service UUID for the
					       // emitter control

typedef enum {
	Sensor_failed_to_start,
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

// Function prototypes
// Function prototypes: Init functions
bool init_GSM();       // innitialises the communication between GSM and server
bool init_bluetooth(); // innitialises the bluetooth module
bool init_sensors();   // starts the sensors
bool send_error(error_codes_t); // sends the error code to server

// Function prototypes: Main functions
void sensor_readings();	     // reads data from sensors
bool bluetooth_to_emitter(); // Attempts to discover emitter device over
			     // bluetooth. If it finds any, it calls
			     // controlEmitters to send control bytes. If a
			     // device is not found or something fails, it
			     // returns false.
bool control_emitters(
    BLEDevice peripheral); // connects to device and sends control bytes to
			   // emitter over bluetooth
