// 1. INCLUDE LIBRARIES
#include <Wire.h>			   // For I2C communication
#include <SPI.h>			   // For SPI communication (SD Card)
#include <SD.h>				   // For SD Card
#include <BH1750.h>			   // For BH1750 light sensors
#include <LiquidCrystal_I2C.h> // For I2C LCD
#include "Control_System.h"	   // For Controlling Servo
#include "Scheduler.h"

void taskSampling();
void taskControl();
void taskLogging();

// 2. DEFINE PINS AND ADDRESSES
const byte SD_CS_PIN = 10;
const byte ACS712_BAT_PIN = A7;
const byte ACS712_PV_PIN = A6;
const byte VOLTAGE_BAT_PIN = A0;
const byte VOLTAGE_PV_PIN = A1;
const byte BH1750_ADDR1 = 0x23; // ADDR pin to GND
const byte BH1750_ADDR2 = 0x5C; // ADDR pin to VCC
const byte LCD_ADDR = 0x27;		// Common LCD address (or 0x3F)
const byte SERVO_PIN = 6;
const uint16_t delaySamplingData = 200;
const uint16_t delayControlSystem = 250;
const uint16_t delayWriteToSD = 5000;

// 3. CONFIGURE SENSORS & OBJECTS
const float R1 = 30000.0;					 // Resistor from input voltage to analog pin
const float R2 = 7500.0;					 // Resistor from analog pin to ground
const float VOLTAGE_CORRECTION_FACTOR = 1.0; // Optional calibration
const float ACS_SENSITIVITY = 0.185;		 // Volts per Amp (185mV/A)
const int ACS_ZERO_OFFSET = 512;			 // Theoretical zero-current ADC reading (1023/2)

// Create objects for our hardware
BH1750 lightMeter1(BH1750_ADDR1);
BH1750 lightMeter2(BH1750_ADDR2);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
ControlSystem control;
File dataFile;
String dataString = ""; // String to hold data for SD card logging
const char *FILENAME = "datalog.csv";
uint32_t prevMillis = 0;
float lux1, lux2, vBatt, iBatt, vPV, iPV;
Scheduler scheduleSampling(200, taskSampling); // 200 milisecond sampling
Scheduler scheduleLogging(3000, taskLogging);  // 3000 milisecond logging
Scheduler scheduleControl(250, taskControl);   // 250 milisecond controlling

// 4. SETUP FUNCTION - Runs once at the beginning
void setup()
{
	Serial.begin(9600);
	Serial.println("Starting data logger...");
	control.begin(SERVO_PIN);

	// Initialize LCD
	lcd.init();
	lcd.backlight();
	lcd.setCursor(0, 0);
	lcd.print("Logger Starting");

	// Initialize I2C bus
	Wire.begin();
	Wire.setTimeout(3000); // 3 sec timeout

	// Initialize BH1750 sensors
	while (!lightMeter1.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
	{
		Serial.println(F("Error initializing BH1750 Sensor 1"));
		lcd.setCursor(0, 1);
		lcd.print("BH1750-1 Fail");
		delay(250);
	}
	Serial.println(F("BH1750 Sensor 1 OK"));
	while (!lightMeter2.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
	{
		Serial.println(F("Error initializing BH1750 Sensor 2"));
		lcd.setCursor(0, 1);
		lcd.print("BH1750-2 Fail");
		delay(250);
	}
	Serial.println(F("BH1750 Sensor 2 OK"));

	// Initialize SD Card
	Serial.print("Initializing SD card...");
	lcd.setCursor(0, 1);
	lcd.print("SD Card...");

	while (!SD.begin(SD_CS_PIN))
	{
		Serial.println("initialization failed!");
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("SD Card Failed!");
		delay(250);
	}
	Serial.println("initialization SD card done.");
	lcd.print("SYSTEM OK!");
	delay(1000);

	// Create header row in CSV file
	dataFile = SD.open(FILENAME, FILE_WRITE);
	if (dataFile)
	{
		dataFile.println("Timestamp (ms),Lux 1,Lux 2,Voltage PV (V),Current PV(A),Voltage BATT (V),Current BATT(A)");
		dataFile.close();
		Serial.println("CSV header written.");
	}
	else
	{
		Serial.println("Error opening datalog.csv");
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("File Error!");
	}

	lcd.clear();
	lcd.print("Setup Complete");
	delay(1000);
}

void taskSampling()
{
	// -------- Read Sensor Values --------
	lux1 = lightMeter1.readLightLevel();
	lux2 = lightMeter2.readLightLevel();

	float vOutBatt = (analogRead(VOLTAGE_BAT_PIN) * 5.0) / 1023.0;
	vBatt = vOutBatt * (R1 + R2) / R2 * VOLTAGE_CORRECTION_FACTOR;
	float vOutPV = (analogRead(VOLTAGE_PV_PIN) * 5.0) / 1023.0;
	vPV = vOutPV * (R1 + R2) / R2 * VOLTAGE_CORRECTION_FACTOR;

	float currentVoltageBatt = (analogRead(ACS712_BAT_PIN) - ACS_ZERO_OFFSET) * (5.0 / 1023.0);
	iBatt = currentVoltageBatt / ACS_SENSITIVITY;
	float currentVoltagePV = (analogRead(ACS712_PV_PIN) - ACS_ZERO_OFFSET) * (5.0 / 1023.0);
	iPV = currentVoltagePV / ACS_SENSITIVITY;

	// -------- Display on LCD --------
	lcd.clear();

	// Line 1: Lux values
	lcd.setCursor(0, 0);
	lcd.print("L1:" + String(lux1, 0)); // Print lux1 with 0 decimal places
	lcd.setCursor(8, 0);
	lcd.print("L2:" + String(lux2, 0)); // Print lux2 with 0 decimal places

	// Line 2: Voltage and Current PV
	lcd.setCursor(0, 1);
	lcd.print("V:" + String(vPV, 2)); // Print voltage with 2 decimal places
	lcd.setCursor(8, 1);
	lcd.print("A:" + String(iPV, 2)); // Print current with 2 decimal places

	//! URGENT THINK HOW TO SERVE THE DATA
	// Line 3: Voltage and Current Batt
	// lcd.setCursor(0, 2);
	// lcd.print("V:" + String(vBatt, 2)); // Print voltage with 2 decimal places
	// lcd.setCursor(8, 2);
	// lcd.print("A:" + String(iBatt, 2)); // Print current with 2 decimal places
}

void taskLogging()
{
	// -------- Log to SD Card --------
	dataString = String(millis()) + "," +
				 String(lux1) + "," + String(lux2) + "," +
				 String(vPV) + "," + String(iPV) + "," +
				 String(vBatt) + "," + String(iBatt);

	dataFile = SD.open(FILENAME, FILE_WRITE);
	if (dataFile)
	{
		dataFile.println(dataString);
		dataFile.close();
		Serial.println(dataString);
	}
	else
	{
		Serial.println("Error writing to SD card.");
		lcd.clear();
		lcd.print("SD Write Error!");
	}
}

void taskControl()
{
	control.runControl(lux1, lux2);
}

void loop()
{
	scheduleControl.runTask();
	scheduleLogging.runTask();
	scheduleSampling.runTask();
}