// 1. INCLUDE LIBRARIES
#include <Wire.h>			   // For I2C communication
#include <SPI.h>			   // For SPI communication (SD Card)
#include <SD.h>				   // For SD Card
#include <BH1750.h>			   // For BH1750 light sensors
#include <LiquidCrystal_I2C.h> // For I2C LCD
#include "Control_System.h"	   // For Controlling Servo

// Forward declarations of task functions
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

// 3. CONFIGURE SENSORS & OBJECTS
const float R1 = 30000.0;					 // Resistor from input voltage to analog pin
const float R2 = 7500.0;					 // Resistor from analog pin to ground
const float VOLTAGE_CORRECTION_FACTOR = 1.0; // Optional calibration
const float ACS_SENSITIVITY = 0.185;		 // Volts per Amp (185mV/A) for 5A module
const int ACS_ZERO_OFFSET = 512;			 // Theoretical zero-current ADC reading (1023/2)

// Create objects for our hardware
BH1750 lightMeter1(BH1750_ADDR1);
BH1750 lightMeter2(BH1750_ADDR2);
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4); // Configured for a 20x4 display
ControlSystem control;
File dataFile;
const char *FILENAME = "datalog.csv";

// Global variables for sensor readings
float lux1, lux2, vBatt, iBatt, vPV, iPV;

// --- millis() Based Task Scheduling ---
unsigned long previousMillisSampling = 0;
unsigned long previousMillisControl = 0;
unsigned long previousMillisLogging = 0;

const long intervalSampling = 200; // Sample data every 200 ms
const long intervalControl = 250;  // Run control logic every 250 ms
const long intervalLogging = 3000; // Log data to SD card every 3 seconds

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
	Wire.setTimeout(500); // 3 sec timeout

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
	Serial.println("Setup Complete");
	delay(1000);
}

void taskSampling()
{
	// -------- Read All Sensor Values --------
	lux1 = lightMeter1.readLightLevel();
	lux2 = lightMeter2.readLightLevel();
	lcd.clear();

	float vOutBatt = (analogRead(VOLTAGE_BAT_PIN) * 5.0) / 1023.0;
	vBatt = vOutBatt * (R1 + R2) / R2 * VOLTAGE_CORRECTION_FACTOR;

	float vOutPV = (analogRead(VOLTAGE_PV_PIN) * 5.0) / 1023.0;
	vPV = vOutPV * (R1 + R2) / R2 * VOLTAGE_CORRECTION_FACTOR;

	float currentVoltageBatt = (analogRead(ACS712_BAT_PIN) - ACS_ZERO_OFFSET) * (5.0 / 1023.0);
	iBatt = currentVoltageBatt / ACS_SENSITIVITY;

	float currentVoltagePV = (analogRead(ACS712_PV_PIN) - ACS_ZERO_OFFSET) * (5.0 / 1023.0);
	iPV = currentVoltagePV / ACS_SENSITIVITY;

	// -------- Display on LCD (NO FLICKER METHOD) --------
	// char buffer[128]; // Buffer to hold formatted string for one line (20 chars + null)

	// Line 1: PV Data
	lcd.setCursor(0, 0);
	lcd.print("P :");
	lcd.print(vPV, 1);
	lcd.print("V ");
	lcd.print(iPV, 2);
	lcd.print("A");

	// Line 2: Battery Data
	lcd.setCursor(0, 1);
	lcd.print("B:");
	lcd.print(vBatt, 1);
	lcd.print("V ");
	lcd.print(iBatt, 2);
	lcd.print("A");

	// Line 3: Lux Data
	lcd.setCursor(0, 2);
	lcd.print("L1:");
	lcd.print(lux1, 0);
	lcd.print(" L2:");
	lcd.print(lux2, 0);
}

void taskLogging()
{
	// Open file, write directly, and close
	dataFile = SD.open(FILENAME, FILE_WRITE);
	if (dataFile)
	{
		// Write data directly to file without creating String objects
		dataFile.print(millis());
		dataFile.print(",");
		dataFile.print(lux1);
		dataFile.print(",");
		dataFile.print(lux2);
		dataFile.print(",");
		dataFile.print(vPV);
		dataFile.print(",");
		dataFile.print(iPV);
		dataFile.print(",");
		dataFile.print(vBatt);
		dataFile.print(",");
		dataFile.println(iBatt);
		dataFile.close();

		// Also print to serial for debugging
		Serial.print(millis());
		Serial.print(",");
		Serial.print(lux1);
		Serial.print(",");
		Serial.print(lux2);
		Serial.print(",");
		Serial.print(vPV);
		Serial.print(",");
		Serial.print(iPV);
		Serial.print(",");
		Serial.print(vBatt);
		Serial.print(",");
		Serial.println(iBatt);
	}
	else
	{
		Serial.println("Error writing to SD card.");
		lcd.setCursor(12, 0);
		lcd.print("SD_WRITE");
	}
}

void taskControl()
{
	control.runControl(lux1, lux2);
}

void loop()
{
	// Get the current time once at the start of the loop
	unsigned long currentMillis = millis();

	// --- Task 1: Data Sampling ---
	if (currentMillis - previousMillisSampling >= intervalSampling)
	{
		previousMillisSampling = currentMillis;
		taskSampling();
	}

	// --- Task 2: Servo Control ---
	if (currentMillis - previousMillisControl >= intervalControl)
	{
		previousMillisControl = currentMillis;
		taskControl();
	}

	// --- Task 3: SD Card Logging ---
	if (currentMillis - previousMillisLogging >= intervalLogging)
	{
		previousMillisLogging = currentMillis;
		taskLogging();
	}
}