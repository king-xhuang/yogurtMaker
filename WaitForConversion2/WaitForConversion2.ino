//
// Sample of using Async reading of Dallas Temperature Sensors
//
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Adafruit_MCP3421.h"

Adafruit_MCP3421 mcp;
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 10

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
 
DeviceAddress tempDeviceAddress;

int  resolution = 10;
unsigned long lastTempRequest = 0;
unsigned long curTime = 0;
unsigned long adcTime = 0;
unsigned long tempTime = 0;
int  delayInMillis = 0;
float temperature = 0.0;
int  idle = 0;
int32_t adcValue = 0;
//
// SETUP
//
void setup(void)
{
  Serial.begin(9600);

  Serial.println("Dallas Temperature Control Library - Async Demo");
  Serial.print("Library Version: ");
  Serial.println(DALLASTEMPLIBVERSION);
  Serial.println("\n");

  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, 10); 
  // Begin can take an optional address and Wire interface
  if (!mcp.begin(0x68, &Wire)) { 
    Serial.println("Failed to find MCP3421 chip");
    while (1) {
      delay(10); // Avoid a busy-wait loop
    }
  }
  Serial.println("MCP3421 Found!");

  // Options: GAIN_1X, GAIN_2X, GAIN_4X, GAIN_8X
  mcp.setGain(GAIN_1X); 

  // The resolution affects the sample rate (samples per second, SPS)
  // Other options: RESOLUTION_14_BIT (60 SPS), RESOLUTION_16_BIT (15 SPS), RESOLUTION_18_BIT (3.75 SPS)
  mcp.setResolution(RESOLUTION_12_BIT); // 240 SPS (12-bit) 
  // Test setting and getting Mode
  mcp.setMode(MODE_ONE_SHOT); // Options: MODE_CONTINUOUS, MODE_ONE_SHOT 
  lastTempRequest = millis();
  mcp.startOneShotConversion();
  
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures(); 
}

void loop(void)
{
  if (mcp.isReady()) {
        adcValue = mcp.readADC(); // Read ADC value
       // uint32_t currentMillis = millis();
        Serial.print("ADC reading: ");
        Serial.println(adcValue);
        // Serial.println( millis() - lastSecond);
         
       
  }
  //(millis() - lastTempRequest >= delayInMillis) // waited long enough??
  if(sensors.isConversionComplete())
  { 
    temperature = sensors.getTempCByIndex(0);
    curTime = millis();
    // Serial.print(" Temperature: ");
   
    // Serial.println(temperature, resolution - 8);
    // Serial.print("  Resolution: ");
    // Serial.println(resolution);
    // Serial.print("c time: ");
    // Serial.println( millis() - lastTempRequest);
    
    idle = 0;
    delay(320);
    // immediately after fetching the temperature we request a new sample
    // in the async modus
    // for the demo we let the resolution change to show differences
    //resolution++;
    if (resolution > 12) resolution = 9;

    sensors.setResolution(tempDeviceAddress, resolution);
     lastTempRequest = millis();
    sensors.requestTemperatures();
     mcp.startOneShotConversion();
    
   
  }

  
  // we can do usefull things here
  // for the demo we just count the idle time in millis
  delay(1); 
}
