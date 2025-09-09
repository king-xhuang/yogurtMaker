//
// Sample of using Async reading of Dallas Temperature Sensors
//
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 10

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

DeviceAddress tempDeviceAddress;

int  resolution = 12;
unsigned long lastTempRequest = 0;
unsigned long timeForConv= 0;
int  delayInMillis = 0;
float temperature = 0.0;
int  idle = 0;
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
  sensors.setResolution(tempDeviceAddress, resolution);
  
  sensors.setWaitForConversion(false);
  lastTempRequest = millis();
  sensors.requestTemperatures();  
}

void loop(void)
{

  if (sensors.isConversionComplete())
  {  
    temperature = sensors.getTempCByIndex(0);
    timeForConv = millis() - lastTempRequest;
    Serial.print(" Temperature: ");
    Serial.println(temperature );

    Serial.print("  Resolution: ");
    Serial.println(resolution); 
    Serial.print("time=");
    Serial.println(timeForConv); 

    // immediately after fetching the temperature we request a new sample
    // in the async modus
    // for the demo we let the resolution change to show differences
    resolution++;
    if (resolution > 12) resolution = 9;

    sensors.setResolution(tempDeviceAddress, resolution);
    lastTempRequest = millis();
    sensors.requestTemperatures(); 
    
  }

  digitalWrite(13, HIGH);
  // we can do usefull things here
  // for the demo we just count the idle time in millis
  delay(1);
  idle++;
}
