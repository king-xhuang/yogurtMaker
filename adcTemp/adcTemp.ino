//
//  using Async reading of Dallas Temperature Sensors DS18B20 and Async reading of MCP3421 adc
//
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Adafruit_MCP3421.h"
#include <PrintEx.h> 
#include <math.h>

Adafruit_MCP3421 mcp;
// Data wire is plugged into port 2 on the Arduino

#define ONE_WIRE_BUS 10
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
 
DeviceAddress tempDeviceAddress;
PrintEx PExSerial = Serial; //Wrap the Serial object in a PrintEx interface.
int  resolution = 10;
unsigned long lastTempRequest = 0;
unsigned long curTime = 0;
unsigned long adcTime = 0;
unsigned long tempTime = 0;
int  delayInMillis = 0;
float temperature = -127.0;
int  idle = 0;
long adcValue = -2000;
boolean adcDone = false;
boolean tempDone = false;
const short WaitNone = 1;
const short WaitAdc = 2;
short waitFor = WaitNone;

#define N_SEGMENTS 15
#define N_CHECK 16

typedef struct {
    double x_start;
    double x_end;
    double slope;
    double intercept;
} Segment;
Segment segments[N_SEGMENTS] = {
    {501.000000, 627.255572, 0.029508, 1.216293},
    {627.255572, 763.201308, 0.040170, -5.471393},
    {763.201308, 776.801195, 0.062038, -22.160905},
    {776.801195, 861.955228, 0.055010, -16.701379},
    {861.955228, 985.258211, 0.034212, 1.225793},
    {985.258211, 1073.848327, 0.039593, -4.076047},
    {1073.848327, 1112.870210, 0.039891, -4.396021},
    {1112.870210, 1113.811833, 0.056683, -23.083279},
    {1113.811833, 1280.274532, 0.059623, -26.358843},
    {1280.274532, 1342.996401, 0.089497, -64.605109},
    {1342.996401, 1408.000123, 0.067858, -35.544605},
    {1408.000123, 1473.999932, 0.075758, -46.666662},
    {1473.999932, 1583.500026, 0.087622, -64.154618},
    {1583.500026, 1675.005538, 0.105624, -92.661051},
    {1675.005538, 1757.500000, 0.130193, -133.814634}
};
double piecewise_fit(double x) {
    if(x <= segments[0].x_start) return segments[0].slope*x + segments[0].intercept;
    if(x >= segments[N_SEGMENTS-1].x_end) return segments[N_SEGMENTS-1].slope*x + segments[N_SEGMENTS-1].intercept;
    int i = 0;
    for( i=0;i<N_SEGMENTS;i++) {
        if(x>=segments[i].x_start && x<=segments[i].x_end) return segments[i].slope*x + segments[i].intercept;
    }
    return 200.0;
}
double A = 3.3170353688e-04;
double B = 2.8188645815e-04;
double C = -2.2394141349e-08;
double steinhart_hart_C(double R_ohm, double A, double B, double C) {
    double lnR = log(R_ohm);                // natural log
    double invT = A + B*lnR + C*lnR*lnR*lnR;
    double T_K = 1.0 / invT;
    return T_K - 273.15;                    // Celsius
}
double steinhartHart_R2C(double R_ohm){
  return steinhart_hart_C( R_ohm,   A,   B,   C);
}
double Vtest = 2505.0;  //# mv
double Vbase = 507.0; // # mv
double R     = 50.06;  // # kOhm
double Rntc  = 0.0;   //# kOhm
double v2Rntc(double v ){
  double Vr = v + Vbase;
    Rntc = R*Vtest/Vr - R;
    return  Rntc*1000;
}
    
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
  double t1 = 0.0;
  double t2= 0.0;
  double t3= 0.0;
  double t4= 0.0;
  double t5= 0.0;

  unsigned long bfC = millis();
 for(int i =0;  i<200;i++){
      t1 = steinhartHart_R3C(75000);
      t2 = steinhartHart_R3C(50000);
      t3 = steinhartHart_R3C(11350.53);
      t4 = steinhartHart_R3C(7754.8);
      t5 = steinhartHart_R3C(5012.6);
 }
  

  unsigned long ct = millis() - bfC;
  Serial.print("time for call steinhartHart_R3C() 1000 times: ");
  Serial.println( ct);
  Serial.println(t1);
  Serial.println(t2);
  Serial.println(t3);
  Serial.println(t4);
  Serial.println(t5);

  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, resolution); 
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

  sensors.setWaitForConversion(false);
  waitFor = WaitNone;
  lastTempRequest = millis(); 
  sensors.requestTemperatures(); 
}

void loop(void)
{ 
  if(sensors.isConversionComplete()  && waitFor == WaitNone)
  { 
    temperature = sensors.getTempCByIndex(0);
    tempTime =  millis() - lastTempRequest;
    
    lastTempRequest = millis();
    mcp.startOneShotConversion();  
    waitFor = WaitAdc;
  } 
  else if (mcp.isReady() && waitFor == WaitAdc) {
         
        adcValue = mcp.readADC(); // Read ADC value
        adcTime = millis() - lastTempRequest;   
        double adc2temp = piecewise_fit(adcValue);
        double adc2tempHS = steinhartHart_R3C(v2Rntc(adcValue));
        double d = adc2tempHS - temperature;
        Serial.print(temperature);
        Serial.print(",   ");
        Serial.print(adcValue);
        Serial.print(",   ");
        Serial.print(adc2temp);
        Serial.print(",   ");
        Serial.print(adc2tempHS);
        Serial.print(",   ");
        Serial.print(d);
        Serial.print(",   ");
        Serial.print(tempTime);
        Serial.print(",");
        Serial.println(adcTime);
        delay(320); 
        // start next cycle
        temperature = -127.0;
        adcValue = -2048;
         
        lastTempRequest = millis();
        sensors.requestTemperatures();
        waitFor = WaitNone;
  }
  //(millis() - lastTempRequest >= delayInMillis) // waited long enough??
  else{
    delay(1); 
  }
  
  // we can do usefull things here
  // for the demo we just count the idle time in millis
 
}
