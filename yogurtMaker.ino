// include the library code: 
#include <OneWire.h>
//#include <DallasTemperature.h> 
#include "TimerOne.h"
#include "Adafruit_MCP3421.h"
#include <PrintEx.h> 
#include <math.h>

#define TEST_WAVE 1  // generate test pules at pin 11,13 to trace cycle and temp req/conversion, heating time

// Data wire is plugged into pin 10 on the Arduino
#define ONE_WIRE_BUS 10  // temp sensor input pin


const byte RedLedPIN =  6;
const byte YellowLedPIN =  4;
const byte GreenLedPIN =  5;
const byte LedSize = 2;

const byte BtPIN= 2;
const byte acSyncPIN =  3;
const byte powerLevelPIN =  7;
const byte RelayPIN = 8;
const byte BuzzerPIN = 9;
const byte WTCyclePIN = 11;  // cycle signal plus out
const byte WTReqPIN = 13;    // temp request conversion plus out

const byte stage0 = 0; // stage 0
const byte stage1 = 1; // stage 1 
const byte sComplete = 2; // stage Complete
const byte sWarning = 5;  //  stage warning 
const byte sInit = 10; // initial stage
volatile byte currentStage = sInit; 

volatile int prog = -1; //program 0 - yogurt maker, 1 - sous vide
// const char *const string_table[] = PROGMEM = {"yogurt maker", "sous vide"};
// char buffer[15];  // make sure this is large enough for the largest string it must hold
//const  String pNames[] PROGMEM = {"yogurt maker", "sous vide"};

const float InvalidTemp = -127.00;
volatile float tempC = InvalidTemp;

const unsigned long minuteInMillis PROGMEM = 60000; 
const unsigned long hourInMillis PROGMEM = 60*60000;
const byte stageLedPins[] = {RedLedPIN, YellowLedPIN};

float targetTemps[]  ={ 87.0, 39.0  }; //in C  yogert ferment: 36 ~ 43° C (96.8 ~ 109.4°F), pasteurizing milk: 71~83°C (160~180°F)  
// float targetTemps[] ={ 50.5, 40.0, 30.0 }; //  test data

unsigned long stageHoldTimes[] = {7*minuteInMillis, 7*60*minuteInMillis  };
//unsigned long stageHoldTimes[] = {10*minuteInMillis, 10*minuteInMillis, 1*minuteInMillis }; //   test data


// float delta = 0.5; 
// float deltaMax = 3.0;
// float deltaTemp = deltaMax;

volatile float lastTempDiff = 0.0;
volatile float tempDiff = 0.0;
Adafruit_MCP3421 mcp;
// // Setup a oneWire instance to communicate with any OneWire devices
// OneWire oneWire(ONE_WIRE_BUS);
// // Pass our oneWire reference to Dallas Temperature. 
// DallasTemperature sensors(&oneWire);

// // Assign the addresses of your 1-Wire temp sensors.
// // See the tutorial on how to obtain these addresses:
// // http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html

// //DeviceAddress thermometer = { 0x28, 0x35, 0x88, 0x43, 0x05, 0x00, 0x00, 0xB0 };
// DeviceAddress thermometer; // = { 0x28, 0x4E, 0xF7, 0x07, 0xD6, 0x01, 0x3C, 0x18 };

int ledState = LOW;             // ledState used to set the LED
unsigned long ledPrevMillis = 0;         
unsigned long ledInterval = 500; // led flush interval in millis

volatile uint8_t  buttonState = HIGH; 
volatile uint8_t  lastButtonState = HIGH;
volatile uint8_t  btDownCount = 0;

unsigned long btPrevMillis = 0;         
unsigned long btInterval = 1000; // button push check interval in millis

unsigned long tempPrevMillis = 0;
unsigned long tempInterval = 500;  // tempreture check interval in millis
volatile float targetTempC = 0;
uint8_t stageLedPin = RedLedPIN;

unsigned long stageStartTime = 0;
unsigned long stageHoldStartTime = 0;
volatile boolean reachTargetTemp = false;
unsigned long stageHoldTime = 1;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean debugPrint = false;

boolean initSetTime = false;
boolean initSetTemp = false;

// int initCheck = 0;
// int checkLimit = 20;
// int deltaChangeCount = 0;
// int deltaChangeCountInMin = 60;

const uint8_t TickCountMax = 60; // 30 ac power line cycle
const uint8_t HalfCycle = TickCountMax/2;
//const uint8_t ReqTempCount = 38; // (TickCountMax - ReqTempCount) * 1/timerFrequency  > temp sensor conversion time
const uint8_t ReqMCPAdcCount = TickCountMax - 2; //TODO

volatile uint8_t timerTickCount = 0;
//uint8_t heatingTickCount = 0;
volatile uint8_t heatingLevel = 0;  // from 0 to  TickCountMax 
 
const uint8_t HeatingTrendNone = 0;  // heater not working
const uint8_t HeatingTrendRise = 1;  // heater keeps on
const uint8_t HeatingTrendFall = 2;  // heater keeps off
const uint8_t HeatingTrendHold = 3;  // heater keeps holding temp
volatile uint8_t heatingTrend = HeatingTrendNone;

const uint8_t trsIdal = 0;
const uint8_t trsWaitForComplete = 2;
volatile uint8_t tempReadStatus = trsIdal; //idal,  1 - send req, 2 - wait, 3 - completed
 
const short WaitNone = 1; 
const short WaitAdcReady = 3;
volatile short waitFor = WaitNone;

volatile unsigned long reqTempTime = 0; 
PrintEx PExSerial = Serial; //Wrap the Serial object in a PrintEx interface.

const double Vtest = 2505.0;  //# mv
const double Vbase = 507.0; // # mv
const double R     = 50.06;  // # kOhm
double Rntc  = 0.0;   //# kOhm
double v2Rntc(double v ){
  double Vr = v + Vbase;
    Rntc = R*Vtest/Vr - R;
    return  Rntc*1000;
}
const int32_t AdcInvalidValue = 2000;
volatile int32_t adcValue = AdcInvalidValue;
void resetAdcNTemp(){
   adcValue = AdcInvalidValue;
   tempC = InvalidTemp;
}
const double A = 3.3170353688e-04;
const double B = 2.8188645815e-04;
const double C = -2.2394141349e-08;
double steinhart_hart_C(double R_ohm, double A, double B, double C) {
    double lnR = log(R_ohm);                // natural log
    double invT = A + B*lnR + C*lnR*lnR*lnR;
    double T_K = 1.0 / invT;
    return T_K - 273.15;                    // Celsius
}
double steinhartHart_R2C(double R_ohm){
  return steinhart_hart_C( R_ohm,   A,   B,   C);
}
float c2F(float C){
  return C*9/5+32;
}
// float getDelta(){// detla is decresed when temp rising and close to the target
//   if (currentStage == 1) {
//     deltaTemp = 0.5;
//     return deltaTemp;
//   }
//   if ( !reachTargetTemp ){
//     deltaTemp = deltaMax;    // this make heating stop earlier at the initial heating phase.
//   }else{
//     deltaChangeCount++;
//     if(deltaChangeCount > deltaChangeCountInMin*1){// change deltaTemp every 3 minutes after reachTargetTemp
//       deltaChangeCount = 0;
//       deltaTemp = deltaTemp/2.0;    
//       if (deltaTemp < delta){
//         deltaTemp = delta;
//       }
//       Serial.print("current delta ");
//       Serial.println(deltaTemp);
//     }  
//   }
//   return deltaTemp; 
// }
// program name
String getPName(byte i){ 
  if (i == 0) return "yogurt maker";
  else return "sous vide";
  
}
// set initial condition for a stage
void startStage(byte sid){  // start a new stage
    
    String s = "#### start Stage ";
    s += String(sid);
    Serial.println(s);
    if (!isWorkStage(sid)) {
      heatingTrend = HeatingTrendNone;
      heaterOn(false);	
      if(sid == sInit) {
        Serial.println(F("push button to start" ));
      }
      else if (sid == sComplete ||  sid == sWarning ) {  
        if (sid == sComplete) Serial.println(F("##### program well done #####" ));    
        else  Serial.println(F("##### program end with Warning !!! #####" ));  

        currentStage = sid;  
        setBuzzer(true);    
      }       
       
      return;
    }
    else // working stages
    {         
      for (byte i = 0; i < LedSize; i++){
        digitalWrite( stageLedPins[i], LOW);
      }      
      if(sid == stage0){
        heatingTrend = HeatingTrendRise;
      }else if(sid == stage1 && prog == 0){ // yogurt maker stage 1
        heatingTrend = HeatingTrendFall;
      }
      reachTargetTemp = false;
      stageStartTime = millis();
      stageHoldTime = stageHoldTimes[sid];
      currentStage = sid;
      targetTempC = targetTemps[sid];
      stageLedPin = stageLedPins[sid];
      setLeds();
      setBuzzer(true);
      Serial.print(F("target temp: " ));
      Serial.print(String(targetTempC));
      Serial.println(F(" C"));
      Serial.print(F( "hold time: "));
      Serial.print(stageHoldTime/minuteInMillis);  
      Serial.println(F(" minutes" ));               
	}
}

void setDoneSignal(){
  heaterOn(false);
  timer1ToggleLeds();
}
void timer1ToggleLeds(){
  for (int i = 0; i < LedSize; i++){     
    digitalWrite(stageLedPins[i], digitalRead(stageLedPins[i] ) ^ 1); 
  }
}
void setBuzzer(boolean on){
  if (on){
	  tone(BuzzerPIN, 500);//TODO
	//tone(BuzzerPIN, 500, 2*minuteInMillis);
  }
  else noTone(BuzzerPIN);
}

void setLeds(){
  for (byte i = 0; i < LedSize; i++){
    if (i < currentStage) digitalWrite( stageLedPins[i], HIGH);
  }
}

boolean isWorkStage(int sid){
  return (sid  == stage0 || sid  == stage1 );
}

boolean isDone(){
  return (currentStage == sComplete || currentStage == sWarning );
}

void adc2Temp(int32_t adcValue){
  double r = v2Rntc(adcValue);
  tempC = steinhartHart_R2C(r);
  unsigned long convTime = millis() - reqTempTime; 
  if(debugPrint){
      Serial.print(F("adc = " ));
    Serial.print(adcValue);
    Serial.print(F(",  r = " ));
    Serial.print(r);
     Serial.print(F(",  tempC = " ));
    Serial.print(tempC);
    Serial.print(F(",  time = " ));
    Serial.println(convTime);
  }
 }

void setup() {
  // start serial port
  Serial.begin(115200);
  
  Serial.println(F("#### yogurt maker powered ON ####"));
  if (!mcp.begin(0x68, &Wire)) { 
    Serial.println(F("Failed to find MCP3421 chip" ));
    while (1) {
      delay(10); // Avoid a busy-wait loop
    }
  }
  Serial.println(F("MCP3421 Found!"));
  // Options: GAIN_1X, GAIN_2X, GAIN_4X, GAIN_8X
  mcp.setGain(GAIN_1X); 

  // The resolution affects the sample rate (samples per second, SPS)
  // Other options: RESOLUTION_14_BIT (60 SPS), RESOLUTION_16_BIT (15 SPS), RESOLUTION_18_BIT (3.75 SPS)
  mcp.setResolution(RESOLUTION_12_BIT); // 240 SPS (12-bit) 

   resetAdcNTemp();
   reqTempTime = millis();
  // Test setting and getting Mode
  mcp.setMode(MODE_CONTINUOUS); // Options: MODE_CONTINUOUS, MODE_ONE_SHOT  
  //mcp.startOneShotConversion();  
  waitFor = WaitAdcReady;
  while ( !mcp.isReady() && waitFor == WaitAdcReady) {  
        delay(1); // Avoid a busy-wait loop 
  }
  adcValue = mcp.readADC(); // Read ADC value  
  waitFor = WaitNone;
  double r = v2Rntc(adcValue);
  tempC = steinhartHart_R2C(r);
  unsigned long convTime = millis() - reqTempTime; 
  PExSerial.printf("first conversion time = %u  \n", convTime );  
   Serial.print(F("adc = " ));
  Serial.print(adcValue);
   Serial.print(F(",  r = " ));
  Serial.print(r);
  Serial.print(F( ",  tempC = "));
  Serial.println(tempC);
  printTemperature(tempC);

  pinMode(BuzzerPIN, OUTPUT); 
  pinMode(RelayPIN, OUTPUT);  // control relay
  pinMode(powerLevelPIN, OUTPUT); // control Triac
  heaterOn(false);

  pinMode(RedLedPIN, OUTPUT);
  pinMode(YellowLedPIN, OUTPUT);  
  //pinMode(GreenLedPIN, OUTPUT);

  pinMode(BtPIN, INPUT_PULLUP);
  pinMode(acSyncPIN, INPUT);
  #ifdef TEST_WAVE
    pinMode(WTCyclePIN, OUTPUT); 
    pinMode(WTReqPIN, OUTPUT);
  #endif   
  // sensors.begin();
  
  // sensors.getAddress(thermometer, 0);
  // sensors.setResolution(thermometer, 10); 
  // sensors.setWaitForConversion(false);
  // reqTempTime = millis();
  // // sensors.requestTemperatures();    
  // while(!sensors.isConversionComplete()){
  //   delay(10);
  // } 
  // tempC = sensors.getTempCByIndex(0);
  // //tempC = sensors.getTempC(thermometer); 
  // unsigned long convTime = millis() - reqTempTime; 
  // PExSerial.printf("first conversion time = %u  \n", convTime );  
  // printTemperature(tempC);
  //PExSerial.printf("Temp = %f \n", tempC);
  if (tempC == -InvalidTemp) { // cannot get temp from sensor
    currentStage = sWarning;  
    startStage(sWarning);   
  } 
  tempReadStatus = trsIdal; 
  tempC = InvalidTemp;

  // reserve 200 bytes for the inputString:
  inputString.reserve(20);

  //Timer1.initialize(10000);         // initialize timer1, and set a 10 ms   period
  //Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  //Timer1.attachInterrupt(timer1Callback);  // attaches callback() as a timer overflow interrupt
  attachInterrupt(digitalPinToInterrupt(BtPIN), onButtonDown, FALLING );
  attachInterrupt(digitalPinToInterrupt(acSyncPIN), onMainLineSync, RISING );

  Serial.println(F("### v3.1 11/25/25 final, push button to start ###" ));
}

void onButtonDown() { // call back for button down
  // start the program or stop the buzze
  if (isRebounce()){
    return;
  }
  Serial.print(F("btDown\nprog ="));
  Serial.println(prog);
  if (currentStage == sInit){
    if ( prog == -1 ){
      setProgram(0); // set default program to 0, if not set yet.
    }  
    startStage(stage0); // push button to kick off first stage
  } 
  else{
    setBuzzer(false);
  }        
}
void onMainLineSync(){
   timerTickCheck();
}
void timer1Callback()
{
  timerTickCheck(); 
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true; 
     
    }else if (inChar == 'A') {
      inputString = "";
    }

  }
} 

void printMenu(){
   Serial.println(F("m    - menu"));
   Serial.println(F("ss   - status"));
   Serial.println(F("std0.7 - tempreture delta to 0.7 C"));
   Serial.println(F("st30 - tempreture to 30 C") );
   Serial.println(F("sm30 - set time to 30 mim "));
   Serial.println(F("sp0  - Yogurt Maker"));
   Serial.println(F( "sp1  - Sous Vide")  );
   Serial.println(F( "sp2  - Kefir"));
   Serial.println(F("dp   - debug print"));
}

boolean checkProgram(){ 
 
  if( prog == 0 ){
    // Serial.print("start ");
    // Serial.println(getPName(p));
    return true;
  }
  else if (prog == 1 )
  {
    if (initSetTime && initSetTemp){
      // Serial.print("start ");
      // Serial.println(getPName(p));
      return true;
    } 
    if (!initSetTime){
      Serial.println("set time: sm");
        
    } 
    if(!initSetTemp){
      Serial.println("set Temp: st");
      
    }
   // delay(5000);
    return false;  
  }
  else
  {    
    Serial.println("set program: sp0/sp1");
    //delay(5000);
    //TODO set default program after 10 checks
    return false;
  } 
}


void handleCommand( String cmd){
    Serial.print(F("you say:" ));
    Serial.println(cmd);
    //printString(cmd);

    if (isCmd(cmd, "m")){
      printMenu();
    }else if (isCmd(cmd, "ss")){
      printStatus();
    }else if (isCmd(cmd, "sm")){
      setTimeInMin(getInt(cmd.substring(2)));
    }
    // else if (isCmd(cmd, "std")){
    //   setHeatingTempDelta(getFloat(cmd.substring(3)));
    // }
    else if (isCmd(cmd, "st")){
      setHeatingTemp(getFloat(cmd.substring(2)));
    }else if (isCmd(cmd, "sp0")){
      setProgram( 0 );
    }
    else if (isCmd(cmd,"sp1")){
      setProgram( 1 );
    }
    else if (isCmd(cmd,"sp2")){
      setProgram( 1 );
      setHeatingTemp(25);
      setTimeInMin(24*60);
    }
    else if (isCmd(cmd,"dp")){
       debugPrint = !debugPrint;
    }
    else if (isCmd(cmd,"sp11")){
      setProgram( 1 );
      setHeatingTemp(25);
      setTimeInMin(10);
    }
    else{
      Serial.print(F("unknown command " ));
      Serial.println( cmd);
      printMenu();
    }
}

int getInt(String s){  
  Serial.print("getInt len ");
  Serial.print(s.length());
  Serial.print(", val ");
  Serial.println(s);
  return s.toInt();
}

float getFloat(String floatAsStr){   
 // String n = s.substring(s.length()-1);
  Serial.print(F("getFloat   " ));
  Serial.print(floatAsStr.length());
  Serial.print(F(", val " ));
  Serial.println(floatAsStr);
  float f = floatAsStr.toFloat();
  Serial.println(f);
  return f;
}

int getNum(String s){
  Serial.println(s.length());
  String n = "";
  for (unsigned int i = 0;  i < (s.length()); i++){
    int c = s.charAt(i);
    if (isDigit(c)  ){
      n += (char)c;        
      dp("d"); 
      
    } else{
      dp("c");  
      
    }   
    dp(String((char)c));   
    dp("-"); 
    dp(String(c)); 
  }
  dpLn("end");  
  //String n = s.substring(2, s.length()-1);
  Serial.print("getNum len ");
  Serial.print(n.length());
  Serial.print(", val ");
  Serial.println(n);
  return n.toInt();
}
boolean isCmd(String cmd, String c){
  // Serial.print(cmd);
  // Serial.print(" indexOf ");
  // Serial.print(c);
  // Serial.print(" = ");
  // int i = cmd.indexOf(c);
  // Serial.println(i);
  // //Serial.println(cmd.substring(i));
  // return i >= 0;
  return cmd.startsWith(c);
}
// void printString(String s){
//   for (int i = 0; i++; i < s.length()){
//       Serial.print("#");
//       Serial.println(s.charAt(i));
     
//   } 
// }

void setProgram( int i){
   
  prog = i;
  Serial.print(F("setProgram =") );
  Serial.println(prog);
}
// void setHeatingTempDelta(float t){
//   Serial.print("setHeatingTempDelta ");
//   Serial.println(t);
//   delta = t;     
// }
void setHeatingTemp(float t){
  Serial.print("setHeatingTemp ");
  Serial.println(t);
  if (prog == 1){
    //float temp = float(t);    
    targetTemps[0] = t;
    targetTemps[1] = 0.0;
   
    initSetTemp = true;
  }else Serial.println(F("cannot set temp, enter 'sp1' to set p =1 first" ));
}

void setTimeInMin(int m){
  Serial.print(F("setTimeInMin "));
  Serial.print(m);
  Serial.println(" min");
  if (prog == 1){
    stageHoldTimes[0] = m*minuteInMillis;
    stageHoldTimes[1] = 1000; 
   
    initSetTime = true;
  }else Serial.println(F("cannot set time, set p =1 first" ));
}



void printStatus(){
  if (prog == 1 || prog == 0 ){
    Serial.println(getPName(prog));
    // temp and time setting
    for(int i = 0; i < 2; i++){ 
      String s = "stage "; 
      s += String(i);
      s += ", Temp ";
      s += targetTemps[i]; //String(targetTemps[i]);
      s += ", time ";       
      s += stageHoldTimes[i]/minuteInMillis; //String(stageHoldTimes[i]/minuteInMillis);
      s += " min";    
      Serial.println(s);
    }
    Serial.print(F("heating trend:  " ));
    if (heatingTrend == HeatingTrendRise){
      Serial.println(F(" Rise" ));
    }else if (heatingTrend == HeatingTrendHold){
      Serial.print(F(" Hold,  temp is "));S
      if (tempDiff >= lastTempDiff){ 
        Serial.println(F("DOWN" ));
      }else{
        Serial.println(F("UP" ));
      }
    } 
    else if (heatingTrend == HeatingTrendFall){
      Serial.println(F(" Fall"));
    } 
    else{
       Serial.println(F(" None" ));
    }
    Serial.print(F(" power level " ));
    Serial.println(heatingLevel);
    Serial.print(F("Power on " ));
    if(heatingTrend == HeatingTrendRise){
      Serial.println(digitalRead(RelayPIN));
    }else{
      Serial.println(digitalRead(powerLevelPIN));
    }
    
    
    if(isWorkStage(currentStage) ){
      int timePassed =  (millis() - stageStartTime)/minuteInMillis;
      // current stage, temp, time passed.
      printTemperature(tempC);
      Serial.print(F("Time passed = "));
      Serial.print(timePassed);
      Serial.println(F( " minutes")); 
    }else{
      Serial.print(F( "work stage "));
      Serial.println(currentStage);
    }

  }else{
    Serial.print(F( "program: "));
    Serial.println(prog);
  }
}


void loop() { 
       
  if (stringComplete){
    handleCommand(inputString);
    inputString = "";
    stringComplete = false; 
  }
   
  // if(waitFor == WaitAdcReq){
  //   Serial.println("startOneShotConversion");
  //   reqTempTime = millis();
  //   //mcp.startOneShotConversion();  // sent an async temp request; Making sure the conversion time is less than checkTemp() calling cycle and keeping them in sync.
  //   waitFor == WaitFor;
  // }
  if (waitFor == WaitAdcReady && mcp.isReady()){ 
    adcValue = mcp.readADC(); // Read ADC value  
    waitFor = WaitNone;
    adc2Temp(adcValue);
    // tempC = steinhartHart_R2C(v2Rntc(adcValue)); 
    // if (debugPrint){  
    //   PExSerial.printf("conversion time = %u \n",  millis() - reqTempTime); 
    // } 
    #ifdef TEST_WAVE
      digitalWrite(WTReqPIN, LOW);  // request begin 
    #endif
    
  } 
  

  //run();
  //testDone();
   //testBlink2();
   //testRelay(); 
  // if (tempReadStatus == trsWaitForComplete && sensors.isConversionComplete()){
  //   tempC = sensors.getTempCByIndex(0);
  //   tempReadStatus = trsIdal;
  //   #ifdef TEST_WAVE
  //   digitalWrite(WTReqPIN, LOW);  // request begin 
  //   #endif
  //   if (debugPrint){  
  //     PExSerial.printf("conversion time = %u \n",  millis() - reqTempTime); 
  //   } 
  // } 

}

void timerTickCheck(){ 
      //  if (!isWorkStage(currentStage)){
      //    blinkStageLed();
      //  } 
      // Serial.print("timerTickCheck ");
      // Serial.println(timerTickCount);
  timerTickCount++; 
  if (timerTickCount >= TickCountMax ){// restart a new a heating cycle
        //if(checkProgram()){ 
        //   timer1ToggleLeds();
        //   //setBuzzer(true);
        // }          
       // }
      #ifdef TEST_WAVE 
        digitalWrite(WTCyclePIN, HIGH); // cycle begin
      #endif
      startCycle();
      blinkStageLed();
  }else{ 
    // if (timerTickCount == ReqTempCount ){
    //   #ifdef TEST_WAVE
    //   digitalWrite(WTReqPIN, HIGH);  // request begin
    //   #endif
    //   reqTempTime = millis();
    //   sensors.requestTemperatures();
    //   tempReadStatus = trsWaitForComplete;  // sent an async temp request; Making sure the conversion time is less than checkTemp() calling cycle and keeping them in sync.
    // }
    if (timerTickCount == ReqMCPAdcCount ){
      #ifdef TEST_WAVE
      digitalWrite(WTReqPIN, HIGH);  // request begin
      #endif
      resetAdcNTemp();
      reqTempTime = millis(); 
      waitFor = WaitAdcReady; // set flag to notify loop to read ADC

    }
    #ifdef TEST_WAVE
      if(timerTickCount == HalfCycle){
        digitalWrite(WTCyclePIN,  LOW); // half cycle
      }
    #endif
    
    if (isWorkStage(currentStage)){
      checkHeatingPower();
    }
    else{
      heaterOn(false);
    }
    
  }     
    
}


void testDone(){   
  startStage(3);
  checkDone();
}

// void testBlink(){
   
//   while(true){
//   checkButton();
//   blinkStageLed();
//   }
// }
// void testBlink2(){
//     checkTemp();
//   checkButton();
//   blinkStageLed();
  
// }
  
  
void testRelay(){ 
    while(true){
      heaterOn(true);
      delay(2000);
      heaterOn(false);
      delay(2000);
    }    
  }

void checkDone(){  
  if (isDone()){
    Serial.println(F("##### program done #####" ));
     while(true){
      setDoneSignal();
     }
  }
}  
  void blinkStageLed(){
    if (currentStage == sInit){
      heaterOn(false);
      unsigned long currentMillis = millis();
      if(currentMillis - ledPrevMillis > ledInterval) {
	       dpLn("push button to start!!!!");
	       ledPrevMillis = currentMillis;   
         if (ledState == HIGH) ledState = LOW;
         else ledState = HIGH;
   	     for (int i = 0; i < LedSize; i++){
           digitalWrite( stageLedPins[i], ledState);
         } 
      }
    }
    else if (currentStage == sWarning){
      // heaterOn(false);
      // setBuzzer(true);
      // while(true){
      //    for (int i = 0; i < LedSize; i++){
      //      digitalWrite( stageLedPins[i], HIGH);
      //   }
      //   delay(200);
      //    for (int i = 0; i < LedSize; i++){
      //      digitalWrite( stageLedPins[i], LOW);
      //   }
      //   delay(200);
      // }
    }
    else if (currentStage == stage0 || currentStage == stage1){ // the current working stage led blinking
	   unsigned long currentMillis = millis();
	   if(currentMillis - ledPrevMillis > ledInterval) {
		    ledPrevMillis = currentMillis;   
		    toggleLed(stageLedPin);
	   }
	}
}
  
void toggleLed(int pin){
   if (ledState == HIGH) ledState = LOW;
   else ledState = HIGH;
   digitalWrite(pin, ledState );
}
void startCycle(){    
    if (heatingTrend == HeatingTrendNone 
    ||  heatingTrend == HeatingTrendFall
    ||  heatingTrend == HeatingTrendHold) {
      heatingLevel = 0;
      heaterOn(false);
    } // for heatingTrend == HeatingTrendRise, do nothing 
    
        
    unsigned long currentMillis = millis();
    timerTickCount = 0;
    if(debugPrint){
      PExSerial.printf("heater power cycle = %u \n", currentMillis - tempPrevMillis); 
      PExSerial.print("Temp = ");
      PExSerial.print(tempC);
      PExSerial.println(" C"); 
    }
    
    tempPrevMillis = currentMillis;

      //checkDone();
    if (isDone()){ 
      setDoneSignal();  
      return;     
    }
    if (!isWorkStage(currentStage)) return;

    if (reachTargetTemp && (currentMillis - stageHoldStartTime) > stageHoldTime){// move to next stage
          
      if ( prog == 1 && currentStage == stage0 ){ // sOUS vide
        currentStage = sComplete; // FORCE stage to sComplete
        startStage(currentStage);        
        return;
      }else if ( prog == 0 ){ // yogurt maker
        if(currentStage == stage1 ){ 
          currentStage = sComplete; // FORCE stage tosComplete
          startStage(currentStage);
          return;
        }else if(currentStage == stage0 ){
          currentStage = stage1;
          startStage(currentStage);
        }
        else return;
      }   
    }
      
    if (tempC == InvalidTemp) { // cannot get temp from sensor
      currentStage = sWarning;  
      startStage(sWarning);   
      return; 
    }
       // do get temp from sensor          
             // turn on/off relay
    lastTempDiff = tempDiff;
     tempDiff = targetTempC - tempC;
    
    // get sensor temp and reset heating level
    if(tempDiff <= 0.0){ // temp higher than target
      if (currentStage == stage0 && !reachTargetTemp){
        reachTarget();
      }
      heatingLevel = 0;// TODO use PID ???
      heaterOn(false); // TODO use PID ???
    }else{ // temp lower than target 
      if (currentStage > stage0 && !reachTargetTemp){ // temp falling to target 
        reachTarget();
      }    
      //TODO use PID ??? 
      //heatingLevel = ( tempDiff * TickCountMax )/tempDiff 
      if(heatingTrend == HeatingTrendRise){
        if (tempDiff <=  0.7){ 
           heatingTrend = HeatingTrendHold;
           Serial.print(F("### Turned Relay Off !!! ####"));
           heatingLevel = 0;
           heaterOn(false);
        }else{
           heatingLevel = TickCountMax + 1;
        }
      }
      if(heatingTrend != HeatingTrendRise){ // heatingTrend = HeatingTrendHold
        if ( tempDiff <= 0.5){
           heatingLevel = 0;
        }
        // else if (tempDiff <= 0.7){
        //   // if ( tempDiff >= lastTempDiff ){ // temp is down
        //    heatingLevel = 40;
        //   // }else{  // temp is up
        //   //   heatingLevel = 0;
        //   // }
        // }   
        else if (tempDiff > 0.5){             
          heatingLevel = TickCountMax; //TODO
        }     
      }
      heaterOn(heatingLevel > 0); 
    }
    if(debugPrint){
      PExSerial.printf("heatingLevel= %u \n", heatingLevel);
    } 
  }
  void checkHeatingPower(){ 
    if(heatingTrend == HeatingTrendRise){
      // never turn off the relay until temp close to target
      // 
    }
    else if(timerTickCount >= heatingLevel ){
      // turn off heater
      heaterOn(false);
    } 
  }
  void checkTemp(){ 
    // if (!isWorkStage(currentStage)) return;
    // timerTickCount++; 
    // if (timerTickCount >= TickCountMax ){// restart a new a heating cycle
    //   startCycle();
    // }else{ 
    //   if (timerTickCount == ReqTempCount ){
    //     tempReadStatus = 1;  // set flag to trigger an async temp request in loop(); Making sure the conversion time is less than checkTemp() calling cycle and keeping them in sync.
    //   }
    //   checkHeatingPower();
    // }    
       //printTemperature(tempC);
  }
    
  void reachTarget(){
  	reachTargetTemp = true;
    Serial.print(F("#### stage "));
    Serial.print(String(currentStage));
    Serial.print(F(" reach target temp " ));  
    Serial.println(String(targetTempC));           
    stageHoldStartTime = millis();
    heatingTrend = HeatingTrendHold; 
  }
  // set heater on/off -- true/false
  void heaterOn(boolean on){
    if (heatingTrend == HeatingTrendRise){
      digitalWrite(powerLevelPIN, LOW);
      if (on) {  
        digitalWrite(RelayPIN, HIGH);
        //digitalWrite(powerLevelPIN, HIGH);
      } else{
        digitalWrite(RelayPIN, LOW); 
        //digitalWrite(powerLevelPIN, LOW);
      }
    }else{ // TODO use triac
      digitalWrite(RelayPIN, LOW); 
      if (on) {  
        //digitalWrite(RelayPIN, HIGH); // TODO use triac
        digitalWrite(powerLevelPIN, HIGH);
      } else{
        //digitalWrite(RelayPIN, LOW); // TODO use triac
        digitalWrite(powerLevelPIN, LOW);
      }
    } 
    
  }
  
void printTemperature(float tempC)
{  
  if (tempC == InvalidTemp) {
    Serial.println(F("Error getting temperature" ));
  } else {
    Serial.print(F("currentStage="));
    Serial.println(currentStage);
    Serial.print(F("C: " ));
    Serial.print(tempC);
    Serial.print(F(" F: " ));
    Serial.println(c2F(tempC)); 
    Serial.print(F("reach Target Temp " ));
    Serial.println(reachTargetTemp);    
  }
  Serial.println("");
  
}
  
bool isRebounce(){
  unsigned long currentMillis = millis();
  if((currentMillis - btPrevMillis) > btInterval){
    btPrevMillis = currentMillis;
    return false;
  }else{
    return true;
  }
}
//  void checkButton(){
//    unsigned long currentMillis = millis();
//     if(currentMillis - btPrevMillis > btInterval) {
//       btPrevMillis = currentMillis;   
       
//       if (isButtonDown()){ 
//         // start the program or stop the buzzer
//         if (currentStage == -1){
//           startStage(0); // push button to kick off first stage
        
//         } 
//         else{
//           setBuzzer(false);
//         }       
//       }
//     }
//  }

 void dpBtDown(){
    // toggleLed(GreenLedPIN);
      Serial.print(F( "Button down "));
      Serial.print(btDownCount);
      Serial.print(F("\n\r" ));
 }
 
 void dpLn(String ss){
  if (debugPrint)   Serial.println(ss); 
 }
 
 void dp(String  ss){
   if (debugPrint)  Serial.print(ss); 
 }
 
 
  


 boolean isButtonDown(){
    buttonState = digitalRead(BtPIN);
  
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    lastButtonState =   buttonState;  
    // if the state has changed, increment the counter
    if (buttonState == LOW) {
      btDownCount++;   
	    dpBtDown();
      return true;          
    }  
   }  
   return false; 
 }
