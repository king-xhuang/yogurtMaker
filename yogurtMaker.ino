// include the library code: 
#include <OneWire.h>
#include <DallasTemperature.h>
#include "TimerOne.h"

// Data wire is plugged into pin 10 on the Arduino
#define ONE_WIRE_BUS 10

const int BtPIN= 2;
const int RedLedPIN =  3;
const int YellowLedPIN =  4;
const int GreenLedPIN =  5;
const int LedSize = 3;

const int RelayPIN = 8;
const int BuzzerPIN = 9;

const int s0 = 0;

const int s1 = 1;
const int s2 = 2;
const int warn = 5;
const int sInit = -1;
int currentStage = -1; 
int p = -1; //program 0 - yogurt maker, 1 - sous vide
const  String pNames[] = {"yogurt maker", "sous vide"};
float tempC = -127.00;

const unsigned long minuteInMillis = 60000;
const unsigned long hourInMillis = 60*60000;
const int stageLedPins[] = {RedLedPIN, YellowLedPIN, GreenLedPIN};
//const float targetTemps[] ={ 37.5, 37.5, 37.5 };
float targetTemps[] ={ 80.0, 40.0, 36.0 }; //c
 
float delta = 0.5; 
float deltaMax = 3.0;
float deltaTemp = deltaMax;
unsigned long stageHoldTimes[] = {10*minuteInMillis, 10*minuteInMillis, 7*60*minuteInMillis };
//const unsigned long stageHoldTimes[] = {1*minuteInMillis, 1*minuteInMillis, 240*minuteInMillis };

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Assign the addresses of your 1-Wire temp sensors.
// See the tutorial on how to obtain these addresses:
// http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html

DeviceAddress thermometer = { 0x28, 0x35, 0x88, 0x43, 0x05, 0x00, 0x00, 0xB0 };

int ledState = LOW;             // ledState used to set the LED
unsigned long ledPrevMillis = 0;         
unsigned long ledInterval = 500; // led flush interval in millis

int buttonState = HIGH; 
int lastButtonState = HIGH;
int btDownCount = 0;

unsigned long btPrevMillis = 0;         
unsigned long btInterval = 100; // button push check interval in millis

unsigned long tempPrevMillis = 0;
unsigned long tempInterval = 500;  // tempreture check interval in millis
float targetTempC = 0;
int stageLedPin = RedLedPIN;

unsigned long stageStartTime = 0;
unsigned long stageHoldStartTime = 0;
boolean reachTargetTemp = false;
unsigned long stageHoldTime = 1;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean debugPrint = false;

boolean initSetTime = false;
boolean initSetTemp = false;
int initCheck = 0;
int checkLimit = 20;
int deltaChangeCount = 0;
int deltaChangeCountInMin = 60;

float getDelta(){// detla is decresed when temp rising and close to the target
  if ( !reachTargetTemp ){
    deltaTemp = deltaMax;    // this make heating stop earlier at the initial heating phase.
  }else{
    deltaChangeCount++;
    if(deltaChangeCount > deltaChangeCountInMin*1){// change deltaTemp every 3 minutes after reachTargetTemp
      deltaChangeCount = 0;
      deltaTemp = deltaTemp/2.0;    
      if (deltaTemp < delta){
        deltaTemp = delta;
      }
      Serial.print("current delta ");
      Serial.println(deltaTemp);
    }  
  }
  return deltaTemp; 
}
String getPName(int p){
  return pNames[p];
}
// set initial condition for a stage
void startStage(int sid){  // start a new stage
    String s = "#### start Stage ";
    s += String(sid);
    Serial.println(s);
    if (!isWorkStage(sid)) {
      Serial.println("not a WorkStage");
      setRelay(false);	
      if(sid == -1 ) {
        Serial.println("push button to start");
      }
      else if (sid == 3 ||  sid == warn) {  
        Serial.println("##### program done #####");        
        currentStage = sid;   
        setBuzzer(true);    
      }       
      return;
    }
    else
    {         
      for (int i = 0; i < 3; i++){
        digitalWrite( stageLedPins[i], LOW);
      }      
      reachTargetTemp = false;
      stageStartTime = millis();
      stageHoldTime = stageHoldTimes[sid];
      currentStage = sid;
      targetTempC = targetTemps[sid];
      stageLedPin = stageLedPins[sid];
      setLeds();
      setBuzzer(true);
      Serial.print( "target temp: ");
      Serial.println(String(targetTempC));
      Serial.print(" c, hold time: ");
      Serial.print(stageHoldTime/minuteInMillis);  
      Serial.println(" minutes");               
	}
}

void setDoneSignal(){
  setRelay(false);
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
  for (int i = 0; i < LedSize; i++){
    if (i < currentStage) digitalWrite( stageLedPins[i], HIGH);
  }
}

boolean isWorkStage(int sid){
  return (sid < 3 && sid > -1);
}

boolean isDone(){
  return (currentStage == 3 || currentStage == warn );
}

void setup() {
  // start serial port
  Serial.begin(115200);
  dpLn("#### yogurt maker powered ON ####");
  // declare pin 9 to be an output:
  pinMode(BuzzerPIN, OUTPUT); 
  pinMode(RelayPIN, OUTPUT);
  pinMode(RedLedPIN, OUTPUT);
  pinMode(YellowLedPIN, OUTPUT);
  pinMode(GreenLedPIN, OUTPUT);
  pinMode(BtPIN, INPUT);
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  sensors.setResolution(thermometer, 10);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  Timer1.initialize(500000);         // initialize timer1, and set a 1/2 second period
  //Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(timer1Callback);  // attaches callback() as a timer overflow interrupt
  attachInterrupt(digitalPinToInterrupt(BtPIN), btDown, FALLING );
}

void btDown() { // call back for button down
  // start the program or stop the buzzer
  Serial.println("btDown");
  if (currentStage == -1){
    if ( p == -1 ){
      setProgram(0); // set default program to 0, if not set yet.
    } 

    startStage(0); // push button to kick off first stage
  } 
  else{
    setBuzzer(false);
  }        
}
void timer1Callback()
{
  run();
  //digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
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
      //stringComplete = true;
      String cmd = inputString;
      handleCommand(cmd);
      inputString = "";
    }else if (inChar == 'A') {
      inputString = "";
    }

  }
}



void printMenu(){
   Serial.println("m    - menu");
   Serial.println("ss   - status");
   Serial.println("std0.7 - tempreture delta to 0.7 C");
   Serial.println("st30 - tempreture to 30 C");
   Serial.println("sm30 - set time to 30 mim ");
   Serial.println("sp0  - Yogurt Maker");
   Serial.println("sp1  - Sous Vide");
   Serial.println("dp   - debug print");
}

boolean setProgram(){
  if (currentStage == sInit){
    timer1ToggleLeds();
    //setBuzzer(true);
  }
  if( p == 0 ){
    // Serial.print("start ");
    // Serial.println(getPName(p));
    return true;
  }
  else if (p == 1 )
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
    delay(5000);
    return false;  
  }
  else
  {    
    Serial.println("set program: sp0/sp1");
    delay(5000);
    //TODO set default program after 10 checks
    return false;
  } 
}


void handleCommand( String cmd){
    Serial.print("you say:");
    Serial.println(cmd);
    printString(cmd);

    if (isCmd(cmd, "m")){
      printMenu();
    }else if (isCmd(cmd, "ss")){
      printStatus();
    }else if (isCmd(cmd, "sm")){
      setTimeInMin(getInt(cmd));
    }
    else if (isCmd(cmd, "std")){
      setHeatingTempDelta(getFloat(cmd));
    }
    else if (isCmd(cmd, "st")){
      setHeatingTemp(getFloat(cmd));
    }else if (isCmd(cmd, "sp0")){
      setProgram( 0 );
    }
    else if (isCmd(cmd,"sp1")){
      setProgram( 1 );
    }else if (isCmd(cmd,"dp")){
       debugPrint = !debugPrint;
    }
    else{
      Serial.print("unknown command ");
      Serial.println( cmd.length());
    }
}

int getInt(String s){   
  String n = s.substring(2, s.length()-1);
  Serial.print("getInt len ");
  Serial.print(n.length());
  Serial.print(", val ");
  Serial.println(n);
  return n.toInt();
}

float getFloat(String s){   
  String n = s.substring(2, s.length()-1);
  Serial.print("getFloat   ");
  Serial.print(n.length());
  Serial.print(", val ");
  Serial.println(n);
  float f = n.toFloat();
  Serial.println(f);
  return f;
}

int getNum(String s){
  Serial.println(s.length());
  String n = "";
  for (int i = 0;  i < (s.length()); i++){
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
void printString(String s){
  for (int i = 0; i++; i < s.length()){
      Serial.print("#");
      Serial.println(s.charAt(i));
     
  } 
}

void setProgram( int i){
  p = i;
}
void setHeatingTempDelta(float t){
  Serial.print("setHeatingTempDelta ");
  Serial.println(t);
  delta = t;     
}
void setHeatingTemp(float t){
  Serial.print("setHeatingTemp ");
  Serial.println(t);
  if (p == 1){
    //float temp = float(t);    
    targetTemps[0] = t;
    targetTemps[1] = 0.0;
    targetTemps[2] = 0.0; 
    initSetTemp = true;
  }else Serial.println("cannot set temp");
}

void setTimeInMin(int m){
  Serial.print("setTimeInMin ");
  Serial.println(m);
  Serial.println(" min");
  if (p == 1){
    stageHoldTimes[0] = m*minuteInMillis;
    stageHoldTimes[1] = 1000;
    stageHoldTimes[2] = 1000;
    initSetTime = true;
  }else Serial.println("cannot set time");
}



void printStatus(){
  if (p == 1 || p == 0 ){
    Serial.println(getPName(p));
    // temp and time setting
    for(int i = 0; i < 3; i++){
      
      String s = "stage "; 
      s += String(i);
      s += ", Temp ";
      s += String(targetTemps[i]);
      s += ", time ";       
      s += String(stageHoldTimes[i]/minuteInMillis);
      s += " min";      
      
      Serial.println(s);
    }
    Serial.print("current tempreture delta ");
    Serial.println(deltaTemp);
    Serial.print("Relay on ");
    Serial.println(digitalRead(RelayPIN));
    if(isWorkStage(currentStage) ){
      int timePassed =  (millis() - stageStartTime)/minuteInMillis;
      // current stage, temp, time passed.
      printTemperature(tempC);
      Serial.print("Time passed = ");
      Serial.print(timePassed);
      Serial.println( " minutes"); 
    }else{
      Serial.print("work stage ");
      Serial.println(currentStage);
    }

  }else{
    Serial.print("program: ");
    Serial.println(p);
  }
}


void loop() {

  //run();
  //testDone();
   //testBlink2();
   //testRelay();
   
}

void run(){
  if(setProgram()){
    //checkDone();
    if (isDone()){
         
      setDoneSignal();       
    }
    else{
      //checkButton();
      checkTemp(); 
      blinkStageLed();
    }    
  }  
}


void testDone(){   
  startStage(3);
  checkDone();
}

void testBlink(){
   
  while(true){
  checkButton();
  blinkStageLed();
  }
}
void testBlink2(){
    checkTemp();
  checkButton();
  blinkStageLed();
  
}
  
  
void testRelay(){ 
    while(true){
      setRelay(true);
      delay(2000);
      setRelay(false);
      delay(2000);
    }    
  }

void checkDone(){  
  if (isDone()){
    Serial.println("##### program done #####");
     while(true){
      setDoneSignal();
     }
  }
}  
  void blinkStageLed(){
    if (currentStage == sInit){
      setRelay(false);
      unsigned long currentMillis = millis();
      if(currentMillis - ledPrevMillis > ledInterval) {
	       dpLn("push button to start!!!!");
	       ledPrevMillis = currentMillis;   
         if (ledState == HIGH) ledState = LOW;
         else ledState = HIGH;
   	     for (int i = 0; i < 3; i++){
           digitalWrite( stageLedPins[i], ledState);
         } 
      }
    }
    else if (currentStage == warn){
      // setRelay(false);
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
    else if (currentStage >= 0 && currentStage < 3){ // the current working stage led blinking
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
  
  void checkTemp(){ 
    //dpLn("checkTemp");   
    if (!isWorkStage(currentStage)) return;
    //dpLn("checkTemp");  
    unsigned long currentMillis = millis();
    if(currentMillis - tempPrevMillis > tempInterval) {// check Temperature 
      tempPrevMillis = currentMillis;   
      if (reachTargetTemp && (currentMillis - stageHoldStartTime) > stageHoldTime){// move to next stage
          
          if ( p == 1 && currentStage == 0 ){ // sOUS vide
            currentStage = 3; // FORCE stage to 3
            startStage(3);
            return;
          }
          
          startStage(currentStage + 1);
          if ( p == 0 && currentStage == 3 ){ // YOGURT MAKER             
            setRelay(false);
            return; 
          } 
        
      }
       sensors.requestTemperatures();       
       tempC = sensors.getTempC(thermometer);
       if (tempC == -127.00) { // cannot get temp from sensor
         currentStage = warn;  
         startStage(warn);   
         return;    
        //  setRelay(false);
        //  setBuzzer(true);
         
       }else{   // do get temp from sensor          
         // turn on/off relay
         if ( tempC > ( targetTempC - getDelta()) ) {
           if (currentStage == 0 && !reachTargetTemp){
             reachTarget();
           }
           setRelay(false);
         }
         else{   
          if (currentStage > 0 && !reachTargetTemp){
            reachTarget();
          }         
          setRelay(true);      }          
       }
       //printTemperature(tempC);
    }
  }
  void reachTarget(){
  	reachTargetTemp = true;
    Serial.print("#### stage ");
    Serial.print(String(currentStage));
    Serial.print(" reach target temp ");  
    Serial.println(String(targetTempC));           
    stageHoldStartTime = millis();
  }
  // set realy on/off -- true/false
  void setRelay(boolean on){
    if (on) {
       digitalWrite(RelayPIN, HIGH);
       //dp("Relay ON ");
    }
    else{
       digitalWrite(RelayPIN, LOW);
       //dp("Relay OFF ");
    }
  }
  
void printTemperature(float tempC)
{  
  if (tempC == -127.00) {
    Serial.println("Error getting temperature");
  } else {
    Serial.print("currentStage=");
    Serial.println(currentStage);
    Serial.print(" C: ");
    Serial.print(tempC);
    Serial.print(" F: ");
    Serial.println(DallasTemperature::toFahrenheit(tempC));
    Serial.print("reach Target Temp ");
    Serial.println(reachTargetTemp);    
  }
  Serial.println("");
  
}
  
 
 void checkButton(){
   unsigned long currentMillis = millis();
    if(currentMillis - btPrevMillis > btInterval) {
      btPrevMillis = currentMillis;   
       
      if (isButtonDown()){ 
        // start the program or stop the buzzer
        if (currentStage == -1){
          startStage(0); // push button to kick off first stage
        
        } 
        else{
          setBuzzer(false);
        }       
      }
    }
 }

 void dpBtDown(){
    // toggleLed(GreenLedPIN);
      Serial.print("Button down ");
      Serial.print(btDownCount);
      Serial.print("\n\r");
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

