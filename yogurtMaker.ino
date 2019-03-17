// include the library code: 
#include <OneWire.h>
#include <DallasTemperature.h>

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

const unsigned long minuteInMillis = 60000;
const int stageLedPins[] = {RedLedPIN, YellowLedPIN, GreenLedPIN};
//const float targetTemps[] ={ 37.5, 37.5, 37.5 };
const float targetTemps[] ={ 80.0, 40.0, 36.0 }; //c
const unsigned long stageHoldTimes[] = {10*minuteInMillis, 10*minuteInMillis, 7*60*minuteInMillis };
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
unsigned long tempInterval = 5000;  // tempreture check interval in millis
float targetTempC = 0;
int stageLedPin = RedLedPIN;

unsigned long stageStartTime = 0;
unsigned long stageHoldStartTime = 0;
boolean reachTargetTemp = false;
unsigned long stageHoldTime = 1;

void startStage(int sid){
    String s = "#### start Stage ";
    s += String(sid);
    dpLn(s);
    if (!isWorkStage(sid)) {
      dpLn("not a WorkStage");
      setRelay(false);	 
      if (sid == 3) {      
        currentStage = sid;   
        setBuzzer(true);    
      }       
      return;
    }else{         
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
          dp( "work stage target temp: ");
          Serial.print(targetTempC);
          dp(", hold time:");
          dpLn(String(stageHoldTime));              
	}
}

void setDoneSignal(){
   setRelay(false);
   for (int i = 0; i < LedSize; i++){
      digitalWrite( stageLedPins[i], HIGH);
	  delay(75);
	  digitalWrite( stageLedPins[i], LOW);
	   
      if (isButtonDown()){ 
        //  stop the buzzer
        setBuzzer(false);  
      }	  
  }
}
void setBuzzer(boolean on){
  if (on){
	tone(BuzzerPIN, 500);
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
  return (currentStage == 3);
}

void setup() {
  // start serial port
  Serial.begin(9600);
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
}

void loop() {
  run();
  //testDone();
   //testBlink2();
   //testRelay();
   
}

void run(){
  checkDone();
  checkButton();
  checkTemp(); 
  blinkStageLed();
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
    dpLn("##### yogurt maker done #####");
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
      setRelay(false);
      setBuzzer(true);
      while(true){
         for (int i = 0; i < LedSize; i++){
           digitalWrite( stageLedPins[i], HIGH);
        }
        delay(200);
         for (int i = 0; i < LedSize; i++){
           digitalWrite( stageLedPins[i], LOW);
        }
        delay(200);
      }
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
    if (!isWorkStage(currentStage)) return;
    
    unsigned long currentMillis = millis();
    if(currentMillis - tempPrevMillis > tempInterval) {
       tempPrevMillis = currentMillis;   
       if (reachTargetTemp && (currentMillis - stageHoldStartTime) > stageHoldTime){
         startStage(currentStage + 1);
	  if (currentStage == 3) {
	    setRelay(false);
	    return;
	  }
       }
       sensors.requestTemperatures();       
       float tempC = sensors.getTempC(thermometer);
       if (tempC == -127.00) { 
         currentStage = warn;         
         setRelay(false);
         setBuzzer(true);
         
       }else{             
         // turn on/off relay
         if ( tempC > targetTempC){
           if (currentStage == 0 && !reachTargetTemp){
             reachTarget();
           }
           setRelay(false);
         }
         else{   
           if (currentStage >0 && !reachTargetTemp){
             reachTarget();
           }         
         setRelay(true);      }          
       }
       printTemperature(tempC);
    }
  }
  void reachTarget(){
  	reachTargetTemp = true;
    dp("#### currentStage ");
    Serial.print(currentStage);
    dp(" reach target temp ");  
    Serial.println(targetTempC);           
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
    Serial.print("Error getting temperature");
  } else {
    Serial.print("currentStage=");
    Serial.print(currentStage);
    Serial.print(" C: ");
    Serial.print(tempC);
    Serial.print(" F: ");
    Serial.print(DallasTemperature::toFahrenheit(tempC));
    
  }
  Serial.print("\n\r");
}
  
 
 void checkButton(){
   unsigned long currentMillis = millis();
    if(currentMillis - btPrevMillis > btInterval) {
      btPrevMillis = currentMillis;   
       
      if (isButtonDown()){ 
        // start the maker or stop the buzzer
        if (currentStage == -1){
          startStage(0);
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
   Serial.println(ss); 
 }
 
 void dp(String  ss){
   Serial.print(ss); 
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

