#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <DigitalIO.h>
#include <avr/wdt.h>

#define NODE_ID 92

#define RELAY_ON 0  // GPIO value to write to turn on attached relay
#define RELAY_OFF 1 // GPIO value to write to turn off attached relay


/******************************************************************************************************/
/*                               				Сенсоры												  */
/******************************************************************************************************/

#define CHILD_ID_ENV_TEMP		10  //Температура в комнате
#define CHILD_ID_INBOX_TEMP		11  //Температура в боксе
#define CHILD_ID_OUTBOX_TEMP	12  //Температура на верхней крышке бокса
#define CHILD_ID_RELAY1_TEMP	13  //Температура на клеммах реле 1
#define CHILD_ID_RELAY2_TEMP	14  //Температура на клеммах реле 2


#define CHILD_ID_LIGHT_RELAY1_STATUS		20  //Фоторезистор на светодиоде статуса реле 1
#define CHILD_ID_LIGHT_RELAY2_STATUS		21  //Фоторезистор на светодиоде статуса реле 1


#define CHILD_ID_RELAY1			30  //Реле 1
#define CHILD_ID_RELAY2			31  //Реле 2


#define CHILD_ID_VOLTMETER5V	40  //Вольтметр на выходе 5В
#define CHILD_ID_VOLTMETER10V	41  //Вольтметр на выходе 10В
#define CHILD_ID_VOLTMETER24V	42  //Вольтметр на выходе 24В
#define CHILD_ID_VOLTMETER48V	43  //Вольтметр на выходе 48В

#define CHILD_ID_COMMONSTATUS         90 

#define REBOOT_CHILD_ID                       100
#define RECHECK_SENSOR_VALUES                 101
#define DISABLE_RELAYTEMPDIFF_CHECK_CHILD_ID  102
#define DISABLE_CASETEMPDIFF_CHECK_CHILD_ID   103

/******************************************************************************************************/
/*                               				IO												      */
/******************************************************************************************************/

#define TEMPERATURE_PIN				4

#define LIGHT_SENSOR_RELAY1        A4
#define LIGHT_SENSOR_RELAY2        A5

#define VOLTMETER5V_PIN            A0
#define VOLTMETER10V_PIN           A1
#define VOLTMETER24V_PIN           A2
#define VOLTMETER48V_PIN           A3

#define RELAY1_PIN				    6
#define RELAY2_PIN				    7


/******************************************************************************************************/
/*                               				Common settings									      */
/******************************************************************************************************/

#define COUNT 50

boolean metric = true;          // Celcius

float R1_5v = 99800.0; // resistance of R1 (100K) -see text!
float R2_5v = 10000.0; // resistance of R2 (10K) - see text!
float R1_10v = 100001.0; // resistance of R1 (100K) -see text!
float R2_10v = 9970.0; // resistance of R2 (10K) - see text!
float R1_24v = 100001.0; // resistance of R1 (100K) -see text!
float R2_24v = 10000.0; // resistance of R2 (10K) - see text!
float R1_48v = 99800.0; // resistance of R1 (100K) -see text!
float R2_48v = 10001.0; // resistance of R2 (10K) - see text!


boolean boolRelayDiffCheckDisabled = false;
boolean boolCaseDiffCheckDisabled = false;
boolean boolRecheckSensorValues = false;


/******************************************************************************************************/
/*                               				Variables		   								      */
/******************************************************************************************************/

boolean boolRecheckRelayStatus=false;

long previousRelayStatusMillis = 0; 
long RelayStatussensorInterval = 60000;     
int lastRelayStatusLightLevel1;             // Holds last light level
int lastRelayStatusLightLevel2;             // Holds last light level



float lastrelay1Temp = -1;
float lastrelay2Temp = -1;
float lastTopCaseTemp = -1;
float lastEnvTemp = -1;
float lastInCaseTemp = -1;

long previousTempMillis = 0;        // last time the sensors are updated
long TempsensorInterval = 20000;     // interval at which we will take a measurement ( 30 seconds)


long previousVoltageMillis = 0;      
long VoltagesensorInterval = 60000; 
float oldValue_voltmeter5v = -1;
float oldValue_voltmeter10v = -1;
float oldValue_voltmeter24v = -1;
float oldValue_voltmeter48v = -1;


long previousTDCMillis=0;
long TDCsensorInterval=300000;

//Вольтметр
// эту константу (typVbg) необходимо откалибровать индивидуально
const float typVbg = 1.120; //1.119; // 1.0 -- 1.2 1.106 1.114 //1.128
float Vcc = 0.0;
float MaxVoltage = 0.0;
int i;
float curVoltage;

double Voltage = 0;





OneWire oneWire(TEMPERATURE_PIN); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

MySensor gw;


MyMessage EnvTempMsg(CHILD_ID_ENV_TEMP, V_TEMP);
MyMessage InboxTempMsg(CHILD_ID_INBOX_TEMP, V_TEMP);
MyMessage OutBoxTempMsg(CHILD_ID_OUTBOX_TEMP, V_TEMP);
MyMessage Relay1TempMsg(CHILD_ID_RELAY1_TEMP, V_TEMP);
MyMessage Relay2TempMsg(CHILD_ID_RELAY2_TEMP, V_TEMP);

MyMessage Relay1StatusLightMsg(CHILD_ID_LIGHT_RELAY1_STATUS, V_LIGHT_LEVEL);
MyMessage Relay2StatusLightMsg(CHILD_ID_LIGHT_RELAY2_STATUS, V_LIGHT_LEVEL);

MyMessage Voltmeter5vMsg(CHILD_ID_VOLTMETER5V, V_VOLTAGE);
MyMessage Voltmeter10vMsg(CHILD_ID_VOLTMETER10V, V_VOLTAGE);
MyMessage Voltmeter24vMsg(CHILD_ID_VOLTMETER24V, V_VOLTAGE);
MyMessage Voltmeter48vMsg(CHILD_ID_VOLTMETER48V, V_VOLTAGE);

MyMessage CriticalAlarmMsg(CHILD_ID_COMMONSTATUS, V_TRIPPED);
MyMessage RelayTempDiffCheckStateMsg(DISABLE_RELAYTEMPDIFF_CHECK_CHILD_ID, V_TRIPPED);
MyMessage CaseTempDiffCheckStateMsg(DISABLE_CASETEMPDIFF_CHECK_CHILD_ID, V_TRIPPED);

void setup() {

    Serial.begin(115200);
    Serial.println("Begin setup");

    // Initialize library and add callback for incoming messages
    gw.begin(incomingMessage, NODE_ID, false);

    // Send the sketch version information to the gateway and Controller
    gw.sendSketchInfo("MetalBox sensor node v2", "1.0");  

    metric = gw.getConfig().isMetric;

    //Relays status light sensors
    pinMode(LIGHT_SENSOR_RELAY1, INPUT);
    pinMode(LIGHT_SENSOR_RELAY2, INPUT);
    gw.present(CHILD_ID_LIGHT_RELAY1_STATUS, S_LIGHT_LEVEL);
    gw.present(CHILD_ID_LIGHT_RELAY2_STATUS, S_LIGHT_LEVEL);  

    
    //present voltmeter sensor
    pinMode(VOLTMETER5V_PIN, INPUT);
    pinMode(VOLTMETER10V_PIN, INPUT);
    pinMode(VOLTMETER24V_PIN, INPUT); 
    pinMode(VOLTMETER48V_PIN, INPUT);

    gw.present(CHILD_ID_VOLTMETER5V, S_MULTIMETER); 
    gw.present(CHILD_ID_VOLTMETER10V, S_MULTIMETER); 
    gw.present(CHILD_ID_VOLTMETER24V, S_MULTIMETER); 
    gw.present(CHILD_ID_VOLTMETER48V, S_MULTIMETER); 


    //temperature sensors
    gw.present(CHILD_ID_ENV_TEMP, S_TEMP);
    gw.present(CHILD_ID_INBOX_TEMP, S_TEMP);
    delay(200);
    gw.present(CHILD_ID_OUTBOX_TEMP, S_TEMP);
    delay(200);    
    gw.present(CHILD_ID_RELAY1_TEMP, S_TEMP);
    delay(500);    
    gw.present(CHILD_ID_RELAY2_TEMP, S_TEMP);
    delay(200);    

    // Relays
    pinMode(RELAY1_PIN, OUTPUT);  
    pinMode(RELAY2_PIN, OUTPUT);         
    gw.present(CHILD_ID_RELAY1, S_LIGHT);    
    gw.present(CHILD_ID_RELAY2, S_LIGHT);  


    //reboot sensor command
    gw.present(REBOOT_CHILD_ID, S_BINARY);  


//critical alarm message sensor
  gw.present(CHILD_ID_COMMONSTATUS, S_DOOR);
 

//disable temp difference check
  gw.present(DISABLE_RELAYTEMPDIFF_CHECK_CHILD_ID, S_LIGHT); 
  gw.present(DISABLE_CASETEMPDIFF_CHECK_CHILD_ID, S_LIGHT); 


//reget sensor values
  gw.present(RECHECK_SENSOR_VALUES, S_LIGHT); 

    //Enable watchdog timer
    wdt_enable(WDTO_8S);

    //set all relays to off state    
    delay(1000);
    digitalWrite(RELAY1_PIN, RELAY_OFF);
    digitalWrite(RELAY2_PIN, RELAY_OFF);

    gw.send(CriticalAlarmMsg.set("0"),true);

    Serial.println("End setup");  


} // end setup




void loop() {


	checkRelayStatus();

 
 	checkVoltage();	

  checkTemp();

  reportTempDiffCheckState();

if (boolRecheckSensorValues)
{
  boolRecheckSensorValues = false;
}

    // Alway process incoming messages whenever possible
    gw.process();

    //Serial.println(rvcc ());
    
    //reset watchdog timer
    wdt_reset();  

}




void incomingMessage(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.

    if ( message.sensor == REBOOT_CHILD_ID && message.getBool() == true ) {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }
     
     if (message.type==V_LIGHT && strlen(message.getString())>0 ) 
     {
         if ( message.sensor == CHILD_ID_RELAY1 ) 
         {
                digitalWrite(RELAY1_PIN, message.getBool()?RELAY_ON:RELAY_OFF);
                boolRecheckRelayStatus = true;
         }
         if ( message.sensor == CHILD_ID_RELAY2 ) 
         {
                digitalWrite(RELAY2_PIN, message.getBool()?RELAY_ON:RELAY_OFF);
                boolRecheckRelayStatus = true;                
         }         
     }

    if ( message.sensor == DISABLE_RELAYTEMPDIFF_CHECK_CHILD_ID ) {
         
         if (message.getBool() == true)
         {
            boolRelayDiffCheckDisabled = true;
         }
         else
         {
            boolRelayDiffCheckDisabled = false;
         }

     }

    if ( message.sensor == DISABLE_CASETEMPDIFF_CHECK_CHILD_ID ) {
         
         if (message.getBool() == true)
         {
            boolCaseDiffCheckDisabled = true;
         }
         else
         {
            boolCaseDiffCheckDisabled = false;
         }

     }

    if ( message.sensor == RECHECK_SENSOR_VALUES ) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;
         }

     }

        return;      
} 


void checkRelayStatus()
{
  
int outputValue= 0; 
int invertedValue; 
    unsigned long currentRelayStatusMillis = millis();
 
  if((currentRelayStatusMillis - previousRelayStatusMillis > RelayStatussensorInterval) || boolRecheckRelayStatus || boolRecheckSensorValues) {
    // Save the current millis 
    previousRelayStatusMillis = currentRelayStatusMillis;   
    // take action here:

  int  lightLevel=0;
  
  if (boolRecheckRelayStatus)
  {
      boolRecheckRelayStatus = false;
      delay(600);
  }

     lightLevel = (1023-analogRead(LIGHT_SENSOR_RELAY1))/10.23; 


            
      if (lightLevel > 0)
      {    

          if (lightLevel != lastRelayStatusLightLevel1) {
           Serial.print("Relay 1 power Lightlevel: ");
            Serial.println(lightLevel);
            gw.send(Relay2StatusLightMsg.set(lightLevel));
            
              
            lastRelayStatusLightLevel1 = lightLevel;
          }
    
    
          lightLevel=0;
      }    

    
     lightLevel = (1023-analogRead(LIGHT_SENSOR_RELAY2))/10.23; 


            
      if (lightLevel > 0)
      {    

          if (lightLevel != lastRelayStatusLightLevel2) {
           Serial.print("Relay 2 status Lightlevel: ");
            Serial.println(lightLevel);
            gw.send(Relay1StatusLightMsg.set(lightLevel));
            
              
            lastRelayStatusLightLevel2 = lightLevel;
          }
    
    
          lightLevel=0;
      }    



  }  
  
}


void checkTemp()
{

    unsigned long currentTempMillis = millis();
    if((currentTempMillis - previousTempMillis > TempsensorInterval) || boolRecheckSensorValues) {
        // Save the current millis
        previousTempMillis = currentTempMillis;
        // take action here:



          // Fetch temperatures from Dallas sensors
          sensors.requestTemperatures();



        float relay1Temp =0, relay2Temp =0, TopCaseTemp = 0, InCaseTemp = 0, EnvTemp = 0;

        relay1Temp = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(0) * 10.)) / 10.; // Temp sensor relay

        Serial.print("Relay1Temp: ");
        Serial.println(relay1Temp);
        if ( ( relay1Temp != lastrelay1Temp || boolRecheckSensorValues ) && relay1Temp != -127.00 && relay1Temp != 85.00 ) {
            gw.send(Relay1TempMsg.set(relay1Temp,1));
            lastrelay1Temp = relay1Temp;
            delay (100);
        } 



        // Если температура на реле больше 70 градусов - отключить реле
        if ( relay1Temp > 70 && !boolRelayDiffCheckDisabled ) 
        {
          //Disconnect 220v input
          digitalWrite(RELAY1_PIN, RELAY_ON);
          gw.send(CriticalAlarmMsg.set("1"),true);
          
        }


        relay2Temp = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(4) * 10.)) / 10.; // Temp sensor relay

        Serial.print("Relay2Temp: ");
        Serial.println(relay2Temp);
        if ((relay2Temp != lastrelay2Temp || boolRecheckSensorValues) && relay2Temp != -127.00 && relay2Temp != 85.00 ) {
            gw.send(Relay2TempMsg.set(relay2Temp,1));
            lastrelay2Temp = relay2Temp;
            delay (100);            
        } 

        // Если температура на реле больше 70 градусов - отключить реле
        if ( relay2Temp > 70 && !boolRelayDiffCheckDisabled )
        {
          //Disconnect 220v input
          digitalWrite(RELAY2_PIN, RELAY_ON);
          gw.send(CriticalAlarmMsg.set("1"),true);
          
        }


        TopCaseTemp = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(1) * 10.)) / 10.; // Temp sensor relay

        Serial.print("Top Case Temp: ");
        Serial.println(TopCaseTemp);
        if ((TopCaseTemp != lastTopCaseTemp || boolRecheckSensorValues ) && TopCaseTemp != -127.00 && TopCaseTemp != 85.00 ) {
            gw.send(OutBoxTempMsg.set(TopCaseTemp,1));
            lastTopCaseTemp = TopCaseTemp;
            delay (100);            
        } 

        // Если температура в верхней части снаружи шкафа больше или равно 100 градусов - отключить реле
        if ( TopCaseTemp >= 100 && !boolCaseDiffCheckDisabled )
        {
          //Disconnect 220v input
          digitalWrite(RELAY1_PIN, RELAY_ON);
          gw.send(CriticalAlarmMsg.set("1"),true);
          
        }

        InCaseTemp = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(2) * 10.)) / 10.; // Temp sensor relay

        Serial.print("Inner Case Temp: ");
        Serial.println(InCaseTemp);
        if ((InCaseTemp != lastInCaseTemp || boolRecheckSensorValues) && InCaseTemp != -127.00 && InCaseTemp != 85.00 ) {
            gw.send(InboxTempMsg.set(InCaseTemp,1));
            lastInCaseTemp = InCaseTemp;
            delay (100);            
        } 

        EnvTemp = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(3) * 10.)) / 10.; // Temp sensor relay

        Serial.print("Environment Temp: ");
        Serial.println(EnvTemp);
        if ((EnvTemp != lastEnvTemp || boolRecheckSensorValues) && EnvTemp != -127.00 && EnvTemp != 85.00 ) {
            gw.send(EnvTempMsg.set(EnvTemp,1));
            lastEnvTemp = EnvTemp;
            delay (100);            
        } 


     
    }    

  
}



void checkVoltage()
{
  
float vout = 0.0;
float vin = 0.0;

int value = 0;  
 
  unsigned long currentVoltageMillis = millis();
  
  if((currentVoltageMillis - previousVoltageMillis > VoltagesensorInterval) || boolRecheckSensorValues)  {
  // Save the current millis
  previousVoltageMillis = currentVoltageMillis;
  


  Vcc = readVcc();   
  
  // считываем точное напряжение
  curVoltage = 0.0;
  for (i = 0; i < COUNT; i++) {
      curVoltage = curVoltage + analogRead(VOLTMETER5V_PIN);
      delay(10);
  }
  curVoltage = curVoltage / COUNT;
  float v  = (curVoltage * Vcc) / 1024.0;
  float v2 = v / (R2_5v / (R1_5v + R2_5v)); 


 if ((v2 != oldValue_voltmeter5v) || boolRecheckSensorValues) 
  {
     // Send in the new value
     gw.send(Voltmeter5vMsg.set(v2,2));   
     oldValue_voltmeter5v = v2;  
         Serial.print("Voltage 5v: ");
         Serial.print(v2);  
         Serial.println(" V");          
  }



Vcc = -1;


  Vcc = readVcc();   
  
  // считываем точное напряжение
  curVoltage = 0.0;
  for (i = 0; i < COUNT; i++) {
      curVoltage = curVoltage + analogRead(VOLTMETER10V_PIN);
      delay(10);
  }
  curVoltage = curVoltage / COUNT;
  v  = (curVoltage * Vcc) / 1024.0;
  v2 = v / (R2_10v / (R1_10v + R2_10v)); 


 if ((v2 != oldValue_voltmeter10v) || boolRecheckSensorValues)
  {
     // Send in the new value
     gw.send(Voltmeter10vMsg.set(v2,2));   
     oldValue_voltmeter10v = v2;  
         Serial.print("Voltage 10v: ");
         Serial.print(v2);  
         Serial.println(" V");          
  }


Vcc = -1;


  Vcc = readVcc();   
  
  // считываем точное напряжение
  curVoltage = 0.0;
  for (i = 0; i < COUNT; i++) {
      curVoltage = curVoltage + analogRead(VOLTMETER24V_PIN);
      delay(10);
  }
  curVoltage = curVoltage / COUNT;
  v  = (curVoltage * Vcc) / 1024.0;
  v2 = v / (R2_24v / (R1_10v + R2_24v)); 


 if ((v2 != oldValue_voltmeter24v) || boolRecheckSensorValues)
  {
     // Send in the new value
     gw.send(Voltmeter24vMsg.set(v2,2));   
     oldValue_voltmeter24v = v2;  
         Serial.print("Voltage 24v: ");
         Serial.print(v2);  
         Serial.println(" V");          
  }


Vcc = -1;


  Vcc = readVcc();   
  
  // считываем точное напряжение
  curVoltage = 0.0;
  for (i = 0; i < COUNT; i++) {
      curVoltage = curVoltage + analogRead(VOLTMETER48V_PIN);
      delay(10);
  }
  curVoltage = curVoltage / COUNT;
  v  = (curVoltage * Vcc) / 1024.0;
  v2 = v / (R2_48v / (R1_48v + R2_48v)); 


 if ((v2 != oldValue_voltmeter48v) || boolRecheckSensorValues)
  {
     // Send in the new value
     gw.send(Voltmeter48vMsg.set(v2,2));   
     oldValue_voltmeter48v = v2;  
         Serial.print("Voltage 48v: ");
         Serial.print(v2);  
         Serial.println(" V");          
  }




 }

  
}


void reportTempDiffCheckState()
{

    unsigned long currentTDCMillis = millis();
    if(currentTDCMillis - previousTDCMillis > TDCsensorInterval) {
        // Save the current millis
        previousTDCMillis = currentTDCMillis;
        // take action here:

       gw.send(RelayTempDiffCheckStateMsg.set(boolRelayDiffCheckDisabled ? "1" : "0" ));  
       gw.send(CaseTempDiffCheckStateMsg.set(boolCaseDiffCheckDisabled ? "1" : "0" ));         
    }    


}

float readVcc() {
  byte i;
  float result = 0.0;
  float tmp = 0.0;

  for (i = 0; i < 20; i++) {
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
        ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
        ADMUX = _BV(MUX3) | _BV(MUX2);
    #else
        // works on an Arduino 168 or 328
        ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif

    delay(3); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    tmp = (high<<8) | low;
    tmp = (typVbg * 1023.0) / tmp;
    result = result + tmp;
    delay(5);
  }

  result = result / 20;
  return result;
}



int determineVQ(int PIN) {
Serial.print("estimating avg. quiscent voltage:");
long VQ = 0;
//read 5000 samples to stabilise value
for (int i=0; i<5000; i++) {
    VQ += analogRead(PIN);
    delay(1);//depends on sampling (on filter capacitor), can be 1/80000 (80kHz) max.
}
VQ /= 5000;
Serial.print(map(VQ, 0, 1023, 0, 5000)); //5000
Serial.println(" mV");
return int(VQ);
}




float rvcc ()
{

ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
ADCSRA |= _BV(ADSC); // начало преобразований
while (bit_is_set(ADCSRA, ADSC)); // измерение
uint8_t low = ADCL; // сначала нужно прочесть ADCL - это запирает ADCH
uint8_t high = ADCH; // разлочить оба
float result = (high<<8) | low;
result = (1.1 * 1023.0 * 1000) / result; // Результат Vcc в милливольтах  

  return result;
  
}


/*
float readCurrent(int PIN) {
int current = 0;
int sensitivity = 100.0;//change this to 100 for ACS712-20A or to 66 for ACS712-30A
//read 5 samples to stabilise value
for (int i=0; i<20; i++) {
    current += analogRead(PIN) - VQ;
    delay(1);
}
current = map(current/20, 0, 1023, 0, 5000); //5000
return float(current)/sensitivity;
}



int determineVQ(int PIN) {
Serial.print("estimating avg. quiscent voltage:");
long VQ = 0;
//read 5000 samples to stabilise value
for (int i=0; i<5000; i++) {
    VQ += analogRead(PIN);
    delay(1);//depends on sampling (on filter capacitor), can be 1/80000 (80kHz) max.
}
VQ /= 5000;
Serial.print(map(VQ, 0, 1023, 0, 5000)); //5000
Serial.println(" mV");
return int(VQ);
}

*/
