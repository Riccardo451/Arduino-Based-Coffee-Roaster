// Fluid Bed Coffee Roaster Arduino Sketch made by Henrik Balle Koefoed v. 12. january 2019

// Change to temperatur reading subroutine for higher data quality on temperatur v. 30. september 2019
// http://www.sinobi.dk/henrik/kafferister1/cofferoaster.ino
// http://www.sinobi.dk/henrik/coffeeroaster1/

/*
  CHANGELOG Riccardo
  =========
  Temperature handling update:

  - Removed 8-sample moving average filtering to reduce PID latency.
  - Temperature is now sampled every 250 ms, matching MAX6675 response capability.
  - Added validation of MAX6675 readings:
      * Reject NaN values.
      * Reject values outside valid temperature range.
      * Reject unrealistic temperature jumps between samples.
  - Invalid readings are ignored and the last valid temperature is kept.
  - Modbus register au16data[2] now contains the latest validated temperature.
  - Temperature remains scaled as °C x100 for Artisan compatibility.

  Result:
  - Reduced temperature feedback delay from approximately 1.5 s to ~250-300 ms.
  - Improved response time for Artisan PID control while maintaining fault protection.
*/

// Links to external libraries
// https://github.com/adafruit/MAX6675-library
// https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino

#include "max6675.h"
#include "ModbusRtu.h"

// data array for modbus network sharing
// third place [2] is used for BTtemperature, [5] for fan speed, [6] for heat
uint16_t au16data[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1 };

Modbus slave;                                     

int thermoBtDO = 12;
int thermoBtCS = 10;
int thermoBtCLK = 13;
MAX6675 thermocoupleBT(thermoBtCLK, thermoBtCS, thermoBtDO);

const uint8_t heatSSR = 4;
const uint8_t fanFET = 5;                                          // Pin 5 or 6 are better for PWM control of DC motors as the frequency is 980 Hz, as opposed to just 490 Hz

unsigned long tempTime=0;

const int FANaccelTime=1200;                                // Acceleration ramp time for FAN speed increase or decrease in milliseconds. Default 1500 mS
int FANspeedmapped;
int oldFANspeedmapped;
int actualFANspeed;
int accel;
unsigned long nextFANrampTime;
  
const int heatPRT=1;                                        // Set Pulse Repitition Time for heater in seconds. Default is one second
int Step=0;
unsigned long EndPulseTime;
unsigned long EndPauseTime;

//////////////////////////////////////////////
// Temperature acquisition settings
const unsigned long TEMP_SAMPLE_INTERVAL_MS = 250;

// Temperature validation limits (stored as °C x100)
const uint16_t TEMP_MIN_VALID = 100;       // 1.00 °C
const uint16_t TEMP_MAX_VALID = 35000;     // 350.00 °C

// Maximum allowed temperature change between samples
// MAX6675 + thermocouple cannot physically jump this fast
const uint16_t TEMP_MAX_STEP = 700;       // 7.00 °C -> 7°C / 0.25s = 28°C/s

uint16_t lastValidTemp = 0;
//////////////////////////////////////////////

void setup() {
  pinMode(heatSSR, OUTPUT);
  pinMode(fanFET, OUTPUT);
  digitalWrite(heatSSR, LOW);
  analogWrite(fanFET,0);
  slave = Modbus(1,0,0);                                    // this is slave @1 and RS-232 or USB-FTDI
  slave.begin( 19200 );                                     // 19200 baud, 8-bits, non, 1-bit stop
  delay(500);
}

void loop() {
   tempReading();                                           // call temperatur reading subroutine
   slave.poll( au16data, 16 );                              // poll MODBUS
   delay(1);
   setFAN(au16data[5]);                                     // call fan driving subrutine with modbus data
   setHeat(au16data[4]);                                    // call heater driving subrutine with modbus data
}


void tempReading()
{
  if (millis() - tempTime >= TEMP_SAMPLE_INTERVAL_MS)
  {
    tempTime = millis();

    float temperatureC = thermocoupleBT.readCelsius();

    // Reject invalid MAX6675 readings
    if (isnan(temperatureC) || temperatureC < 0)
    {
        return;
    }

    uint16_t temperature = (uint16_t)(temperatureC * 100.0);

    // Reject zero and impossible temperatures
    if (temperature < TEMP_MIN_VALID ||
        temperature > TEMP_MAX_VALID)
    {
      return;
    }

    // Reject unrealistic jumps
    if (lastValidTemp != 0)
    {
      uint16_t difference = abs((int)temperature - (int)lastValidTemp);

      if (difference > TEMP_MAX_STEP)
      {
        return;
      }
    }

    // Valid reading: update stored value
    lastValidTemp = temperature;
    au16data[2] = temperature;
  }
}


void setFAN(int FANspeed){
  if(au16data[2] > 12000 && FANspeed < 25){
    FANspeed=25; }                                          // For safety. Ensures fan is kept on at least 25%, as long as bean temp is above 120 C
  FANspeedmapped=map(FANspeed,0,99,0,255);                  // maps 0-99 values from Artisan to 0-255 for dutycycle control of fanspeed output

  if(oldFANspeedmapped != FANspeedmapped){                  // Things to do ones, when fanspeed in MODBUS array has changed
    oldFANspeedmapped = FANspeedmapped;
    if(FANspeedmapped > actualFANspeed){                    // Things to do when motor has to speed up
      accel=FANspeedmapped - actualFANspeed;                // How far away are the motor from desired speed
      accel = constrain(map(accel,0,255,1,3),1,3);                         // Reduce speed delta to individual increments between 1 and 3
      if(actualFANspeed+accel>FANspeedmapped){accel=0;}     // Unless motor allready has more or less reached target
    }
    else{                                                   // Things to do when motor has to slow down
      accel=actualFANspeed-FANspeedmapped;                  // How far away are the motor from desired speed
      accel = constrain(map(accel,0,255,-1,-3),-3,-1);                    // Reduce speed delta to individual decrements between 1 and 3
      if(actualFANspeed+accel<FANspeedmapped){accel=0;}     // Unless motor allready has more or less reached target
    }
    nextFANrampTime = millis();
  }

  if(FANspeedmapped!=actualFANspeed){                       // Are we not on desired speed yet? Then do all this
    if (nextFANrampTime < millis()){                        // Is it time to make a new change to speed?
      nextFANrampTime = millis()+(FANaccelTime/100);
      actualFANspeed=actualFANspeed+accel;                  // Add the acceleration to actual speed variable
      if((actualFANspeed-FANspeedmapped>0&&actualFANspeed-FANspeedmapped<3)||(FANspeedmapped-actualFANspeed>0&&FANspeedmapped-actualFANspeed<3)||accel==0)
        {actualFANspeed=FANspeedmapped;}                    // Unless motor allready is more or less on target, then just make it what is requested
      analogWrite(fanFET, actualFANspeed);                  // Do the actual speed change write
    }
  }
}

void setHeat(int HeatLevel){
   if(HeatLevel==0)                                         // For safety
     {Step=0;}
   switch(Step){
    case 1:                                                 // Calculation case of pulse/pause ratio and starting the pulse
      EndPulseTime=millis()+((HeatLevel+1)*10*heatPRT);     // Calculation of when pulse time shall end
      EndPauseTime=EndPulseTime+((heatPRT*1000)-((HeatLevel+1)*10*heatPRT));  // Calculation of when pause time should end to keep dutycycle ratio
      if(au16data[5]>25 && au16data[2]<26000)               // For safety. Only turn heat on if fanspeed is higher than 25% and BT is lower than 260C
        {digitalWrite(heatSSR, HIGH);}
      if (HeatLevel==99){Step=1;}
      else {Step=2;}
      break;
    case 2:                                                 // Ending the pulse when time is up
      if(millis()>EndPulseTime)
        {digitalWrite(heatSSR, LOW); 
        Step=3;}
      break;
    case 3:                                                 // Ending the pause when time is up
      if(millis()>EndPauseTime)
        {Step=1;}
      break;
    default:
      digitalWrite(heatSSR, LOW);                           // For safety. Keeps heater output at zero when no heat is asked for.
      Step=1;
   }
}
