/*
===============================================================================
SYSTEM OVERVIEW
===============================================================================

                         +----------------+
                         |   Artisan /    |
                         |   MODBUS RTU   |
                         +-------+--------+
                                 |
                                 |
                         au16data[] array
                                 |
              +------------------+------------------+
              |                                     |
              v                                     v
       +-------------+                       +-------------+
       | Fan Control |                       | Heat Control|
       +-------------+                       +-------------+
              |                                     |
              v                                     v
          PWM FAN                              SSR HEATER


Temperature Feedback Path:

        +-------------+
        |  MAX6675    |
        | Thermocouple|
        +------+------+
               |
               v
        Read every 250ms
               |
               v
        NaN / Fault Check
               |
               v
        Range Validation
               |
               v
        Rate-of-change Filter
               |
               v
        EMA Low Pass Filter
               |
               v
        Hysteresis Filter
               |
               v
        Modbus Register
        au16data[2]


===============================================================================
*/

/*
===============================================================================
MODBUS REGISTER MAP

Register | Function
---------+-------------------------
[0]      | Reserved
[1]      | Reserved
[2]      | Bean temperature x100°C
[3]      | Reserved
[4]      | Heater command 0-99%
[5]      | Fan command 0-99%
[6]      | Reserved
[14]     | Modbus parameter
[15]     | Modbus parameter

Temperature example:
2500 = 25.00°C
18000 = 180.00°C

===============================================================================
*/

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

// Temperature acquisition settings
#define TEMP_SAMPLE_INTERVAL_MS   250

#define TEMP_MIN_VALID            100       // 1.00°C
#define TEMP_MAX_VALID            30000   // 300.00°C

#define TEMP_MAX_STEP             1000     // 10.00°C/sample

#define TEMP_HYSTERESIS           25      // 0.25°C (one LSB)


//=============================================================================
// Temperature filter state
//=============================================================================

static uint16_t lastRawTemp = 0;
static bool hasRawTemp = false;

static uint16_t medianBuffer[3];
static uint8_t medianIndex = 0;
static bool medianReady = false;

static uint32_t emaTemp = 0;              // x100 °C
static bool emaInitialized = false;

static uint16_t lastPublishedTemp = 0;

static uint8_t invalidCount = 0;

static uint32_t tempTime = 0;


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


// Median helper
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }

    return b;
}

// Temperature reading

void tempReading()
{
    uint32_t now = millis();

    if ((now - tempTime) < TEMP_SAMPLE_INTERVAL_MS)
        return;

    tempTime += TEMP_SAMPLE_INTERVAL_MS;

    //---------------------------------------------------------------------
    // Read MAX6675
    //---------------------------------------------------------------------

    float temperatureC = thermocoupleBT.readCelsius();

    //---------------------------------------------------------------------
    // NaN / Open Thermocouple
    //---------------------------------------------------------------------

    if (isnan(temperatureC))
    {
        if (++invalidCount >= 5)
        {
            // Optional error indication
            // au16data[2] = 0xFFFF;
        }

        return;
    }

    invalidCount = 0;

    //---------------------------------------------------------------------
    // Convert to centi-degrees
    //---------------------------------------------------------------------

    uint16_t temperature =
        (uint16_t)(temperatureC * 100.0f + 0.5f);

    //---------------------------------------------------------------------
    // Absolute limits
    //---------------------------------------------------------------------

    if (temperature < TEMP_MIN_VALID ||
        temperature > TEMP_MAX_VALID)
        return;

    //---------------------------------------------------------------------
        // Reject impossible jumps
    //---------------------------------------------------------------------

      
       if (hasRawTemp)
      {
          int16_t delta =
              (int16_t)temperature - (int16_t)lastRawTemp;
      
          if (abs(delta) > TEMP_MAX_STEP)
              return;
      }
      
      lastRawTemp = temperature;

    //---------------------------------------------------------------------
    // EMA (15/16)
    //---------------------------------------------------------------------

    if (!emaInitialized)
    {
        emaTemp = temperature;
        emaInitialized = true;
    }
    else
    {
        emaTemp += (3*(temperature - emaTemp)) >> 2; // smoothing factor of 1/2^N ->  >> 1 Fast, >> 5 Slow filter
    }

    uint16_t filteredTemp = (uint16_t)emaTemp;

    //---------------------------------------------------------------------
    // Hysteresis
    //---------------------------------------------------------------------

    if (abs((int16_t)filteredTemp -
            (int16_t)lastPublishedTemp) >= TEMP_HYSTERESIS)
    {
        lastPublishedTemp = filteredTemp;
        au16data[2] = filteredTemp;
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
