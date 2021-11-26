
// boiler main control
const char module[] = "BoilerMain2";
const char buildDate[] = __DATE__  "  "  __TIME__;

// version history at bottom

/* quick finder functions:
  void setup(
  void loop(
  float randomF(
  void selfTest(
  void selfTest1(
  void readTadoAnalog(
  void getTemperatures(
  void readHWTemp(
  void readInputs(
  void setOutputs(
  void updateDisplay(
  void display1(
  void display2(
  void display3(
  void display4(
  void display5(
  void log(
  void logConfig(
  void resetConfig(
  void saveToEprom(
  void loadFromEprom(
  String formatIntMMMSS(
  String formatIntHHMMSS(
  String formatInt(
  String formatIntP(
  String formatFloat(
  void updateDisplayLine(
  String controlStateS(
  void I2CScan(
  void lcdReInit(
  void cycleLog(
  void demandLog(
  void calculate(
*/
// --------  simple i/o pins -----
const int ledPin = 2;
const int analogPin = 36;  // analog input from tado controller - normalised for 0-3.3v range

#define USEBT

#ifdef USEBT 
//------  bluetooth -------
#include "BluetoothSerial.h"
const String btName = "BOILER";     // will append -n from config
BluetoothSerial SerialBT;
#endif

// ------------- EEPROM config ---------------
#include <EEPROM.h>

// ---------   watchdog config
#include <esp_task_wdt.h>
const int WDT_TIMEOUT = 30;

//---------- dallas/onewire config ----------
#include <OneWire.h>
#include <DallasTemperature.h>
// GPIO where the DS18B20 is connected to
const int oneWireBus = 15;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire ds(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&ds);
// data
const int SENSORMAPCOUNT = 5;    // temperature by userid - handle up to this many
int sensorMapping[SENSORMAPCOUNT];
float temperatures[SENSORMAPCOUNT];
// --------- dallas/onwire end --------------

// --------- LCD config------------------
// library used is the tandembite on github marco schwartz
// only niggle line wrap skips a line - same bus drived RTC
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // 4 line x 20 char
// --------- LCD end------------------

// ------------- serial input buffer ----------
const int buffLen = 100;
char buffer[buffLen];
int bPtr = 0; 

// ------ i2c control interface config
#include "PCF8574.h"
const byte PFCADDRESS = 0x20;
PCF8574 PCF_01(PFCADDRESS);
const byte CHVALVEBIT = 0;       // overrides pump to on
const byte HWVALVEBIT = 1;       // override HW valve to on
const byte HWPUMPBIT = 2;        // turn on hw pump
const byte BOILEROFFBIT = 3;     // interrupts thermostat on boiler
const byte CHDEMANDBIT = 4;       // read CH valve state
const byte HWDEMANDBIT = 5;       // read HW valve state
const byte TEMPLOLIMBIT  = 6;   // thermostat in state
const byte READMASK = 0xF0;      // 4 top bits high - must set on write as input

// values to set state
boolean setBoilerOff = false;
boolean setChValveOn = false;
boolean setHwValveOn = false;
boolean setHwPumpOn = false;

//values from read state
boolean extChDemand = false;      // ch demand from backup stat
boolean chDemand = false;         // ch demand from analogue
boolean extHwDemand = false;      // hw demand from hw stats
boolean hwDemand = false;         // hw demand from dallas
boolean tempLowLimit = false;     // kicks on when boiler hits low protect level - overrides setBoilerOff
// and previous
boolean prevChDemand = false;         // ch demand from ch stats
boolean prevHwDemand = false;         // hw demand from hw stats
// ----- i2c control end -------------

// config data from eeprom - use int array
const int configSize = 21;
int configArr[configSize];
const int annaLowIx = 0;
const int annaHighIx = 1;
const int annaHystIx = 2;          // drop under on level for off
const int annaSamplesIx = 3;         // min readings over or under for switch
const int annaChangeIntervalIx = 4;   // limit frequence of change (s)
const int thermostatLowIx = 5;
const int thermostatHighIx = 6;
const int thermostatHwIx = 7;
const int thermostatHystIx = 8;
const int anticycleOnOnIx = 9;
const int anticycleOffOnIx = 10;
const int rateAdjustIx = 11;
const int shutdownTempIx = 12;
const int boilerOffDelayIx = 13;
const int hwPumpHystIx = 14;
const int autoRestartHoursIx = 15;
const int selfTestIx = 16;
const int cpuFreqIx = 17;           //1=40, 2=80, 3=160, 4=240
const int btNameIx = 18;           //- appends -n to nale
const int dsResolutionIx = 19;     //- ds resolution request
const int dsErrWarnLevelIx = 20;

// first 2 chars = code key
const String configNames[configSize] = {
  "al-analogLow     ",
  "ah-analogHigh    ",
  "ay-analogHyst    ",
  "as-analogSamples ",
  "ai-analogChgIntvl",
  "tl-thermostatLow ",
  "th-thermostatHigh",
  "tw-thermostatHW  ",
  "ty-thermostatHyst",
  "a1-anticycleOnOn ",
  "a2-anticycleOffOn",
  "ra-rateAdjust    ",
  "st-shutdownTemp  ",
  "od-boileroffdelay",
  "hh-hwPumpHyst    ",
  "ar-autoRestartHrs",
  "tm-testmode<>0   ",
  "cf-clockfreq     ",
  "bt-bluetooth-n   ",
  "tr-dsresln 9-12  ",
  "dw-ds err level  ",  // 
};
const int configMin[configSize]   = { 0,  50,  1,  1,  5, 40, 50, 50, 1,  30, 10,  0, 20, 0,  0,   1, 0, 1, 0,  9 , 1 }; 
const int configMax[configSize]   = {50, 100, 10, 10, 60, 60, 70, 60, 7, 180, 60, 30, 50, 5, 10, 100, 5, 4, 9, 12 , 5 }; 
const int configReset[configSize] = { 5,  75,  2, 10, 30, 50, 65, 60, 4, 120, 30, 20, 45, 3,  3,  24, 0, 2, 0, 12 , 1 };
String configCodes[configSize];

// state engine - we have 6 states:
// 1 - off - action disabled - outputs off, but monitoring
// 1 - standby - !chDemand && !hwDemand && boiler cool - waiting for on - nothing active
// 2 - shutdown - !chDemand && !hwDemand && boiler still hot - select either or both
// 3 - anticycle - chDemand || hwDemand && holding to anticycle
// 4 - heating  - chDemand || hwDemand && under hightemp
// 5 - cooling  - chDemand || hwDemand  and cooling to hightemp-hysterisis anticycle delay

int controlState = 0;
const int CONTROLSOFF = 1;
const int CONTROLSSTANDBY = 2;
const int CONTROLSSHUTDOWN = 3;
const int CONTROLSANTICYCLE = 4;
const int CONTROLSHEATING = 5;
const int CONTROLSCOOLING = 6;

int prevControlState = 0;
String controlStateS(int state)
{
  switch (state)
  {
    case CONTROLSOFF:       return "off    ";
    case CONTROLSSTANDBY:   return "standby";
    case CONTROLSSHUTDOWN:  return "shutdwn";
    case CONTROLSANTICYCLE: return "anticyc";
    case CONTROLSHEATING:   return "heating";
    case CONTROLSCOOLING:   return "cooling";
    default: return "unknown";
  }
}

// globals
unsigned int elapsedMillis = 0;
const char C = ',';
unsigned int nextMillis;      // used to synch main loop to 1 sec
int sNow;                     // seconds since restart from Millis() - used for all timing
String prevLcdRow[] = {"","","",""};   // previously displays lcd data row by row
const int BOILERTEMPSSIZE = 5;
float boilertemps[BOILERTEMPSSIZE];  // keep last N temps - [0] is latst
float adjustedBoilerTemp;     // after rate adjustment
float boilerTempAdjustment;   // the adjustment made
float highestBoilerTemp;      // max temp since last log
float lowestBoilerTemp;       // min temp since last log
int boilerOnElapsed;          // boiler on to off time
int lastBoilerOnAt;           // when last turn on
int boilerOffElapsed;         // boiler off to on time
int lastBoilerOffAt;          // last turn off
float dutyCycle = 0;          // on / (on + off)%
int nextBoilerOnAt;           // used specifically for anti cycle - earliest allowable on
bool breakShutdownCycle = false;  // if we get trigger of boiler on in shotdown
bool shutdownChValve = false;     // if we include ch valve in shutdown
byte lastInputs = 0;          // last PFC read
// analog input processing
int annaChangeAt = 0;         // when can next switch CH from Tado signal
int annaValues[10];           // must be >= annaSamples Max - raw values from ADC
float annaAverage = 0;        // average value used as 0-100
float boilerOffTemp = 0;      // boiler demand temp C - tirn off point
float boilerOnTemp = 0;       // boiler demand temp C - turn on point

// demand log variables
int nextDemandLogAt = 10;
float lastLogAnnaAverage = 0;
bool logHwDemandAs = false;
bool logChDemandAs = false;
int demandLogInterval = 30;

// log files - referred to by index
const int NUMLOGFILES = 4;
const int CMDLOGFILE = 0;
const int MAINLOGFILE = 1;
const int CYCLELOGFILE = 2;
const int DEMANDLOGFILE = 3;

String logFileHeaders[] = {
  "cmd",
  "log",
  "ontime,offtime,duty,offtemp,tmax,tmin,chdemand,hwdemand",
  "tado,offtemp,chdemand,hwdemand"};

// test stuff
bool stActive = false;
int lastStConfig = 0;

int stControl = 0;     // tado analog
int stTimer;
float stAnnaValue = 0;
float stAnnaMax = 0;

bool stExtChDemand = false;  // keep false
bool stTempLowLimit = false;

int stHwControl = 0;  // HW
int stHwTimer;
bool stExtHwDemand = false;

float stTemp = 20;   // boiler temp

void resetConfig()
{
  cmdlog("resetting config");
  for (int ix = 0; ix < configSize ; ix++)
  {
    configArr[ix] = configReset[ix];
  }
  logConfig();
}

float randomF(float v1, float v2)
{
  float f = random(v1 * 100, v2 * 100);
  return f / 100.0;
}

void selfTest()
{
  bool initMe = false;
  if (lastStConfig != configArr[selfTestIx])
  {
    lastStConfig = configArr[selfTestIx];
    initMe = true;
  }
  switch (configArr[selfTestIx])
  {
    case 1:
      stActive = true;
      selfTest1(initMe);
      break;

    case 2:
      stActive = true;
      selfTest2(initMe);
      break;

    default:
      stActive = false;
      stControl = 0;
      stHwControl = 0;
      break;
  }
}

void selfTest1(bool initMe)
{
  if (initMe)
  {
    demandLogInterval = 10;
    configArr[anticycleOnOnIx] = 20;
    configArr[anticycleOffOnIx] = 10;
  }
  int t;
  switch (stHwControl)
  {
    case 0:
      t = random(30, 60);
      log("ST-1 HW hold OFF " + String(t) + "s");
      stExtHwDemand = false;
      stHwTimer = sNow + t;
      stHwControl++;
      break;
    case 1:
      if (sNow > stHwTimer)
      {
        stHwControl++;
      }
      break;
    case 2:
      t = random(30, 60);
      log("ST-1 HW hold ON " + String(t) + "s");
      stExtHwDemand = true;
      stHwTimer = sNow + t;
      stHwControl++;
      break;
    case 3:
      if (sNow > stHwTimer)
      {
        stHwControl = 0;
      }
      break;
  }

  switch (stControl)
  {
    case 0:
      log("ST-1 Anna hold 0% 10s");
      stAnnaValue = 0;
      stTimer = sNow + 10;
      stControl++;
      break;
    case 1:
      if (sNow > stTimer)
      {
        stControl++;
        stAnnaMax = randomF(40, 90);
        log("ST-1 Anna fast ramp up to " + String(stAnnaMax));
      }
      break;
    case 2:
      stAnnaValue += randomF(2, 5);
      if (stAnnaValue >= stAnnaMax)
      {
        stControl++;
        log("ST-1 Anna hold until cooling");
      }
      break;
    case 3:
      if (controlState == CONTROLSCOOLING)
      {
        stControl++;
        stTimer = sNow + 20;
        log("ST-1 Anna hold " + String(stAnnaValue) + " 20s");
      }
      break;
    case 4:
      if (sNow > stTimer)
      {
        stControl++;
        log("ST-1 Anna slow ramp down to 0%");
      }
      break;
    case 5:
      stAnnaValue -= randomF(0, 2);
      if (stAnnaValue <= 0)
      {
        stAnnaValue = 0;
        stControl++;
        log("ST-1 Anna hold until off");

      }
      break;
    case 6:
      if (hwDemand)
      {
        stControl = 1;
        stTimer = sNow + 20;
        log("ST-1 anna holding to OFF abanndon on HW demand; hold 20s");
      }
      if (controlState == CONTROLSSTANDBY)
      {
        stControl = 1;
        stTimer = sNow + 20;
        log("ST-1 anna holding to OFF complete; hold 20s");
      }
      break;
  }
  switch (controlState)
  {
    case CONTROLSHEATING:
      stTemp += randomF(0.3, 0.9);
      break;
    case CONTROLSSHUTDOWN:
    case CONTROLSCOOLING:
    case CONTROLSANTICYCLE:
      stTemp -= randomF(0.3, 0.9);
    default:
      stTemp = max(stTemp - 0.2, 20.0);
      break;
  }
}

void selfTest2(bool initMe)
{
  if (initMe)
  {
    demandLogInterval = 10;
    configArr[anticycleOnOnIx] = 20;
    configArr[anticycleOffOnIx] = 10;
    stControl = 0;
  }
  int t;

  switch (stControl)
  {
    case 0:
      log("ST-2 hold off 10s");
      stAnnaValue = 0;
      stExtHwDemand = false;
      stTimer = sNow + 10;
      stControl++;
      break;
    case 1:
      if (sNow > stTimer)
      {
        stAnnaValue = 90;
        stExtHwDemand = false;
        stControl++;
        log("ST-2 anna to 90, hw OFF; wait for cooling");
      }
      break;
    case 2:
      if (controlState == CONTROLSCOOLING)
      {
        log("ST-2 hold at anna=90 for 10s");
        stTimer = sNow + 10;
        stControl++;
      }
      break;
    case 3:
      if (sNow > stTimer)
      {
        stAnnaValue = 0;
        stExtHwDemand = false;
        stControl++;
        log("ST-2 anna to 0; hw OFF; wait OFF");
      }
      break;
    case 4:
      if (controlState == CONTROLSSTANDBY)
      {
        log("ST-2 hold in off for 10s");
        stTimer = sNow + 10;
        stControl++;
      }
      break;
      
    case 5:
      if (sNow > stTimer)
      {
        stAnnaValue = 0;
        stExtHwDemand = true;
        stControl++;
        log("ST-2 anna 0; hw to ON; wait cooling");
      }
      break;
    case 6:
      if (controlState == CONTROLSCOOLING)
      {
        log("ST-2 hold with HW for 10s");
        stTimer = sNow + 10;
        stControl++;
      }
      break;
     case 7:
      if (sNow > stTimer)
      {
        stAnnaValue = 0;
        stExtHwDemand = false;
        stControl++;
        log("ST-2 anna 0; hw to OFF; wait till OFF");
      }
      break;
    case 8:
      if (controlState == CONTROLSSTANDBY)
      {
        log("ST-2 back to start");
        stControl = 0;
      }
      break;
  }
  switch (controlState)
  {
    case CONTROLSHEATING:
      stTemp += randomF(0.2, 1.0);
      break;
    case CONTROLSSHUTDOWN:
    case CONTROLSCOOLING:
    case CONTROLSANTICYCLE:
      stTemp -= randomF(0.2, 1.0);
    default:
      stTemp = max(stTemp - 0.5, 20.0);
      break;
  }
}

// logging

void cmdlog(String s)    // for command responses
{
  logIx(CMDLOGFILE, s);
}

void log(String s)    // default to mainlogfile
{
  logIx(MAINLOGFILE, s);
}

void logIx(int ix, String s)
{
  String ss = "f:" + String(ix) + C + s;
  Serial.println(ss);
#ifdef USEBT  
  if (SerialBT.hasClient())
  {
    SerialBT.println(ss);
  }
#else
  Serial2.println(ss);
#endif
}

void cycleLog()   // logs at each boiler off
{
  // "time,ontime,offtime,duty,tdemand,tmax,tmin,chdemand,hwdemand",
  String dataline = String(boilerOnElapsed) + C + String(boilerOffElapsed)
                    + C + String(dutyCycle) + C + String(boilerOffTemp) + C + String(highestBoilerTemp) 
                    + C + String(lowestBoilerTemp) + C + String(chDemand) + C + String(hwDemand);
  logIx(CYCLELOGFILE, dataline);
}

void demandLog()
{
  // log when state change in ch or hw or tado temp shift of 2% time limited
  // make sure log before+after each demand transition for graph
  bool loggIt = false;
  if (chDemand != prevChDemand)
  {
    loggIt = true;
    logChDemandAs = prevChDemand;
  }
  else 
  {
    if (logChDemandAs != chDemand)
    {
      loggIt = true;
    }
    logChDemandAs = chDemand;
  }
  
  if (hwDemand != prevHwDemand)
  {
    loggIt = true;
    logHwDemandAs = prevHwDemand;
  }
  else 
  {
    if (logHwDemandAs != hwDemand)
    {
      loggIt = true;
    }
    logHwDemandAs = hwDemand;
  }
  if (abs(lastLogAnnaAverage - annaAverage) >= 2 && sNow >= nextDemandLogAt)
  {
    loggIt = true; 
  }
  if (loggIt)
  {
    lastLogAnnaAverage = annaAverage;
    nextDemandLogAt = sNow + demandLogInterval;
    // "time,tado,offtemp,chdemand,hwdemand",
    String dataline = String(annaAverage) + C + String(boilerOffTemp)
                    + C + String(logChDemandAs) + C + String(logHwDemandAs);
    logIx(DEMANDLOGFILE, dataline);
  }
}

void logConfig()
{
  for (int ix = 0; ix < configSize; ix++)
  {
    cmdlog(configNames[ix] + " = " + String(configArr[ix]));
  }
}

void saveToEprom()
{
  int im = 0;
  for (int ix = 0; ix < configSize; ix++)
  {
    EEPROM.write(im++, highByte(configArr[ix]));
    EEPROM.write(im++, lowByte(configArr[ix]));
  }
  EEPROM.commit();
  cmdlog("EEPROM save:");
  logConfig();
}

void loadFromEprom()
{
  int im = 0;
  for (int ix = 0; ix < configSize; ix++)
  {
    configArr[ix] =  word( EEPROM.read(im++), EEPROM.read(im++));
    if (configArr[ix] < configMin[ix] || configArr[ix] > configMax[ix])
    {
      configArr[ix] = configReset[ix];
    }
  }
  cmdlog("EEPROM load:");
  logConfig();
}

// ---------- utils -------------------
void handleMapSensors()
{
  if (bPtr == 1)
  {
    mapSensors();
    return;
  }
  if (bPtr != 3 || !isDigit(buffer[1]) || !isDigit(buffer[2]))
  {
    cmdlog("!! not 'mnn'");
    return;
  }
  int ix = buffer[1]-48;
  int ud = buffer[2]-48;
  for (int it = 1; it < 5; it++)
  {
    sensors.setUserDataByIndex(ix, ud);
    int cud = sensors.getUserDataByIndex(ix);
    if (cud == ud)
    {
      log("ix=" + String(ix) + " ud change " + String(cud) + " > " + String(cud)); 
      break;
    }
    else
    {
      log("!! ix=" + String(ix) + " set ud fail: " + String(ud) + " != " + String(cud) + " x" + String(ix)); 
      delay(10);
    } 
  }
  mapSensors();
}


// maps sensor index into sensor mapping on sensor ix
void mapSensors()
{
  // clear mapping
  for (int ix = 0; ix < SENSORMAPCOUNT; ix++)
  {
    sensorMapping[ix] = -1;
  }
  for (int ix = 0; ix < sensors.getDeviceCount(); ix++)
  {
    int ud;
    for (int it = 1; it < 4; it++)
    {
      int ud1 = sensors.getUserDataByIndex(ix);
      delay(10);
      int ud2 = sensors.getUserDataByIndex(ix);
      if (ud1 == ud2)
      {
        ud = ud1;
        break;
      }
      log("!! ix=" + String(ix) + " retry get ud x" + String(it));
    }
    String info = "ix:" + String(ix) + ", t=" + String(sensors.getTempCByIndex(ix)) + ", ud=" + String(ud) + ", ";

    if (ud >= 0 && ud < SENSORMAPCOUNT)
    {
      if (sensorMapping[ud] != -1)
      {
        cmdlog(info + "!!duplicate ud");
      }
      else
      {
        sensorMapping[ud] = ix;
        cmdlog(info + "mapped");
      }
    }
    else
    {
      cmdlog(info + "not mapped");
    }
  }
}

// does the temp reading and store
void getTemperatures()
{
  for (int ix = 0; ix < SENSORMAPCOUNT; ix++)
  {
    if (sensorMapping[ix] >= 0 && sensorMapping[ix] < SENSORMAPCOUNT)
    {
      for (int it = 1; it < 4; it++)
      {
        temperatures[ix] = sensors.getTempCByIndex(sensorMapping[ix]);
        if (temperatures[ix] != -127.0)
        {
          break;
        }
        else
        {
          if (it >= configArr[dsErrWarnLevelIx])    // maybe count as well
          {
            log("!! ds read error ix=" + String(ix) + " x" + String(it));
          }
          delay(10);
        }
      }
    }
  }
  if (stActive)
  {
    temperatures[0] = stTemp;
  }
}

// always updates 20 characters of row
void updateDisplayLine(int row, String nl)
{
  int sIx = -1;                     // idx of start of changed section
  //String nl = nlIn;
  while (nl.length() < 20)    // stretch to 20
  {
     nl+= ' ';
  }
  for (int ix = 0; ix < 20; ix++)
  {
    // check if same..
    if (ix < prevLcdRow[row].length() && prevLcdRow[row][ix] == nl[ix])
    {
      if (sIx >= 0)     
      {
        lcd.setCursor(sIx, row);   // write out what we have and reset
        lcd.print(nl.substring(sIx,ix));
        sIx = -1;
      }
    }
    else  // is diff
    {
      if (sIx < 0)
      {
        sIx = ix;     // start of new different section
      }
    }
  }  
   
  if (sIx >= 0)
  {
    lcd.setCursor(sIx, row);    // write out any remainder
    lcd.print(nl.substring(sIx, 20));
  }
  prevLcdRow[row] = nl;    
}

// display
void updateDisplay()
{
  display1();
}
String formatIntMMMSS(int t)
{
  int seconds = t % 60;
  int minutes = (t - seconds) / 60;
  return formatInt(minutes, 3 ) + ":" + formatIntP('0', seconds, 2);
}
String formatIntHHMMSS(int t)
{
  int seconds = t % 60;
  t = (t - seconds) / 60;
  int minutes = t % 60;
  int hours = (t - minutes) / 60;
  return formatInt(hours, 2) + ":" + formatIntP('0', minutes, 2 ) + ":" + formatIntP('0', seconds, 2);
}

String formatInt(int i, int digits)
{
  return formatIntP(' ', i, digits);
}

String formatIntP(char padd, int i, int digits)
{
  String s = String(i);
  if (s.length() > digits)
  {
    s = "";
    while (s.length() < digits)
    {
      s = "x" + s;
    }
  }
  else
  {
    while (s.length() < digits)
    {
      s = padd + s;
    }
  }
  return s;
}
String formatFloat(float d, int digits, int places)
{
  if ((digits == 2 && d >= 99.9) || (digits == 1 && d >= 9.9))
  {
    places--;
    digits++;
  }
  if (places < 1)
  {
    int i = d;
    return formatInt(i, digits);
  }
  int totLen = digits + 1 + places;
  String s = String(d, places);
  if (s.length() > totLen)
  {
    
    //log("?? display " + s + " " + String(d));
    s = "";
    while (s.length() < totLen )
    {
      s = "*" + s;
    }
  }
  else
  {
    while (s.length() < totLen)
    {
      s = ' ' + s;
    }
  }
  return s;
}

// display temperatures + inputs
void display1()
{
  // first line set low - indicator - act/adjustedment - indicator - set high
  String s;
  switch (controlState)
  {
    case CONTROLSSHUTDOWN:
      s += formatInt(configArr[shutdownTempIx], 2) + " s ";
      break;
    case CONTROLSCOOLING:
      s += formatInt(boilerOnTemp, 2) + " < ";
      break;
    default:
      s += formatInt(boilerOnTemp, 2) + "   ";
  }
  s += formatFloat(boilertemps[0], 2, 1);

  s += "  ";
  s += formatFloat(boilerTempAdjustment, 2, 1);

  switch (controlState)
  {
    case CONTROLSANTICYCLE:
      s += " h " + formatInt(boilerOffTemp, 2);
      break;
    case CONTROLSHEATING:
      s += " > " + formatInt(boilerOffTemp, 2);
      break;
    default:
      s += "   " + formatInt(boilerOffTemp, 2);
      break;
  }
  updateDisplayLine(0,s);

  s = formatIntMMMSS(boilerOnElapsed) + " " + formatIntMMMSS(boilerOffElapsed) + " " + formatFloat(dutyCycle, 3, 0) + "%";
  updateDisplayLine(1,s);

  s = formatFloat(annaAverage, 2, 1) + "%  " + formatFloat(temperatures[1], 2, 1);
  updateDisplayLine(2,s);

  s = "";
  if (chDemand || setChValveOn)
  {
    s += "ch ";
  }
  else
  {
    s += ".. ";
  }

  if (hwDemand || setHwValveOn)
  {
    s += "hw ";
  }
  else
  {
    s += ".. ";
  }
  if (setHwPumpOn)
  {
    s += "hp ";
  }
  else
  {
    s += ".. ";
  }
  if (tempLowLimit)
  {
    s += "lt ";
  }
  else
  {
    s += ".. ";
  }
  s += controlStateS(controlState);
  updateDisplayLine(3,s);
}

/*
void display2()
{
  // was display for changing config
}
void display3()
{
  // had display of last inputs (relevant) sd, rtc, elased time
}
void display4()
{
  // sensor temperatures display - poss keep
  // list temperatures of mapped sensors
}

void display5()
{
  // list sensors as found with temp and selector
  // all sensors so can map - need re-implement
}
*/

// reads the Tado boiler analog singal
void readTadoAnalog()
{
  // take average of last sample values as raw analogue (0-4095)
  long annaTotal = 0;
  for (int ix = configArr[annaSamplesIx] - 1; ix >= 1; ix--)
  {
    // shift down and add up
    annaValues[ix] = annaValues[ix - 1];
    annaTotal += annaValues[ix];
  }
  annaValues[0] = analogRead(analogPin); // take current value
  if (stActive)  // override if self test
  {
    annaValues[0] = stAnnaValue * 4096 / 100;
  }
  annaTotal += annaValues[0];
  // find average - drop sample most away from average
  int tempAverage = annaTotal / configArr[annaSamplesIx];
  int dropIx = 0;
  int maxDiff = 0;
  for (int ix = 0; ix < configArr[annaSamplesIx]; ix++)
  {
    if (abs(annaValues[ix] - annaAverage) > maxDiff)
    {
      maxDiff = abs(annaValues[0] - annaAverage);
      dropIx = ix;
    }
  }
  // take new average from remaining 9 scaled 0-100
  annaTotal -= annaValues[dropIx];
  annaAverage = annaTotal / (configArr[annaSamplesIx] - 1) * 100.0 / 4096.0;

  // determine if chdemand on or off - allow for min switching interval && hysterisis
  if (controlState == CONTROLSOFF)
  {
    chDemand = false;
  }
  else if (annaAverage >= configArr[annaLowIx])
  {
    if (!chDemand)
    {
      if (sNow >= annaChangeAt)
      {
        annaChangeAt = sNow + configArr[annaChangeIntervalIx];
        chDemand = true;
        log("TadoCH > ON");
      }
      //      else
      //      {
      //        log("on t-" + String(annaChangeAt-now));
      //      }
    }
  }
  else if (annaAverage < configArr[annaLowIx] - configArr[annaHystIx])
  {
    if (chDemand)
    {
      if (sNow >= annaChangeAt)
      {
        annaChangeAt = sNow + configArr[annaChangeIntervalIx];
        chDemand = false;
        log("TadoCH > OFF");
      }
      //      else
      //      {
      //        //log("off t-" + String(annaChangeAt-now));
      //      }
    }
  }
  // determine demand boiler temp
  float temp = (annaAverage - configArr[annaLowIx]) / (configArr[annaHighIx] -  configArr[annaLowIx]) *
               (configArr[thermostatHighIx] - configArr[thermostatLowIx]) +  configArr[thermostatLowIx];
  boilerOffTemp = constrain(temp, configArr[thermostatLowIx], configArr[thermostatHighIx]);
  // if HW demand must be sufficient for HW
  if (hwDemand)
  {
    boilerOffTemp = constrain(temp, configArr[thermostatHwIx], configArr[thermostatHighIx]);
  }
  boilerOnTemp = boilerOffTemp - configArr[thermostatHystIx];
}

void readHWTemp()
{
  hwDemand = extHwDemand;
}

void calculate()
{
  bool logged = false;

  // shift history down temp array
  for (int ix = 1; ix < BOILERTEMPSSIZE; ix++)
  {
    boilertemps[ix] = boilertemps[ix - 1];
  }
  boilertemps[0] = temperatures[0];

  // rate is between avg last 2 and avg prev 2
  float change = ((boilertemps[0] + boilertemps[1]) / 2 - (boilertemps[2] + boilertemps[3]) / 2);
  float rate = change / 2;    // as sample interval is 2 secs
  boilerTempAdjustment = rate * configArr[rateAdjustIx];
  adjustedBoilerTemp = boilertemps[0] + boilerTempAdjustment;

  // track high/low
  if (boilertemps[0] > highestBoilerTemp)
  {
    highestBoilerTemp = boilertemps[0];
  }
  if (boilertemps[0] < lowestBoilerTemp)
  {
    lowestBoilerTemp = boilertemps[0];
  }

  if (chDemand != prevChDemand || hwDemand != prevHwDemand)
  {
    //logIx(DEMANDLOGFILE, String(hwDemand) + C + String(chDemand));   ?? check
    String s;
    // been a change
    if (chDemand != prevChDemand)
    {
      s += "CH > " + onOffS(chDemand);
    }
    else
    {
      s += "CH = " + onOffS(chDemand);
    }
    if (hwDemand != prevHwDemand)
    {
      s += ", HW > " + onOffS(hwDemand);
    }
    else
    {
      s += ", HW = " + onOffS(hwDemand);
    }
    log(s);
  }

  if (controlState == CONTROLSOFF)
  {
      setHwPumpOn = false;
      setBoilerOff = false;
      setHwValveOn = false;
      setChValveOn = false;
  }

  else if (!chDemand && !hwDemand)
  {
    // if demand dropped with boiler lit - record cycle.
    if (!chDemand && !hwDemand && (prevChDemand || prevHwDemand) && !setBoilerOff)
    {
      boilerOnElapsed =   sNow - lastBoilerOnAt;
      nextBoilerOnAt = max(nextBoilerOnAt, sNow + configArr[anticycleOffOnIx]);
      lastBoilerOffAt = sNow;
      int fullcyclesec = (boilerOffElapsed + boilerOnElapsed);
      if (fullcyclesec > 0)
      {
        dutyCycle = (boilerOnElapsed * 100) / fullcyclesec;
      }
      else
      {
        dutyCycle = 100;
      }
      cycleLog();
      highestBoilerTemp = 0;
      lowestBoilerTemp = 100;
      log("drop demand when lit; ontime=" + formatIntMMMSS(boilerOnElapsed) + "; dc=" + String(dutyCycle));
        
    }
    setHwPumpOn = false;
    if (boilertemps[0] > configArr[shutdownTempIx] && !breakShutdownCycle)
    {
      if (tempLowLimit)
      {
        breakShutdownCycle = true;
        log("temp low limit in shutdown - stopped..");
      }
      else
      {
        // dump heat cycle
        controlState = CONTROLSSHUTDOWN;
        if (prevControlState != CONTROLSSHUTDOWN)
        {
          shutdownChValve = prevChDemand;
        }
        setChValveOn = shutdownChValve;
        setHwValveOn = true;
        setBoilerOff = true;
      }
    }
    else
    {
      // wait for demand
      controlState = CONTROLSSTANDBY;
      setChValveOn = false;
      setHwValveOn = false;
      setBoilerOff = false;
    }
  }
  else
  {
    breakShutdownCycle = false;
    setChValveOn = false;
    setHwValveOn = false;

    if (!prevChDemand && !prevHwDemand)
    {
      // initial state on coming out of no demand.. can't be fully sure of boiler state
      if (adjustedBoilerTemp >= boilerOffTemp )
      {
        setBoilerOff = true;
      }
      else
      {
        setBoilerOff = false;
      }
      lastBoilerOnAt = sNow;
      lastBoilerOffAt = sNow;
      boilerOnElapsed = 0;
      boilerOffElapsed = 0;
    }

    if (setBoilerOff)   // cooling to hysterisis temp or anticycle or was in shutdown
    {
      if (adjustedBoilerTemp <= boilerOnTemp)
      {
        boilerOffElapsed = sNow - lastBoilerOffAt;
        if (sNow >= nextBoilerOnAt)
        {
          // boiler can come on
          controlState = CONTROLSHEATING;
          setBoilerOff = false;
          lastBoilerOnAt = sNow;
          nextBoilerOnAt =  max(nextBoilerOnAt, sNow + configArr[anticycleOnOnIx]);
          int fullcyclesec = (boilerOffElapsed + boilerOnElapsed);
          if (fullcyclesec > 0)
          {
            dutyCycle = (boilerOnElapsed * 100) / fullcyclesec;
          }
          else
          {
            dutyCycle = 100;
          }
          log(controlStateS(prevControlState) + ">" + controlStateS(controlState) + "; offtime=" + formatIntMMMSS(boilerOffElapsed) + "; dc=" + String(dutyCycle));
          logged = true;
        }
        else
        {
          controlState = CONTROLSANTICYCLE;       // anticycle wait
        }
      }
      else
      {
        controlState = CONTROLSCOOLING;
        boilerOffElapsed = sNow - lastBoilerOffAt;   // still cooling to hys temp
      }
    }
    else
    {
      boilerOnElapsed =   sNow - lastBoilerOnAt;
      if (adjustedBoilerTemp >= boilerOffTemp)
      {
        // boiler off
        controlState = CONTROLSCOOLING;
        setBoilerOff = true;
        nextBoilerOnAt = max(nextBoilerOnAt, sNow + configArr[anticycleOffOnIx]);
        lastBoilerOffAt = sNow;
        int fullcyclesec = (boilerOffElapsed + boilerOnElapsed);
        if (fullcyclesec > 0)
        {
          dutyCycle = (boilerOnElapsed * 100) / fullcyclesec;
        }
        else
        {
          dutyCycle = 100;
        }
        cycleLog();
        highestBoilerTemp = 0;
        lowestBoilerTemp = 100;
        log(controlStateS(prevControlState) + ">" + controlStateS(controlState) + "; ontime=" + formatIntMMMSS(boilerOnElapsed) + "; dc=" + String(dutyCycle));
        logged = true;
      }
      else
      {
        controlState = CONTROLSHEATING;
      }
    }
    // click hw pump on at onTemp - off at same - hysteresis
    if (!hwDemand)
    {
      setHwPumpOn = false;
    }
    else if (!setHwPumpOn)
    {
      if (adjustedBoilerTemp >= boilerOnTemp)
      {
        setHwPumpOn = true;
      }
    }
    else
    {
      if (adjustedBoilerTemp < boilerOnTemp - configArr[hwPumpHystIx])
      {
        setHwPumpOn = false;
      }
    }
  }

  if (!logged && controlState != prevControlState)
  {
    log(controlStateS(prevControlState) + ">" + controlStateS(controlState));
  }
 
}

void lcdReInit()
{
  // lcd
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();
  for (int ix = 0; ix < 4; ix++)
  {
    prevLcdRow[ix] = "";
  }
}



void readInputs()
{
  lastInputs = PCF_01.read8();
  if (stActive)
  {
    extChDemand = stExtChDemand;
    extHwDemand = stExtHwDemand;
    tempLowLimit = stTempLowLimit;
  }
  else
  {
    extChDemand = !bitRead(lastInputs, CHDEMANDBIT);
    extHwDemand = !bitRead(lastInputs, HWDEMANDBIT);
    tempLowLimit = !bitRead(lastInputs, TEMPLOLIMBIT);
  }
}

// assumes we have last state in lastInputs
// only change one at a time - !setboileroff is delayed after valve off
int boilerOffDelayAt = 0;

void setOutputs()
{
  byte existB = READMASK | lastInputs;
  byte newB = setOutputs2(existB);
  if (newB != existB)
  {
    PCF_01.write8(newB);
  }
}

String onOffS(bool b)
{
  if (b) return "ON";
  return "OFF";
}

byte setOutputs2(byte existB)
{
  byte newB = existB;
  bool chv = setChValveOn || chDemand;
  bitWrite(newB, CHVALVEBIT, !chv);
  if (newB != existB)
  {
    if (!chv)
    {
      boilerOffDelayAt = sNow + configArr[boilerOffDelayIx];
    }
    log("CHVALVE > " + onOffS(chv));
    return newB;
  }
  bitWrite(newB, HWVALVEBIT, !setHwValveOn);
  if (newB != existB)
  {
    if (!(setHwValveOn))
    {
      boilerOffDelayAt = sNow + + configArr[boilerOffDelayIx];
    }
    log("HWVALVE > " + onOffS(setHwValveOn));
    return newB;
  }
  bitWrite(newB, HWPUMPBIT,  !setHwPumpOn);
  if (newB != existB)
  {
    log("HWPUMP > " + onOffS(setHwPumpOn));
    return newB;
  }
  if (sNow >= boilerOffDelayAt)
  {
    bitWrite(newB, BOILEROFFBIT, !setBoilerOff);
    if (newB != existB)
    {
      log("BOILEROFF > " + onOffS(setBoilerOff));
      return newB;
    }
  }
  return newB;
}

// scans devices on the i2c bus
void I2CScan()
{
  String s = "I2C scan: ";
  int count = 0;
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response)
    {
      s += String(i, HEX) + "  ";

      count++;
    }
  }
  if (count == 0)
  {
    s += "no devices found";
  }
  log(s);
}


int lastClockFreq = -1;
void setClockFreq()
{
  int freq = 80;
  switch (configArr[cpuFreqIx])
  {
    case 1:
      freq = 40;
      break;
    case 2:
      freq = 80;
      break;
    case 3:
      freq = 160;
      break;
    case 4:
      freq = 240;
      break;
  }
  if (freq != lastClockFreq)
  {
    log("set cpuf = " + String(freq) + ", res=" +  String(setCpuFrequencyMhz(freq)) + ", get=" + String(getCpuFrequencyMhz()));
    lastClockFreq = freq;
  }
}

void dohelp()
{
  cmdlog("help menu");
  cmdlog("? or h - help");
  cmdlog("c - show config values");
  cmdlog("cxx - show xx value");
  cmdlog("cxxnnn.. - set config value");
  cmdlog("e - save config to eprom");
  cmdlog("s - standby mode on/off");
  cmdlog("a - anticycle clear");
  cmdlog("r - reset config");
  cmdlog("l - lcd reinit");
  cmdlog("b - restart");
  cmdlog("m - show sensor map");
  cmdlog("mnn - change sensor ud");
}

void parseCmd()
{
  // first character is command
  switch (buffer[0])
  {
    case '?':
    case 'h':
      dohelp();
      break;
    
    case 'c':
      handleConfig();
      break;

    case 'e':
      saveToEprom();
      break;
    
    case 'a':
      if (controlState == CONTROLSANTICYCLE)
      {
        nextBoilerOnAt = sNow + 2;
        log("anticycle cancelled");
      }
      else
      {
        cmdlog("not in anticycle");
      }
      break;

    case 's':
      if (controlState != CONTROLSOFF)
      {
        controlState = CONTROLSOFF;
        log("state set to off");
      }
      else
      {
        controlState = CONTROLSSTANDBY;
        log("state set to standby");
      }
      break;

    case 'r':
      resetConfig();
      break;

    case 'l':
      lcdReInit();
      break;

    case 'b':
      log("user restart...");
      delay(100);
      ESP.restart();
      break;

    case 'm':
      handleMapSensors();
      break;
   
    default:
      cmdlog("'" + String(buffer[0]) + "' ??");
      break;
  }
  bPtr=0;  // reset buffer pointer
}

void handleConfig()
{
  if (bPtr == 1)
  {
    // just list config
    logConfig();
    return;
  }
  // next 2 char is param
  if (bPtr < 3)
  {
    cmdlog("!! no config code");
    return;
  }
  String s = String(buffer[1]) + String(buffer[2]);
 
  int ip = -1;
  for (int ix = 0; ix < configSize; ix++)
  {
    if (s == configCodes[ix])
    {
      ip = ix;
      break;
    }
  }
  if (ip == -1)
  {
    cmdlog("!! config '" + s + "' unknown");
    return;
  }
  if (bPtr == 3)
  {
    // just show param
    cmdlog(configCodes[ip] + "=" + String(configArr[ip]));
    return;
  }
  // parse value
  int value = 0;
  for (int ix = 3; ix < bPtr; ix++)
  {
    char c = buffer[ix];
    if (isDigit(c))
    {
      value = value*10 + c-48;
    }
    else
    {
      cmdlog("!! '" + String(c) + "' not digit");
      return;
    }
  }
  // change value
  if (value > configMax[ip])
  {
    cmdlog("!! > " + String(configMax[ip]));
    return;
  }
  if (value < configMin[ip])
  {
    cmdlog("!! < " + String(configMin[ip]));
    return;
  }
  int prev = configArr[ip];
  configArr[ip] = value;
  log(configCodes[ip] + ":" + String(prev) + " > " + String(configArr[ip]));
}

void setup()
{
  // basics
  Serial.begin(115200);

  // init from eprom..
  EEPROM.begin(configSize * 2);
  loadFromEprom();

#ifdef USEBT 
   // bluetooth
  if (configArr[btNameIx] > 0)
  {
    String actBtName = btName + "-" + String(configArr[btNameIx]);
    log("enable bluetooth - " + actBtName);
    SerialBT.begin(actBtName);
  }
  else
  {
    log("bluetooth disabled");  
  }
#else
  Serial2.begin(9600);
#endif

  
  // config short codes
  for (int ix = 0; ix < configSize; ix++)
  {
    configCodes[ix] = String(configNames[ix][0]) + String(configNames[ix][1]);
    for (int iy = 0; iy < ix; iy++)
    {
      if (configCodes[ix] == configCodes[iy])
      {
        log("!!duplicate short code '" + configCodes[iy] + "'");
      }
    }
  }
  
  // log data headers
  for (int ix = 0; ix < NUMLOGFILES; ix++)
  {
    logIx(ix, logFileHeaders[ix]);
  }
  log("start of initialize");
  pinMode(ledPin, OUTPUT);

  log(module);
  log(buildDate);

  log("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  

  // diagnostic - scan i2c bus
  I2CScan();

  // lcd
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(module);
  lcd.setCursor(0, 1);
  lcd.print(buildDate);

  // dallas
  // for somereason need to do twice - maybe onewire settling down?
  sensors.begin();
  log("Dallas sensors1: " + String(sensors.getDeviceCount()));
  sensors.begin();
  log("Dallas sensors2: " + String(sensors.getDeviceCount()));
  // map userdata to sensor map
  sensors.setResolution(configArr[dsResolutionIx]);
  mapSensors();
  lcd.setCursor(0, 2);
  lcd.print("sensors = ");
  lcd.print(sensors.getDeviceCount());
  sensors.requestTemperatures(); //  wait while get first readings..
  getTemperatures();
  // initialize
  lastBoilerOnAt = sNow;
  lastBoilerOffAt = sNow;
  highestBoilerTemp = 0;
  lowestBoilerTemp = 100;
  for (int ix = 0; ix < BOILERTEMPSSIZE; ix++)
  {
    boilertemps[ix] = temperatures[0];
  }
  sensors.setWaitForConversion(false); // we request and ask a second later. takes some 700ms for 12 bit
  sensors.requestTemperatures();

  // PFC control interface
  PCF_01.begin(255);  // set all bits on

  delay(2000);
 
}

void loop()
{
  // get time..
  sNow = millis() / 1000;

  //input from serial or BT treated equally
#ifdef USEBT 
  while (Serial.available() > 0 || (SerialBT.hasClient() && SerialBT.available() > 0))
  {
    char c;
    if (Serial.available() > 0)
    {
      c = Serial.read();
    }
    else
    {
      c= SerialBT.read();
    }
#else
  while (Serial.available() > 0 || Serial2.available() > 0)
  {
    char c;
    if (Serial.available() > 0)
    {
      c = Serial.read();
    }
    else
    {
      c= Serial2.read();
    }
#endif
    if ((c == '\r' || c == '\n'))
    {
      if (bPtr > 0)
      {
        parseCmd();
      }
    }
    else if (bPtr < buffLen)
    {
      buffer[bPtr++] = c;
    }
  }

  // check for 1 second boundary
  if (millis() >= nextMillis)
  {
    nextMillis += 1000;
    unsigned int startMillis = millis();
    digitalWrite(ledPin, 1);
    //!!setClockFreq();
    if (controlState == CONTROLSSTANDBY && sNow > configArr[autoRestartHoursIx] * 3600)
    {
      log("scheduled restart..");

      ESP.restart();
    }
    // 1 sec processing
    selfTest();
    readInputs();
    getTemperatures();
    readTadoAnalog();
    readHWTemp(); 
    calculate();
    setOutputs();
    demandLog();
    sensors.requestTemperatures(); // pick up next pass
    updateDisplay();
    digitalWrite(ledPin, 0);
    elapsedMillis = millis() - startMillis;
   
    // previous state for next pass
    prevControlState = controlState;
    prevChDemand = chDemand;
    prevHwDemand = hwDemand;
  }
  
  esp_task_wdt_reset();    // tickle watchdog

  delay(5);
}
/*
  version history

2021-05-18  start version 2.
  - simplify - drop RTC and SD logging - keep rest much the same
  - logging delivered by serial port to capture via logger device

2021-10-07  use bluetooth / serial for control
  
*/
