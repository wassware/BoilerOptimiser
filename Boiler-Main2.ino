
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
  void statusLog(
  void calculate(
*/
// --------  simple i/o pins -----
const int ledPin = 2;
const int analogPin = 36;  // analog input from tado controller - normalised for 0-3.3v range

//------  bluetooth -------
#include "BluetoothSerial.h"
const String btName = "BOILER";     // will append -n from config
BluetoothSerial SerialBT;

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
// sensors are
// 0 = boiler out
// 1 = boiler in
// 2 = hw tank mid
// 3 = hw tank bottom
const int SENSORMAPCOUNT = 6;    // temperature by userid - handle up to this many - but only use 4
int sensorMapping[SENSORMAPCOUNT]; // maps the random order of library to sensor user id
float temperatures[SENSORMAPCOUNT]; // last read temperatures
const int dsBoilerOutIx = 0;           // boiler = 0
const int dsBoilerInIx = 1;             // boiler in
const int dsMidTankIx = 2;
const int dsBottomTankIx = 3;
int nextSensorIx = 1;             // for round robin reas of sensort 1-n
const int dsRetries = 4;
int dsErrorCount[dsRetries];               // count errors at retry level
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
//const byte HWPUMPBIT = 2;        // turn on hw pump
const byte SPAREBIT = 2;
const byte BOILEROFFBIT = 3;     // interrupts thermostat on boiler

// values to set outputs
boolean setBoilerOff = false;    // turns boiler flame off - used as memory of heating / cooling
boolean setChValveOn = false;    // opens HW valve in shutdown - used as memory whether currently in shutdown
boolean setHwValveOn = false;    // opens CH valve in shutdown - ditto
//boolean setHwPumpOn = false;

//values from read state & inputs
int tadoLevel = 0;                // from tado, 1 = off, 2 = heat dump ok, 3 = chDemand 
boolean chEnabled = true;         // control to switch CH off - just does HW actions.
boolean chDemand = false;         // ch demand from analogue
boolean chDumpOk = false;         // enough ch demand from tado to be worth dumping in shutdown
boolean hwDemand = false;         // hw demand from tank top sensor
boolean hwPiggyback = false;      // from bottom tank sensor - hw on if ch on
// and previous
boolean prevChDemand = false;         // ch demand from ch stats
boolean prevHwDemand = false;         // hw demand from hw stats
// ----- i2c control end -------------

const int maxThermostatSamples = 32;

struct ConfigItem
{
  public:
  String name;
  String code;
  int value;
  int defValue;
  int minValue;
  int maxValue;
};
const int configSize = 23;
ConfigItem configArr[configSize];// =  {{"aa","aa-name",5,5,2,10},{"bb","bb-name",5,5,2,10}};

void addToConfig(int ix, String codex, String namex, int minValuex, int defValuex, int maxValuex)
{
  if (ix >= configSize)
  {
    log("!!config full at " + namex);
    return;
  }
  for (int ixx = 0; ixx < configSize; ixx++)
  {
    if (configArr[ixx].code == codex)
    {
      log("!!duplicate config code: " + codex);
    }
  }
  configArr[ix].name = namex;
  configArr[ix].code = codex;
  configArr[ix].value = defValuex;
  configArr[ix].defValue = defValuex;
  configArr[ix].minValue = minValuex;
  configArr[ix].maxValue = maxValuex;
}
// config data from eeprom - use int array

int tadoOffIx = -1;
int tadoChIx = -1;
int tadoHighIx = -1;
int tadoHystIx = -1;             
int tadoIntervalIx = -1;   
int thermostatSamplesIx = -1;    
int boilerLowIx = -1;
int boilerHighIx = -1;
int boilerHwIx = -1;
int boilerHystIx = -1;
int hwTempIx = -1;
int hwHystIx = -1;
int hwBtmTempIx = -1;
int hwIntervalIx = -1; 
int anticycleOnOnIx = -1;
int anticycleOffOnIx = -1;
int rateAdjustIx = -1;
int shutdownTempIx = -1;
int boilerOffDelayIx = -1;
//int hwPumpHystIx = -1;
int autoRestartHoursIx = -1;
int selfTestIx = -1;
int btNameIx = -1;           //- appends -n to nale
int dsErrWarnLevelIx = -1;

// read here for meaning
void InitConfig()
{
  int ix = 0;
  tadoOffIx = ix++;  addToConfig(tadoOffIx, "to", "tadoOff", 0,3,50);      // tado level where zero demand - valves likely closed
  tadoChIx = ix++;  addToConfig(tadoChIx, "tc", "tadoCh", 0,10,50);        // tado level where we kick on boiler at low temp
  tadoHighIx = ix++; addToConfig(tadoHighIx, "th", "tadoHigh", 50,80,100); // max value from tado for boiler at hi temp
  tadoHystIx = ix++;  addToConfig(tadoHystIx, "ty", "tadoHyst", 0, 1, 10); // hysteresis on tado level change
  tadoIntervalIx = ix++;  addToConfig(tadoIntervalIx, "ti", "tadoChgIntvl", 0, 60, 180);  // frequency limit on change
  thermostatSamplesIx = ix++;  addToConfig(thermostatSamplesIx, "ts", "thermostatSamples", 4, 12, maxThermostatSamples);
  boilerLowIx = ix++;  addToConfig(boilerLowIx, "bl", "boilerLowTemp", 40, 50, 60);     // lowest boiler temp in range
  boilerHighIx = ix++;  addToConfig(boilerHighIx, "bh", "boilerHighTemp", 60, 65, 70);  // highest boiler temp in range  
  boilerHwIx = ix++;  addToConfig(boilerHwIx, "tw", "thermostatHW", 55, 60, 70);// boiler temp when just HW 
  boilerHystIx = ix++;  addToConfig(boilerHystIx, "by", "thermostatHyst", 0, 4, 10);     
  hwTempIx = ix++;  addToConfig(hwTempIx, "ht", "hotWaterTemp", 40, 45, 55);
  hwHystIx = ix++;  addToConfig(hwHystIx, "hy", "hotWaterHyst", 0, 2, 10);
  hwBtmTempIx = ix++;  addToConfig(hwBtmTempIx, "bt", "hotWaterBottomTemp", 30, 45, 55);
  hwIntervalIx = ix++;  addToConfig(hwIntervalIx, "hi", "hwChgIntvl", 0, 60, 180);  // frequency limit on change
  anticycleOnOnIx = ix++;  addToConfig(anticycleOnOnIx, "a1", "anticycleOnOn", 30, 120, 180);
  anticycleOffOnIx = ix++;  addToConfig(anticycleOffOnIx, "a2", "anticycleOffOn", 15, 30, 120);
  rateAdjustIx = ix++;  addToConfig(rateAdjustIx, "ra", "rateAdjust", 0, 25, 50);
  shutdownTempIx = ix++;  addToConfig(shutdownTempIx, "st", "shutdownTemp", 40, 45, 60);
  boilerOffDelayIx = ix++;  addToConfig(boilerOffDelayIx, "od", "boileroffdelay", 0, 20, 30);
 // hwPumpHystIx = ix++;  addToConfig(hwPumpHystIx, "hy", "hwPumpHyst", 0, 2, 5);
  autoRestartHoursIx = ix++;  addToConfig(autoRestartHoursIx, "ar", "autoRestartHrs", 1, 24, 1000);
  selfTestIx = ix++;  addToConfig(selfTestIx, "tt", "selfTest", 0, 0, 3);
  btNameIx = ix++;  addToConfig(btNameIx, "bt", "btName", 0, 1, 9);
  dsErrWarnLevelIx = ix++;  addToConfig(dsErrWarnLevelIx, "de", "dsErrlevel", 0, 1, 5);
  log("config items = " + String(ix) + " / " + String(configSize));
}

void resetConfig()
{
  cmdlog("resetting config");
  for (int ix = 0; ix < configSize ; ix++)
  {
    configArr[ix].value = configArr[ix].defValue;
  }
  logConfig();
}

// state engine - state enumerations
int controlState = 0;
const int CONTROLSSTANDBY = 1;    //standby - !chDemand && !hwDemand && boiler cool - waiting for on - nothing active
const int CONTROLSSHUTDOWN = 2;   //shutdown - !chDemand && !hwDemand && boiler still hot - select either or both
const int CONTROLSANTICYCLE = 3;  //anticycle - chDemand || hwDemand && holding to anticycle
const int CONTROLSHEATING = 4;    //heating  - chDemand || hwDemand && under hightemp
const int CONTROLSCOOLING = 5;    //cooling  - chDemand || hwDemand  and cooling to hightemp-hysterisis anticycle delay

int prevControlState = 0;
String controlStateS(int state)
{
  switch (state)
  {
    case CONTROLSSTANDBY:   return "standby";
    case CONTROLSSHUTDOWN:  return "shutdwn";
    case CONTROLSANTICYCLE: return "anticyc";
    case CONTROLSHEATING:   return "heating";
    case CONTROLSCOOLING:   return "cooling";
    default: return "unknown";
  }
}

// globals
unsigned long elapsedMillis = 0;
const char C = ',';
unsigned long nextMillis;      // used to synch main loop to 1 sec
unsigned long sNow;                     // seconds since restart from Millis() - used for all timing
String prevLcdRow[] = {"","","",""};   // previously displays lcd data row by row
float boilertemps[maxThermostatSamples];  // keep last N temps for rate adjustment
float adjustedBoilerTemp;     // after rate adjustment
float boilerTempAdjustment;   // the adjustment made
float highestBoilerTemp;      // max temp since last log
float lowestBoilerTemp;       // min temp since last log
unsigned long boilerOnElapsed;          // boiler on to off time
unsigned long lastBoilerOnAt;           // when last turn on
unsigned long boilerOffElapsed;         // boiler off to on time
unsigned long lastBoilerOffAt;          // last turn off
unsigned long boilerOffDelayUntil = 0;  // when releasing boiler off - delays for a few secs while valves release
float dutyCycle = 0;          // on / (on + off)%
unsigned long nextBoilerOnAt;           // used specifically for anti cycle - earliest allowable on

byte lastInputs = 0;          // last PFC read
// analog input processing
unsigned long lastTadoChange = 0;       // next time we can change Tado demand state - limts change
int lastTadoLevel = 0;
int tadoHyst = 0;
const int tadoSampleSize = 10;            // size of averaging ring buffer
int tadoValueArr[tadoSampleSize];           // ring bufer for average
int tadoValueArrPtr = 0;                     // pointer into it
float tadoAverage = 0;        // average value used as 0-100
float boilerOffTemp = 0;      // boiler demand temp C - tirn off point
float boilerOnTemp = 0;       // boiler demand temp C - turn on point
int sendStatus = 0;           // cmd to send a status output as a status reply each loop - num times

unsigned long lastHwChange = 0;   // for next time allowed to change hw demand



// demand log variables
unsigned long nextDemandLogAt = 10;
float lastLogtadoAverage = 0;
bool logHwDemandAs = false;
bool logChDemandAs = false;
int demandLogInterval = 30;

// log files - referred to by index
const int NUMLOGFILES = 5;
const int CMDLOGFILE = 0;
const int MAINLOGFILE = 1;
const int CYCLELOGFILE = 2;
const int DEMANDLOGFILE = 3;
const int STATUSLOGFILE = 4;

String logFileHeaders[] = {
  "cmd",
  "log",
  "ontime,offtime,duty,offtemp,tmax,tmin,chdemand,hwdemand",
  "tado,offtemp,chdemand,hwdemand",
  "to-define"};

// test stuff
bool stActive = false;
int lastStConfig = 0;

int stControl = 0;     // tado analog
unsigned long stTimer;
float stTadoValue = 0;
float stTadoMax = 0;

bool stTempLowLimit = false;
int stHwControl = 0;  // HW
unsigned long stHwTimer;
bool stExtHwDemand = false;
float stTemp = 20;   // boiler temp
float stHwTemp = 55;   // hw temp
float stHwBtmTemp = 55;   // hw temp
float stHwTempTarget = 40;



float randomF(float v1, float v2)
{
  float f = random(v1 * 100, v2 * 100);
  return f / 100.0;
}

// self test 3 in manual control

void selfTest()
{
  bool initMe = false;
  if (lastStConfig != configArr[selfTestIx].value)
  {
    lastStConfig = configArr[selfTestIx].value;
    initMe = true;
  }
  switch (configArr[selfTestIx].value)
  {
    case 1:
      stActive = true;
      selfTest1(initMe);
      break;

    case 2:
      stActive = true;
      selfTest2(initMe);
      break;

    case 3:
      stActive = true;
      selfTest3(initMe);
      break;

    default:
      stActive = false;
      stControl = 0;
      stHwControl = 0;
      break;
  }
}

void handleSim()
{
  for (int ix = 1; ix < bPtr; ix++)
  {
    char c = buffer[ix];
    if (c == '1') stTemp++;
    if (c == '2') stTemp--;
    if (c == '3') stTadoValue++;
    if (c == '4') stTadoValue--;
    if (c == '5') stHwTemp++;
    if (c == '6') stHwTemp--;
    if (c == '7') stHwBtmTemp++;
    if (c == '8') stHwBtmTemp--;
  }
}

void selfTest3(bool initMe)
{
  if (initMe)
  {
    configArr[anticycleOnOnIx].value = 20;
    configArr[anticycleOffOnIx].value = 10;
    configArr[tadoIntervalIx].value = 10;
    configArr[hwIntervalIx].value = 10;
    configArr[rateAdjustIx].value = 1;
  }
}

void selfTest1(bool initMe)
{
  if (initMe)
  {
    demandLogInterval = 10;
    configArr[anticycleOnOnIx].value = 20;
    configArr[anticycleOffOnIx].value = 10;
    configArr[tadoIntervalIx].value = 10;
    configArr[hwIntervalIx].value = 10;
    configArr[rateAdjustIx].value = 1;
    stHwTemp = configArr[hwTempIx].value+2;
    stHwBtmTemp = configArr[hwBtmTempIx].value-5;
    stHwTempTarget = configArr[hwTempIx].value-configArr[hwHystIx].value-10;
  }
  int t;
  switch (stHwControl)
  {
    case 0:
      t = random(30, 60);
      stHwTemp = configArr[hwTempIx].value+2;
      log("ST-1 HW hold at " + String(stHwTemp) + " for " + String(t) + "s");
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
      stHwTempTarget = configArr[hwTempIx].value-configArr[hwHystIx].value-10;
      log("ST-1 HW drop slowly to  " + String(stHwTempTarget));
      stHwControl++;
      break;
    case 3:
      if (stHwTemp <= stHwTempTarget)
      {
        stHwControl++;
      }
      else
      {
        stHwTemp-= randomF(0.1,0.2);
      }
      break;
    case 4:
      stHwTempTarget = configArr[hwTempIx].value+2;
      log("ST-1 HW rise slowly to  " + String(stHwTempTarget));
      stHwControl++;
      break;
    case 5:
      if (stHwTemp >= stHwTempTarget)
      {
        stHwControl = 0;;
      }
      else
      {
        stHwTemp+= randomF(0.1,0.2);
      }
      break;
  }

  switch (stControl)
  {
    case 0:
      log("ST-1 Tado hold 0 30s");
      stTadoValue = 0;
      stTimer = sNow + 30;
      stControl++;
      break;
    case 1:
      if (sNow > stTimer)
      {
        stControl++;
        stTadoMax = randomF(40, 90);
        log("ST-1 Tado fast ramp up to " + String(stTadoMax));
      }
      break;
    case 2:
      stTadoValue += randomF(2, 5);
      if (stTadoValue >= stTadoMax)
      {
        stControl++;
        log("ST-1 Tado hold until cooling");
      }
      break;
    case 3:
      if (controlState == CONTROLSCOOLING)
      {
        stControl++;
        stTimer = sNow + 20;
        log("ST-1 Tado hold " + String(stTadoValue) + " 20s");
      }
      break;
    case 4:
      if (sNow > stTimer)
      {
        stControl++;
        log("ST-1 Tado slow ramp down to 0%");
      }
      break;
    case 5:
      stTadoValue -= randomF(0, 2);
      if (stTadoValue <= 0)
      {
        stTadoValue = 0;
        stControl++;
        log("ST-1 Tado hold until off");
      }
      break;
    case 6:
      if (hwDemand)
      {
        stControl = 1;
        stTimer = sNow + 20;
        log("ST-1 Tado holding to OFF abanndon on HW demand; hold 20s");
      }
      if (controlState == CONTROLSSTANDBY)
      {
        stControl = 1;
        stTimer = sNow + 20;
        log("ST-1 Tado holding to OFF complete; hold 20s");
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
    configArr[anticycleOnOnIx].value = 20;
    configArr[anticycleOffOnIx].value = 10;
    stHwTemp = configArr[hwTempIx].value+2;
    stHwBtmTemp = configArr[hwBtmTempIx].value-5;
    stControl = 0;
  }
  int t;

  switch (stControl)
  {
    case 0:
      log("ST-2 hold off 10s");
      stTadoValue = 0;
      stExtHwDemand = false;
      stTimer = sNow + 10;
      stControl++;
      break;
    case 1:
      if (sNow > stTimer)
      {
        stTadoValue = 90;
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
        stTadoValue = 0;
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
        stTadoValue = 0;
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
        stTadoValue = 0;
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
  String ss;
 
  ss = "f:" + String(ix) + C + s;    // commands - ignore time
 
  Serial.println(ss);
  Serial2.println(ss);
  if (SerialBT.hasClient())
  {
    SerialBT.println(ss);
  }
}

void statusLog()
{
  //ontemp,current,deltaadj,offtemp,ontime,offtime,duty,tado,intemp,ch,hw,state
  String dataline = String(boilerOnTemp) + C + String(boilertemps[dsBoilerOutIx],2) + C + String(boilerTempAdjustment,2) 
  + C + String(boilerOffTemp) + C + String(boilerOnElapsed) + C + String(boilerOffElapsed)
  + C + String(dutyCycle,1) + C + String(tadoAverage,2) + String(temperatures[dsBoilerInIx])
  + C + String(chDemand || setChValveOn) + C + String(hwDemand || setHwValveOn) + C + controlStateS(controlState);
  logIx(STATUSLOGFILE, dataline);
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
  if (abs(lastLogtadoAverage - tadoAverage) >= 2 && sNow >= nextDemandLogAt)
  {
    loggIt = true; 
  }
  if (loggIt)
  {
    float boilerOffTempT = boilerOffTemp;
    
    lastLogtadoAverage = tadoAverage;
    nextDemandLogAt = sNow + demandLogInterval;
    // "time,tado,offtemp,chdemand,hwdemand",
    String dataline = String(tadoAverage) + C + String(boilerOffTempT)
                    + C + String(logChDemandAs) + C + String(logHwDemandAs);
    logIx(DEMANDLOGFILE, dataline);
  }
}

void logConfig()
{
  int maxLen = 0;
  for (int ix = 0; ix < configSize; ix++)
  {
    if (configArr[ix].name.length() > maxLen)
    {
      maxLen = configArr[ix].name.length();
    }
  }
  for (int ix = 0; ix < configSize; ix++)
  {
    if (configArr[ix].code != "")
    {
    int padd = maxLen - configArr[ix].name.length();
    String s = "";
    for (int ip = 0; ip < padd; ip++)
    {
      s += ".";
    }
    cmdlog(configArr[ix].code + "-" + configArr[ix].name + s + " = " + String(configArr[ix].value) 
    + "  (" + String(configArr[ix].minValue)+ ">" + String(configArr[ix].defValue) + ">" + String(configArr[ix].maxValue) + ")");
    }
  }
}

void saveToEprom()
{
  int im = 0;
  for (int ix = 0; ix < configSize; ix++)
  {
    EEPROM.write(im++, highByte(configArr[ix].value));
    EEPROM.write(im++, lowByte(configArr[ix].value));
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
    configArr[ix].value =  word( EEPROM.read(im++), EEPROM.read(im++));
    if (configArr[ix].value < configArr[ix].minValue || configArr[ix].value > configArr[ix].maxValue)
    {
      configArr[ix].value = configArr[ix].defValue;
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
  for (int it = 1; it <= dsRetries; it++)
  {
    sensors.setUserDataByIndex(ix, ud);
    int cud = sensors.getUserDataByIndex(ix);
    if (cud == ud)
    {
      cmdlog("ix=" + String(ix) + " ud change " + String(cud) + " > " + String(cud)); 
      break;
    }
    else
    {
      cmdlog("!! ix=" + String(ix) + " set ud fail: " + String(ud) + " != " + String(cud) + " x" + String(ix)); 
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
    for (int it = 1; it <= dsRetries; it++)
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
      dsErrorCount[it-1]++;
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
// it takes 30-40 ms to do a get temp - so we always do 0 as its the boiler and then the 
// rest on a round robin each loop
void getTemperatures()
{
  nextSensorIx++;
  if (nextSensorIx < 1 || nextSensorIx > SENSORMAPCOUNT)
  {
    nextSensorIx = 1;
  }
  for (int ix = 0; ix < 2; ix++)
  {
    if (ix == 1)
    {
      ix = nextSensorIx;
    }
    if (sensorMapping[ix] >= 0 && sensorMapping[ix] < SENSORMAPCOUNT)
    {
      for (int it = 1; it <= dsRetries; it++)
      {
        temperatures[ix] = sensors.getTempCByIndex(sensorMapping[ix]);
        if (temperatures[ix] != -127.0)
        {
          break;
        }
        else
        {
          if (it >= configArr[dsErrWarnLevelIx].value)    // maybe count as well
          {
            log("!! ds read error ix=" + String(ix) + " x" + String(it));
            dsErrorCount[it-1]++;
          }
          //delay(10);
        }
      }
    }
  }
  if (stActive)
  {
    temperatures[dsBoilerOutIx] = stTemp;
    temperatures[dsMidTankIx] = stHwTemp;
    temperatures[dsBottomTankIx] = stHwBtmTemp;
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
    //case CONTROLSSHUTDOWN:
    //  s += formatInt(configArr[shutdownTempIx].value, 2) + " s ";
    //  break;
    case CONTROLSSHUTDOWN:
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

  s = formatFloat(tadoAverage, 2, 1) + "  " + formatFloat(temperatures[dsBoilerInIx], 2, 1) 
  + " " + formatFloat(temperatures[dsMidTankIx], 2, 1) + " " + formatFloat(temperatures[dsBottomTankIx], 2, 1);
  updateDisplayLine(2,s);

  s = "";
  if (chDemand)
  {
    s += "CH ";
  }
  else if (setChValveOn)
  {
    s += "ch ";
  }
  else
  {
    s += ".. ";
  }

  if (hwDemand)
  {
    s += "HW ";
  }
  else if (setHwValveOn)
  {
    s += "hw ";
  }
  else
  {
    s += ".. ";
  }
  if (setBoilerOff)
  {
    s += "bo ";
  }
  else
  {
    s += ".. ";
  }

  s += ".. ";

  s += controlStateS(controlState);
  updateDisplayLine(3,s);
}

// reads the Tado boiler analog singal & sets CH demand ch dump 
// contains steps to limit change frequency and a provide hysteresis
void readTadoAnalog()
{
  // take average of last sample values as raw analogue (0-4095)
  tadoValueArrPtr++;
  if (tadoValueArrPtr >= tadoSampleSize)
  {
    tadoValueArrPtr = 0;
  }
  tadoValueArr[tadoValueArrPtr] = analogRead(analogPin); // take current value
  if (stActive)  // override if self test
  {
    tadoValueArr[tadoValueArrPtr] = stTadoValue * 4096 / 100;
  }
  // compute average
  long annaTotal = 0;
  for (int ix = 0 ; ix < tadoSampleSize; ix++)
  {
    annaTotal += tadoValueArr[ix];
  }

  tadoAverage = annaTotal / (tadoSampleSize) * 100.0 / 4096.0;

  // work from here with tadoAverage...
  if (!chEnabled)
  {
    // CH is off - just driven by HW demand
    tadoLevel = 1;
    tadoHyst = 0;
    lastTadoChange = 0;
  }
  else if (sNow > (lastTadoChange + configArr[tadoIntervalIx].value));
  {
    // only worth checking if beyond change interval moratorium
    // we have 3 state levels:
    // state 1 - below off - no demand - no dump
    // state 2 - above off - below ch - no demand - can dump
    // state 3 = above CH - ch demand. 
    // hyst will be 0 or -ve
    int newTadoLevel;
    if (tadoAverage < (configArr[tadoOffIx].value + tadoHyst))
    {
      newTadoLevel = 1;
    }
    else if (tadoAverage >= (configArr[tadoChIx].value + tadoHyst))
    {
      newTadoLevel = 3;
    }
    else
    {
      newTadoLevel = 2;
    }
   
    if (newTadoLevel != tadoLevel)
    {
      // has changed - moratorium for further change..
      lastTadoChange = sNow;
      if (newTadoLevel > tadoLevel)
      {
        // rising
        tadoHyst = - configArr[tadoHystIx].value;
      } 
      else if (newTadoLevel < tadoLevel)
      {
        // falling
        tadoHyst = 0;
      }
      tadoLevel = newTadoLevel;
      switch (tadoLevel)
      {
        case 1:
          log("Tado > OFF");
          break;
        case 2:
          log("Tado > DUMP");
          break;
        case 3:
          log("Tado > CHDEMAND");
          break;
        default:
          log("Tado > ?????");
          break;
      }
    }
  }
  switch (tadoLevel)
  {
    case 2:
      chDumpOk = true;
      chDemand = false;
      break;
    case 3:
      chDumpOk = false;
      chDemand = true;
      break;
    default:
      chDumpOk = false;
      chDemand = false;
      break;
  }
  // all done.
}

void readHWTemp()
{
  // have 2 stats on tank - mid and bottom. 
  // Mid used to set HW demand and boiler tenmperature
  // Bottom used to turn HW on in parallel with heating 

  
  if (sNow > (lastHwChange + configArr[hwIntervalIx].value));
  {
    if (temperatures[dsMidTankIx] > float(configArr[hwTempIx].value))
    {
      if (hwDemand)
      {
        hwDemand = false;
        log("HW > OFF");
        lastHwChange =  sNow;
      }
    }
    else if (temperatures[dsMidTankIx] < float(configArr[hwTempIx].value - configArr[hwHystIx].value))
    {
      if (!hwDemand)
      {
        hwDemand = true;
        log("HW > ON");
        lastHwChange =  sNow;
      }
    }
    if (temperatures[dsBottomTankIx] > float(configArr[hwBtmTempIx].value))
    {
      if (hwPiggyback)
      {
        hwPiggyback = false;
        log("HWPiggyback > OFF");
        lastHwChange =  sNow;
      }
    }
    else if (temperatures[dsBottomTankIx] < float(configArr[hwBtmTempIx].value - configArr[hwHystIx].value))
    {
      if (!hwPiggyback)
      {
        hwPiggyback = true;
        lastHwChange =  sNow;
        log("HWPiggyback > ON");
      }
    }
  }
}

void calculate()
{
  bool logged = false;

  // shift history down temp array
  int samples = configArr[thermostatSamplesIx].value;
  for (int ix = samples-1; ix >= 1; ix--)
  {
    boilertemps[ix] = boilertemps[ix - 1];
  }
  boilertemps[0] = temperatures[0];

  // rate is between avg first 2 and avg last 2 - therefore over (samples-2) seconds
  float rate = (boilertemps[0] + boilertemps[1] - boilertemps[samples-2] - boilertemps[samples-1]) / float((samples-2) * 2);
  boilerTempAdjustment = rate * float(configArr[rateAdjustIx].value);
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

  if (!chDemand && !hwDemand)
  {
    // no demand from HW or CH
    // work out shutdown temps..
    boilerOnTemp = configArr[shutdownTempIx].value;     // end shutdown on fall
    boilerOffTemp = boilerOnTemp + configArr[boilerHystIx].value;    // is shutdown restart if temp rises
    
    if ((prevChDemand || prevHwDemand) && !setBoilerOff)
    {
      // demand dropped with boiler lit - record cycle.
      boilerOnElapsed =   sNow - lastBoilerOnAt;
      nextBoilerOnAt = max(nextBoilerOnAt, sNow + configArr[anticycleOffOnIx].value);
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

    // use boiler hyst to limit switching
    if (adjustedBoilerTemp <= boilerOnTemp)
    {
      // reached end of shutdown 
      controlState = CONTROLSSTANDBY;
      setChValveOn = false;
      setHwValveOn = false;
      setBoilerOff = false;
      boilerOffDelayUntil = sNow + 5;    // makes sure we hold boiler off till valves release switch
    }
    else if ((setChValveOn || setHwValveOn) && setBoilerOff)
    {
      // already in shutdown
      controlState = CONTROLSSHUTDOWN;
      setChValveOn = chDumpOk;         // might change during shotdown
      setHwValveOn = !setChValveOn;
    }
    else if (adjustedBoilerTemp >= boilerOffTemp)
    {
      // can enter new shutdown cycle
      // dump heat cycle - either to CH if Tado > off or to CH
      controlState = CONTROLSSHUTDOWN;
      setChValveOn = chDumpOk;
      setHwValveOn = !setChValveOn;
      setBoilerOff = true;
    }
  }
  else
  {
    // there is demand 
    setChValveOn = false;
    setHwValveOn = false;

    // determine demand boiler temp
    if (hwDemand & !chDemand)
    {
      // HW demand only - set to HW heating temperature otherwise CH derived temp prevails.
      boilerOffTemp = configArr[boilerHwIx].value;
    }
    else
    { 
      boilerOffTemp = (tadoAverage - configArr[tadoChIx].value) / (configArr[tadoHighIx].value -  configArr[tadoChIx].value) *
               (configArr[boilerHighIx].value - configArr[boilerLowIx].value) +  configArr[boilerLowIx].value;
      boilerOffTemp = constrain(boilerOffTemp, configArr[boilerLowIx].value, configArr[boilerHighIx].value);
    }
    boilerOnTemp = boilerOffTemp - configArr[boilerHystIx].value;

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
          nextBoilerOnAt =  max(nextBoilerOnAt, sNow + configArr[anticycleOnOnIx].value);
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
        nextBoilerOnAt = max(nextBoilerOnAt, sNow + configArr[anticycleOffOnIx].value);
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

    // can piggy back HW?
    if (chDemand && hwPiggyback)
    {
      setHwValveOn = true;
    }
   
    // click hw pump on at onTemp - off at same - hysteresis
  //  if (!hwDemand)
  //  {
  //    setHwPumpOn = false;
  //  }
  //  else if (!setHwPumpOn)
  //  {
  //    if (adjustedBoilerTemp >= boilerOnTemp)
  //    {
  //      setHwPumpOn = true;
  //    }
  //  }
  //  else
  //  {
     // if (adjustedBoilerTemp < boilerOnTemp - configArr[hwPumpHystIx].value)
     // {
     //   setHwPumpOn = false;
     // }
  //  }
    //setHwPumpOn = false;    // disable for now
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

/*
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
*/

// assumes we have last state in lastInputs
// only change one at a time

void setOutputs()
{
  byte existB = PCF_01.read8();
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
    log("CHVALVE > " + onOffS(chv));
    return newB;
  }
  bitWrite(newB, HWVALVEBIT, !setHwValveOn);
  if (newB != existB)
  {
    log("HWVALVE > " + onOffS(setHwValveOn));
    return newB;
  }
  //bitWrite(newB, HWPUMPBIT,  !setHwPumpOn);
  //if (newB != existB)
  //{
  //  log("HWPUMP > " + onOffS(setHwPumpOn));
  //  return newB;
  //}
  if (sNow < boilerOffDelayUntil && !setBoilerOff)
  {
    return newB;      // don't release yet! 
  }
  bitWrite(newB, BOILEROFFBIT, !setBoilerOff);
  if (newB != existB)
  {
    log("BOILEROFF > " + onOffS(setBoilerOff));
    return newB;
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

/*
int lastClockFreq = -1;
void setClockFreq()
{
  int freq = 80;
  switch (configArr[cpuFreqIx].value)
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
*/

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
  cmdlog("d - diagnostics");
  cmdlog("t - sendstatus");
  cmdlog("q - simulation secial");
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

    case 'q':
      handleSim();
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
      if (chEnabled)
      {
        chEnabled = false;
        log("ch disabled");
      }
      else
      {
        chEnabled = true;
        log("ch enabled");
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

    case 'd':   //!! redo better
      {
        String ss = "DS errors: ";
        for (int ix = 0; ix < 4; ix++)
        {
          ss+= String(dsErrorCount[ix]) + "; ";
        }
        cmdlog(ss);
        break;
      }

    case 't':
      sendStatus++;;
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
    if (s == configArr[ix].code)
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
    cmdlog(configArr[ip].code + "=" + String(configArr[ip].value));
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
  if (value > configArr[ip].maxValue)
  {
    cmdlog("!! > " + String(configArr[ip].maxValue));
    return;
  }
  if (value < configArr[ip].minValue)
  {
    cmdlog("!! < " + String(configArr[ip].minValue));
    return;
  }
  int prev = configArr[ip].value;
  configArr[ip].value = value;
  log(configArr[ip].code + ":" + String(prev) + " > " + String(configArr[ip].value));
}

void setup()
{
  // basics
  Serial.begin(115200);
  Serial2.begin(115200);

  // PFC control interface
  PCF_01.begin(255);  // set all bits on
  
  InitConfig();

  // init from eprom..
  EEPROM.begin(configSize * 2);
  loadFromEprom();

   // bluetooth
  if (configArr[btNameIx].value > 0)
  {
    String actBtName = btName + "-" + String(configArr[btNameIx].value);
    log("enable bluetooth - " + actBtName);
    SerialBT.begin(actBtName);
  }
  else
  {
    log("bluetooth disabled");  
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
  // try n times max - should get required devices
  for (int it=1; it <= dsRetries; it++)
  {
    sensors.begin();
    log("Dallas sensors: " + String(sensors.getDeviceCount()));
    if (sensors.getDeviceCount() == SENSORMAPCOUNT)
    {
      break;
    }
    log("!! device count <> " + String(SENSORMAPCOUNT) + " x=" + String(it));
    dsErrorCount[it-1]++;
    delay(10);
  }
  // map userdata to sensor map
  //sensors.setResolution(configArr[dsResolutionIx].value);
  mapSensors();
  lcd.setCursor(0, 2);
  lcd.print("sensors = ");
  lcd.print(sensors.getDeviceCount());
  sensors.setWaitForConversion(false); // we request and ask a second later. takes some 700ms for 12 bit
  sensors.requestTemperatures(); 
  delay(1000);                         //  wait while get first readings..
  getTemperatures();
  // initialize
  lastBoilerOnAt = sNow;
  lastBoilerOffAt = sNow;
  highestBoilerTemp = 0;
  lowestBoilerTemp = 100;
  for (int ix = 0; ix < configArr[thermostatSamplesIx].value; ix++)
  {
    boilertemps[ix] = temperatures[0];
  }
  delay(1000);
  sensors.requestTemperatures();
  
  delay(1000);
}



void loop()
{
  // get time..
  sNow = millis() / 1000;

  //input from serial or BT treated equally
  while (Serial.available() || Serial2.available() || (SerialBT.hasClient() && SerialBT.available()))
  {
    char c;
    if (Serial.available())
    {
      c = Serial.read();
    }
    else if (Serial2.available())
    {
      c = Serial2.read();
    }
    else
    {
      c= SerialBT.read();
    }

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
    digitalWrite(ledPin, 1);
    
    unsigned int startMillis = millis();
    nextMillis += 1000;
    
    //!!setClockFreq();
    if (controlState == CONTROLSSTANDBY && sNow > configArr[autoRestartHoursIx].value * 3600)
    {
      log("scheduled restart..");

      ESP.restart();
    }
    // 1 sec processing
    getTemperatures();
    selfTest();
    //!readInputs();
    readTadoAnalog();
    readHWTemp(); 
    calculate();
    setOutputs();
    demandLog();
    updateDisplay();
   
    // previous state for next pass
    prevControlState = controlState;
    prevChDemand = chDemand;
    prevHwDemand = hwDemand;
    sensors.requestTemperatures(); // pick up next pass

    esp_task_wdt_reset();    // tickle watchdog

    elapsedMillis = millis() - startMillis;
    if (sendStatus > 0)
    {
      statusLog();
      sendStatus--;;
    }
    digitalWrite(ledPin, 0);
  }
  
  

  delay(5);
}
/*
  version history

2021-05-18  start version 2.
  - simplify - drop RTC and SD logging - keep rest much the same
  - logging delivered by serial port to capture via logger device

2021-10-07  use bluetooth / serial for control

2022-02-19  add status log , also use serial2, remove time from log lines
  
*/
