/**
*  @file    NutrientEOL.ino
*  @author  peter c
*  @date    10/25/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  NEOL Arduino Nano
** @section HISTORY
** 2017Oct25 - created
*/
#include <HardwareSerial.h>
#include <TimeAlarms.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> // for RTC
#include <Streaming.h>

#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>

#include "PlantModbus.h"




// comment out to not include terminal processing
//#define PROCESS_TERMINAL
//#define TRACE_1WIRE
//#define TRACE_ANALOGS
//#define TRACE_DISCRETES
//#define TRACE_MODBUS


// comment out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate

#define ALARM_REFRESH_INTERVAL 100 // ms
// One Wire - Hydroponic temperatures
// 
#define TEMPERATURE1 8 // pin
#define TEMPERATURE2 7 // pin
#define TEMPERATURE3 14 // pin
#define TEMPERATURE4 4 // pin

#define ONE_TEMPERATURE_PRECISION 9

OneWire oneWireBus1(TEMPERATURE1);
OneWire oneWireBus2(TEMPERATURE2);
OneWire oneWireBus3(TEMPERATURE3);
OneWire oneWireBus4(TEMPERATURE4);
DallasTemperature T1_TT004(& oneWireBus1);
DallasTemperature N1_TT004(& oneWireBus2);
DallasTemperature N2_TT004(& oneWireBus3);
DallasTemperature N3_TT004(& oneWireBus4);


DA_DiscreteOutput T1_XY004 = DA_DiscreteOutput(3, LOW); // V1
DA_DiscreteOutput N1_XY004 = DA_DiscreteOutput(11, LOW); // V2
DA_DiscreteOutput N2_XY004 = DA_DiscreteOutput(10, LOW); // V3
DA_DiscreteOutput N3_XY004 = DA_DiscreteOutput(9, LOW); // V4

DA_AnalogInput T1_PT004 = DA_AnalogInput(A1, 0.0, 5.); // min max
DA_AnalogInput N1_PT004 = DA_AnalogInput(A2, 0.0, 5.); // min max
DA_AnalogInput N2_PT004 = DA_AnalogInput(A6, 0.0, 5.); // min max
DA_AnalogInput N3_PT004 = DA_AnalogInput(A7, 0.0, 5.); // min max

#ifdef PROCESS_TERMINAL
DA_DiscreteOutput LED = DA_DiscreteOutput( 13, HIGH);  // for debugging
#endif


// HEARTBEAT

unsigned long heartBeat = 0;

struct _AlarmEntry
{
  time_t epoch;
  AlarmId id = dtINVALID_ALARM_ID;
  bool firstTime = true;
};

typedef _AlarmEntry AlarmEntry;
AlarmEntry onRefreshAnalogs; // sonar and 1-wire read refresh
AlarmEntry onFlowCalc; // flow calculations
#ifdef PROCESS_TERMINAL
HardwareSerial *tracePort = & Serial2;
#endif


void onEdgeDetect(bool state, int pin)
{
  #ifdef PROCESS_TERMINAL
  *tracePort << "Edge Detection:" << state << " on pin " << pin << endl;
  #endif
}

void printOneWireAddress(HardwareSerial *tracePort, DeviceAddress aDeviceAddress, bool aCR)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (aDeviceAddress[i] < 16)
      *tracePort << '0';
    tracePort -> print(aDeviceAddress[i], HEX);
  }
  if (aCR)
    *tracePort << endl;
}

void init1WireTemperatureSensor(DallasTemperature * sensor, int idx)
{
  DeviceAddress address;
  sensor -> begin();
  if (sensor -> getAddress(address, 0))
  {

#ifdef TRACE_1WIRE
    *tracePort << "Channel " << idx << " 1Wire Temperature initialized. Address =  ";
    printOneWireAddress(tracePort, address, true);
#endif

    sensor -> setResolution(address, ONE_TEMPERATURE_PRECISION);
  }
  else
  {

#ifdef TRACE_1WIRE
    *tracePort << "Unable to find address for 1Wire Temperature Device @ " << idx << endl;
#endif

  }
}

void initHydroponicOneWireTemps()
{
  init1WireTemperatureSensor(& T1_TT004, 1);
  init1WireTemperatureSensor(& N1_TT004, 2);
  init1WireTemperatureSensor(& N2_TT004, 3);
  init1WireTemperatureSensor(& N3_TT004, 4);
}

int polling; // 1=polling analogs, 2=polling digitals, -1 nothing
int currentTogglePin; // -1 none, >0 pin
void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort -> begin(9600);

#endif

#ifdef PROCESS_MODBUS
  slave.begin(19200);
#endif

  randomSeed(analogRead(3));
  onRefreshAnalogs.id = Alarm.timerRepeat(POLL_CYCLE_SECONDS, doOnPoll);
  initHydroponicOneWireTemps();
}

void loop()
{

#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif

  Alarm.delay(ALARM_REFRESH_INTERVAL);
}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{
  doReadAnalogs();
  doProcess1WireTemperatures();
  heartBeat++;


}



void doPoll1WireTemperature( DallasTemperature *sensor, int idx ) 
{
  sensor->requestTemperatures();
  #ifdef TRACE_1WIRE
    *tracePort << "Temperature " << idx << " = " << sensor->getTempCByIndex(0) << " C" << endl;
  #endif
}


void doProcess1WireTemperatures()
{

  doPoll1WireTemperature( &T1_TT004, 0);
  doPoll1WireTemperature( &N1_TT004, 1);
  doPoll1WireTemperature( &N2_TT004, 2);
  doPoll1WireTemperature( &N3_TT004, 3); 
}

void doReadAnalogs()
{
  T1_PT004.refresh();
  N1_PT004.refresh();
  N2_PT004.refresh();


  #ifdef TRACE_ANALOGS
      T1_PT004.serialize( tracePort, true);
      N1_PT004.serialize( tracePort, true);
      N2_PT004.serialize( tracePort, true);
      N3_PT004.serialize( tracePort, true);
  #endif

}

void doUpdateOutputs()
{
}

// 
/*
** Modbus related functions
*/

#ifdef PROCESS_MODBUS
void refreshModbusRegisters()
{
  /*
  #define HR_TEMPERATURE1 HOLDING_REGISTER_READ_OFFSET
  #define HR_TEMPERATURE2 HR_TEMPERATURE1 + 1
  #define HR_TEMPERATURE3 HR_TEMPERATURE2 + 1
  #define HR_TEMPERATURE4 HR_TEMPERATURE3 + 1
  #define HR_PRESSURE1 HR_TEMPERATURE4 + 1
  #define HR_PRESSURE2 HR_PRESSURE1 + 1
  #define HR_PRESSURE3 HR_PRESSURE2 + 1
  #define HR_PRESSURE4 HR_PRESSURE3 + 1
*/
  blconvert.val = T1_TT004.getTempCByIndex(0) * 100;
  modbusRegisters[HR_TEMPERATURE1] = blconvert.regsl[0];
  modbusRegisters[HR_TEMPERATURE1 + 1] = blconvert.regsl[1];

  blconvert.val = N1_TT004.getTempCByIndex(0) * 100;
  modbusRegisters[HR_TEMPERATURE2] = blconvert.regsl[0];
  modbusRegisters[HR_TEMPERATURE2 + 1] = blconvert.regsl[1];

  blconvert.val = N2_TT004.getTempCByIndex(0) * 100;
  modbusRegisters[HR_TEMPERATURE3] = blconvert.regsl[0];
  modbusRegisters[HR_TEMPERATURE3 + 1] = blconvert.regsl[1];

  blconvert.val = N3_TT004.getTempCByIndex(0) * 100;
  modbusRegisters[HR_TEMPERATURE4] = blconvert.regsl[0];
  modbusRegisters[HR_TEMPERATURE4 + 1] = blconvert.regsl[1];

  blconvert.val = T1_PT004.getScaledSample() * 100;
  modbusRegisters[HR_PRESSURE1] = blconvert.regsl[0];
  modbusRegisters[HR_PRESSURE1 + 1] = blconvert.regsl[1];

  blconvert.val = N1_PT004.getScaledSample() * 100;
  modbusRegisters[HR_PRESSURE2] = blconvert.regsl[0];
  modbusRegisters[HR_PRESSURE2 + 1] = blconvert.regsl[1];

  blconvert.val = N2_PT004.getScaledSample() * 100;
  modbusRegisters[HR_PRESSURE3] = blconvert.regsl[0];
  modbusRegisters[HR_PRESSURE3 + 1] = blconvert.regsl[1];

  blconvert.val = heartBeat;
  modbusRegisters[HR_PRESSURE4] = blconvert.regsl[0];
  modbusRegisters[HR_PRESSURE4 + 1] = blconvert.regsl[1];

  

  // modbusRegisters[HR_LED_DUTY_CYCLE] = plantStrip.getDutyCycle();
  // modbusRegisters[HR_LED_DUTY_CYCLE_PERIOD] = dutyCycleChangeAlarm.epoch;
  /*
  blconvert.val = lightsOn.epoch;
  modbusRegisters[HR_LED_ON_TIME] = blconvert.regsl[0];
  modbusRegisters[HR_LED_ON_TIME + 1] = blconvert.regsl[1];
  blconvert.val = lightsOff.epoch;
  modbusRegisters[HR_LED_OFF_TIME] = blconvert.regsl[0];
  modbusRegisters[HR_LED_OFF_TIME + 1] = blconvert.regsl[1];
  blconvert.val = now();
  modbusRegisters[HR_CURRENT_TIME] = blconvert.regsl[0];
  modbusRegisters[HR_CURRENT_TIME + 1] = blconvert.regsl[1];
  */
  // modbusRegisters[ CS_LED_STATUS ] = plantStrip.isLightsOn();
  // modbusRegisters[ CS_LED_STATUS ] = plantStrip.isLightsOn();
  // writeModbusCoil( COIL_STATUS_READ_WRITE_OFFSET, CS_LED_STATUS, plantStrip.isLightsOn());
  // modbusRegisters[HR_AMBIENT_TEMPERATURE] = (int) TE_001.getScaledSample() * 10;
  // modbusRegisters[HR_SOIL_MOISTURE] = (int) QE_001.getScaledSample() * 10;
  // bitWrite(modbusRegisters[ COIL_STATUS_READ_OFFSET], CS_SET_LED_ON, plantStrip.isLightsOn());
}


bool getModbusCoilValue( unsigned short startAddress, unsigned short bitPos)
{

  //*tracePort << "reading at " << startAddress << " bit offset " << bitPos << "value=" << bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 ) << endl;

  return ( bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 ) );
}

void writeModbusCoil( unsigned short startAddress, unsigned short bitPos, bool value )
{
  bitWrite(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 , value) ;
}

void checkAndActivateDO( unsigned int bitOffset, DA_DiscreteOutput *aDO )
{
  // look for a change from 0 to 1
   if(  getModbusCoilValue( COIL_STATUS_READ_WRITE_OFFSET, bitOffset )  )
   {
          aDO->activate();
    #ifdef TRACE_MODBUS
      *tracePort << "Activate DO:" ;
       aDO->serialize( tracePort, true);
            LED.activate();
    #endif

 
      writeModbusCoil( COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
   }
}

void checkAndResetDO( unsigned int bitOffset, DA_DiscreteOutput *aDO )
{ 
    // look for a change from 0 to 1
   if(  getModbusCoilValue( COIL_STATUS_READ_WRITE_OFFSET, bitOffset )  )
   {
          aDO->reset();
        #ifdef TRACE_MODBUS
      *tracePort << "Reset DO:" ;
      aDO->serialize( tracePort, true);
      LED.reset();
    #endif

 
      writeModbusCoil( COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
   }
}




void processValveCommands()
{
  checkAndActivateDO( VALVE1_OPEN, &T1_XY004 );
  checkAndResetDO( VALVE1_CLOSE, &T1_XY004 );  

  checkAndActivateDO( VALVE2_OPEN, &N1_XY004 );
  checkAndResetDO( VALVE2_CLOSE, &N1_XY004 ); 

  checkAndActivateDO( VALVE3_OPEN, &N2_XY004 );
  checkAndResetDO( VALVE3_CLOSE, &N2_XY004 ); 
  
  checkAndActivateDO( VALVE4_OPEN, &N3_XY004 );
  checkAndResetDO( VALVE4_CLOSE, &N3_XY004 ); 
 
}

void processModbusCommands()
{
  processValveCommands();  
}

#endif

