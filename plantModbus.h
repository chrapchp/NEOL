/**
 * @file 	plantModbus.h
 * @version     0.1
 * @date        2017May3
 * @author 	pjc

 *
 * @description
 *  Helpers for plant lighting and control using Modbus
 *
 * Using arduino modbus implementation @
 * https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
*/


#include <SoftwareSerial.h>
#include <ModbusRtu.h>

 
#define COIL_STATUS_READ_WRITE_OFFSET 0
#define COIL_STATUS_WRITE_START_BIT	(16*5)  // 80 coils/bits for reads then followed by write memory for coils

#define HOLDING_REGISTER_READ_OFFSET 10		// start read holding regisers
#define HOLDING_REGISTER_WRITE_OFFSET 30


#define HR_TEMPERATURE1    	HOLDING_REGISTER_READ_OFFSET
#define HR_TEMPERATURE2  	HR_TEMPERATURE1 + 2
#define HR_TEMPERATURE3    	HR_TEMPERATURE2 + 2
#define HR_TEMPERATURE4  	HR_TEMPERATURE3 + 2
#define HR_PRESSURE1		HR_TEMPERATURE4 + 2
#define HR_PRESSURE2		HR_PRESSURE1    + 2
#define HR_PRESSURE3		HR_PRESSURE2    + 2
#define HR_PRESSURE4		HR_PRESSURE3    + 2
#define HR_HEARTBEAT		HR_PRESSURE4    + 2

//  HOLDING_REGISTER_WRITE_OFFSET + LAST #DEFINE IN THE LIST ON TOP.
//  IF YOU ADD MORE ENSURE THE CHANGE IS MADE HERE 
#define MODBUS_REG_COUNT HOLDING_REGISTER_WRITE_OFFSET + HR_HEARTBEAT + 1
// coil write commands
#define VALVE1_OPEN			COIL_STATUS_WRITE_START_BIT //bit position
#define VALVE1_CLOSE		VALVE1_OPEN 	+ 1		
#define VALVE2_OPEN			VALVE1_CLOSE 	+ 1  
#define VALVE2_CLOSE		VALVE2_OPEN 	+ 1  
#define VALVE3_OPEN			VALVE2_CLOSE 	+ 1  
#define VALVE3_CLOSE		VALVE3_OPEN     + 1
#define VALVE4_OPEN			VALVE3_CLOSE 	+ 1  
#define VALVE4_CLOSE		VALVE4_OPEN     + 1 




//#define CS_RESET_TO_DEFAULTS	 CS_SET_LED_OFF + 1

// coil statuses
//#define CS_LED_STATUS			COIL_STATUS_READ_WRITE_OFFSET
/*

#define HR_AMBIENT_TEMPERATURE HOLDING_REGISTER_READ_OFFSET
#define HR_SOIL_MOISTURE	   HR_AMBIENT_TEMPERATURE + 1
#define HR_LED_DUTY_CYCLE	   HR_SOIL_MOISTURE + 1
#define HR_LED_DUTY_CYCLE_PERIOD HR_LED_DUTY_CYCLE + 1
#define HR_CURRENT_TIME			HR_LED_DUTY_CYCLE_PERIOD + 1   // UTC
#define HR_LED_ON_TIME			HR_CURRENT_TIME + 2
#define HR_LED_OFF_TIME			HR_LED_ON_TIME + 2

#define HR_SET_TIME				HOLDING_REGISTER_WRITE_OFFSET  // UTC
#define HR_SET_LED_ON_TIME		HR_SET_TIME + 2
#define HR_SET_LED_OFF_TIME		HR_SET_LED_ON_TIME + 2
#define HR_SET_DUTY_CYCLE		HR_SET_LED_OFF_TIME + 2
#define HR_SET_DUTY_CYCLE_PERIOD HR_SET_DUTY_CYCLE + 1

#define MODBUS_REG_COUNT HOLDING_REGISTER_WRITE_OFFSET + HR_SET_DUTY_CYCLE_PERIOD + 1

// coil write commands
#define CS_SET_LED_ON			 COIL_STATUS_WRITE_START_BIT //bit position
#define CS_SET_LED_OFF			 CS_SET_LED_ON + 1
#define CS_RESET_TO_DEFAULTS	 CS_SET_LED_OFF + 1

// coil statuses
#define CS_LED_STATUS			COIL_STATUS_READ_WRITE_OFFSET

*/
#define MB_SLAVE_ID				1
#define MB_SERIAL_PORT			0



union
{
  int regsf[2];
  float val;
} 
bfconvert;


union
{
  int regsl[2];
  long val;
} 
blconvert;

uint16_t modbusRegisters[MODBUS_REG_COUNT];

Modbus slave(MB_SLAVE_ID, MB_SERIAL_PORT); 


