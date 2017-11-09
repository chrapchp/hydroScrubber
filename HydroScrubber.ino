

/**
*  @file    hydroScrubber.ino
*  @author  peter c
*  @date    11/08/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
* Based on HVAC board layout. Makeup air, exhaust air, and scrubber share similar IO
* Scruber has a CO2 sensor and 3 FP, the others don't have a CO2 sensor and have just 2 DPs
*
** @section HISTORY
** 2017Nov8 - created
*/
#include <HardwareSerial.h>

#include <Streaming.h>
#include <Adafruit_Sensor.h>
#include <DHT.h> // DHT-22 humidity sensor
#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>
#include <flowmeter.h>
#include <DA_NonBlockingDelay.h>


#include "UnitModbus.h"
// comment out to  include terminal processing for debugging
 //#define PROCESS_TERMINAL
 //#define TRACE_1WIRE
//#define TRACE_ANALOGS
 //define TRACE_FLOW_SENSOR
// #define TRACE_DISCRETES
//#define TRACE_MODBUS_COILS
//#define TRACE_MODBUS_HR
// comment//out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate



#define CO2_INTERRUPT_PIN 3 
#define ENABLE_CO2_SENSOR_RISING_INTERRUPTS attachInterrupt(digitalPinToInterrupt(CO2_INTERRUPT_PIN), on_B1R1_1A_AT_001_Rising, RISING)
#define ENABLE_CO2_SENSOR_FALLING_INTERRUPTS attachInterrupt(digitalPinToInterrupt(CO2_INTERRUPT_PIN), on_B1R1_1A_AT_001_Falling, FALLING)
#define DISABLE_CO2_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt(CO2_INTERRUPT_PIN))


volatile unsigned long timeOnStart = 0 ;
volatile unsigned long timeOffStart = 0;
volatile unsigned long timeOn = 0 ;
volatile unsigned long timeOff = 0;


// 
// 
// /#define FAN_SPEED_INTERUPT   2 // pin does not work return 0 for now to host

#define HUMIDITY_INTAKE_FEED 9 // pin


// DHT-22 - one wire type humidity sensor (won't work with one wire lib)
DHT B1R1_1A_AT_002_DHT = DHT(HUMIDITY_INTAKE_FEED, DHT22);
float B1R1_1A_AT_002 = NAN; // or B1R1-1A-AT-007 for right
float B1R1_1A_TT_009 = NAN; // or B1R1-1A-TT-014 for right


DA_AnalogInput B1R1_1A_PDT_001 = DA_AnalogInput(A3, 0.0, 1023.); // min max
DA_AnalogInput B1R1_1A_PDT_002 = DA_AnalogInput(A3, 0.0, 1023.); // min max
DA_AnalogInput B1R1_1A_PDT_003 = DA_AnalogInput(A3, 0.0, 1023.); // min max

// PWM control
#define B1R1_1A_SY_002_PIN 10 // Pin 10 for PWM to control fan speed
unsigned short B1R1_1A_SY_002 = 0; // current duty cycle to write to fan


DA_DiscreteOutput B1R1_1A_XY_001 = DA_DiscreteOutput(12, HIGH); // V1


// HEARTBEAT
unsigned int heartBeat = 0;


// poll I/O every 2 seconds
DA_NonBlockingDelay pollTimer = DA_NonBlockingDelay( POLL_CYCLE_SECONDS*1000, &doOnPoll);



#ifdef PROCESS_TERMINAL
HardwareSerial *tracePort = &Serial;
#endif


void on_B1R1_1A_AT_001_Rising()
{

  unsigned long timestamp = micros();
  //timeCycle = micros();
  timeOnStart = timestamp;
  timeOff = timeOnStart - timeOffStart;
  ENABLE_CO2_SENSOR_FALLING_INTERRUPTS;
}


void on_B1R1_1A_AT_001_Falling()
{

  unsigned long timestamp = micros();

  timeOn = timestamp - timeOnStart;
  timeOffStart = timestamp; 
  ENABLE_CO2_SENSOR_RISING_INTERRUPTS;
}

void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort -> begin(9600);
#endif

#ifdef PROCESS_MODBUS
  slave.begin(MB_SPEED);
#endif

  pinMode(B1R1_1A_SY_002, OUTPUT);
  randomSeed(analogRead(3));

ENABLE_CO2_SENSOR_RISING_INTERRUPTS;
  // humidity sensors start
  B1R1_1A_AT_002_DHT.begin();


}

void loop()
{

#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif
  pollTimer.refresh();



  analogWrite(B1R1_1A_SY_002_PIN, B1R1_1A_SY_002);
}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{
  doReadAnalogs();
  B1R1_1A_AT_002 = B1R1_1A_AT_002_DHT.readHumidity();
  B1R1_1A_TT_009 = B1R1_1A_AT_002_DHT.readTemperature();

   #ifdef TRACE_1WIRE
  *tracePort << "B1R1_1A_PDT_001:" << B1R1_1A_AT_002  ; 
  *tracePort << " B1R1_1A_TT_009:" << B1R1_1A_TT_009  ;

  #endif
  heartBeat++;
}



void doReadAnalogs()
{
  B1R1_1A_PDT_001.refresh();

  B1R1_1A_PDT_002.refresh();
    B1R1_1A_PDT_003.refresh();
#ifdef TRACE_ANALOGS
  B1R1_1A_PDT_001.serialize(tracePort, true);
    B1R1_1A_PDT_002.serialize(tracePort, true);
      B1R1_1A_PDT_003.serialize(tracePort, true);
#endif

}

// 
/*
** Modbus related functions
*/

#ifdef PROCESS_MODBUS
void refreshModbusRegisters()
{

  modbusRegisters[B1R1_1A_PDT_001_MB] =  B1R1_1A_PDT_001.getRawSample();
    modbusRegisters[B1R1_1A_PDT_002_MB] = B1R1_1A_PDT_002.getRawSample();
      modbusRegisters[B1R1_1A_PDT_003_MB] = B1R1_1A_PDT_003.getRawSample();
  modbusRegisters[B1R1_1A_AT_002_MB] =  B1R1_1A_AT_002 * 100;
  modbusRegisters[B1R1_1A_TT_009_MB] = B1R1_1A_TT_009 * 100;



  //modbusRegisters[HR_FLOW1] = B1R1_1A_FT_001.getCurrentPulses();
  modbusRegisters[B1R1_1A_ST_002_MB] =  -1; // not implemented
    modbusRegisters[HEART_BEAT] = heartBeat;
modbusRegisters[B1R1_1A_AT_001_MB] =  ( 2000 * ( timeOn - .002)/(timeOn+timeOff - .004) ) * 10;


}


void processSetFanSpeedCommand()
{
  B1R1_1A_SY_002 = modbusRegisters[B1R1_1A_SY_002_MB];
    #ifdef TRACE_MODBUS_HR
  *tracePort << "Set Fan Speed :" << B1R1_1A_SY_002 << endl;
  #endif
}

void processModbusCommands()
{

  processSetFanSpeedCommand();
}

#endif
