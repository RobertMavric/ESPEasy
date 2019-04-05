#ifdef USES_P183
//#######################################################################################################
//################### Plugin 183 PZEM-004T AC Current and Voltage measurement sensor ####################
//#######################################################################################################
//
// This plugin is interfacing with multiple PZEM-004T Sesor with softserial communication as the sensor
// has an UART pinout (TX/RX/VCC/GND)
// PZEM-004T selected with VCC pin
//


#include <ESPeasySoftwareSerial.h>

#ifndef CONFIG
#define CONFIG(n) (Settings.TaskDevicePluginConfig[event->TaskIndex][n])
#endif

ESPeasySoftwareSerial* swSerial;

typedef struct  {
    uint8_t command;
    uint8_t addr[4];
    uint8_t data;
    uint8_t crc;
} PZEMCommand;

#define PZEM_DEFAULT_READ_TIMEOUT 1000
#define PZEM_ERROR_VALUE -1.0

#define PZEM_VOLTAGE (uint8_t)0xB0
#define RESP_VOLTAGE (uint8_t)0xA0

#define PZEM_CURRENT (uint8_t)0xB1
#define RESP_CURRENT (uint8_t)0xA1

#define PZEM_POWER   (uint8_t)0xB2
#define RESP_POWER   (uint8_t)0xA2

#define PZEM_ENERGY  (uint8_t)0xB3
#define RESP_ENERGY  (uint8_t)0xA3

#define PZEM_SET_ADDRESS (uint8_t)0xB4
#define RESP_SET_ADDRESS (uint8_t)0xA4

#define PZEM_POWER_ALARM (uint8_t)0xB5
#define RESP_POWER_ALARM (uint8_t)0xA5

#define RESPONSE_SIZE sizeof(PZEMCommand)
#define RESPONSE_DATA_SIZE (RESPONSE_SIZE - 2)

#define PZEM_BAUD_RATE 9600

float voltage();
float current();
float power();
float energy();

uint8_t del10s = 0;

unsigned long _readTimeOut = PZEM_DEFAULT_READ_TIMEOUT;

void send( uint8_t cmd);
bool recieve(uint8_t resp, uint8_t *data = 0);
uint8_t crc(uint8_t *data, uint8_t sz);

#define PLUGIN_183
#define PLUGIN_ID_183        183
#define PLUGIN_183_DEBUG     false   //activate extra log info in the debug
#define PLUGIN_NAME_183       "Voltage & Current (AC) - PZEM-004T"
#define PLUGIN_VALUENAME1_183 "Voltage(V)"
#define PLUGIN_VALUENAME2_183 "Current(A)"
#define PLUGIN_VALUENAME3_183 "Power(W)"
#define PLUGIN_VALUENAME4_183 "Energy(Wh)"

// local parameter for this plugin
#define PZEM_MAX_ATTEMPT      3

boolean Plugin_183(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
      {
        Device[++deviceCount].Number = PLUGIN_ID_183;
        Device[deviceCount].Type = DEVICE_TYPE_DUMMY;
        Device[deviceCount].VType = SENSOR_TYPE_QUAD;
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = true;
        Device[deviceCount].ValueCount = 4;
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = true;
        Device[deviceCount].GlobalSyncOption = true;
        break;
      }

    case PLUGIN_GET_DEVICENAME:
      {
        string = F(PLUGIN_NAME_183);
        break;
      }

    case PLUGIN_GET_DEVICEVALUENAMES:
      {
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_183));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_183));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[2], PSTR(PLUGIN_VALUENAME3_183));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[3], PSTR(PLUGIN_VALUENAME4_183));
        break;
      }

    case PLUGIN_WEBFORM_LOAD:
      {
        if(event->TaskIndex == 0){
          addHtml(F("<TR><TD>SoftSerial RX-Pin:<TD>"));
          addPinSelect(false, "rx_pin", CONFIG(0));
          addHtml(F("<TR><TD>SoftSerial TX-Pin:<TD>"));
          addPinSelect(false, "tx_pin", CONFIG(1));
        }
        addHtml(F("<TR><TD>SoftSerial Dir-Pin:<TD>"));
        addPinSelect(false, "dir_pin", CONFIG(2));
        success = true;
        break;
      }

    case PLUGIN_WEBFORM_SAVE:
      {
        if(event->TaskIndex == 0){
          CONFIG(0) = getFormItemInt(F("rx_pin"));
          CONFIG(1) = getFormItemInt(F("tx_pin"));
        } else {
          CONFIG(0) = -1;
          CONFIG(1) = -1;
        }
        CONFIG(2) = getFormItemInt(F("dir_pin"));

        int pzemRXpin = CONFIG(0);
        int pzemTXpin = CONFIG(1);

        if(pzemRXpin != -1 && pzemTXpin != -1){
          if(swSerial)delete(swSerial);
          swSerial = new ESPeasySoftwareSerial(pzemRXpin, pzemTXpin);
          swSerial->begin(9600);
        }

        success = true;
        break;
      }

    case PLUGIN_READ:
      {
        success = true;
        if(swSerial){
          float pzVoltage = Plugin183_ReadVoltage(CONFIG(2));
      		float pzCurrent = Plugin183_ReadCurrent(CONFIG(2));
      		float pzPower   = Plugin183_ReadPower(CONFIG(2));
      		float pzEnergy  = Plugin183_ReadEnergy(CONFIG(2));
          String log = F("PZEM004T: got values");
          log += F("V="); log += pzVoltage;
          log += F("C="); log += pzCurrent;
          log += F("P="); log += pzPower;
          log += F("E="); log += pzEnergy;
          addLog(LOG_LEVEL_INFO, log);

          if(pzVoltage == -1 || pzCurrent == -1 || pzPower == -1)success = false;
          else{
            UserVar[event->BaseVarIndex]     = pzVoltage;
            UserVar[event->BaseVarIndex + 1] = pzCurrent;
            UserVar[event->BaseVarIndex + 2] = pzPower;
            if (pzEnergy>=0)  UserVar[event->BaseVarIndex + 3] = pzEnergy;
          }
        }

        break;
      }
      case PLUGIN_ONCE_A_SECOND:
      {
        success = true;
        break;
      }
    case PLUGIN_INIT:
      {
          int pzemRXpin = CONFIG(0);
          int pzemTXpin = CONFIG(1);
          if(pzemRXpin != -1 && pzemTXpin != -1 && swSerial == 0){
            swSerial = new ESPeasySoftwareSerial(pzemRXpin, pzemTXpin);
            swSerial->begin(9600);
            String log = F("PZEM004T: Initialized");
            log += F(" - RX-Pin="); log += pzemRXpin;
            log += F(" - TX-Pin="); log += pzemTXpin;
            addLog(LOG_LEVEL_INFO, log);

          }

        success = true;
        break;
      }

  }
  return success;
}

//************************************//
//***** reading values functions *****//
//************************************//

// NOTE: readings are attempted only PZEM_AMX_ATTEMPT times

float Plugin183_ReadVoltage(uint8_t index) {
  int counter = 0;
	float reading = -1.0;
  digitalWrite(index, HIGH);
  pinMode(index, OUTPUT);
	do {
		reading = voltage();
		wdt_reset();
		counter++;
	} while (counter < PZEM_MAX_ATTEMPT && reading < 0.0);
	return reading;
}

float Plugin183_ReadCurrent(uint8_t index) {
	int counter = 0;
	float reading = -1.0;
	do {
		reading = current();
		wdt_reset();
		counter++;
	} while (counter < PZEM_MAX_ATTEMPT && reading < 0.0);
  return reading;
}

float Plugin183_ReadPower(uint8_t index) {
  int counter = 0;
	float reading = -1.0;
	do {
		reading = power();
		wdt_reset();
		counter++;
	} while (counter < PZEM_MAX_ATTEMPT && reading < 0.0);
  return reading;
}

float Plugin183_ReadEnergy(uint8_t index) {
	int counter = 0;
	float reading = -1.0;
	do {
		reading = energy();
		wdt_reset();
		counter++;
	} while (counter < PZEM_MAX_ATTEMPT && reading < 0.0);
  pinMode(index, INPUT);
  digitalWrite(index, LOW);
	return reading;
}

float voltage()
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(PZEM_VOLTAGE);
    if(!recieve(RESP_VOLTAGE, data))
        return PZEM_ERROR_VALUE;

    return (data[0] << 8) + data[1] + (data[2] / 10.0);
}

float current()
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(PZEM_CURRENT);
    if(!recieve(RESP_CURRENT, data))
        return PZEM_ERROR_VALUE;

    return (data[0] << 8) + data[1] + (data[2] / 100.0);
}

float power()
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(PZEM_POWER);
    if(!recieve(RESP_POWER, data))
        return PZEM_ERROR_VALUE;

    return (data[0] << 8) + data[1];
}

float energy()
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(PZEM_ENERGY);
    if(!recieve(RESP_ENERGY, data))
        return PZEM_ERROR_VALUE;

    return ((uint32_t)data[0] << 16) + ((uint16_t)data[1] << 8) + data[2];
}

void send(uint8_t cmd)
{
    PZEMCommand pzem;

    pzem.command = cmd;
    pzem.addr[0] = 192;
    pzem.addr[1] = 168;
    pzem.addr[2] = 1;
    pzem.addr[3] = 1;
    pzem.data = 0;

    uint8_t *bytes = (uint8_t*)&pzem;
    pzem.crc = crc(bytes, sizeof(pzem) - 1);

    while(swSerial->available())
        swSerial->read();

    swSerial->write(bytes, sizeof(pzem));
}

bool recieve(uint8_t resp, uint8_t *data)
{
    uint8_t buffer[RESPONSE_SIZE];

    unsigned long startTime = millis();
    uint8_t len = 0;
    while((len < RESPONSE_SIZE) && (millis() - startTime < _readTimeOut))
    {
        if(swSerial->available() > 0)
        {
            uint8_t c = (uint8_t)swSerial->read();
            if(!c && !len)
                continue; // skip 0 at startup
            buffer[len++] = c;
            startTime = millis();
        }
        yield();	// do background netw tasks while blocked for IO (prevents ESP watchdog trigger)
    }

    if(len != RESPONSE_SIZE)
        return false;

    if(buffer[6] != crc(buffer, len - 1))
        return false;

    if(buffer[0] != resp)
        return false;

    if(data)
    {
        for(uint8_t i=0; i<RESPONSE_DATA_SIZE; i++)
            data[i] = buffer[1 + i];
    }

    return true;
}

uint8_t crc(uint8_t *data, uint8_t sz)
{
    uint16_t crc = 0;
    for(uint8_t i=0; i<sz; i++)
        crc += *data++;
    return (uint8_t)(crc & 0xFF);
}
#endif
