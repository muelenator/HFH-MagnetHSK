#include "MagnetHSK_framework.h"

// Fn to handle a local command write.
// This gets called when a local command is received
// with data (len != 0).
int handleLocalWrite(uint8_t localCommand, uint8_t *data, uint8_t len) {
  switch (localCommand) {
  case eSetPriority: {
    setCommandPriority((housekeeping_prio_t *)data);
    return 0;
  }
  case eTestMode: {
    enterTestMode(data, len);
    return 2;
  }
  default:
    return -EBADCOMMAND;
  }
}

// Fn to handle a local command read.
// This gets called when a local command is received
// with no data (len == 0)
// buffer contains the pointer to where the data
// will be written.
// int returns the number of bytes that were copied into
// the buffer, or -EBADCOMMAND if there's no command there.

int handleLocalRead(uint8_t localCommand, uint8_t *buffer) {
  switch (localCommand) {
  /* System functions */
  case ePingPong: {
    return 0;
  }
  case eReset: {
    SysCtlReset();
    return 0;
  }

  /* Resistance measurements */
  case eResistanceCh3: {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 3);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead, sizeof(ResRead));
  }
  case eResistanceCh6: {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 6);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead, sizeof(ResRead));
  }
  case eResistanceCh9: {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 9);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead, sizeof(ResRead));
  }
  case eResistanceCh12: {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 12);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead, sizeof(ResRead));
  }
  case eResistanceCh16: {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 16);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead, sizeof(ResRead));
  }
  case eResistanceCh20: {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 20);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead, sizeof(ResRead));
  }

  /* Temperature measurements */
  case eTempCh3: {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 3);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempCh6: {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 6);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempCh9: {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 9);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempCh12: {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 12);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempCh16: {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 16);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempCh20: {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 20);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }

  /* Flow readings */
  case eFlow1: {
    double gasdata[4];
    char gastype[100];
    char errorcode[100];

    uint32_t FlowRead = readFlow(1, gasdata, gastype, errorcode);

    memcpy(buffer, (uint8_t *)&gasdata, sizeof(gasdata));
    memcpy(buffer + sizeof(gasdata), (uint8_t *)&gastype, sizeof(gastype));
    memcpy(buffer + sizeof(gasdata) + sizeof(gastype), (uint8_t *)&errorcode,
           sizeof(errorcode));

    return (sizeof(gasdata) + sizeof(gastype) + sizeof(errorcode));
  }
  case eFlow2: {
    double gasdata[4];
    char gastype[100];
    char errorcode[100];

    uint32_t FlowRead = readFlow(2, gasdata, gastype, errorcode);

    memcpy(buffer, (uint8_t *)&gasdata, sizeof(gasdata));
    memcpy(buffer + sizeof(gasdata), (uint8_t *)&gastype, sizeof(gastype));
    memcpy(buffer + sizeof(gasdata) + sizeof(gastype), (uint8_t *)&errorcode,
           sizeof(errorcode));

    return (sizeof(gasdata) + sizeof(gastype) + sizeof(errorcode));
  }

  /* Temperature probes */
  case eTempProbe1: {
    uint32_t TempRead = tempSensorVal(1, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempProbe2: {
    uint32_t TempRead = tempSensorVal(2, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempProbe3: {
    uint32_t TempRead = tempSensorVal(3, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempProbe4: {
    uint32_t TempRead = tempSensorVal(4, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempProbe5: {
    uint32_t TempRead = tempSensorVal(5, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempProbe6: {
    uint32_t TempRead = tempSensorVal(6, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempProbe7: {
    uint32_t TempRead = tempSensorVal(7, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempProbe8: {
    uint32_t TempRead = tempSensorVal(8, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempProbe9: {
    uint32_t TempRead = tempSensorVal(9, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }
  case eTempProbe10: {
    uint32_t TempRead = tempSensorVal(10, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead, sizeof(TempRead));
  }

  /* Pressure Readings */
  case ePressure: {
    double pressureValue;
    double temperature;
    char typeOfPressure;
    char errorcode[100];

    readPressureSensor(&pressureValue, &temperature, &typeOfPressure,
                       errorcode);

    memcpy(buffer, (uint8_t *)&pressureValue, sizeof(pressureValue));
    memcpy(buffer + sizeof(pressureValue), (uint8_t *)&temperature,
           sizeof(temperature));
    memcpy(buffer + sizeof(pressureValue) + sizeof(temperature) +
               sizeof(typeOfPressure),
           (uint8_t *)&errorcode, sizeof(errorcode));

    return (sizeof(pressureValue) + sizeof(temperature) +
            sizeof(typeOfPressure) + sizeof(errorcode));
  }

  default:
    return -EBADCOMMAND;
  }
}
