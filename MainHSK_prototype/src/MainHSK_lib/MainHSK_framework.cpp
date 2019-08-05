/* MainHSK_framework.cpp
 *
 * MainHSK will handle local reads/writes as follows
 */

#include "MainHSK_framework.h"

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
  // case eAutoPriorityPeriod:
  // {
  // 	got an autoPriorityPeriod command
  // 	return setAutoPriorityPeriods(data, len);
  // }
  case ePacketCount: {
    return EBADLEN;
  }
  case eRandomTest: {
    return EBADLEN;
  }
  default:
    return EBADCOMMAND;
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
  case eIntSensorRead: {
    uint32_t TempRead = analogRead(TEMPSENSOR);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eMapDevices: {
    return whatToDoIfMap(buffer);
  }
  case eReset: {
    SysCtlReset();
    return 0;
  }
  // case eAutoPriorityPeriod:
  // {
  // 	memcpy(buffer, (uint8_t *) currentAutoPriorityPeriods,
  // sizeof(currentAutoPriorityPeriods)); 	return
  // sizeof(currentAutoPriorityPeriods);
  // }
  case ePacketCount: {
    memcpy(buffer, (uint8_t *)&currentPacketCount, sizeof(currentPacketCount));
    return sizeof(currentPacketCount);
  }
  case eRandomTest: {
    int numBytes = rand() % MAX_PACKET_LENGTH * (1 - 1 / 254);
    uint8_t Bytes[numBytes];
    for (int i = 0; i < numBytes; i++) {
      Bytes[i] = rand();
    }
    memcpy(buffer, (uint8_t *)&Bytes, numBytes);
    return numBytes;
  }

  default:
    return EBADCOMMAND;
  }
}

/* Function flow:
 * --Fills outgoing header w/protocol standard
 * --Fills outgoing data array with its internal device list
 *
 * Function params:
 * hdr_out:		Pointer to outgoing packet header
 * downDevs:	The device's 2d array of downstream devices
 * num:			Total number of downstream devices
 *
 *  */
int whatToDoIfMap(uint8_t *data) {
  uint8_t num = 0;
  /* Fill in data array with device list */
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 254; j++) {
      if (addressList[i][j] != 0) {
        *(data + num++) = j;
      }
    }
  }
  return num;
}
