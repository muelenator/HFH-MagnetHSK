/* MainHSK_framework.cpp
 *
 * MainHSK will handle reads/writes as follows
 */

#pragma once

#define NUM_LOCAL_CONTROLS 12

#include <Arduino.h>
#include <driverlib/sysctl.h>

#include "MainHSK_protocol.h"
#include <Core_protocol.h>

// extern setAutoPriorityPeriods();
extern void enterTestMode(uint8_t *data, uint8_t len);
extern void setCommandPriority(housekeeping_prio_t * prio);

extern uint8_t addressList[7][254];
extern uint8_t localControlPriorities[NUM_LOCAL_CONTROLS];

int handleLocalWrite(uint8_t localCommand, uint8_t *data, uint8_t len);
int handleLocalRead(uint8_t localCommand, uint8_t *buffer);
int whatToDoIfMap(uint8_t *data);
