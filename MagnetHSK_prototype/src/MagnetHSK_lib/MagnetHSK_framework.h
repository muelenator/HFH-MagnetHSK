/* MainHSK_framework.cpp
 *
 * MainHSK will handle reads/writes as follows
 */

#pragma once

#define NUM_LOCAL_CONTROLS 12

#include <Arduino.h>
#include <driverlib/sysctl.h>

#include "MagnetHSK_protocol.h"
#include <Core_protocol.h>

#include "PressureAndFlow.h"
#include "TempSensors.h"
#include "configConstants.h"
#include "supportFunctions.h"

// Not sure where other constants are stored, this can be moved -L
const int thermalPin = 38;

// extern setAutoPriorityPeriods();
extern void enterTestMode(uint8_t *data, uint8_t len);

extern uint8_t localControlPriorities[NUM_LOCAL_CONTROLS];

int handleLocalWrite(uint8_t localCommand, uint8_t *data, uint8_t len);
int handleLocalRead(uint8_t localCommand, uint8_t *buffer);
