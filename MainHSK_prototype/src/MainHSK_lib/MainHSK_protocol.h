/*
 * MainHSK_protocol.cpp
 *
 * Declares the interface protocol for cross device communication.
 *
 * Housekeeping packet consists of header
 * 0-255 payload bytes
 * 1 byte CRCS (or checksum)
 */

/*****************************************************************************
 * Defines
 ****************************************************************************/
#pragma once
#include <stdint.h>

/*******************************************************************************
 * Typedef enums
 *******************************************************************************/

/* Command definitions */
typedef enum MainHSK_cmd {
  // 2-248 are board-specific: these are test commands
  eIntSensorRead = 2,
  eMapDevices = 3,
  eHeaterControl = 4,
  ePacketCount = 5,
  eRandomTest = 6
} MainHSK_cmd;
