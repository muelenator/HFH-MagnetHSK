/*
 * MagResponse.h
 *
 * Declares a set of functions to act as responses to received commands and
 * error protocols
 *
 */

#pragma once

#include <Arduino.h>

/* MagnetHSK specific */
#include <MagProtocol.h>
#include <supportFunctions.h>
#include <configConstants.h>
#include <PressureAndFlow.h>
#include <TempSensors.h>

/* If these commands are received */
void whatToDoIfResistance(housekeeping_hdr_t * hdr_out, uint8_t device);
void whatToDoIfTemp(housekeeping_hdr_t * hdr_out, uint8_t device);
bool whatToDoIfFlow(housekeeping_hdr_t * hdr_out, uint8_t device);
void whatToDoIfTempProbe(housekeeping_hdr_t * hdr_out, uint8_t device);
void whatToDoIfPressure(housekeeping_hdr_t * hdr_out);


/* If these commands are received */
void whatToDoIfPingPong(housekeeping_hdr_t * hdr_out);

void whatToDoIfSetPriority(housekeeping_prio_t * hdr_prio, housekeeping_hdr_t * hdr_out, uint8_t * comPriorList);

void whatToDoIfTestMode(uint16_t * numTestPackets_p, housekeeping_hdr_t * hdr_out);

bool checkThatPriority(housekeeping_hdr_t * hdr_in, housekeeping_hdr_t * hdr_out, uint8_t numSends);

/* If the criteria for these errors is fulfilled  */
void error_badDest(housekeeping_hdr_t * hdr_in, housekeeping_hdr_t * hdr_out, housekeeping_err_t * hdr_err);
void error_badCommand(housekeeping_hdr_t * hdr_in, housekeeping_hdr_t * hdr_out, housekeeping_err_t * hdr_err);
void error_badLength(housekeeping_hdr_t * hdr_in, housekeeping_hdr_t * hdr_out, housekeeping_err_t * hdr_err);
void error_badArgs(housekeeping_hdr_t * hdr_in, housekeeping_hdr_t * hdr_out, housekeeping_err_t * hdr_err);
/* Matches on-board data with outgoing packets */
void matchData(housekeeping_hdr_t * hdr_out);
