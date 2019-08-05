/*
 * MagnetHSK_prototype.ino
 *
 * Initiates serial ports & follows HSK protocol for command responses and error
 * reporting. This program can be used on other devices by changing the device
 * address (myID) and the downStream serial connection (direct line to the SFC)
 *
 * CONSTANTS:
 * --PACKETMARKER is defined in Cobs_encoding.h
 * --MAX_PACKET_LENGTH is defined in PacketSerial
 * --NUM_LOCAL_CONTROLS is defined in MagnetHSK_framework.h
 * --BAUD rates are defined here
 *
 */

/* Everyone uses these */
#include <Core_protocol.h>
#include <PacketSerial.h>

/* These are device specific */
#include "src/MagnetHSK_lib/MagnetHSK_framework.h"
#include "src/MagnetHSK_lib/MagnetHSK_protocol.h"

#include "src/MagnetHSK_lib/PressureAndFlow.h"
#include "src/MagnetHSK_lib/configFunctions.h"
#include "src/MagnetHSK_lib/supportFunctions.h"

/*******************************************************************************
 * Defines
 *******************************************************************************/
#define DOWNBAUD 115200
#define UPBAUD 115200

/* Name of this device */
housekeeping_id myID = eMagnetHsk;

uint8_t outgoingPacket[MAX_PACKET_LENGTH];

size_t hdr_size = sizeof(housekeeping_hdr_t); // size of the header

uint8_t addressList[7][254] = {0}; // List of all upstream devices

uint8_t localControlPriorities[NUM_LOCAL_CONTROLS] = {0}; // Priority settings

housekeeping_hdr_t *hdr_in; // Pointer to incoming header

/* Declare instances of PacketSerial to set up the serial lines */
PacketSerial downStream1;
PacketSerial upStream1;
PacketSerial upStream2;
PacketSerial upStream3;
PacketSerial upStream4;
PacketSerial upStream5;
PacketSerial upStream6;
PacketSerial upStream7;

PacketSerial *serialDevices[7] = {&upStream1, &upStream2, &upStream3,
                                  &upStream4, &upStream5, &upStream6,
                                  &upStream7};
/*******************************************************************************
 * Setup
 *******************************************************************************/
void setup() {
  Serial3.begin(DOWNBAUD);
  downStream1.setStream(&Serial3);
  downStream1.setPacketHandler(&checkHdr);

  //	upStream1.setStream(&Serial);
  //	upStream1.setPacketHandler(&checkHdr);

  Serial.begin(UPBAUD);
  upStream4.setStream(&Serial);
  upStream4.setPacketHandler(&checkHdr);

  Serial4.begin(UPBAUD);
  upStream5.setStream(&Serial4);
  upStream5.setPacketHandler(&checkHdr);

  Serial7.begin(UPBAUD);
  upStream7.setStream(&Serial7);
  upStream7.setPacketHandler(&checkHdr);

  pinMode(7, INPUT);
  // pinMode(6,OUTPUT);
  // digitalWrite(6,LOW);
  initRS232();
  Initialize_TM4C123();
  configure_channels();
  configure_memory_table();
  configure_global_parameters();
}

/*******************************************************************************
 * Main program
 ******************************************************************************/
void loop() {
  /* Continuously read in one byte at a time until a packet is received */
  if (downStream1.update() != 0)
    badPacketReceived(&downStream1);
  if (upStream1.update() != 0)
    badPacketReceived(&upStream1);
  if (upStream2.update() != 0)
    badPacketReceived(&upStream2);
  if (upStream3.update() != 0)
    badPacketReceived(&upStream3);
  if (upStream4.update() != 0)
    badPacketReceived(&upStream4);
  if (upStream5.update() != 0)
    badPacketReceived(&upStream5);
  if (upStream6.update() != 0)
    badPacketReceived(&upStream6);
  if (upStream7.update() != 0)
    badPacketReceived(&upStream7);
}

/*******************************************************************************
 * Packet handling functions
 *******************************************************************************/
/* To be executed when a packet is received
 *
 * Function flow:
 * --Creates default header & error header values that most board functions will
 * use
 * --Checks if the message was intended for this device
 *      --If so, check if it was a send priority command
 *          --If not, execute the command by calling CommandCenter().
 *            priority request,
 *      --If not, forward on the packet based on where it came from
 *
 * Function params:
 * sender:		PacketSerial instance where the message came from
 * buffer:		The decoded packet
 * len:			The size (in bytes) of the decoded packet
 *
 *
 * */
 void checkHdr(const void *sender, const uint8_t *buffer, size_t len) {
   hdr_in = (housekeeping_hdr_t *)buffer;
   /* Check if the message is for this device. If so, check & execute command */
   if (hdr_in->dst == myID || hdr_in->dst == eBroadcast) {
     /* Check for data corruption */
     if (verifyChecksum((uint8_t *)buffer)) {
       /* Check for bad length	*/
       if (hdr_in->len != len - 4 - 1) {
         badPacketReceived((PacketSerial *)sender);
         return;
       }

       /* Forward downstream if eBroadcast */
       if (hdr_in->dst == eBroadcast) {
         upStream1.send(buffer, len);
         upStream2.send(buffer, len);
         upStream3.send(buffer, len);
         upStream4.send(buffer, len);
         upStream5.send(buffer, len);
         upStream6.send(buffer, len);
         upStream7.send(buffer, len);
       }

       /* If a send priority command is received */
       if ((int)hdr_in->cmd <= 253 && (int)hdr_in->cmd >= 250) {
         handlePriority((hdr_in->cmd - 249) % 4);
       }
       /* Otherwise just execute the command */
       else {
         if (handleLocalCommand(hdr_in, (uint8_t *)hdr_in + hdr_size) < 0) {
           buildError(EBADCOMMAND);
           fillChecksum((uint8_t *)outgoingPacket);
           downStream1.send(outgoingPacket, hdr_size + 4 + 1);
         }
       }
     }

     /* If the checksum didn't match, throw a bad args error */
     else {
       buildError(EBADARGS);
       fillChecksum((uint8_t *)outgoingPacket);
       downStream1.send(outgoingPacket, hdr_size + 4 + 1);
     }
   }

   /* If the message wasn't meant for this device pass it along */
   else {
     if (sender == &downStream1) {
       /* Send upstream away from SFC */
       forwardUp(buffer, len);
     }
     /* Send  downstream towards SFC */
     else
       forwardDown(buffer, len, sender);
   }
 }

 // got a priority request from destination dst
 int handlePriority(uint8_t priority) {
   for (int i = FIRST_LOCAL_COMMAND; i < NUM_LOCAL_CONTROLS; i++) {
     hdr_in->cmd = i;
     if (localControlPriorities[i] == priority || !priority) {
       if (handleLocalCommand(hdr_in, (uint8_t *)hdr_in + hdr_size) < 0) {
         buildError(EBADCOMMAND);
         fillChecksum((uint8_t *)outgoingPacket);
         downStream1.send(outgoingPacket, hdr_size + 4 + 1);
       }
     }
   }
 }

 int handleLocalCommand(housekeeping_hdr_t *hdr, uint8_t *data) {
   int retval;

   if (hdr->len) {
     if (retval = handleLocalWrite(hdr->cmd, data, hdr->len))
       return retval;
   } else {
     // local read. by definition these always go downstream.
     housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *)outgoingPacket;
     uint8_t *respData = outgoingPacket + hdr_size;

     respHdr->src = myID;
     respHdr->dst = hdr->src;

     retval = handleLocalRead(hdr->cmd, respData);

     if (retval >= 0) {
       respHdr->cmd = hdr->cmd;
       respHdr->len = (uint8_t)retval;
     } else {
       buildError(retval);
     }

     fillChecksum(outgoingPacket);
     downStream1.send(outgoingPacket, hdr_size + respHdr->len + 1);
   }
   return 0;
 }

 /******************************************************************************
  * Packet checking functions
  *****************************************************************************/

 /* Function flow:
  * --Forwards the packet towards the SFC
  *
  * Function params:
  * buffer:		The decoded packet received
  * len:			Size (in bytes) of the incoming packet above
  * sender:		PacketSerial instance (serial line) where the message
  * was received
  *
  */
 void forwardDown(const uint8_t *buffer, size_t len, const void *sender) {
   /* Continue to send the message */
   downStream1.send(buffer, len);
   checkDownBoundDst(sender);
 }

 /* Function flow:
  * --Checks if the intended destination is a device known to be attached
  *      --If it is known, send the packet down the correct serial line towards
  * that device
  *      --If it isn't, throw an error
  *
  * Function params:
  * buffer:		The decoded packet received
  * len:			Size (in bytes) of the incoming packet above
  *
  */
 void forwardUp(const uint8_t *buffer, size_t len) {
   int bus = checkUpBoundDst();
   if (bus)
     serialDevices[bus]->send(buffer, len);
 }

 /* Function flow:
  * --Checks to see if the downstream device that sent the message is known
  *      --If not, add it to the list of known devices
  *      --If yes, just carry on
  * --Executed every time a packet is received from downStream
  *
  * Function params:
  * sender:		PacketSerial instance (serial line) where the message
  * was received
  *
  */
 void checkDownBoundDst(const void *sender) {
   for (int i = 0; i < 7; i++) {
     if (serialDevices[i] == (PacketSerial *)sender) {
       if (addressList[i][hdr_in->src] == 0) {
         addressList[i][hdr_in->src] = (uint8_t)hdr_in->src;
         return;
       }
     }
   }
 }

 /* Function flow:
  * --Checks each serial line's list of devices for the intended destination of a
  * packet
  * --If a line has that device known, a number corresonding to that serial line
  * is returned
  *      --In the array 'serialDevices'
  * --If the device is not known, an error is thrown
  *
  */
 int checkUpBoundDst() {
   for (int i = 0; i < 7; i++) {
     if (addressList[i][hdr_in->dst] != 0) {
       return i;
     }
   }

   /* If it wasn't found, throw an error */
   buildError(EBADDEST);

   housekeeping_hdr_t *hdr_out = (housekeeping_hdr_t *)outgoingPacket;
   fillChecksum(outgoingPacket);
   downStream1.send(outgoingPacket, hdr_size + hdr_out->len + 1);

   return 0;
 }

 /* Function flow:
  * --Find the device address that produced the error
  * --Execute the bad length function & send the error to the SFC
  *
  * Function params:
  * sender:		PacketSerial instance which triggered the error protocol
  *
  *
  * Send an error if a packet is unreadable in some way */
 void badPacketReceived(PacketSerial *sender) {
   int i = 0;
   for (i = 0; i < 7; i++) {
     if (sender == serialDevices[i]) {
       hdr_in->src = addressList[i][0];
       break;
     }
   }

   if (i == 7)
     hdr_in->src = eSFC;

   buildError(EBADLEN);

   fillChecksum((uint8_t *)outgoingPacket);
   downStream1.send(outgoingPacket, hdr_size + 4 + 1);
 }

 void buildError(int error) {
   housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *)outgoingPacket;
   housekeeping_err_t *err = (housekeeping_err_t *)(outgoingPacket + hdr_size);
   respHdr->dst = eSFC;
   respHdr->cmd = eError;
   respHdr->len = 4;
   err->src = hdr_in->src;
   err->dst = hdr_in->dst;
   err->cmd = hdr_in->cmd;
   err->error = error;
 }

 /******************************************************************************
  * System functions for all devices
  *****************************************************************************/

 void enterTestMode(uint8_t *data, uint8_t len) {
   if (len == 2) {
     housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *)outgoingPacket;
     uint8_t *respData = outgoingPacket + hdr_size;

     respHdr->dst = eSFC;
     respHdr->src = myID;
     respHdr->cmd = eTestMode;
     respHdr->len = 2;

     uint16_t numTestPackets = (uint16_t) * (data + 1) << 8;
     numTestPackets |= *(data);

     while (numTestPackets) {
       memcpy(respData, (uint8_t *)&numTestPackets, sizeof(numTestPackets));
       fillChecksum(outgoingPacket);
       downStream1.send(outgoingPacket, hdr_size + respHdr->len + 1);
       numTestPackets--;
     }
   }
 }

 void setCommandPriority(housekeeping_prio_t *prio) {
   localControlPriorities[prio->command] = prio->prio_type;

   housekeeping_hdr_t *hdr = (housekeeping_hdr_t *)outgoingPacket;
   hdr->dst = eSFC;
   hdr->src = myID;
   hdr->cmd = eSetPriority;
   hdr->len = sizeof(housekeeping_prio_t);

   housekeeping_prio_t *RespPrio =
       (housekeeping_prio_t *)(outgoingPacket + hdr_size);
   RespPrio->command = prio->command;
   RespPrio->prio_type = prio->prio_type;

   fillChecksum(outgoingPacket);
   downStream1.send(outgoingPacket, hdr_size + hdr->len + 1);
 }
