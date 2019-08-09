/*
 * MainHSK_prototype.ino
 *
 * Initiates serial ports & follows HSK protocol for command responses and error
 * reporting. This program can be used on other devices by changing the device
 * address (myID) and the downStream serial connection (direct line to the SFC)
 *
 * CONSTANTS:
 * --PACKETMARKER is defined in Cobs_encoding.h
 * --MAX_PACKET_LENGTH is defined in PacketSerial
 * --NUM_LOCAL_CONTROLS is defined here
 * --FIRST_LOCAL_COMMAND is defined here
 * --TEST_MODE_PERIOD is defined here
 * --BAUD rates are defined here
 *
 */

/* Everyone uses these */
#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>

/* These are device specific */
#include "src/MainHSK_lib/MainHSK_protocol.h"

/*******************************************************************************
 * Defines
 *******************************************************************************/
#define TEST_MODE_PERIOD 100  // period in milliseconds between testmode packets being sent
#define NUM_LOCAL_CONTROLS 12 // how many commands total are local to the board
#define FIRST_LOCAL_COMMAND 2 // value of hdr->cmd that is the first command local to the board

#define DOWNBAUD 1292000 // Baudrate to the SFC
#define UPBAUD 115200    // Baudrate to upsteam devices

/* Name of this device */
housekeeping_id myID = eMainHsk;

uint8_t outgoingPacket[MAX_PACKET_LENGTH];

size_t hdr_size = sizeof(housekeeping_hdr_t); // size of the header

uint8_t addressList[7][254] = {0}; // List of all upstream devices

uint16_t currentPacketCount = 0; // How many packets have been sent

uint8_t localControlPriorities[NUM_LOCAL_CONTROLS] = {0}; // Priority settings

autoPriorityPeriods_t currentAutoPriorityPeriods = {0}; // Current auto-priority settings

housekeeping_hdr_t *hdr_in; // Pointer to incoming header

/* Timing variables for autopriority functions */
uint32_t nextLowPacket = 0;
uint32_t nextMedPacket = 0;
uint32_t nextHighPacket = 0;

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
void setup()
{
  Serial.begin(DOWNBAUD);
  downStream1.setStream(&Serial);
  downStream1.setPacketHandler(&checkHdr);

  //	upStream1.setStream(&Serial);
  //	upStream1.setPacketHandler(&checkHdr);

  Serial1.begin(UPBAUD);
  upStream2.setStream(&Serial1);
  upStream2.setPacketHandler(&checkHdr);

  Serial2.begin(UPBAUD);
  upStream3.setStream(&Serial2);
  upStream3.setPacketHandler(&checkHdr);

  Serial3.begin(UPBAUD);
  upStream4.setStream(&Serial3);
  upStream4.setPacketHandler(&checkHdr);

  Serial4.begin(UPBAUD);
  upStream5.setStream(&Serial4);
  upStream5.setPacketHandler(&checkHdr);

  Serial5.begin(UPBAUD);
  upStream6.setStream(&Serial5);
  upStream6.setPacketHandler(&checkHdr);

  Serial7.begin(UPBAUD);
  upStream7.setStream(&Serial7);
  upStream7.setPacketHandler(&checkHdr);
}

/*******************************************************************************
 * Main program
 ******************************************************************************/
void loop()
{
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

  checkAutoPriority();
}

/*******************************************************************************
 * Packet handling functions--Device specific
 *******************************************************************************/
void checkHdr(const void *sender, const uint8_t *buffer, size_t len)
{
  hdr_in = (housekeeping_hdr_t *)buffer;
  /* Check if the message is for this device. If so, check & execute command */
  if (hdr_in->dst == myID || hdr_in->dst == eBroadcast)
  {
    /* Check for data corruption */
    if (verifyChecksum((uint8_t *)buffer))
    {
      /* Check for bad length	*/
      if (hdr_in->len != len - 4 - 1)
      {
        badPacketReceived((PacketSerial *)sender);
        return;
      }

      /* Forward downstream if eBroadcast */
      if (hdr_in->dst == eBroadcast)
      {
        upStream1.send(buffer, len);
        upStream2.send(buffer, len);
        upStream3.send(buffer, len);
        upStream4.send(buffer, len);
        upStream5.send(buffer, len);
        upStream6.send(buffer, len);
        upStream7.send(buffer, len);
      }

      /* If a send priority command is received */
      if ((int)hdr_in->cmd <= 253 && (int)hdr_in->cmd >= 250)
        handlePriority((hdr_in->cmd - 249) % 4); // passes the priority #
      /* Otherwise just execute the command */
      else
        handleLocalCommand(hdr_in, (uint8_t *)hdr_in + hdr_size);
    }

    /* If the checksum didn't match, throw a bad args error */
    else
    {
      buildError(EBADARGS);
      fillChecksum((uint8_t *)outgoingPacket);
      downStream1.send(outgoingPacket, hdr_size + 4 + 1);
    }
  }

  /* If the message wasn't meant for this device pass it along */
  else
  {
    if (sender == &downStream1)
      forwardUp(buffer, len); // Send upstream away from SFC
    else
      forwardDown(buffer, len, sender); // Send downstream towards SFC
  }
}

// Fn to handle a local command write.
// This gets called when a local command is received
// with data (len != 0).
int handleLocalWrite(uint8_t localCommand, uint8_t *data, uint8_t len)
{
  switch (localCommand)
  {
  case eSetPriority:
  {
    housekeeping_prio_t *in_prio = (housekeeping_prio_t *)data;
    housekeeping_prio_t *out_prio = (housekeeping_prio_t *)(outgoingPacket + hdr_size);
    localControlPriorities[in_prio->command] = in_prio->prio_type;

    memcpy(out_prio, (uint8_t *)in_prio, sizeof(housekeeping_prio_t));
    return sizeof(housekeeping_prio_t);
  }
  case eTestMode:
  {
    return handleTestMode(data, len);
  }
  case ePacketCount:
  {
    return EBADLEN;
  }
  case eRandomTest:
  {
    return EBADLEN;
  }
  case eAutoPriorityPeriod:
  {
    memcpy((uint16_t *)&currentAutoPriorityPeriods, data, len);
    return 0;
  }
  case eReset:
  {
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
// the buffer, or -EBADCOMMAND if there's no command there
int handleLocalRead(uint8_t localCommand, uint8_t *buffer)
{
  switch (localCommand)
  {
  case ePingPong:
  {
    return 0;
  }
  case eSetPriority:
  {
    return EBADLEN;
  }
  case eIntSensorRead:
  {
    uint32_t TempRead = analogRead(TEMPSENSOR);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eMapDevices:
  {
    return whatToDoIfMap(buffer);
  }
  case ePacketCount:
  {
    memcpy(buffer, (uint8_t *)&currentPacketCount, sizeof(currentPacketCount));
    return sizeof(currentPacketCount);
  }
  case eRandomTest:
  {
    int numBytes = rand() % MAX_PACKET_LENGTH * (1 - 1 / 254) - 2;
    for (int i = 0; i < numBytes; i++)
      *(buffer + i) = rand();
    return numBytes;
  }
  case eAutoPriorityPeriod:
  {
    memcpy(buffer, (uint8_t *)&currentAutoPriorityPeriods, sizeof(autoPriorityPeriods_t));
    return sizeof(autoPriorityPeriods_t);
  }
  case eReset:
  {
    SysCtlReset();
    return 0;
  }
  default:
    return EBADCOMMAND;
  }
}

/******************************************************************************
 * System functions for all devices
 *****************************************************************************/
void handleLocalCommand(housekeeping_hdr_t *hdr, uint8_t *data)
{
  int retval;

  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *)outgoingPacket;
  uint8_t *respData = outgoingPacket + hdr_size;

  if (hdr->len) // local write
    retval = handleLocalWrite(hdr->cmd, data, hdr->len);
  else // local read. by definition these always go downstream.
    retval = handleLocalRead(hdr->cmd, respData);

  if (retval >= 0)
  {
    respHdr->src = myID;
    respHdr->dst = hdr->src;
    respHdr->cmd = hdr->cmd;
    respHdr->len = retval;
  }
  else
    buildError(retval);

  fillChecksum(outgoingPacket);
  downStream1.send(outgoingPacket, hdr_size + respHdr->len + 1);

  currentPacketCount++;
}

int handleTestMode(uint8_t *data, uint8_t len)
{
  if (len == 2)
  {
    housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *)outgoingPacket;
    uint8_t *respData = outgoingPacket + hdr_size;

    respHdr->dst = eSFC;
    respHdr->src = myID;
    respHdr->cmd = eTestMode;
    respHdr->len = sizeof(uint16_t);

    uint16_t numTestPackets = (uint16_t) * (data + 1) << 8 | *(data);
    uint32_t timelastpacket = millis();

    while (numTestPackets)
    {
      if (millis() - timelastpacket > TEST_MODE_PERIOD)
      {
        memcpy(respData, (uint8_t *)&numTestPackets, sizeof(numTestPackets));
        fillChecksum(outgoingPacket);
        downStream1.send(outgoingPacket, hdr_size + respHdr->len + 1);

        numTestPackets--;
        currentPacketCount++;
        timelastpacket = millis();
      }
    }
    return sizeof(uint16_t);
  }
  else
    return EBADLEN;
}

// got a priority request from destination dst
void handlePriority(uint8_t priority)
{
  for (int i = FIRST_LOCAL_COMMAND; i < NUM_LOCAL_CONTROLS; i++)
  {
    hdr_in->cmd = i;
    if (localControlPriorities[i] == priority || !priority)
      handleLocalCommand(hdr_in, (uint8_t *)hdr_in + hdr_size);
  }
}

/******************************************************************************
 * Device specific functions
 *****************************************************************************/
int whatToDoIfMap(uint8_t *data)
{
  uint8_t num = 0;
  /* Fill in data array with device list */
  for (int i = 0; i < 7; i++)
  {
    for (int j = 0; j < 254; j++)
    {
      if (addressList[i][j] != 0)
      {
        *(data + num++) = j;
      }
    }
  }
  return num;
}

void checkAutoPriority()
{
  // this function should check if autopriority packets need to be sent and call functions to send those packets.
  // CHECK ROLLOVER
  // check low, med, and high
  housekeeping_hdr_t fakeHdr;
  fakeHdr.src = eSFC;
  fakeHdr.dst = myID;
  fakeHdr.len = 0;
  hdr_in = &fakeHdr;

  // HIGH
  if (currentAutoPriorityPeriods.hiPriorityPeriod >= MIN_PERIOD)
  {
    if (long(millis() - nextHighPacket) > 0)
    {
      // function for sending low priority packets, acts as if it was sent by sfc as query for sending that priority packets.
      handlePriority(eHiPriority);
      nextHighPacket += currentAutoPriorityPeriods.hiPriorityPeriod;
    }
  }
  // MEDIUM
  if (currentAutoPriorityPeriods.medPriorityPeriod >= MIN_PERIOD)
  {
    if (long(millis() - nextMedPacket) > 0)
    {
      // function for sending low priority packets, acts as if it was sent by sfc as query for sending that priority packets.
      handlePriority(eMedPriority);
      nextMedPacket += currentAutoPriorityPeriods.medPriorityPeriod;
    }
  }
  // LOW
  if (currentAutoPriorityPeriods.lowPriorityPeriod >= MIN_PERIOD)
  {
    if (long(millis() - nextLowPacket) > 0)
    {
      // function for sending low priority packets, acts as if it was sent by sfc as query for sending that priority packets.
      handlePriority(eLowPriority);
      nextLowPacket += currentAutoPriorityPeriods.lowPriorityPeriod;
    }
  }
}
/******************************************************************************
 * Packet checking functions
 *****************************************************************************/

/* --Forwards the packet towards the SFC
 *
 * Function params:
 * buffer:		The decoded packet received
 * len:			Size (in bytes) of the incoming packet above
 * sender:		PacketSerial instance (serial line) where the message was received
 *
 */
void forwardDown(const uint8_t *buffer, size_t len, const void *sender)
{
  /* Continue to send the message */
  downStream1.send(buffer, len);
  checkDownBoundDst(sender);
}

/* --Checks if the intended destination is a device known to be attached
 *      --If it is known, send the packet down the correct serial line towards
 * that device
 *      --If it isn't, throw an error
 *
 * Function params:
 * buffer:		The decoded packet received
 * len:			Size (in bytes) of the incoming packet above
 *
 */
void forwardUp(const uint8_t *buffer, size_t len)
{
  int bus = checkUpBoundDst();
  if (bus)
    serialDevices[bus]->send(buffer, len);
}

/* --Checks to see if the downstream device that sent the message is known
 *      --If not, add it to the list of known devices
 *      --If yes, just carry on
 * --Executed every time a packet is received from downStream
 *
 * Function params:
 * sender:		PacketSerial instance (serial line) where the message
 * was received
 *
 */
void checkDownBoundDst(const void *sender)
{
  for (int i = 0; i < 7; i++)
  {
    if (serialDevices[i] == (PacketSerial *)sender)
    {
      if (addressList[i][hdr_in->src] == 0)
      {
        addressList[i][hdr_in->src] = (uint8_t)hdr_in->src;
        return;
      }
    }
  }
}

/* --Checks each serial line's list of devices for the intended destination of a
 * packet
 * --If a line has that device known, a number corresonding to that serial line
 * is returned
 *      --In the array 'serialDevices'
 * --If the device is not known, an error is thrown
 *
 */
int checkUpBoundDst()
{
  for (int i = 0; i < 7; i++)
  {
    if (addressList[i][hdr_in->dst] != 0)
    {
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

/* --Find the device address that produced the error
 * --Execute the bad length function & send the error to the SFC
 *
 * Function params:
 * sender:		PacketSerial instance which triggered the error protocol
 *
 *
 * Send an error if a packet is unreadable in some way */
void badPacketReceived(PacketSerial *sender)
{
  int i = 0;
  for (i = 0; i < 7; i++)
  {
    if (sender == serialDevices[i])
    {
      hdr_in->src = addressList[i][0];
      break;
    }
  }

  if (i == 7)
    hdr_in->src = eSFC;

  buildError(EBADLEN);

  fillChecksum((uint8_t *)outgoingPacket);
  downStream1.send(outgoingPacket, hdr_size + 4 + 1);
  currentPacketCount++;
}

void buildError(int error)
{
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
