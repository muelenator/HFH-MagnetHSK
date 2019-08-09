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
#include "src/MagnetHSK_lib/MagnetHSK_protocol.h"

#include "src/MagnetHSK_lib/PressureAndFlow.h"
#include "src/MagnetHSK_lib/configFunctions.h"
#include "src/MagnetHSK_lib/supportFunctions.h"
#include "src/MagnetHSK_lib/configConstants.h"
#include "src/MagnetHSK_lib/TempSensors.h"

/*******************************************************************************
 * Defines
 *******************************************************************************/
#define TEST_MODE_PERIOD 100      // period in milliseconds between testmode packets being sent
#define NUM_LOCAL_CONTROLS 26 + 6 // how many commands total are local to the board
#define FIRST_LOCAL_COMMAND 2     // value of hdr->cmd that is the first command local to the board

#define DOWNBAUD 115200
#define UPBAUD 115200

/* Name of this device */
housekeeping_id myID = eMagnetHsk;

uint8_t outgoingPacket[MAX_PACKET_LENGTH];

size_t hdr_size = sizeof(housekeeping_hdr_t); // size of the header

uint8_t addressList[7][254] = {0}; // List of all upstream devices

uint8_t localControlPriorities[NUM_LOCAL_CONTROLS] = {0}; // Priority settings

housekeeping_hdr_t *hdr_in; // Pointer to incoming header

const int thermalPin = 38;

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
}

/*******************************************************************************
 * Packet handling functions
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
    handleTestMode(data, len);
    return 2;
  }
  case eResistanceCh3:
  {
    return EBADLEN;
  }
  case eResistanceCh6:
  {
    return EBADLEN;
  }
  case eResistanceCh9:
  {
    return EBADLEN;
  }
  case eResistanceCh12:
  {
    return EBADLEN;
  }
  case eResistanceCh16:
  {
    return EBADLEN;
  }
  case eResistanceCh20:
  {
    return EBADLEN;
  }
  case eTempCh3:
  {
    return EBADLEN;
  }
  case eTempCh6:
  {
    return EBADLEN;
  }
  case eTempCh9:
  {
    return EBADLEN;
  }
  case eTempCh12:
  {
    return EBADLEN;
  }
  case eTempCh16:
  {
    return EBADLEN;
  }
  case eTempCh20:
  {
    return EBADLEN;
  }
  case eFlow1:
  {
    return EBADLEN;
  }
  case eFlow2:
  {
    return EBADLEN;
  }
  case eTempProbe1:
  {
    return EBADLEN;
  }
  case eTempProbe2:
  {
    return EBADLEN;
  }
  case eTempProbe3:
  {
    return EBADLEN;
  }
  case eTempProbe4:
  {
    return EBADLEN;
  }
  case eTempProbe5:
  {
    return EBADLEN;
  }
  case eTempProbe6:
  {
    return EBADLEN;
  }
  case eTempProbe7:
  {
    return EBADLEN;
  }
  case eTempProbe8:
  {
    return EBADLEN;
  }
  case eTempProbe9:
  {
    return EBADLEN;
  }
  case eTempProbe10:
  {
    return EBADLEN;
  }
  case ePressure:
  {
    return EBADLEN;
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
  /* System functions */
  case ePingPong:
  {
    return 0;
  }
  case eSetPriority:
  {
    return EBADLEN;
  }

  /* Resistance measurements */
  case eResistanceCh3:
  {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 3);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead);
  }
  case eResistanceCh6:
  {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 6);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead);
  }
  case eResistanceCh9:
  {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 9);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead);
  }
  case eResistanceCh12:
  {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 12);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead);
  }
  case eResistanceCh16:
  {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 16);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead);
  }
  case eResistanceCh20:
  {
    uint32_t ResRead = returnResistance(CHIP_SELECT, 20);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    return sizeof(ResRead);
  }

  /* Temperature measurements */
  case eTempCh3:
  {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 3);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempCh6:
  {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 6);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempCh9:
  {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 9);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempCh12:
  {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 12);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempCh16:
  {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 16);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempCh20:
  {
    uint32_t TempRead = returnTemperature(CHIP_SELECT, 20);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }

  /* Flow readings */
  case eFlow1:
  {
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
  case eFlow2:
  {
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
  case eTempProbe1:
  {
    uint32_t TempRead = tempSensorVal(1, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempProbe2:
  {
    uint32_t TempRead = tempSensorVal(2, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempProbe3:
  {
    uint32_t TempRead = tempSensorVal(3, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempProbe4:
  {
    uint32_t TempRead = tempSensorVal(4, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempProbe5:
  {
    uint32_t TempRead = tempSensorVal(5, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempProbe6:
  {
    uint32_t TempRead = tempSensorVal(6, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempProbe7:
  {
    uint32_t TempRead = tempSensorVal(7, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempProbe8:
  {
    uint32_t TempRead = tempSensorVal(8, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempProbe9:
  {
    uint32_t TempRead = tempSensorVal(9, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }
  case eTempProbe10:
  {
    uint32_t TempRead = tempSensorVal(10, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    return sizeof(TempRead);
  }

  /* Pressure Readings */
  case ePressure:
  {
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
