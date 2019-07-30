/*
 * MagResponse.cpp
 *
 * Defines a set of functions to act as responses to received commands for the
 * magnet housekeeping board
 *
 */

/*******************************************************************************
* Defines
*******************************************************************************/
#include <MagResponse.h>

/* Buffer for outgoing data */
uint8_t outgoingData [255] = {0};

const int thermalPin = 38; // Not sure where other constants are stored, this can be moved -L

/*******************************************************************************
* Functions
*******************************************************************************/
void whatToDoIfResistance(housekeeping_hdr_t * hdr_out, uint8_t device)
{
	/* Variables for performing a resistance reading + storing it in an array of uint8_t */
	uint32_t ResRead;
	uint32_t * tmp;

	hdr_out->dst = eSFC;
	hdr_out->cmd = device;
	hdr_out->len = 4;

	switch (device)
	{
	case eResistanceCh3:
	{
		ResRead = returnResistance(CHIP_SELECT, 3);
		break;
	}
	case eResistanceCh6:
	{
		ResRead = returnResistance(CHIP_SELECT, 6);
		break;
	}
	case eResistanceCh9:
	{
		ResRead = returnResistance(CHIP_SELECT, 9);
		break;
	}
	case eResistanceCh12:
	{
		ResRead = returnResistance(CHIP_SELECT, 12);
		break;
	}
	case eResistanceCh16:
	{
		ResRead = returnResistance(CHIP_SELECT, 16);
		break;
	}
	case eResistanceCh20:
	{
		ResRead = returnResistance(CHIP_SELECT, 20);
		break;
	}
	}

	tmp = &ResRead;

	/* Fills outgoing data buffer */
	for(int i=0; i < hdr_out->len; i++)
	{
		outgoingData[i] = *tmp;
		*tmp = *tmp>>8;
	}

	matchData(hdr_out);
}

void whatToDoIfTemp(housekeeping_hdr_t * hdr_out, uint8_t device)
{
	/* Variables for performing a resistance reading + storing it in an array of uint8_t */
	uint32_t TempRead;
	uint32_t * tmp;

	hdr_out->dst = eSFC;
	hdr_out->cmd = device;
	hdr_out->len = 4;

	switch (device)
	{
	case eTempCh3:
	{
		TempRead = returnTemperature(CHIP_SELECT, 3);
		break;
	}
	case eTempCh6:
	{
		TempRead = returnTemperature(CHIP_SELECT, 6);
		break;
	}
	case eTempCh9:
	{
		TempRead = returnTemperature(CHIP_SELECT, 9);
		break;
	}
	case eTempCh12:
	{
		TempRead = returnTemperature(CHIP_SELECT, 12);
		break;
	}
	case eTempCh16:
	{
		TempRead = returnTemperature(CHIP_SELECT, 16);
		break;
	}
	case eTempCh20:
	{
		TempRead = returnTemperature(CHIP_SELECT, 20);
		break;
	}
	}

	tmp = &TempRead;

	/* Fills outgoing data buffer */
	for(int i=0; i < hdr_out->len; i++)
	{
		outgoingData[i] = *tmp;
		*tmp = *tmp>>8;
	}

	matchData(hdr_out);
}

bool causeWeNeedTwoPackets;

bool whatToDoIfFlow(housekeeping_hdr_t * hdr_out, uint8_t device)
{
	uint32_t FlowRead;
	uint8_t * tmp;

	hdr_out->dst = eSFC;
	hdr_out->cmd = device;
	hdr_out->len = 208;

	// Used for readout
	double gasdata[4];
	char gastype[100];
	char errorcode[100];

	switch (device)
	{
	case eFlow1:
	{
		FlowRead = readFlow(1, gasdata, gastype, errorcode);
		break;
	}
	case eFlow2:
	{
		FlowRead = readFlow(2, gasdata, gastype, errorcode);
		break;
	}
	}

	tmp = (uint8_t *) &gasdata[0];

	if (FlowRead)
	{
		/* Fills outgoing data buffer */
		for (int i=0; i < 256; i++)
		{
			/* Relay back pressure data */
			outgoingData[i] = *tmp;
			*tmp = *tmp >> 8;

			causeWeNeedTwoPackets = true;
		}
	}

	if (!causeWeNeedTwoPackets)
	{
		for (int i=0; i < 200; i++)
		{
			outgoingData[i] = gastype[i];
			outgoingData[100 + i] = errorcode[i];
		}

		causeWeNeedTwoPackets = false;
	}

	matchData(hdr_out);

	return causeWeNeedTwoPackets;
}
void whatToDoIfTempProbe(housekeeping_hdr_t * hdr_out, uint8_t device)
{
	/* Variables for performing a resistance reading + storing it in an array of uint8_t */
	uint32_t TempRead;
	uint32_t * tmp;

	hdr_out->dst = eSFC;
	hdr_out->cmd = device;
	hdr_out->len = 4;

	switch (device)
	{
	case eTempProbe1:
	{
		TempRead = tempSensorVal(1, thermalPin);
		break;
	}
	case eTempProbe2:
	{
		TempRead = tempSensorVal(2, thermalPin);
		break;
	}
	case eTempProbe3:
	{
		TempRead = tempSensorVal(3, thermalPin);
		break;
	}
	case eTempProbe4:
	{
		TempRead = tempSensorVal(4, thermalPin);
		break;
	}
	case eTempProbe5:
	{
		TempRead = tempSensorVal(5, thermalPin);
		break;
	}
	case eTempProbe6:
	{
		TempRead = tempSensorVal(6, thermalPin);
		break;
	}
	case eTempProbe7:
	{
		TempRead = tempSensorVal(7, thermalPin);
		break;
	}
	case eTempProbe8:
	{
		TempRead = tempSensorVal(8, thermalPin);
		break;
	}
	case eTempProbe9:
	{
		TempRead = tempSensorVal(9, thermalPin);
		break;
	}
	case eTempProbe10:
	{
		TempRead = tempSensorVal(10, thermalPin);
		break;
	}
	}

	/* Fills outgoing data buffer */
	for (int i=0; i < hdr_out->len; i++)
	{
		outgoingData[i] = *tmp;
		*tmp = *tmp>>8;
	}

	matchData(hdr_out);
}

void whatToDoIfPressure(housekeeping_hdr_t * hdr_out)
{
	uint32_t PressureRead;
	uint8_t * tmp1;
	uint8_t * tmp2;

	hdr_out->dst = eSFC;
	hdr_out->cmd = ePressure;
	hdr_out->len = 117;

	double pressureValue;
	double temperature;
	char typeOfPressure;
	char errorcode[100];

	readPressureSensor(&pressureValue, &temperature, &typeOfPressure, errorcode);

	tmp1 = (uint8_t *) &pressureValue;
	tmp2 = (uint8_t *) &temperature;

	for (int i=0; i < 100; i++)
	{
		outgoingData[17 + i] = errorcode[i];

		if (i < 8)
		{
			outgoingData[i] = *tmp1;
			*tmp1 = *tmp1>>8;

			outgoingData[8 + i] = *tmp2;
			*tmp2 = *tmp2>>8;
		}
	}

	outgoingData[16] = typeOfPressure;

	matchData(hdr_out);
}


/*******************************************************************************
* Functions
*******************************************************************************/
/* Function flow:
 * --Fills outgoing header w/protocol standard
 * --Board response to ping pong command return
 *
 * Function params:
 * hdr_out:		Pointer to outgoing packet header
 */
void whatToDoIfPingPong(housekeeping_hdr_t * hdr_out)
{
	/* Create the header */
	hdr_out->dst = eSFC; // Intended destination of packet
	hdr_out->cmd = ePingPong; // Command for what do to with packet
	hdr_out->len = 0; // Size of data after the header
}

/* Function flow:
 * --Finds the priority settings from the incoming packet
 * --Changes the intended command's priority
 * --Fills outgoing packet header w/protocol standard
 * --Sets outgoing data bytes to the new priority settings for SFC confirmation
 *      --Not part of HSK protocol
 *
 * Function params:
 * hdr_prio:	Pointer to the priority header attached to the incoming packet
 * hdr_out:		Pointer to outgoing packet header
 * comPriorList:	List of all command priorities
 *
 *  */
void whatToDoIfSetPriority(housekeeping_prio_t * hdr_prio, housekeeping_hdr_t * hdr_out,
                           uint8_t * comPriorList)
{
	comPriorList[(uint8_t) hdr_prio->command] = (uint8_t) hdr_prio->prio_type;

	hdr_out->dst = eSFC;
	hdr_out->cmd = eSetPriority;
	hdr_out->len = 2;

	outgoingData[0] = hdr_prio->command;
	outgoingData[1] = hdr_prio->prio_type;

	matchData(hdr_out);
}

void whatToDoIfTestMode(uint16_t* numTestPackets_p, housekeeping_hdr_t* hdr_out)
{
	// set the output header based on what PSA said
	// the length can be 2 to send back the number of test packets that still need to be sent.
	// the incoming packet has two extra bytes, so its length was 2 and that was just at first to decide how many packets to send.
	hdr_out->dst = eSFC;
	hdr_out->cmd = eTestMode;
	hdr_out->len = 2;
	/* Fill outgoing data with the numTestPackets*/
	uint8_t tmpVal = *numTestPackets_p;
	uint8_t * tmp = &tmpVal;
	for (int i = 0; i < hdr_out->len; i++)
	{
		outgoingData[i] = *tmp;
		*tmp = *tmp >> 8;
	}
	matchData(hdr_out);
}

/* Function flow:
 * --Checks if this device has any commands of a certain priority
 * --Checks if the incoming destination is eBroadcast
 * --Overall, makes sure the device responds when it has no commands of a certain
 *   priority only if the destination is not eBroadcast
 *
 * Function params:
 * hdr_in:		Pointer to the incoming packet header
 * hdr_out:		Pointer to the outgoing packet header
 * numSends:	Number of commands this device has of a certain priority
 *
 *
 */
bool checkThatPriority(housekeeping_hdr_t * hdr_in, housekeeping_hdr_t * hdr_out,
                       uint8_t numSends)
{
	if (hdr_in->dst != eBroadcast && numSends == 0)
	{
		hdr_out->dst = eSFC;
		hdr_out->cmd = hdr_in->cmd;
		hdr_out->len = 0;
		return true;
	}
	else return false;
}

/* These functions are called if the device has detected an error.
 * They follow the HSK error protocol by filling the outgoing header &
 * outgoing error header.
 *
 *  */
void error_badDest(housekeeping_hdr_t * hdr_in, housekeeping_hdr_t * hdr_out
                   , housekeeping_err_t * hdr_err)
{
	/* Outgoing housekeeping header */
	hdr_out->dst = eSFC;
	hdr_out->cmd = eError;
	hdr_out->len = 4;
	/* Outgoing error diagnostic */
	hdr_err->src = (uint8_t) hdr_in->src;
	hdr_err->cmd = (uint8_t) hdr_in->cmd;
	hdr_err->error = (uint8_t) EBADDEST;
}

void error_badLength(housekeeping_hdr_t * hdr_in, housekeeping_hdr_t * hdr_out
                     , housekeeping_err_t * hdr_err)
{
	/* Outgoing housekeeping header */
	hdr_out->dst = eSFC;
	hdr_out->cmd = eError;
	hdr_out->len = 4;
	/* Outgoing error diagnostic */
	hdr_err->src = (uint8_t) hdr_in->src;
	hdr_err->cmd = (uint8_t) hdr_in->cmd;
	hdr_err->error = (uint8_t) EBADLEN;
}

void error_badCommand(housekeeping_hdr_t * hdr_in, housekeeping_hdr_t * hdr_out
                      , housekeeping_err_t * hdr_err)
{
	/* Outgoing housekeeping header */
	hdr_out->dst = eSFC;
	hdr_out->cmd = eError;
	hdr_out->len = 4;
	/* Outgoing error diagnostic */
	hdr_err->src = (uint8_t) hdr_in->src;
	hdr_err->cmd = (uint8_t) hdr_in->cmd;
	hdr_err->error = (uint8_t) EBADCOMMAND;
}

void error_badArgs(housekeeping_hdr_t * hdr_in, housekeeping_hdr_t * hdr_out
                   , housekeeping_err_t * hdr_err)
{
	/* Outgoing housekeeping header */
	hdr_out->dst = eSFC;
	hdr_out->cmd = eError;
	hdr_out->len = 4;
	/* Outgoing error diagnostic */
	hdr_err->src = (uint8_t) hdr_in->src;
	hdr_err->cmd = (uint8_t) hdr_in->cmd;
	hdr_err->error = (uint8_t) EBADARGS;
}


/* Function flow:
 * --Dereferences the outgoing packet data location and fills it with the data
 *   from a function which was executed prior
 *
 * Function params:
 * hdr_out:		Pointer to the outgoing packet header
 *
 */
void matchData(housekeeping_hdr_t * hdr_out)
{
	/* Each command response sets a data length */
	for (int i = 0; i < hdr_out->len; i++)
	{
		/* Dereference the outgoing packet's data location & fills the spot */
		*((uint8_t *)hdr_out+4+i) = outgoingData[i];
	}
}
