///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#include "Sample.h"

DUOLEDSeq ledSequence[] = {
	{ 50,  0,  0, },
	{  0, 50,  0, },
	{  0,  0, 50, },
	{  0, 50,  0, },
};

void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
	printf("DUO Timestamp: %10.1f ms\n", pFrameData->timeStamp/10.0f);
	printf("  LEDs: [%3d%%][%3d%%][%3d%%]\n", 
			ledSequence[pFrameData->ledSeqTag].ledPwmValue[0],
			ledSequence[pFrameData->ledSeqTag].ledPwmValue[1],
			ledSequence[pFrameData->ledSeqTag].ledPwmValue[2]);
	printf("------------------------------------------------------\n");
}

int main(int argc, char* argv[])
{
	printf("DUOLib Version:       v%s\n", GetDUOLibVersion());

	DUOResolutionInfo ri;
	// Select 320x240 resolution with 2x2 binning capturing at 10FPS
	if(EnumerateDUOResolutions(&ri, 1, 320, 240, DUO_BIN_HORIZONTAL2+DUO_BIN_VERTICAL2, 10))
	{
		DUOInstance duo;
		// Open DUO
		if(OpenDUO(&duo))
		{
			char tmp[260];
			// Get some DUO parameter values
			GetDUODeviceName(duo, tmp);
			printf("DUO Device Name: '%s'\n", tmp);

			GetDUODeviceName(duo, tmp);
			printf("DUO Serial Number: %s\n", tmp);

			GetDUOFirmwareVersion(duo, tmp);
			printf("DUO Firmware Version: v%s\n", tmp);

			GetDUOFirmwareBuild(duo, tmp);
			printf("DUO Firmware Build:   %s\n", tmp);

			printf("\nHit any key to start capturing");
			_getch();
			printf("\n");

			// Set selected resolution
			SetDUOResolutionInfo(duo, ri);

			// Set the LED sequence
			SetDUOLedPWMSeq(duo, ledSequence, sizeof(ledSequence)/sizeof(DUOLEDSeq));

			// Start capture (no callback function)
			if(StartDUO(duo, DUOCallback, NULL))
			{
				_getch();
				// Stop capture
				StopDUO(duo);
				// Close DUO
				CloseDUO(duo);
			}
		}
	}
	return 0;
}
