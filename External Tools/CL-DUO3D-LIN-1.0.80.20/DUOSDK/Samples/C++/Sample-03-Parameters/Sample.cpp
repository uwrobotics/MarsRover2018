///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#include "Sample.h"

int main(int argc, char* argv[])
{
	printf("DUOLib Version:       v%s\n", GetDUOLibVersion());

	DUOResolutionInfo ri;
	// Select 320x240 resolution with 2x2 binning capturing at 10FPS
	if(EnumerateDUOResolutions(&ri, 1, 320, 240, DUO_BIN_HORIZONTAL2+DUO_BIN_VERTICAL2, 30))
	{
		DUOInstance duo;
		// Open DUO
		if(OpenDUO(&duo))
		{
			char tmp[260];
			// Get some DUO parameter values
			GetDUODeviceName(duo, tmp);
			printf("DUO Device Name:      '%s'\n", tmp);

			GetDUODeviceName(duo, tmp);
			printf("DUO Serial Number:    %s\n", tmp);

			GetDUOFirmwareVersion(duo, tmp);
			printf("DUO Firmware Version: v%s\n", tmp);

			GetDUOFirmwareBuild(duo, tmp);
			printf("DUO Firmware Build:   %s\n", tmp);

			printf("\nHit any key to start capturing");
			_getch();
			printf("\n");

			// Set selected resolution
			SetDUOResolutionInfo(duo, ri);

			float ledPwm = 30;
			// Set the LED brightness value in %
			SetDUOLedPWM(duo, ledPwm);
			// Start capture (no callback function)
			if(StartDUO(duo, NULL, NULL))
			{
				printf("Use '+' to increase the brightness of the LEDs\n");
				printf("Use '-' to decrease the brightness of the LEDs\n");
				printf("Use '<Esc>' to exit the program\n\n");
				int ch;
				do
				{
					ch = _getch();
					if(ch == '-') ledPwm > 0 ? ledPwm-- : 0;
					else if(ch == '+') ledPwm < 100 ? ledPwm++ : 0;
					printf("LED: %3d%%\r", (int)ledPwm);
					SetDUOLedPWM(duo,ledPwm);
				}while(ch != 27);
				// Stop capture
				StopDUO(duo);
				// Close DUO
				CloseDUO(duo);
			}
		}
	}
	return 0;
}
