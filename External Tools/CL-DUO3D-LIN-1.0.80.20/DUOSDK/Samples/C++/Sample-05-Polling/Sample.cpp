///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#include "Sample.h"

int main(int argc, char* argv[])
{
	printf("DUOLib Version:       v%s\n", GetDUOLibVersion());

	// Open DUO camera and start capturing
	if(!OpenDUOCamera(320, 240, 15))
	{
		printf("Could not open DUO camera\n");
		return 0;
	}
	
	int duoFrameNum = 0;

	// Run capture loop until <Esc> key is pressed
	while(!_kbhit() || _getch() != 27)
	{
		// Capture DUO frame
		PDUOFrame pFrameData = GetDUOFrame();

		// Process DUO frame
		if(pFrameData != NULL)
		{
			printf("DUO Frame #%d\n", duoFrameNum++);
			printf("  Timestamp:          %10.1f ms\n", pFrameData->timeStamp/10.0f);
			printf("  Frame Size:         %dx%d\n", pFrameData->width, pFrameData->height);
			printf("  Left Frame Buffer:  %p\n", pFrameData->leftData);
			printf("  Right Frame Buffer: %p\n", pFrameData->rightData);
			printf("------------------------------------------------------\n");
		}
	}

	// Close DUO camera
	CloseDUOCamera();
	return 0;
}
