///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#include "Sample.h"

#define WIDTH	320
#define HEIGHT	240
#define FPS		60

DUOLEDSeq ledSequence[] = 
{
	{ 100, 100, 100, },
	{   0,   0,   0, },
};

int main(int argc, char* argv[])
{
	printf("DUOLib Version:       v%s\n", GetDUOLibVersion());

	// Open DUO camera and start capturing
	if(!OpenDUOCamera(WIDTH, HEIGHT, FPS))
	{
		printf("Could not open DUO camera\n");
		return 0;
	}
	printf("\nHit <ESC> to exit.\n");
	
	// Set exposure and LED brightness
	SetGain(0);
	SetExposure(50);

	// Set the LED sequence
	SetDUOLedPWMSeq(_duo, ledSequence, sizeof(ledSequence)/sizeof(DUOLEDSeq));

	// Create image headers for left, right and background frames
	Mat left, right;
	Mat leftBg, rightBg;

	// Run capture loop until <Esc> key is pressed
	while((cvWaitKey(1) & 0xff) != 27)
	{
		// Capture DUO frame
		PDUOFrame pFrameData = GetDUOFrame();
		if(pFrameData == NULL) continue;

		if(pFrameData->ledSeqTag == 0)
		{
			// LEDs are on
			left = Mat(Size(WIDTH, HEIGHT), CV_8UC1, pFrameData->leftData);
			right = Mat(Size(WIDTH, HEIGHT), CV_8UC1, pFrameData->rightData);
		}
		else
		{
			// LEDs are off
			leftBg = Mat(Size(WIDTH, HEIGHT), CV_8UC1, pFrameData->leftData);
			rightBg = Mat(Size(WIDTH, HEIGHT), CV_8UC1, pFrameData->rightData);
		}

		// Display background subtracted images
		if(!left.empty() && !leftBg.empty() && 
		   !right.empty() && !rightBg.empty())
		{
			imshow("Left", left - leftBg);
			imshow("Right", right - rightBg);
		}
	}
	destroyAllWindows();

	// Close DUO camera
	CloseDUOCamera();
	return 0;
}
