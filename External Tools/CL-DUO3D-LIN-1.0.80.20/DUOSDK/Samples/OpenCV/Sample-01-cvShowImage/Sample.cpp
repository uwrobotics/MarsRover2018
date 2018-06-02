///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#include "Sample.h"

#define WIDTH	320
#define HEIGHT	240
#define FPS		30

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
	
	// Create OpenCV windows
	cvNamedWindow("Left");
	cvNamedWindow("Right");

	// Create image headers for left & right frames
	IplImage *left, *right;

	// Set exposure and LED brightness
	SetGain(0);
	SetExposure(50);
	SetLed(25);

	bool first = true;
	// Run capture loop until <Esc> key is pressed
	while((cvWaitKey(1) & 0xff) != 27)
	{
		// Capture DUO frame
		PDUOFrame pFrameData = GetDUOFrame();
		if(pFrameData == NULL) continue;

		// Set the image data
		if(first)
		{
			first = false;
			left = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
			right = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
		}
		left->imageData = (char*)pFrameData->leftData;
		right->imageData = (char*)pFrameData->rightData;

		// Process images here (optional)

		// Display images
		cvShowImage("Left", left);
		cvShowImage("Right", right);
	}
	cvDestroyAllWindows();

	// Release image headers
	cvReleaseImageHeader(&left);
	cvReleaseImageHeader(&right);

	// Close DUO camera
	CloseDUOCamera();
	return 0;
}
