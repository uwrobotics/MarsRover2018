///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#include "Sample.h"

#define WIDTH	320
#define HEIGHT	240
#define FPS		30

Vec3b HSV2RGB(float hue, float sat, float val)
{
	float x, y, z;

	if(hue == 1) hue = 0;
	else         hue *= 6;

	int i = static_cast<int>(floorf(hue));
	float f = hue - i;
	float p = val * (1 - sat);
	float q = val * (1 - (sat * f));
	float t = val * (1 - (sat * (1 - f)));

	switch(i)
	{
		case 0: x = val; y = t; z = p; break;
		case 1: x = q; y = val; z = p; break;
		case 2: x = p; y = val; z = t; break;
		case 3: x = p; y = q; z = val; break;
		case 4: x = t; y = p; z = val; break;
		case 5: x = val; y = p; z = q; break;
	}
	return Vec3b((uchar)(x * 255), (uchar)(y * 255), (uchar)(z * 255));
}

int main(int argc, char* argv[])
{
	// Build color lookup table for depth display
	Mat colorLut = Mat(cv::Size(256, 1), CV_8UC3);
	for(int i = 0; i < 256; i++)
		colorLut.at<Vec3b>(i) = (i==0) ? Vec3b(0, 0, 0) : HSV2RGB(i/256.0f, 1, 1);

	// Open Dense3D
	Dense3DMTInstance dense3d;
	if(!Dense3DOpen(&dense3d))
	{
		printf("Could not open Dense3DMT library\n");
		return 1;
	}

	DUOInstance duo = GetDUOInstance(dense3d);

	char tmp[260];
	GetDUODeviceName(duo,tmp);
	printf("DUO Device Name:      '%s'\n", tmp);
	GetDUOSerialNumber(duo, tmp);
	printf("DUO Serial Number:    %s\n", tmp);
	GetDUOFirmwareVersion(duo, tmp);
	printf("DUO Firmware Version: v%s\n", tmp);
	GetDUOFirmwareBuild(duo, tmp);
	printf("DUO Firmware Build:   %s\n", tmp);
	printf("DUOLib Version:       v%s\n", GetDUOLibVersion());
	printf("Dense3DMT Version:    v%s\n", Dense3DGetLibVersion());

	// Set the Dense3D license (visit https://duo3d.com/account)
	if(!SetDense3DLicense(dense3d, "XXXXX-XXXXX-XXXXX-XXXXX-XXXXX")) // <-- Put your Dense3D license
	{
		printf("Invalid or missing Dense3D license. To get your license visit https://duo3d.com/account\n");
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}
	// Set the image size
	if(!SetDense3DImageInfo(dense3d, WIDTH, HEIGHT, FPS))
	{
		printf("Invalid image size\n");
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}

	// Set Dense3D parameters
	Dense3DParams params;
	params.scale = 0;
    params.mode = 0;
    params.numDisparities = 2;
    params.sadWindowSize = 6;
    params.preFilterCap = 28;
    params.uniqenessRatio = 27;
    params.speckleWindowSize = 52;
    params.speckleRange = 14;
	if(!SetDense3Params(dense3d, params))
    {
        printf("GetDense3Params error\n");
		// Close Dense3D library
		Dense3DClose(dense3d);
        return 1;
    }

	// Queue used to receive Dense3D frames
	Dense3DFrameQueue d3dq;

	// Start DUO capture and Dense3D processing
	if(!Dense3DStart(dense3d, [](const PDense3DFrame pFrameData, void *pUserData)
					{
                        D3DFrame frame;
                        Size frameSize(pFrameData->duoFrame->width, pFrameData->duoFrame->height);
                        frame.leftImg = Mat(frameSize, CV_8U, pFrameData->duoFrame->leftData);
                        frame.rightImg = Mat(frameSize, CV_8U, pFrameData->duoFrame->rightData);
                        frame.disparity = Mat(frameSize, CV_32F, pFrameData->disparityData);
                        frame.depth = Mat(frameSize, CV_32FC3, pFrameData->depthData);
                        ((Dense3DFrameQueue*)pUserData)->push(frame);
					}, &d3dq))
	{
		printf("Dense3DStart error\n");
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}

	// Set exposure, LED brightness and camera orientation
	SetDUOExposure(duo, 85);
	SetDUOLedPWM(duo, 28);
	SetDUOVFlip(duo, true);

	// Run capture loop until <Esc> key is pressed
	while((cvWaitKey(1) & 0xff) != 27)
	{
		D3DFrame d3DFrame;
		if(!d3dq.pop(d3DFrame))
			continue;

		Mat disp8;
		d3DFrame.disparity.convertTo(disp8, CV_8UC1, 255.0/(params.numDisparities*16));
		Mat rgbBDisparity;
		cvtColor(disp8, rgbBDisparity, COLOR_GRAY2BGR);
		LUT(rgbBDisparity, colorLut, rgbBDisparity);

		// Display images
		imshow("Left Image", d3DFrame.leftImg);
		imshow("Right Image", d3DFrame.rightImg);
		imshow("Disparity Image", rgbBDisparity);
	}
	destroyAllWindows();

	Dense3DStop(dense3d);
	Dense3DClose(dense3d);
	return 0;
}
