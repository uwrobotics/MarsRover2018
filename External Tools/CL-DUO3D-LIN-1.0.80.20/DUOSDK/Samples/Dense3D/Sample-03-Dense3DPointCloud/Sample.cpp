///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#include "Sample.h"
using namespace cl;

#define WIDTH	320
#define HEIGHT	240
#define FPS     30

int main(int argc, char* argv[])
{
	printf("Dense3D Point Cloud Program\n");

	Dense3DMTInstance dense3d;
	if(!Dense3DOpen(&dense3d))
	{
		printf("Could not open Dense3DMT\n");
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

	if(!SetDense3DLicense(dense3d, "XXXXX-XXXXX-XXXXX-XXXXX-XXXXX")) // <-- Put your Dense3D license
	{
		printf("Invalid or missing Dense3D license. To get your license visit https://duo3d.com/account\n");
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}

	if(!SetDense3DImageInfo(dense3d, WIDTH, HEIGHT, FPS))
	{
		printf("SetDense3DImageInfo error\n");
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}

	Dense3DParams params;
	params.scale = 3;
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

	if(!Dense3DStart(dense3d, [](const PDense3DFrame pFrameData, void *pUserData)
			{
		                D3DFrame frame;
		                Size frameSize(pFrameData->duoFrame->width, pFrameData->duoFrame->height);
		                frame.leftImg = Mat(frameSize, CV_8U, pFrameData->duoFrame->leftData);
		                frame.depth = Mat(frameSize, CV_32FC3, pFrameData->depthData);
		                ((Dense3DFrameQueue*)pUserData)->push(frame);
			}, &d3dq))
	{
		printf("Dense3DStart error\n");
		return 1;
	}

	// Set exposure, LED brightness and camera orientation
	SetDUOExposure(duo, 85);
	SetDUOLedPWM(duo, 28);
	SetDUOVFlip(duo, false);

	DUOResolutionInfo ri;
	GetDUOResolutionInfo(duo, &ri);
	double fov[4];
	GetDUORectifiedFOV(duo, fov);

	CloudViewer viewer;
	viewer.setFov(fov[0], fov[1]);

	// Setup idle callback
	viewer.onIdle([&]()
	{
		D3DFrame d3DFrame;
		if(!d3dq.pop(d3DFrame))
			return;

		// Update point cloud
		viewer.addData(d3DFrame.leftImg, d3DFrame.depth);
	});
	viewer.onExit([&]()
	{
		Dense3DStop(dense3d);
 		Dense3DClose(dense3d);
	});

	// Run viewer loop
	viewer.run();
	return 0;
}
