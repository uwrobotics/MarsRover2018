///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#ifndef SAMPLE_H
#define SAMPLE_H

#include <queue>
#include <mutex>

// Include DUO API header file
#include <DUOLib.h>
// Include Dense3D API header file
#include <Dense3DMT.h>

#include <opencv2/opencv.hpp>
using namespace cv;

typedef struct
{
    Mat leftImg, rightImg;
    Mat disparity;
    Mat depth;
}D3DFrame;

class Dense3DFrameQueue 
{
	std::mutex mutex;
	std::queue<D3DFrame> queue;
public:
	void push(D3DFrame frame)
	{
		mutex.lock();
		queue.push(frame);
		mutex.unlock();
	}
	bool pop(D3DFrame &frame)
	{
        if(queue.size() == 0)
            return false;
        mutex.lock();
        frame = queue.front();
        queue.pop();
        mutex.unlock();
        return true;
	}
};

#endif // SAMPLE_H
