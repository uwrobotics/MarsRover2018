///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#ifndef SAMPLE_H
#define SAMPLE_H

// Include some generic header files
#if defined(WIN32)
	#include <SDKDDKVer.h>
	#include <stdio.h>
	#include <conio.h>
#elif defined(__linux__) || defined(__APPLE__)
	#include <stdio.h>
	#include <termios.h>
	static struct termios _old, _new;
	/* Initialize new terminal i/o settings */
	void initTermios(int echo) 
	{
	  tcgetattr(0, &_old); /* grab old terminal i/o settings */
	  _new = _old; /* make new settings same as old settings */
	  _new.c_lflag &= ~ICANON; /* disable buffered i/o */
	  _new.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
	  tcsetattr(0, TCSANOW, &_new); /* use these new terminal i/o settings now */
	}
	/* Restore old terminal i/o settings */
	void resetTermios(void) 
	{
	  tcsetattr(0, TCSANOW, &_old);
	}
	/* Read 1 character - echo defines echo mode */
	char getch_(int echo) 
	{
	  char ch;
	  initTermios(echo);
	  ch = getchar();
	  resetTermios();
	  return ch;
	}
	/* Read 1 character without echo */
	char _getch(void) 
	{
	  return getch_(0);
	}
#endif

// Include DUO API header file
#include <DUOLib.h>

#endif // SAMPLE_H
