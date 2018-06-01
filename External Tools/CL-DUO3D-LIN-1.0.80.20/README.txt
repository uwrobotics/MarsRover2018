Ubuntu 14.04.03 x64

Pre-requisites:

1) Install the following pre-requisites:
	>sudo apt-get install qt5-default

2) You must load and install the DUO kernel module. For instructions take a look at DUODriver/ReadMe.txt file.

3) Once the driver is loaded, plug in DUO device and verify that the node 'duo0' appears in /dev directory.

4) Use DUO with the DUODashboard and DUOCalibration applications.

Development:

0) Install dev tools:
	>sudo apt-get install build-essential gdb cmake

1) Install OpenCV:
	>sudo apt-get install libopencv-dev

2) Install Qt5:
	>sudo apt-get install qt5-default qtcreator

3) Build the DUOSDK code samples
