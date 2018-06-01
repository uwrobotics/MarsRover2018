------------------------------------
DUO SDK Compilation
------------------------------------
Samples Overview - OpenCV
------------------------------------

This project demonstrate simple usage of DUO with OpenCV (C++) and CMake.

------------------------------------
Compilation Instructions
------------------------------------

Requirements:

	- Visual Studio 2012
	- Download and install CMake and add to the PATH
	- OpenCV 2.4.7.2+
	
	------------------------------------
	Building Samples
	------------------------------------

	All Platforms: 

	1) Double-click the BuildAll scripts to compile
	1) Samples will be generated in the bin folder
	3) Plug in your DUO Device and run the samples

	Windows:

	1) Install Visual Studio and CMake
	2) Download and extract OpenCV 2.4.7.2 into C:\OpenCV\2.4.7.2
	3) BuildAll-x86.cmd or BuildAll-x64.cmd
	4) Run the generated exe files in the bin folder
	
	Linux:

	1) Install build-essential, CMake, libgtk2.0-dev	
	2) Download and build OpenCV 2.4.7 from source
	3) $ sh ./BuildAll.sh

	OSX:
	
	1) Install XCode and CMake
	2) Download and build OpenCV 2.4.7 from source
	3) $ sh ./BuildAll.sh
	
------------------------------------
References
------------------------------------
http://duo3d.com/docs/articles/api
http://duo3d.com/docs/articles/sdk
http://duo3d.com/docs/articles/samples
http://duo3d.com/docs/articles/samples-opencv
------------------------------------