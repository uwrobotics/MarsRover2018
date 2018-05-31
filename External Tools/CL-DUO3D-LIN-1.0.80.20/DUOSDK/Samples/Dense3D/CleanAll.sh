#bin/bash
find . -name "CMakeFiles" -type d | xargs /bin/rm -rf -v
find . -name "cmake_install.cmake" -or -name "CMakeCache.txt" -type f | xargs /bin/rm -f -v
find . -name "Makefile" -or -name "Makefile.Debug" -or -name "Makefile.Release" -type f | xargs /bin/rm -f -v
find . -name ".DS_Store" -type f | xargs /bin/rm -f -v
find bin/. -name "Sample-*" -type f | xargs /bin/rm -f -v

find . -name "Release" -type d | xargs /bin/rm -rf -v
find . -name "Win32" -type d | xargs /bin/rm -rf -v
find . -name "*.dir" -type d | xargs /bin/rm -rf -v
find . -name "*.sln" -type f | xargs /bin/rm -f -v
find . -name "*.vcxproj*" -type f | xargs /bin/rm -f -v
