cmake_minimum_required (VERSION 2.8)
project(DUOSamples)

# Get DUO_SDK install path from environment variable if set
if(DEFINED ENV{DUO_SDK})
  set(DUO_SDK $ENV{DUO_SDK} )
else()
  set(DUO_SDK ${CMAKE_CURRENT_SOURCE_DIR}/../../SDK)
endif()

include_directories(${DUO_SDK}/include)

# Detect Compiler / Arch and add libraries
if(WIN32)
  if(CMAKE_CL_64) 
    set(DUOLib_ARCH x64)
  else() 
    set(DUOLib_ARCH x86)
  endif()
  find_library(DUO_LIBRARY DUOLib HINTS ${DUO_SDK}/windows/${DUOLib_ARCH})
  find_library(DENSE3D_LIBRARY Dense3D HINTS ${DUO_SDK}/windows/${DUOLib_ARCH})
  find_library(DENSE3DMT_LIBRARY Dense3DMT HINTS ${DUO_SDK}/windows/${DUOLib_ARCH})
  # Copy DUOLib.dll
  file(TO_NATIVE_PATH ${DUO_SDK}/windows/${DUOLib_ARCH}/DUOLib.dll SRC)
  file(TO_NATIVE_PATH ${CMAKE_BINARY_DIR}/../bin/${DUOLib_ARCH}/ DST)
  execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${SRC} ${DST}) 
  # Copy Dense3D.dll
  file(TO_NATIVE_PATH ${DUO_SDK}/windows/${DUOLib_ARCH}/Dense3D.dll SRC)
  execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${SRC} ${DST}) 
  # Copy Dense3D.dll
  file(TO_NATIVE_PATH ${DUO_SDK}/windows/${DUOLib_ARCH}/Dense3DMT.dll SRC)
  execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${SRC} ${DST}) 
  
  SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY
   ${PROJECT_BINARY_DIR}/../bin/${DUOLib_ARCH}
   CACHE PATH
   "Single Directory for all Libraries"
   )

elseif(APPLE)
  set(DUOLib_ARCH x64)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
  find_library(DUO_LIBRARY DUO HINTS ${DUO_SDK}/osx/${DUOLib_ARCH})
  find_library(DENSE3D_LIBRARY Dense3D HINTS ${DUO_SDK}/osx/${DUOLib_ARCH})
  find_library(DENSE3DMT_LIBRARY Dense3DMT HINTS ${DUO_SDK}/osx/${DUOLib_ARCH})
elseif(UNIX)
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL armv7l OR CMAKE_SYSTEM_PROCESSOR STREQUAL aarch64)
    set(DUOLib_ARCH arm)
  else()
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(DUOLib_ARCH x64)
    else() 
      set(DUOLib_ARCH x86)
    endif()
  endif()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
  find_library(DUO_LIBRARY DUO HINTS ${DUO_SDK}/linux/${DUOLib_ARCH})
  find_library(DENSE3D_LIBRARY Dense3D HINTS ${DUO_SDK}/linux/${DUOLib_ARCH})
  find_library(DENSE3DMT_LIBRARY Dense3DMT HINTS ${DUO_SDK}/linux/${DUOLib_ARCH})
endif()

# Set output directory for binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin/${DUOLib_ARCH} )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../bin/${DUOLib_ARCH} )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../bin/${DUOLib_ARCH} )

add_subdirectory(Sample-01-Dense3D)
add_subdirectory(Sample-02-Dense3DMT)
add_subdirectory(Sample-03-Dense3DPointCloud)