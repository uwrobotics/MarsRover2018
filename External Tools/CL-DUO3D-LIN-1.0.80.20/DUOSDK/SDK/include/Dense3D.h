//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This library is part of Dense3D SDK that allows the use of Dense3D devices in your own applications
//
// For updates and file downloads go to: http://duo3d.com/
//
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _DENSE3D_H
#define _DENSE3D_H

#include <stdint.h>

#ifdef WIN32
    #ifdef DENSE3D_DLL
        #define API_FUNCTION(type)	__declspec(dllexport) type __cdecl
    #else
        #define API_FUNCTION(type)	__declspec(dllimport) type __cdecl
    #endif
#else
    #define API_FUNCTION(type)	 __attribute__((visibility("default"))) type
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Dense3D C API
extern "C" {

// Dense3D instance
typedef void *Dense3DInstance;

typedef struct  
{
    float x;
    float y;
    float z;
}Dense3DDepth, *PDense3DDepth;

// Dense3D error codes
enum Dense3DErrorCode
{
    DENSE3D_NO_ERROR,
    DENSE3D_INVALID_DENSE3D_INSTANCE,
    DENSE3D_ERROR_CREATING_DENSE3D_INSTANCE,
    DENSE3D_INVALID_LICENSE,
    DENSE3D_INVALID_PARAMETER,
    DENSE3D_INVALD_IMAGE_POINTER,
    DENSE3D_INVALD_DEPTH_DATA_POINTER,
    DENSE3D_INVALID_IMAGE_SIZE,
    DENSE3D_INVALD_PLY_FILE_NAME,
    DENSE3D_ERROR_EXPORTING_PLY_FILE
};

// Dense3D error code
API_FUNCTION(Dense3DErrorCode) Dense3DGetErrorCode();

// Dense3D library version
API_FUNCTION(char*) Dense3DGetLibVersion();

// Dense3D initialization
API_FUNCTION(bool) Dense3DOpen(Dense3DInstance *dense3D);
API_FUNCTION(bool) Dense3DClose(Dense3DInstance dense3D);

// Dense3D processing
API_FUNCTION(bool) Dense3DGetDepth(Dense3DInstance dense3D, uint8_t *leftImage, uint8_t *rightImage,
                                   float *disparityData, PDense3DDepth depthData);
// Dense3D data access
API_FUNCTION(bool) Dense3DSavePLY(Dense3DInstance dense3D, char *plyFile);

// Dense3D parameters

// Get Dense3D image size
API_FUNCTION(bool) GetDense3DImageSize(Dense3DInstance dense3D, uint32_t *w, uint32_t *h);
// Get Dense3D image scale [0, 3] - [No Scale, Scale X, Scale Y, Scale X&Y]
API_FUNCTION(bool) GetDense3DScale(Dense3DInstance dense3D, uint32_t *val);
// Get Dense3D processing mode [0, 3] - [BM, SGBM, BM_HQ, SGBM_HQ]
API_FUNCTION(bool) GetDense3DMode(Dense3DInstance dense3D, uint32_t *val);
// Get Dense3D pre-filter cap [1, 63]
API_FUNCTION(bool) GetDense3DPreFilterCap(Dense3DInstance dense3D, uint32_t *val);
// Get Dense3D number of disparities [2, 16] resulting in [32, 256] disparities
API_FUNCTION(bool) GetDense3DNumDisparities(Dense3DInstance dense3D, uint32_t *val);
// Get Dense3D SAD window size [2, 10] resulting in [5, 21] window sizes
API_FUNCTION(bool) GetDense3DSADWindowSize(Dense3DInstance dense3D, uint32_t *val);
// Get Dense3D uniqueness ratio [1, 100]
API_FUNCTION(bool) GetDense3DUniqunessRatio(Dense3DInstance dense3D, uint32_t *val);
// Get Dense3D speckle window size [0, 256]
API_FUNCTION(bool) GetDense3DSpeckleWindowSize(Dense3DInstance dense3D, uint32_t *val);
// Get Dense3D speckle range [0, 32]
API_FUNCTION(bool) GetDense3DSpeckleRange(Dense3DInstance dense3D, uint32_t *val);

// Set Dense3D license
API_FUNCTION(bool) SetDense3DLicense(Dense3DInstance dense3D, const char *license);
// Set Dense3D image size
API_FUNCTION(bool) SetDense3DImageSize(Dense3DInstance dense3D, uint32_t w, uint32_t h);
// Set Dense3D stereo camera parameters from DUOLib
API_FUNCTION(bool) SetDense3DCalibration(Dense3DInstance dense3D, void *stereo);
// Set Dense3D image scale [0, 3] - [No Scale, Scale X, Scale Y, Scale X&Y]
API_FUNCTION(bool) SetDense3DScale(Dense3DInstance dense3D, uint32_t val);
// Set Dense3D processing mode [0, 3] - [BM, SGBM, BM_HQ, SGBM_HQ]
API_FUNCTION(bool) SetDense3DMode(Dense3DInstance dense3D, uint32_t val);
// Set Dense3D pre-filter cap [1, 63]
API_FUNCTION(bool) SetDense3DPreFilterCap(Dense3DInstance dense3D, uint32_t val);
// Set Dense3D number of disparities [2, 16] resulting in [32, 256] disparities
API_FUNCTION(bool) SetDense3DNumDisparities(Dense3DInstance dense3D, uint32_t val);
// Set Dense3D SAD window size [2, 10] resulting in [5, 21] window sizes
API_FUNCTION(bool) SetDense3DSADWindowSize(Dense3DInstance dense3D, uint32_t val);
// Set Dense3D uniqueness ratio [1, 100]
API_FUNCTION(bool) SetDense3DUniquenessRatio(Dense3DInstance dense3D, uint32_t val);
// Set Dense3D speckle window size [0, 256]
API_FUNCTION(bool) SetDense3DSpeckleWindowSize(Dense3DInstance dense3D, uint32_t val);
// Set Dense3D speckle range [0, 32]
API_FUNCTION(bool) SetDense3DSpeckleRange(Dense3DInstance dense3D, uint32_t val);

} // extern "C"

#endif // _DENSE3D_H
