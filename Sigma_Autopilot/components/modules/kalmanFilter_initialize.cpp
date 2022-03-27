// File: kalmanFilter_initialize.cpp

// Include Files
#include "rt_nonfinite.h"
#include "kalmanFilter.h"
#include "kalmanFilter_initialize.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void kalmanFilter_initialize()
{
  rt_InitInfAndNaN(8U);
}

//
// File trailer for kalmanFilter_initialize.cpp
//
// [EOF]
//
