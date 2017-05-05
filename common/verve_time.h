#ifndef VERVE_TIME_H_
#define VERVE_TIME_H_
#include "simplelinklibrary.h"

 //*****************************************************************************
 // Assert Define
 //*****************************************************************************
#define ASSERT_ON_ERROR(line_number, error_code) \
            {\
                /* Handling the error-codes is specific to the application */ \
                if (error_code < 0) return error_code; \
                /* else, continue w/ execution */ \
            }

//*****************************************************************************
// Date and Time Global
//*****************************************************************************
SlDateTime_t dateTime;
long GetCurrentTime();

#endif
