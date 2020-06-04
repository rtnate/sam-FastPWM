#ifndef _FAST_PWM_H_
#define _FAST_PWM_H_


#include <Arduino.h>
#include <sam.h>

#if defined(SAMD21_SERIES)
#include "SAMD/FastPWMSamD.h"
typedef SamD::FastPWMPin FastPWMPin;
// #elif defined(__SAMD51__)
#else
#error "Platform Not Supported"
#endif
#endif