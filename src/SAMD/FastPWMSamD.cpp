/*
*   Copyright (c) 2020 RT ElecTRonix.  All rights reserved.
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy
*   of this software and associated documentation files (the "Software"), to deal
*   in the Software without restriction, including without limitation the rights
*   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*   copies of the Software, and to permit persons to whom the Software is
*   furnished to do so, subject to the following conditions:
*   
*   The above copyright notice and this permission notice shall be included in all
*   copies or substantial portions of the Software.
*   
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*   SOFTWARE.
*/

#include "FastPWMSamD.h"
//Include Pin Peripheral
#include "wiring_private.h"

using namespace SamD;

/**
 * @brief Checks if a pin has the provided attribute
 * 
 * @param PIN_ATTRIBUTES The pin's attribute flags
 * @param ATTRIBUTE The attribute to verify
 */
#define PIN_ATTR_HAS(PIN_ATTRIBUTES, ATTRIBUTE) ((PIN_ATTRIBUTES & ATTRIBUTE) == ATTRIBUTE)

bool FastPWMPin::tcEnabled[NoTimers] = {0};

const uint16_t FastPWMPin::GCLK_CLKCTRL_IDs[] = {
    GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
    GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
    GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
    GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
    GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
    GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
    GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
    GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC7
};

FastPWMPin::FastPWMPin(uint32_t pinNo): pin(pinNo), updateFunction(&FastPWMPin::onUpdateNull), timerModule{nullptr}
{
}

void FastPWMPin::enableTC(Tc* timer, uint32_t tcChannel)
{
    // Disable The Timer While We Update
    timer->COUNT16.CTRLA.bit.ENABLE = 0;
    //Wait for sync
    while (timer->COUNT16.STATUS.bit.SYNCBUSY);
    // Set Timer counter Mode to 16 bits, normal PWM
    timer->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_NPWM;
    //Wait for sync
    while (timer->COUNT16.STATUS.bit.SYNCBUSY);
    // Set the initial value
    timer->COUNT16.CC[tcChannel].reg = 0;
    //Wait for sync
    while (timer->COUNT16.STATUS.bit.SYNCBUSY);
    // Enable TCx
    timer->COUNT16.CTRLA.bit.ENABLE = 1;
    //Wait for sync
    while (timer->COUNT16.STATUS.bit.SYNCBUSY);
}

void FastPWMPin::enableTCC(Tcc* timer, uint32_t tcChannel)
{
    //Disable the module while we update it
    timer->CTRLA.bit.ENABLE = 0;
    //Wait for sync
    while (timer->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
    // Set TCCx as normal PWM
    timer->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
    //Wait for sync
    while (timer->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
    // Set the initial value
    timer->CC[tcChannel].reg = 0;
    //Wait for sync
    while (timer->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
    // Set PER to maximum counter value (resolution : 0xFFFF)
    timer->PER.reg = 0xFFFF;
    //Wait for sync
    while (timer->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
    // Enable TCCx
    timer->CTRLA.bit.ENABLE = 1;
    //Wait for sync
    while (timer->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

void FastPWMPin::enableTimer(uint32_t pwmChannel, uint32_t tcNum, uint32_t tcChannel)
{
    //Set the enabled flag for this timer
    tcEnabled[tcNum] = true;
    //Enabed TCC Clock For Pin
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
    //Wait For Sync
	while (GCLK->STATUS.bit.SYNCBUSY == 1);
    //If the timer is greater than the last TCC, it bust be a plain ole TC
    if (tcNum >= TCC_INST_NUM) 
    {
        enableTC(timerModule.tc, tcChannel);
	} 
    //Otherwise its a sweet, fancy TCC
    else 
    {
        enableTCC(timerModule.tcc, tcChannel);
	}
}

void FastPWMPin::getPinData(uint32_t pin, uint32_t &attribute, uint32_t& pwmChannel, uint32_t &tcNum, uint32_t &tcChannel)
{
    PinDescription pinDesc = g_APinDescription[pin];
    attribute = pinDesc.ulPinAttribute;
    pwmChannel = pinDesc.ulPWMChannel;
    tcNum = GetTCNumber(pinDesc.ulPWMChannel);
    tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
}

void FastPWMPin::init()
{
    //Reterive pin metadata
    uint32_t attr, pinTcNum, pinTcChannel, pinPwmChannel;
    getPinData(pin, attr, pinPwmChannel, pinTcNum, pinTcChannel);
    //Store pin data to member variables
    tcNum = pinTcNum;
    tcChannel = pinTcChannel;

    //If this pin has PWM ability, go ahead and the start that engine, baby
    if (PIN_ATTR_HAS(attr, PIN_ATTR_PWM))
    {
        //Set the pin multiplexer
        setPinPeripheral(attr);
        //If the pin's TC number is greater than the last TCC module, 
        //it must be a plain TC module
        if (pinTcNum >= TCC_INST_NUM) 
        {
            timerModule.tc = (Tc*)GetTC(pinPwmChannel);
            updateFunction = &FastPWMPin::onUpdateTC;
        } 
        //Otherwisw, it must be a TCC module
        else 
        {
            timerModule.tcc = (Tcc*) GetTC(pinPwmChannel);
            updateFunction = &FastPWMPin::onUpdateTCC;
        }
        //If this pin's module hasn't been enabled by this class yet,
        //Start up that PWM engine, baby
        if(!tcEnabled[pinTcNum])
        {
            enableTimer(pinPwmChannel, pinTcNum, pinTcChannel);
        }
    }
    //If the pin is not a PWM pin, set the update pointer to do nothing and return
    else
    {
        updateFunction = &FastPWMPin::onUpdateNull;
    }
}

void FastPWMPin::setPinPeripheral(uint32_t attr)
{
    
	if (PIN_ATTR_HAS(attr,PIN_ATTR_TIMER)) 
    {
#if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
	      // Compatibility for cores based on SAMD core <=1.6.2
	      if (pinDesc.ulPinType == PIO_TIMER_ALT) {
	        pinPeripheral(pin, PIO_TIMER_ALT);
	      } 
          else
          {
	        pinPeripheral(pin, PIO_TIMER);
	      }
#else
        pinPeripheral(pin, PIO_TIMER);
#endif
	} 
    else if (PIN_ATTR_HAS(attr, PIN_ATTR_TIMER_ALT))
    {
        //this is on an alt timer
        pinPeripheral(pin, PIO_TIMER_ALT);
	}
	else
    {
	    return;
	}
}


void FastPWMPin::onUpdateNull(FastPWMPin& instance, uint32_t value)
{
    return;
}

void FastPWMPin::onUpdateTC(FastPWMPin& instance, uint32_t value)
{
    auto timer = instance.timerModule.tc;
    auto ch = instance.tcChannel;
    //Update the timer Compare Channel and Wait for sync
    timer->COUNT16.CC[ch].reg = (uint32_t)value;
    while (timer->COUNT16.STATUS.bit.SYNCBUSY);
}

void FastPWMPin::onUpdateTCC(FastPWMPin& instance, uint32_t value)
{
    auto timer = instance.timerModule.tcc;
    auto ch = instance.tcChannel;
    //Lock updates while buffer is updated
    timer->CTRLBSET.bit.LUPD = 1;
    //Wait for sync
    while (timer->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
        ;
    //Update the buffered register with the new value
    timer->CCB[ch].reg = (uint32_t)value;
    //Wait for sync
    while (timer->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
        ;
    //Unlock and enable update
    timer->CTRLBCLR.bit.LUPD = 1;
    //Wait for sync
    while (timer->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
        ;
}

