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

/**
 * @file FastPWMSamD.h  
 * @author Nate Taylor (nate@rtelectronix.com)
 * @brief Declaration of the SamD::FastPWMPin class
 * @version 1.0
 * @date 2020-06-04
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef _FAST_PWM_SAMD_H_
#define _FAST_PWM_SAMD_H_

#include <Arduino.h>

namespace SamD
{
    /**
     * @brief Class providing faster PWM pin access for changing duty cycle during interrupts
     */
    class FastPWMPin
    {
        public:
            /**
             * @brief Construct a new FastPWMPin object
             * 
             * @param pinNo The Arduino pin number
             */
            FastPWMPin(uint32_t pinNo);

            /**
             * @brief Initializes the pin for PWM
             * 
             */
            void init();
            
            /**
             * @brief Updates the duty cycle to the supplied value
             * 
             * @param value The new duty cycle (16 bits)
             */
            void update(uint32_t value)
            {
                //if (!booted) return;
                //updateTest(pwmChannel, tcNum, tcChannel, value);
                updateFunction(*this, value);
            }

            /**
             * @brief Check if the pin is ready to output PWM
             * 
             * @return true If the pin is ready.
             * @return false If it is not.
             */
            bool ready()
            {
                if (updateFunction == &FastPWMPin::onUpdateNull) return false;
                else return true;
            }

        protected:
            /**
             * @brief The number of available timers
             */
            static constexpr unsigned int NoTimers = TCC_INST_NUM+TC_INST_NUM;

            /**
             * @brief Static member that logs whether or not a Timer module has been booted
             */
            static bool tcEnabled[NoTimers];

            /**
             * @brief Static Member Defining The Timers Correct GCLK IDs
             */
            static const uint16_t GCLK_CLKCTRL_IDs[];

            /**
             * @brief Function pointer to the current valid update function.
             * 
             *  This function points to a static function that takes the active instance.
             *  This ABI is better optimized then the C++ implicit this on the Cortex-M0.
             */
            void (*updateFunction)(FastPWMPin& instance, uint32_t value);

            /**
             * @brief Union that stores the active timer module, either a Tc* or a Tcc*
             */
            union TimerModule
            {
                Tc* tc;
                Tcc* tcc;
            } timerModule;

            /**
             * @brief The active Arduino pin number
             */
            uint8_t pin;

            /**
             * @brief The output channel of the Timer that the pin is attached to
             */
            uint32_t tcChannel;

            /**
             * @brief The index of the pins Timer module
             */
            uint32_t tcNum;

            /**
             * @brief Sets the pin multiplexer to the correct peripheral
             * 
             * @param attr The pins attributes for selecting the right multiplexer setting
             */
            void setPinPeripheral(uint32_t attr);

            /**
             * @brief Enables the timer for the current pin
             * 
             * @param pwmChannel The pin's PWM Channel
             * @param tcNum The pin's timer index
             * @param tcChannel The pin's timer output channel
             */
            void enableTimer(uint32_t pwmChannel, uint32_t tcNum, uint32_t tcChannel);

            /**
             * @brief Target function for the updateFunction pointer when no PWM is active
             * 
             * @param instance The current FastPWMPin instance
             * @param value The value to update the PWM Duty cycle to
             */
            static void onUpdateNull(FastPWMPin& instance, uint32_t value);

            /**
             * @brief Target function for the updateFunction pointer when PWM is on a plain Timer/Counter module
             * 
             * @param instance The current FastPWMPin instance
             * @param value The value to update the PWM Duty cycle to
             */
            static void onUpdateTC(FastPWMPin& instance, uint32_t value);

            /**
             * @brief Target function for the updateFunction pointer when ePWM is on a Timer/Counter/Control module
             * 
             * @param instance The current FastPWMPin instance
             * @param value The value to update the PWM Duty cycle to
             */
            static void onUpdateTCC(FastPWMPin& instance, uint32_t value);

            /**
             * @brief Enables a Timer/Counter module for PWM
             * 
             * @param timer A pointer to the timer module to enable
             * @param tcChannel The PWM output channel for the pin
             */
            static void enableTC(Tc* timer, uint32_t tcChannel);

            /**
             * @brief Enables a Timer/Counter/Control module for PWM
             * 
             * @param timer A pointer to the timer module to enable
             * @param tcChannel The PWM output channel for the pin
             */
            static void enableTCC(Tcc* timer, uint32_t tcChannel);

            /**
             * @brief Get the data describing the properties of the given pin
             * 
             * @param pin The Arduino pin number
             * @param attribute (out) The pin's attribute flags
             * @param pwmChannel (out) The pin's PWM Channel
             * @param tcNum (out) The pin's timer module index number
             * @param tcChannel The pin's timer output channel
             */
            static void getPinData(uint32_t pin, uint32_t &attribute, uint32_t& pwmChannel, uint32_t &tcNum, uint32_t &tcChannel);
    };
}
#endif