#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"



#define MCLK_PIN	18
#define MCLK_TIMER	5
#define MCLK_TIMER_SEG AM_HAL_CTIMER_TIMERB

#define BCLK_PIN_INV	13
#define BCLK_TIMER_INV	0
#define BCLK_TIMER_SEG_INV AM_HAL_CTIMER_TIMERB
#define BCLK_TIMER_CLOCK_INV   0x1B//TMRB0CLK is B5

#define REF_TIMER	2
#define REF_TIMER_SEG AM_HAL_CTIMER_TIMERA
#define REF_TIMER_CLOCK   0x1B//TMRA2CLK is B5

#define LRCLK_PIN	31
#define LRCLK_TIMER	3
#define LRCLK_TIMER_SEG AM_HAL_CTIMER_TIMERA
#define LRCLK_TIMER_INT AM_HAL_CTIMER_INT_TIMERA3
#define LRCLK_TIMER_CLOCK   0x15//TMRA3CLK is A2

#define SDATA_PIN   27
#define SDATA_TIMER	1
#define SDATA_TIMER_CLOCK   0x17//TMRA1CLK is A2

#define BUF_SIZE		256

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************



void pwm_out(void)
{


	//
	// Configure the output pin.
	//
	am_hal_ctimer_output_config(MCLK_TIMER,
	                            MCLK_TIMER_SEG,
	                            MCLK_PIN,
	                            AM_HAL_CTIMER_OUTPUT_NORMAL,
	                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);

	//
	// Configure a timer to drive the LED.
	//
	am_hal_ctimer_config_single(MCLK_TIMER,               // ui32TimerNumber
	                            MCLK_TIMER_SEG,           // ui32TimerSegment
	                            (AM_HAL_CTIMER_FN_PWM_REPEAT    |   // ui32ConfigVal
	                             AM_HAL_CTIMER_HFRC_12MHZ|
	                             AM_HAL_CTIMER_INT_ENABLE) );

	//
	// Set up initial timer periods.
	//
	am_hal_ctimer_period_set(MCLK_TIMER,
	                         MCLK_TIMER_SEG, 2, 1); //MCKL 4MHz
	am_hal_ctimer_aux_period_set(MCLK_TIMER,
	                         MCLK_TIMER_SEG, 2, 1);
//////////////////////////////////////////////////////////////////////////////////

//No need REF_TIMER pin output
#if 0
	//
	// Configure the output pin.
	//
	am_hal_ctimer_output_config(REF_TIMER,
	                            REF_TIMER_SEG,
	                            46,
	                            AM_HAL_CTIMER_OUTPUT_NORMAL,
	                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);
#endif
	//
	// Configure a timer to drive the LED.
	//
	am_hal_ctimer_config_single(REF_TIMER,               // ui32TimerNumber
	                            REF_TIMER_SEG,           // ui32TimerSegment
	                            (AM_HAL_CTIMER_FN_PWM_REPEAT    |   // ui32ConfigVal
	                             _VAL2FLD(CTIMER_CTRL0_TMRA0CLK, REF_TIMER_CLOCK) | 
						AM_HAL_CTIMER_INT_ENABLE));
						//AM_HAL_CTIMER_INT_ENABLE | AM_HAL_CTIMER_PIN_INVERT) );

	//
	// Set up initial timer periods.
	//
	am_hal_ctimer_period_set(REF_TIMER,
	                         REF_TIMER_SEG, 3, 2);
	am_hal_ctimer_aux_period_set(REF_TIMER,
	                         REF_TIMER_SEG, 3, 2);

	//////////////////////////////////////////////////////////////////////////////////

	//
	// Configure the output pin.
	//	
	am_hal_ctimer_output_config(BCLK_TIMER_INV,
	                            BCLK_TIMER_SEG_INV,
	                            BCLK_PIN_INV,
	                            AM_HAL_CTIMER_OUTPUT_NORMAL,
	                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);

	//
	// Configure a timer to drive the LED.
	//
	am_hal_ctimer_config_single(BCLK_TIMER_INV,               // ui32TimerNumber
	                            BCLK_TIMER_SEG_INV,           // ui32TimerSegment
	                            (AM_HAL_CTIMER_FN_PWM_REPEAT    |   // ui32ConfigVal
	                             _VAL2FLD(CTIMER_CTRL0_TMRA0CLK, BCLK_TIMER_CLOCK_INV) | 
						//AM_HAL_CTIMER_INT_ENABLE));
						AM_HAL_CTIMER_INT_ENABLE | AM_HAL_CTIMER_PIN_INVERT) );

	//
	// Set up initial timer periods.
	//
	am_hal_ctimer_period_set(BCLK_TIMER_INV,
	                         BCLK_TIMER_SEG_INV, 3, 2);//BCLK INV
	am_hal_ctimer_aux_period_set(BCLK_TIMER_INV,
	                         BCLK_TIMER_SEG_INV, 3, 2);//BCLK INV

	//////////////////////////////////////////////////////////////////////////////////

	//
	// Start the timer.
	//
	am_hal_ctimer_start(MCLK_TIMER, MCLK_TIMER_SEG);
	am_hal_ctimer_start(REF_TIMER, REF_TIMER_SEG);

	am_hal_ctimer_start(BCLK_TIMER_INV, BCLK_TIMER_SEG_INV);


}


void
initialize_pattern64_counter(uint32_t ui32TimerNumber,
                           uint32_t ui32TimerSegment,
                           uint64_t ui64Pattern,
                           uint32_t ui32PatternLen,
                           uint32_t ui32Trigger,
                           uint32_t ui32OutputPin,
                           uint32_t ui32PatternClock)
{
    //
    // Set up timer.
    //
    am_hal_ctimer_clear(ui32TimerNumber, ui32TimerSegment);

    am_hal_ctimer_config_single(ui32TimerNumber, ui32TimerSegment,
                              (AM_HAL_CTIMER_FN_PTN_REPEAT    |AM_HAL_CTIMER_INT_ENABLE |
                               ui32PatternClock) );
	

    //
    // Set the pattern in the CMPR registers.
    //
    am_hal_ctimer_compare_set(ui32TimerNumber, ui32TimerSegment, 0, 
                            (uint32_t)(ui64Pattern & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, ui32TimerSegment, 1, 
                            (uint32_t)((ui64Pattern >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, ui32TimerSegment, 0, 
                            (uint32_t)((ui64Pattern >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, ui32TimerSegment, 1, 
                            (uint32_t)((ui64Pattern >> 48) & 0xFFFF));
    //
    // Set the timer trigger and pattern length.
    //
    am_hal_ctimer_config_trigger(ui32TimerNumber, ui32TimerSegment,
                               ( (ui32PatternLen << CTIMER_AUX0_TMRA0LMT_Pos) |
                                 ( ui32Trigger << CTIMER_AUX0_TMRA0TRIG_Pos) ) );

    //
    // Configure timer output pin.
    //
    am_hal_ctimer_output_config(ui32TimerNumber, ui32TimerSegment, ui32OutputPin, 
                              AM_HAL_CTIMER_OUTPUT_NORMAL, 
                              AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);

    //
    // Start the timer.
    //
    am_hal_ctimer_start(ui32TimerNumber, ui32TimerSegment);
}

void
global_disable(void)
{
    CTIMER->GLOBEN = 0x0;
}

void
global_enable(void)
{
    CTIMER->GLOBEN = 0xffff;
}

void I2Smaster_init(void)
{
	//
	// Disable all the counters.
	//
	global_disable();
	pwm_out();	

	initialize_pattern64_counter(LRCLK_TIMER, LRCLK_TIMER_SEG, 0xFFFFFFFF00000000, 
	               63, CTIMER_AUX0_TMRB0TRIG_DIS, LRCLK_PIN,
	               _VAL2FLD(CTIMER_CTRL0_TMRA0CLK, LRCLK_TIMER_CLOCK));//Timer3 Clock source is CTIMERA2 OUT
	
	//
	// Enable all the counters.
	global_enable();//I2S starts

}


void I2Smaster_deinit(void)
{
	//
	// Disable all the counters.
	//
	global_disable();
}

