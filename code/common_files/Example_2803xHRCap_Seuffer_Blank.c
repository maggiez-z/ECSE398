//###########################################################################
//
//!  \addtogroup f2803x_example_list
//!  <h1>HRCAP Capture HRPWM Pulses (hrcap_capture_hrpwm)</h1>
//!
//!  This program generates a high-resolution PWM output on ePWM1A
//!  (with falling edge moving by 8 HRPWM MEP steps per period), and
//!  the HRCAP is configured to generate an interrupt on  either rising
//!  edges OR falling edges to continuously capture up to 5 PWM periods
//!  (5 high pulses and 5 low pulses) and calculate the high 
//!  resolution pulse widths in integer + fractional HCCAPCLK cycles
//!  in Q16 format.
//!
//!  Monitor pulsewidthlow and pulsewidthhigh in the Watch Window (pulsewidthlow
//!  gradually decreases and pulsewidthhigh gradually increases as CMPAHR moves
//!  the falling edge MEP steps.) Another option is to monitor periodwidth in
//!  the Watch Window, which should not change much because the period stays
//!  the same.
//!
//! - User must set 
//!   - \#define FALLTEST 1 and RISETEST 0 for falling edge interrupts
//!   - \#define RISETEST 1 and FALLTEST 0 for rising edge interrupts
//! - To measure PERIOD, user must set:
//!  \#define PERIODTEST 1 
//! -  To measure high pulse width or low pulse width, user must set:
//!  \#define PERIODTEST 0 
//!
//!  To determine the actual pulse \f$ \frac{width}{period} \f$ time in 
//!  \f$ \frac{pulsewidthlow}{pulsewidthhigh}\f$ period: \n
//!  \f$ pulse width in seconds = pulsewidth[n] * (1 HCCAPCLK cycle) \f$ \n
//!  (i.e. 1 HCCAPCLK cycle is 8.33 ns for 120 MHz HCCAPCLK)
//!
//!  PLL2 is configured such that:
//!  - PLL2 = 60 MHz using INTOSC1 (10 MHz) as source
//!  - HCCAPCLK = 120 MHz
//!
//! \note 
//! - THE FOLLOWING DEFINITION AND STRUCTURE MUST BE DEFINED IN CODE
//! IN ORDER TO USE THE HCCAL LIBRARY
//!   - \#define NUM_HRCAP 5        // \# of HRCAP modules + 1 (4 HRCAP's on 2806x + 1 = 5)
//!   - volatile struct HRCAP_REGS *HRCAP[NUM_HRCAP] = {0, \&HRCap1Regs, \&HRCap2Regs, \&HRCap3Regs, \&HRCap4Regs};
//!  - Because there is no EMU FREE/STOP support in the HRCAP peripheral,
//!  the HRCAP results cannot easily be seen in "Real-time continuous refresh"
//!  debug mode.  The only way to see accurate results of the capture is to
//!  set a breakpoint in the HRCAP ISR immediately after data has been captured.
//!  Otherwise the results read at any other time in the program via the debugger 
//!  could be mid-capture between one period and the next, and therefore bogus.
//!
//!  \b External \b Connections \n
//!  - HRCAP1 is on GPIO26
//!  - ePWM1A is on GPIO0
//!  - Connect output of ePWM1A to input of HRCAP1 (GPIO0->GPIO26)
//!
//!  \b Watch \b Variables \n
//!  - pulsewidthlow
//!    - Pulse Width of Low Pulses (# of HCCAPCLK cycles - int + frac)
//!      in Q16 format (i.e. upper 16-bits integer, lower 16-bits fraction)
//!  - pulsewidthhigh
//!    - Pulse Width of High Pulses (# of HCCAPCLK cycles - int + frac)
//!      in Q16 format (i.e. upper 16-bits integer, lower 16-bits fraction)
//!  - periodwidth
//!    - Period Width (# of HCCAPCLK cycles - int + frac)
//!      in Q16 format (i.e. upper 16-bits integer, lower 16-bits fraction)
//
//###########################################################################
// $TI Release: F2803x C/C++ Header Files and Peripheral Examples V127 $
// $Release Date: March 30, 2013 $
//###########################################################################

#include "DSP2803x_Device.h"         // DSP2803x Headerfile
#include "DSP2803x_Examples.h"       // DSP2803x Examples Headerfile
#include "DSP2803x_EPwm_defines.h" 	 // useful defines for initialization
#include "HCCal_Type0_V1.h"

// Define whether falling edge interrupts or rising edge interrupts are used:
//---------------------------------------------------------------------------
#define FALLTEST 0
#define RISETEST 0

// Define whether to capture period or pulse widths:
//---------------------------------------------------------------------------
#define PERIODTEST 0

// Definitions used in the code
#define HCCAPCLK_PLLCLK 1  // HCCAPCLK = PLL2CLK (CLKIN * PLL2 MULT)
#define HCCAPCLK_SYSCLK 0  // HCCAPCLK = SYSCLK2 (CLKIN * PLL2 MULT / SYSCLK2DIV)


// !!IMPORTANT!!
// The following definition and struct must be defined in order to use the HCCal library
//---------------------------------------------------------------------------------------
#define NUM_HRCAP 3        // # of HRCAP modules + 1 (2 HRCAP's on 2803x + 1 = 3)
volatile struct HRCAP_REGS *HRCAP[NUM_HRCAP] = {0, &HRCap1Regs, &HRCap2Regs};

// Function Prototypes
void HRCAP2_Config(void);
void HRPWM1_Config(Uint16 period);
__interrupt void HRCAP2_Isr (void);
void PWM2_Config(Uint16 blank_period);
void InitSysCtrl_xtal(void);
void hrcap_pwm_init(void);

// Global variables used in program
Uint16 first;              // Count and Dly captured by 1st RISE/FALL event after cal, soft reset, or clock enable
                           // is invalid and therefore ignored. (equals # cycles from reset, cal, clock enable to edge
						   // instead of valid pulse width)

Uint16 counter;            // Increments CMPAHR by 8 MEP steps with each period
Uint16 datacounter;        // Counts 5 periods then resets.
Uint32 pulsewidthlow[5];   // Stores 5 low pulses in # of HCCAPCLK cycles (Q16 format: int + fraction)
Uint32 pulsewidthhigh[5];  // Stores 5 high pulses in # of HCCAPCLK cycles (Q16 format: int + fraction)
Uint32 periodwidth[5];     // Stores 5 period widths in # of HCCAPCLK cycles (Q16 format: int + fraction)
Uint32 periodwidthrise[5];     // Stores 5 period widths in # of HCCAPCLK cycles (Q16 format: int + fraction)

Uint32 highpulse[100];  // Stores 100 high pulses in # of HCCAPCLK cycles (Q16 format: int + fraction)
Uint16 array_index = 0;
Uint16 pulse_array_updated = 0;
Uint32 total_counter = 0;  // Counts total number of reads
Uint32 error_counter = 0;  // Counts total number of errors
Uint32 good_counter = 0;   // Counts total number of good results
Uint16 start_cap = 0;

Uint16 start_avg=0, start_flag=1;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - GENERAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


//#define FIX 1
#define PLL2SRC_X1_CRYSTAL 2  // X1 clock source (crystal oscillator) is selected as clock to PLL2 (Eva-Board, ToSch)
// Specify the PLL2 control register divide select (SYSCLK2DIV2DIS) and (PLL2MULT) values.
#define PLL2_SYSCLK2DIV2DIS   	0 	// PLL2 Output /2
//#define PLL2_SYSCLK2DIV2DIS  	1 	// PLL2 Output /1


Uint16 FIRST_PULSE_WIDTH = 4;

//Uint32 OFFSET_ADJ = 0x00001581;	// Corresponds to 0.7 ns fixed offset  added by the PWM2 output relative to function generator output (observed at room temp)
Uint32 OFFSET_ADJ = 0x0000;	// Corresponds to 0.0 ns fixed offset  added by the PWM2 output relative to function generator output (observed at room temp)




void HRPWM1_Config(Uint16 period)
{
// ePWM1 register configuration with HRPWM
// ePWM1A toggle low/high with MEP control on Falling edge

	EPwm1Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;	        // set Immediate load
	EPwm1Regs.TBPRD = period-1;		                    // PWM frequency = 1 / period
	EPwm1Regs.CMPA.half.CMPA = period / 2;              // set duty 50% initially
    EPwm1Regs.CMPA.half.CMPAHR = (0 << 8);              // initialize HRPWM extension
	EPwm1Regs.TBPHS.all = 0;
	EPwm1Regs.TBCTR = 0;

	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;


	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

	EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;                 // PWM toggle low/high
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;


	EALLOW;
	EPwm1Regs.HRCNFG.all = 0x0;
	EPwm1Regs.HRCNFG.bit.EDGMODE = HR_FEP;			   // HR MEP control on Falling edge
	EPwm1Regs.HRCNFG.bit.CTLMODE = HR_CMP;
	EPwm1Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO;
	EDIS;

	EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;             // Enable TBCLKSYNC
    EDIS;

}

void PWM2_Config(Uint16 blank_period)
{

	EPwm2Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;	        // set Immediate load
	EPwm2Regs.TBPRD = 0xFFFF;
	EPwm2Regs.CMPA.half.CMPA = 19;              // set duty 50% initially
	EPwm2Regs.TBPHS.all = 0;
	EPwm2Regs.TBCTR = 0;

	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0x7;	// /14

	EPwm2Regs.TBCTL.bit.CLKDIV = 0x2;	// /4


	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

	EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;

//    EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;                                 // This ensures that when trip is cleared, PWM output goes to zero – indicating end of first pulse

//OPTION # 1: Using TZCTL[DCAEVT1]

    EALLOW;
// Define an event (DCAEVT1) based on TZ1 Input
    EPwm2Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_TZ1;// DCAH = TZ1 input – Select the TZ input used for this function
    EPwm2Regs.TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI;  // DCAEVT1 = DCAH high (will become active when TZ1 input goes high)
//    EPwm2Regs.TZDCSEL.bit.DCAEVT2 = TZ_DCAH_LOW;  // DCAEVT2 = DCAH low (will become active when TZ1 input goes low)
    EPwm2Regs.DCACTL.bit.EVT1SRCSEL = DC_EVT_FLT;// DCAEVT1 = DC_EVT_FLT (filtered)
    EPwm2Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;          // Take async path

//    EPwm2Regs.DCACTL.bit.EVT2SRCSEL = DC_EVT2;// DCAEVT2 = DC_EVT2 (not filtered)
//    EPwm2Regs.DCACTL.bit.EVT2FRCSYNCSEL = DC_EVT_ASYNC;          // Take async path


// Following code for the sync mechanism based on the same trigger event – high TZ1
    EPwm2Regs.DCACTL.bit.EVT1SYNCE = 1;                                                    // Sync enabled

// What do we want the DCAEVT1 event to do?
    EPwm2Regs.TZCTL.bit.DCAEVT1 = TZ_FORCE_HI;                                     // EPWMxA – go high. Connect PWMnA output to HRCAP input externally.
    EPwm2Regs.TZCTL.bit.DCAEVT2 = TZ_NO_CHANGE;                               // Do nothing
    EPwm2Regs.TZCTL.bit.TZA = TZ_NO_CHANGE;                                          // Do nothing

//===========================================================================
// Event Filtering Configuration
    EPwm2Regs.DCFCTL.bit.SRCSEL = DC_SRC_DCAEVT1;
    EPwm2Regs.DCFCTL.bit.BLANKE = DC_BLANK_ENABLE;
//    EPwm2Regs.DCFCTL.bit.BLANKINV = 1;		//window is inverted
    EPwm2Regs.DCFCTL.bit.PULSESEL = DC_PULSESEL_ZERO;

    EPwm2Regs.DCFOFFSET = FIRST_PULSE_WIDTH;     // Blanking Window Offset = 24 or 25 (~400 ns) i.e. at least equal to pulse width of the first pulse in the series of pulses
//	EPwm2Regs.DCFOFFSET = 0;     // 0 when window is inverted

    EPwm2Regs.DCFWINDOW = blank_period; // When PWM2 CLKDIV is divide by 4

    EPwm2Regs.HRCNFG.bit.SELOUTB = 1;	// PWM2B is inverted version of PWM2A for easy viewing on the scope

    EDIS;

}


void HRCAP2_Config(void)
{
   EALLOW;

    HRCap2Regs.HCCTL.bit.SOFTRESET = 1;
	HRCap2Regs.HCCTL.bit.HCCAPCLKSEL = 1;  // HCCAPCLK = PLLCLK = SYSCLK x 2

	//***********************************************
	//Following added by HN
//	    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 0;     // Turn on XTALOSC

	//***********************************************

//    #if RISETEST
	HRCap2Regs.HCCTL.bit.RISEINTE = 1;     // Enable Rising Edge Capture Event Interrupt
	HRCap2Regs.HCCTL.bit.FALLINTE = 0;	   // Disable Falling Edge Capture Event Interrupt
//   #elif FALLTEST
//	HRCap2Regs.HCCTL.bit.FALLINTE = 1;	   // Enable Falling Edge Capture Event Interrupt
//	HRCap2Regs.HCCTL.bit.RISEINTE = 0;     // Disable Rising Edge Capture Event Interrupt
//    #endif
	HRCap2Regs.HCCTL.bit.OVFINTE = 0;	   // Enable Interrupt on 16-bit Counter Overflow Event

	// PLL2 = X1 or XCLKIN (20 Mhz) * 6 / 1 = 120 MHz
//	   InitPll2(PLL2SRC_X1_CRYSTAL, 6, PLL2_SYSCLK2DIV2DIS); // Eva-Board ToSch

   HRCap2Regs.HCICLR.all = 0xFFFF;        // Clear all interrupt flags (RISEOVF, COUNTEROVF, FALL, RISE, INT)
   EDIS;

}

__interrupt void HRCAP2_Isr (void)
{
	EALLOW;
if (start_cap) // Only read the HRCAP input when start_cap is set. Set this only after turning on function generator.
{
//	if (HRCap2Regs.HCIFR.bit.RISEOVF == 1) {
//		ESTOP0;                           // Another rising edge detected before ISR serviced.
//   }

//	highpulse[array_index] = HighPulseWidth1((Uint16 *)&HRCap2Regs);
//	highpulse[array_index] = HighPulseWidth0((Uint16 *)&HRCap2Regs);
	highpulse[array_index] = PeriodWidthRise0((Uint16 *)&HRCap2Regs);
	highpulse[array_index] = highpulse[array_index] - OFFSET_ADJ;

	pulse_array_updated = 1;

	 if (highpulse[array_index] < 0x38403D70 && HRCap2Regs.HCIFR.bit.COUNTEROVF != 1) 		// Only if captured period is less than 120us
	 {
//	   if ((highpulse[array_index] > 0x1A598000) && (highpulse[array_index] < 0x1A5AA000)) //+/- 2ns for 56us pulse
//	   if ((highpulse[array_index] > Lower_Limit) && (highpulse[array_index] < Upper_Limit)) //+/- 2ns for 56us pulse
//		 good_counter++;
//	   else error_counter++;

	   if (array_index == 99) // if whole measurement array has been filled
		  {
		    array_index = 0;
			start_avg = start_flag?1:2; // For the first time here start_flag is 1 ==> start_avg will be set to 1
			start_flag = 0;	  // second time onwards start_avg will be set to 2 making sure that temp_avg is only initialized the 1st time around
		  }
	   else array_index++;
	 }
	   total_counter++;

}
	HRCap2Regs.HCICLR.bit.RISEOVF=1;  // Clear RISE flag
	HRCap2Regs.HCICLR.bit.FALL=1;  // Clear FALL flag
	HRCap2Regs.HCICLR.bit.RISE=1;  // Clear RISE flag
	HRCap2Regs.HCICLR.bit.COUNTEROVF = 1; // Clear CounterOVF flag

    HRCap2Regs.HCICLR.bit.INT=1;   // Clear HRCAP interrupt flag
	PieCtrlRegs.PIEACK.bit.ACK4=1; // Acknowledge PIE Group 4 interrupts.


	EDIS;
}

void InitSysCtrl_xtal(void)
{

   // Disable the watchdog
   DisableDog();

   // *IMPORTANT*
   // The Device_cal function, which copies the ADC & oscillator calibration values
   // from TI reserved OTP into the appropriate trim registers, occurs automatically
   // in the Boot ROM. If the boot ROM code is bypassed during the debug process, the
   // following function MUST be called for the ADC and oscillators to function according
   // to specification. The clocks to the ADC MUST be enabled before calling this
   // function.
   // See the device data manual and/or the ADC Reference
   // Manual for more information.

        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1; // Enable ADC peripheral clock
        (*Device_cal)();
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0; // Return ADC clock to original state
        EDIS;

   // Select External Oscillator as Clock Source, and turn off all unused clocks to
   // conserve power.
   XtalOscSel();

   // Initialize the PLL control: PLLCR and CLKINDIV
   // DSP28_PLLCR and DSP28_CLKINDIV are defined in DSP2803x_Examples.h
   // Note: if board has 12MHz crystal, change it to 10 to get 120MHz, use 6 for 20MHz crystal
   InitPll(6,DSP28_DIVSEL);		// 20 MHz crystal
   // Initialize the peripheral clocks
   InitPeripheralClocks();
}

void hrcap_pwm_init(void)
{
	Uint16 status;

    status = 0;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;              // Disable TBCLKSYNC
    EDIS;

    // Start/Stop is connected to GPIO11 directly (no blanking)
    // Comment out PWM2 blanking
    //PWM2_Config(50);	// PWM2 period. 4500d ~ 50 us - used for blanking at /1 CLKDIV
    //   PWM2_Config(225);	// PWM2 period. 225d ~ 10 us at /4 CLKDIV - used for blanking
	   	   	   	   	   	   	// 225d ~ 2.5 us at /1 CLKDIV
    HRPWM1_Config(300);	                   // EPWM1 output waveform, Period = 300

    while (status!=2)                       // While calibration is incomplete
	{
       status = HRCAP_Cal(1,HCCAPCLK_PLLCLK, &EPwm7Regs);
       if (status == HCCAL_ERROR)
       {
	       ESTOP0;                        // If there is an error on HRCAP, stop and check 98 MHz < PLL2CLK < 120 MHz .
       }
    }

    HRCAP2_Config();                       // Configure HRCAP4 Module

}

