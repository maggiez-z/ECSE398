/*
 * state_machine.c
 *
 *  Created on: Jan 9, 2014
 *      Author: a0415888
 */

#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include "DSP2803x_Device.h"         // DSP2803x Headerfile
#include "DSP2803x_Examples.h"       // DSP2803x Examples Headerfile
#include "DSP2803x_EPwm_defines.h" 	 // useful defines for initialization
#include "HCCal_Type0_V1.h"
#include "lmp91400_spi.h"
#include "TI_LMP91400.h"

// Definitions used in the code
#define HCCAPCLK_PLLCLK 1  // HCCAPCLK = PLL2CLK (CLKIN * PLL2 MULT)
#define MAX_STR_LENGTH 32

// CLKS_PER_US = number of ticks per micro second
#define CLKS_PER_US 60

// Extern variables
extern Uint16 start_avg;
extern Uint16 start_flag;
extern Uint16 pulse_array_updated;
extern Uint32 highpulse[];
extern Uint16 array_index;
extern Uint16 start_cap;
extern uint8_t reg_local_copy[];
extern Uint16 first;
extern Uint16 overflow_count;

// Extern Functions
extern void scia_xmit(int a);
extern void scia_msg(char *msg);
extern uint8_t handleHostCommand(uint8_t *, uint16_t );
extern uint16_t timer_trigger_freq;

// Global variables used

Uint32 temp_avg = 0, Avg_Cap = 0, Upper_Limit = 0x38403D70, Lower_Limit = 0x0;
Uint16 avg_index;
Uint16 Recalculate_avg = 1;
uint16_t TDC1000_HV_Driver_Enable1 = 0;
uint16_t TDC1000_HV_Driver_Enable2 = 0;
uint16_t TDC1000_HV_Driver_Enable1_Period = 0;
uint16_t TDC1000_HV_Driver_Enable2_Period = 0;

#if 0
Uint16 first_time_measure = 1;
char msg_out[32];
char cmdResponseString[MAX_STR_LENGTH] = "";
#endif
extern volatile Uint16 single_shot_measure_state;
extern volatile Uint16 continuous_trigger_state;
extern volatile Uint16 tof_graph_state;
extern volatile Uint16 temp_measure_state;
extern volatile Uint16 interleaved_temp_measure;
extern volatile Uint16 measure_one_temp;
extern volatile Uint16 number_of_stops_received;
extern uint16_t count_per_temp;
uint16_t nsamp_done = 0;
uint16_t c_task_timeout = 0;
uint16_t c_task_data2host_pending = 0;
extern Uint16 relay_pin_high_count;
extern Uint16 relay_pin_state;
extern Uint16 level_demo_relay_control;
// -------------------------------- FRAMEWORK --------------------------------------
// State Machine function prototypes
//----------------------------------------------------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
void C0(void);	//state C0

// A branch states
void A1(void);	//state A1
void A2(void);	//state A2
void A3(void);	//state A3
void A4(void);	//state A4

// B branch states
void B1(void);	//state B1
void B2(void);	//state B2
void B3(void);	//state B3
void B4(void);	//state B4

// C branch states
void C1(void);	//state C1
void C2(void);	//state C2
void C3(void);	//state C3
void C4(void);	//state C4

// Function Prototypes
void state_machine_task_timing_init(void);

// Variable declarations
void (*Alpha_State_Ptr)(void);				// Base States pointer
void (*A_Task_Ptr)(void);					// State pointer A branch
void (*B_Task_Ptr)(void);					// State pointer B branch
void (*C_Task_Ptr)(void);					// State pointer C branch

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - GENERAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// -------------------------------- FRAMEWORK --------------------------------------

int16 VTimer0[4];			// Virtual Timers slaved of CPU Timer 0 (A events)
int16 VTimer1[4]; 			// Virtual Timers slaved of CPU Timer 1 (B events)
int16 VTimer2[4]; 			// Virtual Timers slaved of CPU Timer 2 (C events)

void Wait(unsigned long);

//#define FIX 1
#define PLL2SRC_X1_CRYSTAL 2  // X1 clock source (crystal oscillator) is selected as clock to PLL2 (Eva-Board, ToSch)
// Specify the PLL2 control register divide select (SYSCLK2DIV2DIS) and (PLL2MULT) values.
#define PLL2_SYSCLK2DIV2DIS   	0 	// PLL2 Output /2
//#define PLL2_SYSCLK2DIV2DIS  	1 	// PLL2 Output /1

//#define      mSec0_01           900           // 10 uS
//#define      mSec0_05          4500           // 50 uS
//#define      mSec30           2700000         // 30 mS
//Uint16 FIRST_PULSE_WIDTH = 4;

//Uint32 OFFSET_ADJ = 0x00001581;	// Corresponds to 0.7 ns fixed offset  added by the PWM2 output relative to function generator output (observed at room temp)
//Uint32 OFFSET_ADJ = 0x0000;	// Corresponds to 0.0 ns fixed offset  added by the PWM2 output relative to function generator output (observed at room temp)

//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION
//=================================================================================
// NOTE: 10/2/14:vishy: state m/c change, no A-tasks, use only 2 states B & C, use CPU Timer 0 for Wait function
//--------------------------------- FRAMEWORK -------------------------------------
void A0(void) {
	// loop rate synchronizer for A-tasks
	if (CpuTimer0Regs.TCR.bit.TIF == 1) {
		CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*A_Task_Ptr)();		// jump to an A Task (A1,A2,A3,...)
		//-----------------------------------------------------------

		VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
	}

	Alpha_State_Ptr = &B0;		// Comment out to allow only A tasks
}

void B0(void) {
	// loop rate synchronizer for B-tasks
	if (CpuTimer1Regs.TCR.bit.TIF == 1) {
		CpuTimer1Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// jump to a B Task (B1,B2,B3,...)
		//-----------------------------------------------------------
		VTimer1[0]++;			// virtual timer 1, instance 0
		VTimer1[1]++;// virtual timer 1, instance 1 (used by DSP280xx_SciCommsGui.c)
	}

	Alpha_State_Ptr = &C0;		// Allow C state tasks
}

void C0(void) {
	// loop rate synchronizer for C-tasks
	if (CpuTimer2Regs.TCR.bit.TIF == 1) {
		CpuTimer2Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*C_Task_Ptr)();		// jump to a C Task (C1,C2,C3,...)
		//-----------------------------------------------------------
		VTimer2[0]++;			//virtual timer 2, instance 0 (spare)
	}
    // NOTE: 10/2/14:vishy: state m/c change, Skip state A0, use only 2 states B & C, use CPU Timer 0 for Wait function
	//Alpha_State_Ptr = &A0;	// Back to State A0
	Alpha_State_Ptr = &B0;	// Back to State B0
}

//=================================================================================
//	A - TASKS
// NOTE: 10/2/14:vishy: state m/c change, Skip A tasks, use only 2 states B & C, use CPU Timer 0 for Wait function
//=================================================================================
//--------------------------------------------------------
void A1(void) //
//=====================================================================
{
#if 0
	Uint16 ix, send_response = 0;
	Uint32 retry = 0;
	Uint16 rx_error;
	if (start_avg == 1)	// start_avg is used to make sure that the captured period array has been filled completely before
			{				// starting to calculate the average captured value
		temp_avg = highpulse[0];// initialize temp_avg to 1st element of the captured period array
		start_avg = 2;// This makes sure that temp_avg is only initialized once
	}

	if (start_avg == 2 && Recalculate_avg)// Recalculate_avg should be Set from watch window whenever new avg/limits need to be calculated
			{
		avg_index = 0;
		for (avg_index = 0; avg_index < 100; avg_index++) {
			temp_avg = (temp_avg + highpulse[avg_index]) >> 1;
		}

		Avg_Cap = temp_avg;

		//	Lower_Limit = Avg_Cap - 0x00013333;		// +/- 10 ns
		//	Upper_Limit = Avg_Cap + 0x00013333;		// +/- 10 ns

		//	Lower_Limit = Avg_Cap - 0x00009999;		// +/- 5 ns
		//	Upper_Limit = Avg_Cap + 0x00009999;		// +/- 5 ns

		Lower_Limit = Avg_Cap - 0x00003D70;		// +/- 2 ns
		Upper_Limit = Avg_Cap + 0x00003D70;		// +/- 2 ns

//		Lower_Limit = Avg_Cap - 0x00001EB8;		// +/- 1 ns
//		Upper_Limit = Avg_Cap + 0x00001EB8;		// +/- 1 ns

		Recalculate_avg = 0;
	}
#endif
#if 0 // fixes first tof read error
	if ((single_shot_measure_state == 2) && (first_time_measure == 1))
	{
		first_time_measure = 0;
		scia_msg(cmdResponseString);
		GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1; // Toggle GPIO8
		for(ix=0; ix < MAX_STR_LENGTH; ix++)
			cmdResponseString[ix] = NULL;
	}
#endif
#if 0
    if ((pulse_array_updated == 1) && (single_shot_measure_state == 2))
    {
		pulse_array_updated = 0;
		single_shot_measure_state = 0;
		start_cap = 0;
		if (array_index == 0)
			ix = 99;
		else
			ix = array_index-1;
		cmdResponseString[8] = highpulse[ix] >> 24;
		cmdResponseString[9] = highpulse[ix] >> 16;
		cmdResponseString[10] = highpulse[ix] >> 8;
		cmdResponseString[11] = highpulse[ix];
		scia_msg(cmdResponseString);
		GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1; // Toggle GPIO8
		for(ix=0; ix < MAX_STR_LENGTH; ix++)
			cmdResponseString[ix] = NULL;
    }

	ix = 0; retry = 0; rx_error = 0;
	if (SciaRegs.SCIFFRX.bit.RXFFST > 0)
	{
		while (ix != 32)
		{
			if (SciaRegs.SCIFFRX.bit.RXFFST > 0)
				cmdResponseString[ix++] = SciaRegs.SCIRXBUF.all;
			else if (++retry == 500000)
			{
				rx_error = 1;
				break;
			}
		}
	}
	if (ix == 32)
	{
		send_response = (Uint16) handleHostCommand((uint8_t *) cmdResponseString, ix);
		if (send_response)
			scia_msg(cmdResponseString);
		for(ix=0; ix < MAX_STR_LENGTH; ix++)
			cmdResponseString[ix] = NULL;
	} else
	{
		if (rx_error)
		{
			for(ix=0; ix < MAX_STR_LENGTH; ix++)
				cmdResponseString[ix] = 0xFF;
			scia_msg(cmdResponseString);
		}
	}
#endif
	//-------------------
	A_Task_Ptr = &A1;
	//-------------------
}

/*
 //=====================================================================
 void A2(void)  // SPARE
 //-----------------------------------------------------------------
 {
 //-------------------
 A_Task_Ptr = &A1;	// To make task A3 active, change &A1 to &A3
 //-------------------
 }
 */

//%%%%%%%%%%%%%%%    B-Tasks:   %%%%%%%%%%%%%%%%%%%%%%%%%
//=====================================================================
void B1(void) //  Run calibration routine periodically in background to completion
//=====================================================================
{
	Uint16 status;
	// Run calibration routine periodically in background using HRCAP1 internally tied to HRPWM8 output.
	status = 0;                                  // New calibration not complete
	while (status != 2)                       // While calibration is incomplete
	{
		status = HRCAP_Cal(1, HCCAPCLK_PLLCLK, &EPwm7Regs);
		if (status == HCCAL_ERROR) {
			ESTOP0;
			// If there is an error on HRCAP, stop and check 98 MHz < PLL2CLK < 120 MHz .
		}
	}

	B_Task_Ptr = &B1;
	//-----------------
}
/*
 //=====================================================================
 void B2(void) // SPARE
 //=====================================================================
 {
 //-----------------
 B_Task_Ptr = &B1;
 //-----------------
 }
 */

//%%%%%%%%%%%%%%%    C-Tasks:   %%%%%%%%%%%%%%%%%%%%%%%%%
//=====================================================================
//=====================================================================
void C1(void)
//=====================================================================
{
	if ((continuous_trigger_state == 1) || (tof_graph_state == 1) || (single_shot_measure_state == 1) || (temp_measure_state == 1))
	{
		if ((tof_graph_state == 1) || (single_shot_measure_state == 1) || (temp_measure_state == 1))
		{
			// start capture signal for HRCAP2_Isr
			start_cap = 1;
			first = 0;
			overflow_count = 0;
			number_of_stops_received = 0;

			if (c_task_data2host_pending == 1)
				c_task_timeout = 1;
			else
			{
				c_task_data2host_pending = 1;
				c_task_timeout = 0;
			}
			if (interleaved_temp_measure == 1)
			{
				if (nsamp_done == count_per_temp)
				{
					// make sure previous one_temp is done
					if (measure_one_temp == 0)
					{
						// config2 for temp_measurement
						TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x40);
						// config3 for RTD1 only and PT1000
					    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, reg_local_copy[TI_LMP91400_CONFIG3_REG] | 0x40);
						// set temp_measure_state for this measurement (reset once data sent out)
						measure_one_temp = 1;
						nsamp_done = 0;
					}
				} else
					nsamp_done++;
			}
			// slow things down
			if (temp_measure_state == 1)
			{
				// config2 for temp_measurement
				TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x40);
				// config3 for RTD1 only and PT1000
			    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, reg_local_copy[TI_LMP91400_CONFIG3_REG]);
			}
		}

	    // Handle HV Driver Enable Command
	    if ((TDC1000_HV_Driver_Enable1 == 1) || (TDC1000_HV_Driver_Enable2 == 1))
	    {
	      if (TDC1000_HV_Driver_Enable1 == 1)
	        GpioDataRegs.GPASET.bit.GPIO1 = 1;
	      if (TDC1000_HV_Driver_Enable2 == 1)
	        GpioDataRegs.GPASET.bit.GPIO3 = 1;
	      // we need totally about 20us delay before start pulse
	      DELAY_US(20);
	    }

		// Trigger the AFE every 500ms
		GpioDataRegs.GPASET.bit.GPIO22 = 1;
		// D3 specific change: use both GPIO24 and GPIO22 for now as trigger
		GpioDataRegs.GPASET.bit.GPIO24 = 1;
		DELAY_US(1);                          //<<-- causes illegal trap on c2000+tdc1000 board
		GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;


	    // Handle HV Driver Enable Command
	    if ((TDC1000_HV_Driver_Enable1 == 1) || (TDC1000_HV_Driver_Enable2 == 1))
	    {
	      if ((TDC1000_HV_Driver_Enable1 == 1) && (TDC1000_HV_Driver_Enable2 == 1))
	      {
	        if (TDC1000_HV_Driver_Enable1_Period > TDC1000_HV_Driver_Enable2_Period)
	        	Wait(TDC1000_HV_Driver_Enable1_Period);
	        	//DELAY_US(40);
	        else
	        	Wait(TDC1000_HV_Driver_Enable2_Period);
	        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
	        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
	      } else if (TDC1000_HV_Driver_Enable1 == 1)
	      {
	    	  Wait(TDC1000_HV_Driver_Enable1_Period);
	    	  //DELAY_US(40);
	          GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
	      } else
	      {
	    	  Wait(TDC1000_HV_Driver_Enable2_Period);
	          GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
	      }
	    }

		if ((level_demo_relay_control == 1) && (relay_pin_state == 1))
		{
			relay_pin_high_count++;
			// assuming 300ms timer freq this is the safe max to allow
			if (relay_pin_high_count >= 37)
			{
				GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;     // Clear Relay Pin
				relay_pin_high_count = 0;
			}
		}
	}
	//-----------------
	C_Task_Ptr = &C1;
	//-----------------
}
/*
 //=====================================================================
 void C2(void) //  SPARE (not active)
 //=====================================================================
 {
 //-----------------
 C_Task_Ptr = &C3;
 //-----------------
 }


 //=====================================================================
 void C3(void) //  SPARE (not active)
 //=====================================================================
 {
 //-----------------
 C_Task_Ptr = &C1;
 //-----------------
 }*/

void state_machine_task_timing_init(void) {

	// Timing sync for background loops
	// Timer period definitions found in PeripheralHeaderIncludes.h
	// NOTE: 10/2/14:vishy: state m/c change, Skip A tasks, use only 2 states B & C, use CPU Timer 0 for Wait function
	// CpuTimer0Regs.PRD.all = mSec1;					// A tasks

	CpuTimer1Regs.PRD.all = mSec50;				// B tasks
//	CpuTimer2Regs.PRD.all = mSec500; 				// C tasks
	// initial timer_trigger_freq = 4 & modified by GUI commands
	CpuTimer2Regs.PRD.all = (timer_trigger_freq + 1) * mSec100;

	// Tasks State-machine init
	// NOTE: 10/2/14:vishy: state m/c change, Skip A tasks, use only 2 states B & C, use CPU Timer 0 for Wait function
	// Alpha_State_Ptr = &A0;
	Alpha_State_Ptr = &B0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;
	C_Task_Ptr = &C1;

	VTimer0[0] = 0;
	VTimer1[0] = 0;
	VTimer1[1] = 0;
	VTimer2[0] = 0;

}


void Wait(unsigned long int us)
{
  CpuTimer0Regs.PRD.all = us * CLKS_PER_US;
  CpuTimer0Regs.TCR.bit.TRB = 1;    // reload cpu timer0 with new period
  CpuTimer0Regs.TCR.bit.TIF = 1;    // clear flag
  while (CpuTimer0Regs.TCR.bit.TIF != 1);
  CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag
}
