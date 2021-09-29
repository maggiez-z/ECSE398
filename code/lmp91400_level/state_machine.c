/*
 * state_machine.c
 *
 *  Created on: Jan 9, 2014
 *      Author: a0415888
 */

#include "DSP2803x_Device.h"         // DSP2803x Headerfile
#include "DSP2803x_Examples.h"       // DSP2803x Examples Headerfile
#include "DSP2803x_EPwm_defines.h" 	 // useful defines for initialization
#include "HCCal_Type0_V1.h"
#include "stdio.h"

// Definitions used in the code
#define HCCAPCLK_PLLCLK 1  // HCCAPCLK = PLL2CLK (CLKIN * PLL2 MULT)
// Extern variables
extern Uint16 start_avg;
extern Uint16 start_flag;
extern Uint16 pulse_array_updated;
extern Uint32 highpulse[];
extern Uint16 array_index;
extern Uint16 start_cap;

// Extern Functions
extern void scia_xmit(int a);
extern void scia_msg(char *msg);

// Global variables used

Uint32 temp_avg = 0, Avg_Cap = 0, Upper_Limit = 0x38403D70, Lower_Limit = 0x0;
Uint16 avg_index;
Uint16 Recalculate_avg = 1;
char msg_out[32];

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

	Alpha_State_Ptr = &A0;	// Back to State A0
}

//=================================================================================
//	A - TASKS
//=================================================================================
//--------------------------------------------------------
void A1(void) //
//=====================================================================
{
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

	if (pulse_array_updated == 1)
	{
		Uint16 ix;
		pulse_array_updated = 0;
		if (array_index == 0)
			ix = 99;
		else
			ix = array_index-1;
		sprintf(msg_out, "%8lu\r\n", highpulse[ix]);
        scia_msg(msg_out);
        GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1; // Toggle GPIO8
	}

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
	// start capture signal for HRCAP2_Isr
	start_cap = 1;
	// Trigger the AFE every 500ms
	GpioDataRegs.GPADAT.bit.GPIO22 = 1;
//	DELAY_US(1); <<-- causes illegal trap on c2000+tdc1000 board
	GpioDataRegs.GPADAT.bit.GPIO22 = 0;
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
	CpuTimer0Regs.PRD.all = mSec1;					// A tasks
	CpuTimer1Regs.PRD.all = mSec50;				// B tasks
	CpuTimer2Regs.PRD.all = mSec500; 				// C tasks

	// Tasks State-machine init
	Alpha_State_Ptr = &A0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;
	C_Task_Ptr = &C1;

	VTimer0[0] = 0;
	VTimer1[0] = 0;
	VTimer1[1] = 0;
	VTimer2[0] = 0;

}


