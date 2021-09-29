//###########################################################################
//
//!  \addtogroup f2803x_example_list
//!  <h1>SPI Digital Loop Back(spi_loopback)</h1>
//!
//!  This program uses the internal loop back test mode of the peripheral. 
//!  Other then boot mode pin configuration, no other hardware configuration
//!  is required. Interrupts are not used.
//!
//!  A stream of data is sent and then compared to the received stream.
//!  The sent data looks like this: \n
//!  0000 0001 0002 0003 0004 0005 0006 0007 .... FFFE FFFF \n
//!  This pattern is repeated forever.
//!  
//!  \b Watch \b Variables \n     
//!  - \b sdata , sent data
//!  - \b rdata , received data
//		
////###########################################################################		
// $TI Release: F2803x C/C++ Header Files and Peripheral Examples V127 $
// $Release Date: March 30, 2013 $
//###########################################################################
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "TI_LMP91400.h"
#include "lmp91400_spi.h"
#include "host_interface.h"
#include "DSP2803x_GlobalPrototypes.h"

// Definitions used in the code
#define MAX_STR_LENGTH 32

// Function Prototypes
extern void HRCAP2_Config(void);
extern void HRPWM1_Config(Uint16 period);
extern void PWM2_Config(Uint16 blank_period);
extern void InitSysCtrl_xtal(void);
extern void InitSysCtrl_ExtOsc(void);
__interrupt void HRCAP2_Isr (void);
__interrupt void xint1_isr(void);
extern void hrcap_pwm_init(void);
extern void state_machine_task_timing_init(void);
extern void scia_init(void);
extern void scia_fifo_init(void);
extern void scia_xmit(int a);
extern void scia_msg(char *msg);

void error(void);
void TI_LMP91400_reg_init(void);

// extern variable declarations
extern void (*Alpha_State_Ptr)(void);				// Base States pointer
extern Uint16 datacounter;
extern Uint16 first;

extern Uint16 start_avg;
extern Uint16 start_flag;
extern volatile Uint16 pulse_array_updated;
extern volatile Uint32 highpulse[];
extern volatile Uint16 array_index;
extern Uint16 start_cap;
extern volatile Uint16 save_ovflow_count;
extern uint8_t reg_local_copy[];
extern uint16_t count_per_temp;
extern uint16_t c_task_timeout;
extern uint16_t c_task_data2host_pending;
extern uint16_t generate_software_reset;

void Example_MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;


volatile Uint16 single_shot_measure_state = 0;
volatile Uint16 continuous_trigger_state = 0;
volatile Uint16 tof_graph_state = 0;
volatile Uint16 temp_measure_state = 0;
volatile Uint16 interleaved_temp_measure = 0;
volatile Uint16 measure_one_temp = 0;
volatile Uint16 number_of_stops_received = 0;
Uint16 level_demo_relay_control = 0;
Uint16 relay_pin_high_count = 0;
Uint16 relay_pin_state = 0;
Uint16 start_relay_first_time = 1;

char msg_out[32];
char cmdResponseString[MAX_STR_LENGTH] = "";

uint16_t reg0, reg1;
void main(void)
{
	Uint16 ix, ixm1, num_of_valid_lt_pulses;
	Uint16 send_response = 0;
	Uint16 data_packet_ready = 0;
	Uint16 i;
	Uint32 retry = 0;
	Uint16 rx_error;

// Step 0: by Vishy
//	memcpy(&RamfuncsLoadStart, &RamfuncsLoadEnd, (Uint32)RamfuncsRunStart);
	Example_MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);


// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2803x_SysCtrl.c file.
//   InitSysCtrl();

   InitSysCtrl_xtal();
//   InitSysCtrl_ExtOsc();



// Step 2. Initalize GPIO: 
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example
// Setup only the GP I/O only for SPI-A functionality
// This function is found in DSP2803x_Spi.c
   InitSpiaGpio();

// These functions are in the F2806x_EPwm.c file
   InitHRCapGpio();
// Not using blanking, disable pwm2
   InitEPwm2Gpio();

// For this example, only init the pins for the SCI-A port.
// This function is found in the DSP2803x_Sci.c file.
   InitSciaGpio();

   EALLOW;
   GpioCtrlRegs.GPAQSEL1.bit.GPIO2 = 3;		// Asynch to SYSCLKOUT GPIO2 (PWM2A)

// GPIO-12 - PIN FUNCTION = (TZ1)
   GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;		// Enable pull-up on GPIO12 (HRCAP4)
   GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;		// Asynch to SYSCLKOUT GPIO12
   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;	// 0=GPIO,  1=TZ1,  2=CANTX-B,  3=SPISIMO-B

   EDIS;

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
   DINT;

// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the DSP2803x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;
   
// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2803x_DefaultIsr.c.
// This function is found in DSP2803x_PieVect.c.
   InitPieVectTable();

   state_machine_task_timing_init();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.

   EALLOW;	// This is needed to write to EALLOW protected registers
   PieVectTable.HRCAP2_INT = &HRCAP2_Isr;
   PieVectTable.XINT1 = &xint1_isr;
   EDIS;   // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2803x_InitPeripherals.c
// InitPeripherals(); // Not required for this example
   spi_fifo_init();	  // Initialize the Spi FIFO
   spi_init();		  // init SPI
   hrcap_pwm_init();  // init HRCAP1, HRCAP2, PWM2

   scia_fifo_init();       // Initialize the SCI FIFO
   scia_init();            // Initalize SCI

// Step 5. User specific code:
// Interrupts are not used in this example. 
#if 1
// Configure GPIOs 20-22 as a GPIO output pins (AFE_EN, AFE_RST, AFE_TRIG)
   EALLOW;
//   GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPASET.bit.GPIO20 = 1;     // Set TDC1000 enable pin
   GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;    // GPIO20 = GPIO20
   GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;     // GPIO = output

//   GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPASET.bit.GPIO21 = 1;     // Reset TDC1000
//   GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;   // Remove Reset LMP91400
   GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;    // GPIO21 = GPIO21
   GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;     // GPIO = output

//   GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;     // Clear TDC1000 trigger
   GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;    // GPIO22 = GPIO22
   GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;     // GPIO = output

// D3 specific change: use GPIO24 for trigger instead of GPIO22
// For now both GPIO22 and GPIO24 are assigned trigger function
//  GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;     // Clear TDC1000 trigger
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;    // GPIO24 = GPIO24
   GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;     // GPIO = output

   GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;     // Remove Reset LMP91400

//   Configure GPIO0 to control the level demo motor relay
   GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;     // Clear Relay Pin
   GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;    // GPIO0 = GPIO0
   GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;     // GPIO = output

//   Configure GPIO1 to control the HV Driver EN1 pin: both Carsten's board and D3
   GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;     // Clear pin
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;    // GPIO1 = GPIO1
   GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;     // GPIO = output

//   Configure GPIO3 to control the HV Driver EN2 pin: only Carsten's board
   GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;     // Clear pin
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;    // GPIO3 = GPIO3
   GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;     // GPIO = output

//   Configure GPIO7 to control the TDC1000 CHSWP pin: both Sergio's and D3 boards
   GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPASET.bit.GPIO7 = 1;     // Set the pin << Need to be initally high
   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;    // GPIO7 = GPIO7
   GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;     // GPIO = output

//   Configure GPIO32 to control the CAN Driver Standby pin: only D3 board
  GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;  // Enable pullup on GPIO
  GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;     // Clear pin: D3 wants this pin driven low
  GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;    // GPIO32 = GPIO32
  GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;     // GPIO = output

// Configure GPIO8 (on GPA) as a GPIO output pin to control LED
   GpioDataRegs.GPASET.bit.GPIO8 = 1;                           // LED off
   GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;

// Configure GPIO9 (on GPA) as a GPIO input pin to handle TDC_ERRB
   GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;        // GPIO
   GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;         // input
   GpioCtrlRegs.GPAQSEL1.bit.GPIO9 = 0;       // XINT1 Synch to SYSCLKOUT only

// GPIO9 is XINT1
   GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 9;  // XINT1 is GPIO9

// Configure XINT1
   XIntruptRegs.XINT1CR.bit.POLARITY = 0;     // Falling edge interrupt

// Enable XINT1
   XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable XINT1

   EDIS;
#endif

   TI_LMP91400_reg_init();

#if 1
   // for testing
   reg0 = TI_LMP91400_SPIReadReg(TI_LMP91400_CONFIG1_REG);
   reg1 = TI_LMP91400_SPIReadReg(TI_LMP91400_CONFIG4_REG);

   if ((reg0 != 0x41) || (reg1 != 0x5F)) error();
   GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;                           // Device initialized: Light the LED

#endif

   // Initialize variables
   datacounter = 0;  // marks how many pulses have been captured
   first = 0;        // marks first captured data after a SOFTRESET to discard

   // Enable interrupts required for this example
   // Enable XINT1 in the PIE: Group 1 interrupt 4
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
   PieCtrlRegs.PIEIER1.bit.INTx4 = 1;   // Enable PIE Gropu 1 INT4
   PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8
   IER |= M_INT1;                       // Enable CPU INT1
   IER|=M_INT4;                         // Enable CPU INT5
   EINT;                                // Enable Global Interrupts

	// start capture signal for HRCAP2_Isr
    start_cap = 0;
    // gui controllable
    generate_software_reset = 0;

	// for temperature sensor testing
	//TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x40); // tx2 is 04, tx1 is 0
	//temp_measure_state = 1;

   // we want to be extra sure before turning on the relay, so look for
   // at least 5 low threshold pulses
   num_of_valid_lt_pulses = 0;
   for (;;)
   {
	    if ((pulse_array_updated == 1) && (c_task_data2host_pending == 1))
	    {
			if (single_shot_measure_state == 1)
			{
				single_shot_measure_state = 0;
				start_cap = 0;
			}
			if ((temp_measure_state == 1) || (measure_one_temp == 1))
			{
				//number_of_stops_received++;
				if (number_of_stops_received >= 5)
				{
				start_cap = 0;
				pulse_array_updated = 0;
				c_task_data2host_pending = 0;
				c_task_timeout = 0;
				number_of_stops_received = 0;
				{
					// temp mode stop pulses = 5
					if (array_index < 5)
						ix = 95+array_index;
					else
						ix = array_index-5;
					for(i = 0; i < 5; i++)
					{
						cmdResponseString[8+4*i] = highpulse[ix] >> 24;
						cmdResponseString[9+4*i] = highpulse[ix] >> 16;
						cmdResponseString[10+4*i] = highpulse[ix] >> 8;
						cmdResponseString[11+4*i] = highpulse[ix];
						if (ix+1 == 100)
							ix = 0;
						else
							ix++;
					}
					data_packet_ready = 1;
		            // reset state to 0
					// do it for temp_measure_state as well to slow things down
					if ((measure_one_temp == 1) || (temp_measure_state == 1))
					{
						// config2 back to tof measurement
						TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, reg_local_copy[TI_LMP91400_CONFIG2_REG]);
						// config3 back to tof measurement
						 TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, reg_local_copy[TI_LMP91400_CONFIG3_REG]);
						measure_one_temp = 0;
					}
					// Let host know if it is interleaved temp packet by putting count_per_temp in 7
					cmdResponseString[7] = count_per_temp;
				}
				}
			} else
			{
			start_cap = 0;
			pulse_array_updated = 0;
			c_task_data2host_pending = 0;
			c_task_timeout = 0;
			if (array_index == 0)
					ix = 99;
			else
				ix = array_index-1;
			// Let host know it is tof packet and not temp packet
			cmdResponseString[7] = 0;
			cmdResponseString[8] = highpulse[ix] >> 24;
			cmdResponseString[9] = highpulse[ix] >> 16;
			cmdResponseString[10] = highpulse[ix] >> 8;
			cmdResponseString[11] = highpulse[ix];
			cmdResponseString[12] = save_ovflow_count >> 8;
			cmdResponseString[13] = save_ovflow_count;
			data_packet_ready = 1;
			}
			if (level_demo_relay_control == 1)
			{
				if (ix == 0)
					ixm1 = 99;
				else
					ixm1 = ix-1;
				//if ((highpulse[ix] < 0x2EE00000) && (save_ovflow_count == 0))            // < 100us
				// if montonically falling, check difference is within 100us
				if ((highpulse[ixm1] >= highpulse[ix]) && ((highpulse[ixm1] - highpulse[ix]) <= 0x2EE00000) && (save_ovflow_count ==0))
				{
					// wait for 5 continuous pulses to hit this threshold
					if ((highpulse[ix] < 0x46500000) && (highpulse[ixm1] < 0x46500000) && (save_ovflow_count == 0))            // < 150us
					{
						// number of valid low threshold pulses
						if ((num_of_valid_lt_pulses < 5) && (!start_relay_first_time))
							num_of_valid_lt_pulses++;
						else
						{
							GpioDataRegs.GPASET.bit.GPIO0 = 1;     // Set Relay Pin
							relay_pin_state = 1;
							relay_pin_high_count = 0;
							num_of_valid_lt_pulses = 0;
							start_relay_first_time = 0;
						}
					}
				//else if (highpulse[ix] > 0xEA600000)       // > 500us
				//else if ((highpulse[ix] > 0x19400000) && (save_ovflow_count >= 1))       // > 600us
				// else if ((highpulse[ix] > 0x46500000) && (save_ovflow_count >= 1))       // > 700us
				} else
				{
					num_of_valid_lt_pulses = 0;
					if ((highpulse[ix] >= highpulse[ixm1]) && ((highpulse[ix] - highpulse[ixm1]) <= 0x2EE00000))
				// if montonically rising, check their difference is within 100us
					{
					// wait for 2 continuous pulses to hit this threshold
						if ((highpulse[ix] > 0x4F000000) && (highpulse[ixm1] > 0x4F000000) && (save_ovflow_count >= 1))       // > 650us
						{
							GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;     // Clear Relay Pin
							relay_pin_state = 0;
							relay_pin_high_count = 0;
						}
					} else if (((highpulse[ixm1] >= highpulse[ix]) && ((highpulse[ixm1] - highpulse[ix]) > 0x2EE00000)) ||
						   ((highpulse[ix] >= highpulse[ixm1]) && ((highpulse[ix] - highpulse[ixm1]) > 0x2EE00000)))
					// if montonically rising or falling and difference between adjacent samples is > 100us, send an empty packet
					{
						cmdResponseString[8] = 0;
						cmdResponseString[9] = 0;
						cmdResponseString[10] = 0;
						cmdResponseString[11] = 0;
						cmdResponseString[12] = 0;
						cmdResponseString[13] = 0;
					}
				}
			}

			save_ovflow_count = 0;
#if 0
			if ((temp_measure_state == 2) || (measure_one_temp == 2))
			{
				// temp mode stop pulses = 5
				if (array_index < 5)
					ix = 95+array_index;
				else
					ix = array_index-5;
				for(i = 0; i < 5; i++)
				{
					cmdResponseString[8+4*i] = highpulse[ix] >> 24;
					cmdResponseString[9+4*i] = highpulse[ix] >> 16;
					cmdResponseString[10+4*i] = highpulse[ix] >> 8;
					cmdResponseString[11+4*i] = highpulse[ix];
					if (ix+1 == 100)
						ix = 0;
					else
						ix++;
				}
				data_packet_ready = 1;
	            // reset state to 0
				if (measure_one_temp == 2)
				{
					// config2 back to tof measurement
					TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, reg_local_copy[TI_LMP91400_CONFIG2_REG]);
					// config3 back to tof measurement
					 TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, reg_local_copy[TI_LMP91400_CONFIG3_REG]);
					measure_one_temp = 0;
				} else
					temp_measure_state = 1;
				// Let host know if it is interleaved temp packet by putting count_per_temp in 7
				cmdResponseString[7] = count_per_temp;
			} else
				// Let host know it is tof packet and not temp packet
				cmdResponseString[7] = 0;
#endif
			if (data_packet_ready == 1)
			{
				scia_msg(cmdResponseString);
				data_packet_ready = 0;
				GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1; // Toggle GPIO8
				for(ix=0; ix < MAX_STR_LENGTH; ix++)
				cmdResponseString[ix] = NULL;
			}
	    }
	    if ((c_task_data2host_pending == 1) && (c_task_timeout == 1))
	    {
			if (single_shot_measure_state == 1)
			{
				single_shot_measure_state = 0;
				start_cap = 0;
			}
			if (measure_one_temp == 1)
			{
				// config2 back to tof measurement
				TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, reg_local_copy[TI_LMP91400_CONFIG2_REG]);
				// config3 back to tof measurement
				 TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, reg_local_copy[TI_LMP91400_CONFIG3_REG]);
				measure_one_temp = 0;
			}
			start_cap = 0;
			pulse_array_updated = 0;
	    	c_task_timeout = 0;
	    	c_task_data2host_pending = 0;
	    	// send an empty packet to keep communication alive
			scia_msg(cmdResponseString);
			// clear error: needed if user has selected timeout disable in the gui
			// and hence tdc1000 errob pin never goes high
			// TI_LMP91400_SPIWriteReg(TI_LMP91400_ERROR_FLAGS_REG, 0x03);
			GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1; // Toggle GPIO8
			// make sure relay is off
			//if (level_demo_relay_control == 1)
			//GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;     // Clear Relay Pin

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
			SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
		}
		if (ix == 32)
		{
			send_response = (Uint16) handleHostCommand((uint8_t *) cmdResponseString, ix);
			if (send_response)
				scia_msg(cmdResponseString);
			for(ix=0; ix < MAX_STR_LENGTH; ix++)
				cmdResponseString[ix] = NULL;
			if (generate_software_reset)
			{
				generate_software_reset = 0;
				// wait for UART communication to host to complete
				// takes 26ms at 9600 baud to send 32bytes
				DELAY_US(30000);
		        EALLOW;
		        SysCtrlRegs.WDCR= 0x0040;
		        EDIS;
			}
		} else
		{
			if (rx_error)
			{
				for(ix=0; ix < MAX_STR_LENGTH; ix++)
					cmdResponseString[ix] = 0xFF;
				scia_msg(cmdResponseString);
				for(ix=0; ix < MAX_STR_LENGTH; ix++)
					cmdResponseString[ix] = NULL;
			}
		}

  		// State machine entry & exit point
   		//===========================================================
   		(*Alpha_State_Ptr)();	// jump to an Alpha state (A0,B0,...)
   		//===========================================================

 	}

} 	


void TI_LMP91400_reg_init(void)
{
// 2MHz test cell (yellow)
#if 0
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG0_REG, 0x24); // 4pulses
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG1_REG, 0x41); // -> 44 to 40 (1stop)
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x00); // tx2 is 04, tx1 is 0
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, 0x0B); // enable blanking, 320mv threshold
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG4_REG, 0x5F); // 5e (group) -> 1e (edge mode)
  TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF1_REG, 0x80);
  TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF0_REG, 0x1E);
  TI_LMP91400_SPIWriteReg(TI_LMP91400_ERROR_FLAGS_REG, 0x01);
  TI_LMP91400_SPIWriteReg(TI_LMP91400_TIMEOUT_REG, 0x3B);
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CLOCK_RATE_REG, 0x01);
#endif
  // 1MHz test cell (green)
#if 1
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG0_REG, 0x44); // 4pulses
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG1_REG, 0x41); // -> 44 to 40 (1stop)
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x00); // tx2 is 04, tx1 is 0
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, 0x0C); // enable blanking, 320mv threshold
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG4_REG, 0x5F); // 5e (group) -> 1e (edge mode)
    TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF1_REG, 0x40);
    TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF0_REG, 0x1E);
    TI_LMP91400_SPIWriteReg(TI_LMP91400_ERROR_FLAGS_REG, 0x03);
    TI_LMP91400_SPIWriteReg(TI_LMP91400_TIMEOUT_REG, 0x23);
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CLOCK_RATE_REG, 0x01);
#endif
    // Auto Demo for Eric
#if 0
      TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG0_REG, 0x44); // 3pulses
      TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG1_REG, 0x41); // -> 44 to 41 (1stop)
      TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x00); // tx2 is 04, tx1 is 0
      TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, 0x06); // enable blanking, 320mv threshold
      TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG4_REG, 0x5F); // 5e (group) -> 1e (edge mode)
      TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF1_REG, 0xC0);
      TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF0_REG, 0x28);
      TI_LMP91400_SPIWriteReg(TI_LMP91400_ERROR_FLAGS_REG, 0x00);
      TI_LMP91400_SPIWriteReg(TI_LMP91400_TIMEOUT_REG, 0x21);
      TI_LMP91400_SPIWriteReg(TI_LMP91400_CLOCK_RATE_REG, 0x01);
#endif
      // conti 76cm tank setup
#if 0
        TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG0_REG, 0x5F); //
        TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG1_REG, 0x41); // -> 44 to 40 (1stop)
        TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x00); // tx2 is 04, tx1 is 0
        TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, 0x03); //
        TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG4_REG, 0x5F); // 5e (group) -> 1e (edge mode)
        TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF1_REG, 0xE3);
        TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF0_REG, 0xFF);
        TI_LMP91400_SPIWriteReg(TI_LMP91400_ERROR_FLAGS_REG, 0x03);
        TI_LMP91400_SPIWriteReg(TI_LMP91400_TIMEOUT_REG, 0x13);
        TI_LMP91400_SPIWriteReg(TI_LMP91400_CLOCK_RATE_REG, 0x03);
#endif
        // Convergence Level Demo with Relay setup
#if 0
          TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG0_REG, 0x48); //
          TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG1_REG, 0x41); // -> 44 to 40 (1stop)
          TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x00); // tx2 is 04, tx1 is 0
          TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, 0x04); //
          TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG4_REG, 0x5F); // 5e (group) -> 1e (edge mode)
          TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF1_REG, 0xE3);
          TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF0_REG, 0xFF);
          TI_LMP91400_SPIWriteReg(TI_LMP91400_ERROR_FLAGS_REG, 0x03);
          TI_LMP91400_SPIWriteReg(TI_LMP91400_TIMEOUT_REG, 0x23);
          TI_LMP91400_SPIWriteReg(TI_LMP91400_CLOCK_RATE_REG, 0x01);
#endif
   // Fluid ID Test Demo 9/23/14
#if 0
            TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG0_REG, 0x45); // 5pulses
            TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG1_REG, 0x41); // -> 44 to 40 (1stop)
            TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x00); // tx2 is 04, tx1 is 0
            TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, 0x02); //
            TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG4_REG, 0x5F); // 5e (group) -> 1e (edge mode)
            TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF1_REG, 0x60);
            TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF0_REG, 0x1E);
            TI_LMP91400_SPIWriteReg(TI_LMP91400_ERROR_FLAGS_REG, 0x03);
            TI_LMP91400_SPIWriteReg(TI_LMP91400_TIMEOUT_REG, 0x23);
            TI_LMP91400_SPIWriteReg(TI_LMP91400_CLOCK_RATE_REG, 0x01);
#endif
}


// Step 7. Insert all local Interrupt Service Routines (ISRs) and functions here:	

void error(void)
{
	GpioDataRegs.GPASET.bit.GPIO8 = 1;          // Device in error: Turn off the LED
    __asm("     ESTOP0");						// Test failed!! Stop!
    for (;;);
}

__interrupt void xint1_isr(void)
{

	TI_LMP91400_SPIWriteReg(TI_LMP91400_ERROR_FLAGS_REG, 0x03);

	// Acknowledge this interrupt to get more from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
/*------------------------------------------------------------------
  Simple memory copy routine to move code out of flash into SARAM
-----------------------------------------------------------------*/

void Example_MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }
    return;
}
//===========================================================================
// No more.
//===========================================================================



