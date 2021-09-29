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
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "TI_LMP91400.h"
#include "lmp91400_spi.h"

// Function Prototypes
extern void HRCAP2_Config(void);
extern void HRPWM1_Config(Uint16 period);
extern void PWM2_Config(Uint16 blank_period);
extern void InitSysCtrl_xtal(void);
__interrupt void HRCAP2_Isr (void);
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

uint16_t reg0, reg1;
void main(void)
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2803x_SysCtrl.c file.
//   InitSysCtrl();

   InitSysCtrl_xtal();



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
   GpioDataRegs.GPADAT.bit.GPIO20 = 1;     // LMP91400 in standby
   GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;    // GPIO20 = GPIO20
   GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;     // GPIO = output

//   GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPADAT.bit.GPIO21 = 1;     // Reset LMP91400
//   GpioDataRegs.GPADAT.bit.GPIO21 = 0;   // Remove Reset LMP91400
   GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;    // GPIO21 = GPIO21
   GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;     // GPIO = output

//   GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPADAT.bit.GPIO22 = 0;     // Clear Lmp91400 trigger
   GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;    // GPIO22 = GPIO22
   GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;     // GPIO = output

   GpioDataRegs.GPADAT.bit.GPIO21 = 0;     // Remove Reset LMP91400

// Configure GPIO8 (on GPA) as a GPIO output pin to control LED
   GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;
   EDIS;
#endif

   TI_LMP91400_reg_init();

#if 1
   // for testing
   reg0 = TI_LMP91400_SPIReadReg(TI_LMP91400_CONFIG1_REG);
   reg1 = TI_LMP91400_SPIReadReg(TI_LMP91400_CONFIG4_REG);

   if ((reg0 != 0x40) && (reg1 != 0x5F)) error();
   // Device initialized: Light the LED
   GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;
#endif

   // Initialize variables
   datacounter = 0;  // marks how many pulses have been captured
   first = 0;        // marks first captured data after a SOFTRESET to discard

   // Enable interrupts required for this example
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
   PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8
   IER|=M_INT4;                         // Enable CPU INT5
   EINT;                                // Enable Global Interrupts

   for (;;)
   {
  		// State machine entry & exit point
   		//===========================================================
   		(*Alpha_State_Ptr)();	// jump to an Alpha state (A0,B0,...)
   		//===========================================================

 	}

} 	


void TI_LMP91400_reg_init(void)
{
// 2MHz test cell (yellow)
#if 1
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG0_REG, 0x24); // 4pulses
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG1_REG, 0x40); // -> 44 to 40 (1stop)
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x04); // tx2 is 04, tx1 is 0
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, 0x0B); // enable blanking, 320mv threshold
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG4_REG, 0x5F); // 5e (group) -> 1e (edge mode)
  TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF1_REG, 0x80);
  TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF0_REG, 0x1E);
  TI_LMP91400_SPIWriteReg(TI_LMP91400_ERROR_FLAGS_REG, 0x01);
  TI_LMP91400_SPIWriteReg(TI_LMP91400_TIMEOUT_REG, 0x3B);
  TI_LMP91400_SPIWriteReg(TI_LMP91400_CLOCK_RATE_REG, 0x01);
#endif
  // 1MHz test cell (green)
#if 0
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG0_REG, 0x44); // 4pulses
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG1_REG, 0x41); // -> 44 to 40 (1stop)
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG2_REG, 0x04); // tx2 is 04, tx1 is 0
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG3_REG, 0x0C); // enable blanking, 320mv threshold
    TI_LMP91400_SPIWriteReg(TI_LMP91400_CONFIG4_REG, 0x5F); // 5e (group) -> 1e (edge mode)
    TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF1_REG, 0x40);
    TI_LMP91400_SPIWriteReg(TI_LMP91400_TOF0_REG, 0x1E);
    TI_LMP91400_SPIWriteReg(TI_LMP91400_ERROR_FLAGS_REG, 0x01);
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



//===========================================================================
// No more.
//===========================================================================



