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
#include "lmp91400_spi.h"
void error(void);
uint16_t reg0, reg1;
void main(void)
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2803x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO: 
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example  
// Setup only the GP I/O only for SPI-A functionality
// This function is found in DSP2803x_Spi.c
   InitSpiaGpio();

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
	
// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2803x_InitPeripherals.c
// InitPeripherals(); // Not required for this example
   spi_fifo_init();	  // Initialize the Spi FIFO
   spi_init();		  // init SPI

// Step 5. User specific code:
// Interrupts are not used in this example. 
#if 1
// Configure GPIO20-22 as a GPIO output pins (AFE_EN, AFE_RST, AFE_TRIG)
   EALLOW;
   GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPADAT.bit.GPIO20 = 1;   // LMP91400 in standby
   GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;  // GPIO20 = GPIO20
   GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;   // GPIO = output

   GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPADAT.bit.GPIO21 = 1;   // Reset LMP91400
//   GpioDataRegs.GPADAT.bit.GPIO21 = 0;   // Remove Reset LMP91400
   GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;  // GPIO21 = GPIO21
   GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;   // GPIO = output

   GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO
   GpioDataRegs.GPADAT.bit.GPIO22 = 0;   // Clear Lmp91400 trigger
   GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;  // GPIO22 = GPIO22
   GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;   // GPIO = output

   GpioDataRegs.GPADAT.bit.GPIO21 = 0;   // Remove Reset LMP91400
   EDIS;
#endif
   reg0 = TI_LMP91400_SPIReadReg(0);
   reg1 = TI_LMP91400_SPIReadReg(1);
   if (reg0 != 0x45) error();
   TI_LMP91400_SPIWriteReg(0, 0x41);
   reg0 = TI_LMP91400_SPIReadReg(0);
   TI_LMP91400_SPIWriteReg(0, 0x43);
   reg1 = TI_LMP91400_SPIReadReg(0);
 //  reg1 = reg0;

   if ((reg1 != 0x43) && (reg0 != 0x41)) error();
//   TI_LMP91400_SPIWriteReg(0, 0x0042);
//   reg0 = TI_LMP91400_SPIReadReg(0);
//   if (reg0 != 0x0041) error();
   for (;;)
   {
	   __asm("     NOP" );
	   __asm("     NOP" );
	   __asm("     NOP" );
   }
} 	

// Step 7. Insert all local Interrupt Service Routines (ISRs) and functions here:	

void error(void)
{
    __asm("     ESTOP0");						// Test failed!! Stop!
    for (;;);
}







//===========================================================================
// No more.
//===========================================================================


