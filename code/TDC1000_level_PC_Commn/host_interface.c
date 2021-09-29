//******************************************************************************
//  Description:  This file contains functions that setups timerA0 for sampling.
//  Also, there are queue functions to queue and dequeue samples. Timer isr queues
//  the samples, while the usb communication task unqueues it to transmit out.
//  MSP430/LMP91400 Interface Code Library v1.0
// 
//
//   Vishy Natarajan
//   Texas Instruments Inc.
//   February 2012
//   Built with CCE Version: 4.2 and IAR Embedded Workbench Version:  5.3x
//******************************************************************************
/*  Copyright 2011-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user who
  downloaded the software, his/her employer (which must be your employer) and
  Texas Instruments Incorporated (the "License"). You may not use this Software
  unless you agree to abide by the terms of the License. The License limits your
  use, and you acknowledge, that the Software may not be modified, copied or
  distributed unless embedded on a Texas Instruments microcontroller which is 
  integrated into your product. Other than for the foregoing purpose, you may 
  not use, reproduce, copy, prepare derivative works of, modify, distribute, 
  perform, display or sell this Software and/or its documentation for any 
  purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL TEXAS
  INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL
  EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT
  LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL
  DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS,
  TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT
  LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

#include <stdint.h>
#include <cstring>
//#include <cstdio>
#include <stdio.h>
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "TI_LMP91400.h"
#include "lmp91400_spi.h"
#include "host_interface.h"

#define ONE_MS 24000                      // 1ms at 24MHz
#define ONE_100_uS 2400                   // 100us delay at 24MHz  
#define MAX_STOP_PULSES 6

extern Uint16 start_cap;
extern Uint16 pulse_array_updated;
extern Uint16 array_index;
extern Uint32 highpulse[];
extern volatile Uint16 single_shot_measure_state;
extern volatile Uint16 tof_graph_state;
extern volatile Uint16 continuous_trigger_state;
extern volatile Uint16 temp_measure_state;
extern volatile Uint16 interleaved_temp_measure;
extern volatile Uint16 measure_one_temp;
extern uint16_t nsamp_done;
extern uint16_t c_task_timeout;
extern uint16_t c_task_data2host_pending;
extern Uint16 level_demo_relay_control;
extern Uint16 first;
extern Uint16 start_relay_first_time;
extern uint16_t TDC1000_HV_Driver_Enable1;
extern uint16_t TDC1000_HV_Driver_Enable2;
extern uint16_t TDC1000_HV_Driver_Enable1_Period;
extern uint16_t TDC1000_HV_Driver_Enable2_Period;

extern void TI_LMP91400_reg_init(void);

void handle_reInit(uint8_t *pBuf, uint16_t pSize);
uint8_t char2nibble(uint8_t db);
uint8_t reg_local_copy[10] = {0x45, 0x40, 0x00, 0x03, 0x1F, 0x00, 0x00, 0x00, 0x19, 0x00};        // default settings
uint8_t LMP91400_address; 
uint16_t timer_trigger_freq = 4; // default 500ms trigger update freq
uint16_t count_per_temp = 0;
uint16_t generate_software_reset = 0;


// Parse Command, then execute
uint8_t handleHostCommand(uint8_t *pBuf, uint16_t pSize)
{
  uint8_t host_response = 0;
  uint16_t word_data;
  uint8_t nxt_byte, byte_data;
  
  nxt_byte = char2nibble(pBuf[0]);
  nxt_byte = (nxt_byte << 4) + char2nibble(pBuf[1]);
  switch(nxt_byte)
  {
    case Command_LoopPacket:  
    {
      host_response = 1;
      break;
    }
    case Command_ReInit: 
    {
      handle_reInit(pBuf, pSize);
      host_response = 1;
      break;
    }    
    case Command_SPI_Byte_Write:
    {
      LMP91400_address = char2nibble(pBuf[2]);
      LMP91400_address = (LMP91400_address << 4) + char2nibble(pBuf[3]);
      
      byte_data = char2nibble(pBuf[4]);
      byte_data = (byte_data <<4) + char2nibble(pBuf[5]);
      TI_LMP91400_SPIWriteReg(LMP91400_address, byte_data);
      reg_local_copy[LMP91400_address] = byte_data;
      host_response = 1;
      break;
    }
    case Command_SPI_Byte_Read:
    {
      LMP91400_address = char2nibble(pBuf[2]);
      LMP91400_address = (LMP91400_address << 4) + char2nibble(pBuf[3]);
      byte_data = TI_LMP91400_SPIReadReg(LMP91400_address);
      pBuf[8] = byte_data;
      host_response = 1;
      break;
    }
    case Command_Start_TOF_One_Shot:
    {
      
      // start capture signal for HRCAP2_Isr
      //start_cap = 1;
      //first = 0;
      // Trigger the AFE
      //GpioDataRegs.GPASET.bit.GPIO22 = 1;
      //	DELAY_US(1); <<-- causes illegal trap on c2000+tdc1000 board
      //GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
      single_shot_measure_state = 1;
      c_task_data2host_pending = 0;
      c_task_timeout = 0;
      host_response = 0;
      break;
    }
    case Command_Start_TOF_Graph:
    {
      // low byte first
      word_data = char2nibble(pBuf[4]);
      word_data = (word_data <<4) + char2nibble(pBuf[5]);
      // high byte next
      word_data = (word_data <<4) + char2nibble(pBuf[2]);
      word_data = (word_data <<4) + char2nibble(pBuf[3]);
      if (word_data != 0)
      {
    	  interleaved_temp_measure = 1;
    	  nsamp_done = 0;
    	  measure_one_temp = 0;
    	  count_per_temp = word_data;
      } else
      {
    	  count_per_temp = 0;
      }
      c_task_data2host_pending = 0;
      c_task_timeout = 0;
      tof_graph_state = 1;
      // start capture signal for HRCAP2_Isr
      // start_cap = 1;
      // first = 0;
      // turn on/off the level demo relay control: 0 off, 1 on
      level_demo_relay_control = 0;
      if (level_demo_relay_control)
      {
    	  // init timer trigger freq = 300ms for level demo & modified by GUI commands
    	  timer_trigger_freq = 2;
    	  CpuTimer2Regs.PRD.all = (timer_trigger_freq + 1) * mSec100;
    	  start_relay_first_time = 1;
      }
      host_response = 1;
      break;
    }
    case Command_End_TOF_Graph:
    {
      tof_graph_state = 0;
      start_cap = 0;
      interleaved_temp_measure = 0;
      measure_one_temp = 0;
      count_per_temp = 0;
      nsamp_done = 0;
      level_demo_relay_control = 0;
      GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;     // Clear Relay Pin
      host_response = 1;
      break;
    }
    case Command_Start_Continuous_Trigger:
    {
      continuous_trigger_state = 1;
      host_response = 1;
      break;
    }
    case Command_Stop_Continuous_Trigger:
    {
      continuous_trigger_state = 0;
      host_response = 1;
      break;
    }
  case Command_Firmware_Version_Read:
    {
      pBuf[8] = Firmware_VersionA;
      pBuf[9] = Firmware_VersionB;
      pBuf[10] = Firmware_VersionC;
      pBuf[11] = Firmware_VersionD;
      host_response = 1;
      break;
    }
  case Command_LED_Toggle:
    {
      // toggle LED
      GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1;
      host_response = 1;
      break;
    }
  case Command_C2000SPI_Config_Read:
    {
      pBuf[8] = SpiaRegs.SPICTL.all;
      pBuf[9] = SpiaRegs.SPIBRR;
      pBuf[10] = SpiaRegs.SPIBRR >> 8;
      host_response = 1;
      break;
    }
  case Command_C2000SPI_Config_Write:
    {
      byte_data = char2nibble(pBuf[2]);
      byte_data = (byte_data <<4) + char2nibble(pBuf[3]); 
      byte_data &= 0xC0;                                                       // make sure only top 2 bits are valid
      // low byte first
      word_data = char2nibble(pBuf[6]);
      word_data = (word_data <<4) + char2nibble(pBuf[7]);
      // high byte next
      word_data = (word_data <<4) + char2nibble(pBuf[4]);
      word_data = (word_data <<4) + char2nibble(pBuf[5]);      
      if (word_data < 1)
        word_data = 1;

      SpiaRegs.SPICCR.all =0x000F;	             // Reset on, rising edge, 16-bit char bits
      SpiaRegs.SPICTL.all |= byte_data;   	     // Enable master mode, high (not normal) phase,
                                                   // enable talk, and SPI int disabled.
      SpiaRegs.SPIBRR = word_data;				// vishy: 6MHz spi assuming 60MHz system clock
      SpiaRegs.SPICCR.all =0x008F;		         // vishy: Relinquish SPI from Reset, no loopback, 16-bits

      host_response = 1;
      break;
    }
  case Command_Set_Timer_Trigger_Freq:
   {
     // low byte first
     word_data = char2nibble(pBuf[4]);
     word_data = (word_data <<4) + char2nibble(pBuf[5]);
     // high byte next
     word_data = (word_data <<4) + char2nibble(pBuf[2]);
     word_data = (word_data <<4) + char2nibble(pBuf[3]);
     // don't allow user to change trigger freq for level relay demo
     if (level_demo_relay_control != 1)
     {
    	 timer_trigger_freq = word_data;

    	 // initial timer_trigger_freq = 5 & modified by GUI commands
    	 CpuTimer2Regs.PRD.all = (timer_trigger_freq + 1) * mSec100;
     }

     host_response = 1;
     break;
   }
  case Command_Read_Timer_Trigger_Freq:
   {
     pBuf[8] = timer_trigger_freq;
     pBuf[9] = timer_trigger_freq >> 8;
     host_response = 1;
     break;
   }
  case Command_Reset_TDC1000:
    {
      GpioDataRegs.GPASET.bit.GPIO21 = 1;     // Reset LMP91400
      // wait for 10usec
      DELAY_US(10L);
      GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;     // Remove Reset LMP91400
      // give atleast 500us
      DELAY_US(500L);
      TI_LMP91400_reg_init();
      host_response = 1;
      break;
    }
  case Command_Start_Temp_Measure_Graph:
  	{
  		temp_measure_state = 1;
        // start capture signal for HRCAP2_Isr
        // start_cap = 1;
        // first = 0;
        c_task_data2host_pending = 0;
        c_task_timeout = 0;
  		host_response = 1;
  		break;
    }
  case Command_End_Temp_Measure_Graph:
    {
    	temp_measure_state = 0;
        start_cap = 0;
    	host_response = 1;
    	break;
    }
  case Command_Set_Relay_Control:
    {
    	level_demo_relay_control = 1;
	    host_response = 1;
	    break;
    }
  case Command_Clear_Relay_Control:
    {
    	level_demo_relay_control = 0;
	    host_response = 1;
	    break;
    }
  case Command_Generate_Software_Reset:
    {
        generate_software_reset = 1;
	    host_response = 1;
	    break;
    }
  case Command_Enable_HV_DRIVER_EN1:
    {
      // get hv driver enable period in us
      // low byte first
      word_data = char2nibble(pBuf[4]);
      word_data = (word_data <<4) + char2nibble(pBuf[5]);
      // high byte next
      word_data = (word_data <<4) + char2nibble(pBuf[2]);
      word_data = (word_data <<4) + char2nibble(pBuf[3]);

      TDC1000_HV_Driver_Enable1 = 1;
      TDC1000_HV_Driver_Enable1_Period = word_data;
      host_response = 1;
      break;
    }
  case Command_Disable_HV_DRIVER_EN1:
    {
      TDC1000_HV_Driver_Enable1 = 0;
      host_response = 1;
      break;
    }
  case Command_Enable_HV_DRIVER_EN2:
    {
      // get hv driver enable period in us
      // low byte first
      word_data = char2nibble(pBuf[4]);
      word_data = (word_data <<4) + char2nibble(pBuf[5]);
      // high byte next
      word_data = (word_data <<4) + char2nibble(pBuf[2]);
      word_data = (word_data <<4) + char2nibble(pBuf[3]);

      TDC1000_HV_Driver_Enable2 = 1;
      TDC1000_HV_Driver_Enable2_Period = word_data;
      host_response = 1;
      break;
    }
  case Command_Disable_HV_DRIVER_EN2:
    {
      TDC1000_HV_Driver_Enable2 = 0;
      host_response = 1;
      break;
    }
  default:
  	{
  		host_response = 1;
  		break;
    }
  }
  return (host_response);
}

void handle_reInit(uint8_t *pBuf, uint16_t pSize)
{
}
      
uint8_t char2nibble(uint8_t db)
{
  if ((db >= '0') && (db <= '9'))
    return (db-'0');
  else if ((db >= 'A') && (db <= 'F'))
    return (db-'A'+0xa);
  else if ((db >= 'a') && (db <= 'f'))
    return (db-'a'+0xa);
  else
    return (db);
}
