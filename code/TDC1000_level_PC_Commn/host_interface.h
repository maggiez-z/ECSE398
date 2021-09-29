//------------------------------------------------------------------------------
//  Description:  Header file for host_interface.h
//
//  MSP430/LMP93601 Interface Code Library v1.1
//
//   Vishy Natarajan
//   Texas Instruments Inc.
//   February 2012
//   Built with CCE Version: 4.2 and IAR Embedded Workbench Version:  5.3x
//------------------------------------------------------------------------------
// Change Log:
//------------------------------------------------------------------------------
// Version:  1.00
// Comments: Initial Release Version
//------------------------------------------------------------------------------

#define Firmware_VersionA 0
#define Firmware_VersionB 1                            // Major Rev: A + B
#define Firmware_VersionC 0
#define Firmware_VersionD 42                            // Minor Rev: C + D

uint8_t handleHostCommand(uint8_t *pBuf, uint16_t pSize);

//******************************************************************************
//Command and Packet definition
// 

#define Command_LoopPacket 0                // Loop back entire packet
											    // PacketBuffer[0],[1] Command
											    // 

#define Command_ReInit 1                    // Controls the firmware settings
											    // PacketBuffer[0],[1] Command
                                                                                            // PacketBuffer[1] Look for external trigger or not in continuous sampling 
                                                                                            // PacketBuffer[2]     													                                            	                                                	
                                                	                                                                                                    
#define Command_SPI_Byte_Write 2            // Writes data to SPI port
											    // PacketBuffer[0],[1] Command
											    // PacketBuffer[1],[2] SPI Address Byte											    
                                                                                            // PacketBuffer[3],[4] SPI Write Data Byte
  

#define Command_SPI_Byte_Read 3             // Read byte data from SPI port 
											    // PacketBuffer[0],[1] Command
											    // PacketBuffer[1],[2] SPI Address Byte
                                                                                            // PacketBuffer[8] Response: SPI Data Byte Read from device

#define Command_Start_Continuous_Trigger 4   // start continuous trigger (no c2000 measurement data response)
											                                                // PacketBuffer[0],[1] Command
											                                                //
                                                                                            // PacketBuffer[8] Response:

#define Command_Start_TOF_One_Shot 5                 //Start TOF command
											    // PacketBuffer[0],[1] Command
											    // PacketBuffer[2],[3] Expected num of stop pulses
                                                                             
                                                                                            // PacketBuffer[8] Response: start-stop1 data low byte 
                                                                                            // PacketBuffer[9] Response: start-stop1 data high byte
                                                                                            // PacketBuffer[10] Response: start-stop2 data low byte (only if numstop >= 2)
                                                                                            // PacketBuffer[11] Response: start-stop2 data high byte
                                                                                            // PacketBuffer[12] Response: start-stop3 data low byte (only if numch == 3)
                                                                                            // PacketBuffer[13] Response: start-stop3 data high byte


#define Command_Start_TOF_Graph 6   // Start tof graph
											                                                // PacketBuffer[0],[1] Command
											                                                //

                                                                                            // PacketBuffer[8] Response:


#define Command_End_TOF_Graph 7   // stop tof graph
											                                                // PacketBuffer[0],[1] Command
											                                                //
                                                                                            // PacketBuffer[8] Response:

#define Command_Stop_Continuous_Trigger 8   // stop continuous trigger
											                                                // PacketBuffer[0],[1] Command
											                                                //
                                                                                            // PacketBuffer[8] Response: 

#define Command_Firmware_Version_Read 9     // Read firmware version 
											    // PacketBuffer[0],[1] Command
                                                                                            // PacketBuffer[8] Response: Firmware VersionA 
                                                                                            // PacketBuffer[9] Response: Firmware VersionB
                                                                                            // PacketBuffer[10] Response: Firmware VersionC 
                                                                                            // PacketBuffer[11] Response: Firmware VersionD

#define Command_LED_Toggle 0x0A                // Toggle LED
											    // PacketBuffer[0],[1] Command
											    //

#define Command_C2000SPI_Config_Read 0x0B      // Read C2000 SPI config info
											    // PacketBuffer[0],[1] Command                            
                                                                                            // PacketBuffer[8] Response: SPI Clock Config   
                                                                                            // PacketBuffer[9] Response: SPI clock divider (low byte)
                                                                                            // PacketBuffer[10] Response: SPI clock divider (high byte)

#define Command_C2000SPI_Config_Write 0x0C      // Write C2000 SPI config info
											    // PacketBuffer[0],[1] Command
											    // PacketBuffer[2],[3] SPI Clock Config
                                                                                            // PacketBuffer[4],[5] SPI Clock divider low byte
                                                                                            // PacketBuffer[6],[7] SPI Clock divider high byte

#define Command_Start_Temp_Measure_Graph 0x0D   // Start Temp Measure graph
											                                                // PacketBuffer[0],[1] Command
											                                                //

                                                                                            // PacketBuffer[8] Response:


#define Command_End_Temp_Measure_Graph 0x0E   // Stop Temp Measure graph
											                                                // PacketBuffer[0],[1] Command
											                                                //
                                                                                            // PacketBuffer[8] Response:
#define Command_Set_Relay_Control 0x10            // Close the relay
                                                                                            // PacketBuffer[0],[1] Command
                                                                                            //
                                                                                            // PacketBuffer[8] Response:

#define Command_Clear_Relay_Control 0x11            // Open the relay
                                                                                            // PacketBuffer[0],[1] Command
                                                                                            //
                                                                                            // PacketBuffer[8] Response:

#define Command_Generate_Software_Reset 0x12     // generate software reset using watchdog
                                                                                            // PacketBuffer[0],[1] Command
                                                                                            //
                                                                                            // PacketBuffer[8] Response:
#define Command_Enable_HV_DRIVER_EN1 0x13        // HV Driver enable EN1
                                                                                            // PacketBuffer[0],[1] Command
                                                                                            // PacketBuffer[2],[3] Enable Period in us
                                                                                            // PacketBuffer[8] Response:
#define Command_Disable_HV_DRIVER_EN1 0x14       // HV Driver disable EN1
                                                                                            // PacketBuffer[0],[1] Command
                                                                                            //
                                                                                            // PacketBuffer[8] Response:
#define Command_Enable_HV_DRIVER_EN2 0x15        // HV Driver enable EN2
                                                                                            // PacketBuffer[0],[1] Command
                                                                                            // PacketBuffer[2],[3] Enable Period in us
                                                                                            // PacketBuffer[8] Response:
#define Command_Disable_HV_DRIVER_EN2 0x16       // HV Driver disable EN2
                                                                                            // PacketBuffer[0],[1] Command
                                                                                            //
                                                                                            // PacketBuffer[8] Response:
#define Command_Set_Timer_Trigger_Freq 0x18
                                                                                            // PacketBuffer[0],[1] Command
                                                                                            // PacketBuffer[2],[3] trigger freq low byte
                                                                                            // PacketBuffer[4],[5] trigger freq high byte

#define Command_Read_Timer_Trigger_Freq 0x1B
                                                                                            // PacketBuffer[0],[1] Command
                                                                                            // PacketBuffer[8],[9] Timer Trigger Freq byte


#define Command_Reset_TDC1000 0x1F
                                                                                            // PacketBuffer[0],[1] Command
                                                                                            //
