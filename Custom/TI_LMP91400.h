//----------------------------------------------------------------------------
// Register addresses, names, and masks for the analog-front-end TI LMP91400.
//
// Define where we are getting information from
//
//
#ifndef HEADER_FILE_TI_LMP91400_H

#define HEADER_FILE_TI_LMP91400_H

/************************************************************
* TI LMP91400 REGISTER SET ADDRESSES
************************************************************/

#define TI_LMP91400_CONFIG0_REG                        (0x00)                  //  
#define TI_LMP91400_CONFIG1_REG                        (0x01)                  //  
#define TI_LMP91400_CONFIG2_REG                        (0x02)                  //  
#define TI_LMP91400_CONFIG3_REG                        (0x03)                  //  
#define TI_LMP91400_CONFIG4_REG                        (0x04)                  //  
#define TI_LMP91400_TOF1_REG                           (0x05)                  //  
#define TI_LMP91400_TOF0_REG                           (0x06)                  //  
#define TI_LMP91400_ERROR_FLAGS_REG                    (0x07)                  // 
#define TI_LMP91400_TIMEOUT_REG                        (0x08)                  //  
#define TI_LMP91400_CLOCK_RATE_REG                     (0x09)                  //  

// READ and WRITE
#define LMP91400_READ_BIT                              (0xBF)                  // bit 6 is 0 for read
#define LMP91400_WRITE_BIT                             (0x40)                  // bit 6 is 1 for write

#endif
