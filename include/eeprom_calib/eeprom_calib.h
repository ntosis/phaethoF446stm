/*
 * eeprom_calib.h
 *
 *  Created on: Jul 5, 2016
 *      Author: unix
 */

#ifndef EEPROM_CALIB_H_
#define EEPROM_CALIB_H_

#define _start 0x800C000
#include <hardware_init.h>
#include <string.h>
// Create structure
typedef struct {
    uint16_t virtualAddress;
    uint8_t  oneLevelSystem_C;
    float K_P_Htng;
    float K_I_Htng;
    float K_D_Htng;
    float K_P_Coolg;
    float K_I_Coolg;
    float K_D_Coolg;
    uint8_t smartCnt_C;
    uint8_t debugInfosFlag_C;
    uint8_t statusOfThisBlock;
} CAL_PARAM;

void initCAL(void);
void checkForaValidBlockInEEm(void);
void copyBlockFromEEmtoRam(void);
void copyInitCALtoRam(void);

extern CAL_PARAM CALinRAM;
extern CAL_PARAM  const CALinEE;
extern CAL_PARAM  const *p;

#define K_P_Htng (p->K_P_Htng)
#define K_I_Htng (p->K_I_Htng)
#define K_D_Htng (p->K_D_Htng)
#define K_P_Coolg (p->K_P_Coolg)
#define K_I_Coolg (p->K_I_Coolg)
#define K_D_Coolg (p->K_D_Coolg)
#define oneLevelSystem_C (p->oneLevelSystem_C)
#define smartCnt_C (p->smartCnt_C)
#define debugInfosFlag_C p->debugInfosFlag_C)
#endif /* EEPROM_CALIB_H_ */

