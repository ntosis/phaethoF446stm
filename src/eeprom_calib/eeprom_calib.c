#include "eeprom_calib.h"
#include "eeprom_em.h"
// Create variable in EEPROM with initial values
const CAL_PARAM CALinEE = {0x8888, 1,1.56,0.001,800,1.56,0.001,800,7,1,0} ;
// Create pointer for the calibration parameters
const CAL_PARAM  *p=&CALinEE;
CAL_PARAM CALinRAM;

void initCAL(void) {

    CAL_PARAM tmp;

    EE_ReadBlockInEEm((uint16_t*)&tmp,sizeof(CAL_PARAM),0x8888);

    if(tmp.statusOfThisBlock==0x01) {

	memcpy(&CALinRAM,&tmp,sizeof(CAL_PARAM));

    }
    else {

	memcpy(&CALinRAM,&CALinEE,sizeof(CAL_PARAM));

    }


}




