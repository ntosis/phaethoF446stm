#include "heatingSys.h"

bool autoProgramSelected=false;
bool manualProgramSelected=false;
bool autoProgramTimeEnabled=false;
bool stateOfHeatingSystem=false;
// On/Off time Array
// Sunday->Saturday ontime/offtime
  const uint16_t onTimesAM[7]  =
  //ontime
  {845,550,550,550,550,550,845};

  const uint16_t offTimesAM[7]  =
  //offtime
  {1200,645,645,645,645,645,1200};

  const uint16_t onTimesPM[7]  =
  //ontime
  {1300,1630,1630,1630,1630,1630,1400} ;

  const uint16_t offTimesPM[7]  =
  //offtime
  {2200,2200,2200,2200,2200,2300,2300};

void updateSollTemperature() {

  	 if (TurnDetected&&stateOfProgram) {

  		 smartCntFlag=1;

  		 SOLLtemperature=SOLLtemperature+(up);

  		     if(up<0) {
  			 smartCntDown=smartCntDown+abs(up);
  		     } else if(up>0) {
  			smartCntUp=smartCntUp+abs(up);
  		     }
  		 //showSolltemp();
  		 up=0; 			//reset up counter
  	 	 }
       TurnDetected = false;    // do NOT repeat IF loop until new rotation detected

  }
/*
void autoProgram() {


  uint16_t convertedTime= GetHH()*100 +GetMM();

  uint16_t onTimeAM= pgm_read_word_near(onTimesAM + (GetDoW()-1));
  uint16_t offTimeAM= pgm_read_word_near(offTimesAM + (GetDoW()-1));
  uint16_t onTimePM= pgm_read_word_near(onTimesPM + (GetDoW()-1));
  uint16_t offTimePM= pgm_read_word_near(offTimesPM + (GetDoW()-1));

  if((onTimeAM<convertedTime&&convertedTime<offTimeAM)||(onTimePM<convertedTime&&convertedTime<offTimePM)) {
	  autoProgramTimeEnabled=true;

  }
  else
    {
	  autoProgramTimeEnabled=false;
  }

}
*/
void initLEDs(void){

         GPIO_InitTypeDef GPIO_InitStruct;
	 /*Configure GPIO pin : Arduino Connector A1_Pin (PA1) */
         GPIO_InitStruct.Pin = GPIO_PIN_1;
         GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
         GPIO_InitStruct.Pull = GPIO_PULLUP;
         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
         HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

         /*Configure GPIO pin : Arduino Connector A2_Pin (PA4) */
         GPIO_InitStruct.Pin = GPIO_PIN_4;
         GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
         GPIO_InitStruct.Pull = GPIO_PULLUP;
         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
         HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

         /*Configure GPIO pin : Arduino Connector A0_Pin (PA0) */
         GPIO_InitStruct.Pin = GPIO_PIN_0;
         GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
         GPIO_InitStruct.Pull = GPIO_PULLUP;
         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
         HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

         /*Configure GPIO pin Output Level */
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

         LEDfunction();
}

void LEDfunction(void) {
	if(!stateOfProgram) {
		__LOW(RED_LED);
		__HIGH(GREEN_LED);
	}
	else if((stateOfProgram)&&(!selectProgram)){
		__HIGH(RED_LED);
		__LOW(GREEN_LED);
	}
	else if((stateOfProgram)&&(selectProgram)) {
		__TOGGLE(GREEN_LED);
		__HIGH(RED_LED);
	}
	if(Ctrl_Subsystem_Y.Out1==true) {
		__TOGGLE(BLUE_LED);
	}
	else if(Ctrl_Subsystem_Y.Out1==false) {
		__HIGH(BLUE_LED);
	}
}

