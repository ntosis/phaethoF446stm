/**
  ******************************************************************************
  * @file    st7735.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    24-November-2014
  * @brief   This file includes the driver for ST7735 LCD mounted on the Adafruit
  *          1.8" TFT LCD shield (reference ID 802).
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ili9481.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup ST7735
  * @brief      This file provides a set of functions needed to drive the
  *             ST7735 LCD.
  * @{
  */

/** @defgroup ST7735_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup ST7735_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup ST7735_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup ST7735_Private_Variables
  * @{
  */
enum {
  MemoryAccessControlNormalOrder,
  MemoryAccessControlReverseOrder
} MemoryAccessControlRefreshOrder;

enum {
  MemoryAccessControlColorOrderBGR,
  MemoryAccessControlColorOrderRGB
} MemoryAccessControlColorOrder;

enum {
    ColumnAddressSet = 0x2a,
    PageAddressSet = 0x2b,
    MemoryWrite = 0x2c,
    MemoryAccessControl = 0x36,
    WriteDisplayBrightness = 0x51
} ILI9341Register;
/*
static lcdProperties_t  lcdProperties = { 240, 320, true, true, true };
static lcdOrientation_t lcdOrientation = LCD_ORIENTATION_PORTRAIT;
*/
static unsigned char lcdPortraitConfig = 0;
static unsigned char lcdLandscapeConfig = 0;


LCD_DrvTypeDef   st7735_drv =
{
  st7735_Init,
  0,
  st7735_DisplayOn,
  st7735_DisplayOff,
  st7735_SetCursor,
  st7735_WritePixel,
  0,
  st7735_SetDisplayWindow,
  st7735_DrawHLine,
  st7735_DrawVLine,
  st7735_GetLcdPixelWidth,
  st7735_GetLcdPixelHeight,
  st7735_DrawBitmap,
};

static uint16_t ArrayRGB[320] = {0};

/**
* @}
*/

/** @defgroup ST7735_Private_FunctionPrototypes
  * @{
  */

/**
* @}
*/

/** @defgroup ST7735_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the ST7735 LCD Component.
  * @param  None
  * @retval None
  */
void st7735_Init(void)
{
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void st7735_DisplayOn(void)
{
  uint8_t data = 0;
  LCD_IO_WriteReg(LCD_REG_19);
  HAL_Delay(10);
  LCD_IO_WriteReg(LCD_REG_41);
  HAL_Delay(10);
  LCD_IO_WriteReg(LCD_REG_54);
  data = 0xC0;
  LCD_IO_WriteMultipleData(&data, 1);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void st7735_DisplayOff(void)
{
  uint8_t data = 0;
  LCD_IO_WriteReg(LCD_REG_19);
  HAL_Delay(10);
  LCD_IO_WriteReg(LCD_REG_40);
  HAL_Delay(10);
  LCD_IO_WriteReg(LCD_REG_54);
  data = 0xC0;
  LCD_IO_WriteMultipleData(&data, 1);
}

/**
  * @brief  Sets Cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @retval None
  */
void st7735_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
}

/**
  * @brief  Writes pixel.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  RGBCode: the RGB pixel color
  * @retval None
  */
void st7735_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode)
{
  uint8_t data = 0;
  if((Xpos >= ST7735_LCD_PIXEL_WIDTH) || (Ypos >= ST7735_LCD_PIXEL_HEIGHT))
  {
    return;
  }

  /* Set Cursor */
  st7735_SetCursor(Xpos, Ypos);

  data = RGBCode >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = RGBCode;
  LCD_IO_WriteMultipleData(&data, 1);
}


/**
  * @brief  Writes to the selected LCD register.
  * @param  LCDReg: Address of the selected register.
  * @param  LCDRegValue: value to write to the selected register.
  * @retval None
  */
void LCD_IO_WriteReg(uint8_t Reg){

         __LOW(LCD_ChipSelect); //THIS IS D5 arduino like pin, hier is used as CS for the LCD. CS =LOW=LISTEN
         __LOW(LCD_CMD); //LCD_CMD pin = LOW = Send Command
         HAL_SPI_Transmit(&SpiHandle,(uint8_t)Reg, 1,500);
         //__LOW(LCD_CMD);
         __HIGH(LCD_ChipSelect);
         __HIGH(LCD_CMD);


}
void st7735_WriteReg(uint8_t LCDReg, uint8_t LCDRegValue)
{
    //LCD_IO_WriteReg((uint8_t)Data);
     __LOW(LCD_ChipSelect); //THIS IS D5 arduino like pin, hier is used as CS for the LCD. CS =LOW=LISTEN
     __LOW(LCD_CMD); //LCD_CMD pin = LOW = Send Command
     HAL_SPI_Transmit(&SpiHandle,(uint8_t)LCDReg, 1,500);
     //__LOW(LCD_CMD);
     LCD_IO_WriteMultipleData(&LCDRegValue, 1);
     __HIGH(LCD_ChipSelect);
     __HIGH(LCD_CMD);
}

void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size) {
    __LOW(LCD_ChipSelect); //THIS IS D5 arduino like pin, hier is used as CS for the LCD. CS =LOW=LISTEN
    __LOW(LCD_CMD); //LCD_CMD pin = LOW = Send Command
    HAL_SPI_Transmit(&SpiHandle,(uint8_t*)pData, Size,500);
    __HIGH(LCD_ChipSelect);
    __HIGH(LCD_CMD);
}
void LCD_IO_WriteData(uint8_t *pData, uint32_t Size) {
    __LOW(LCD_ChipSelect); //THIS IS D5 arduino like pin, hier is used as CS for the LCD. CS =LOW=LISTEN
    __LOW(LCD_CMD); //LCD_CMD pin = LOW = Send Command
    HAL_SPI_Transmit(&SpiHandle,(uint8_t*)pData, Size,500);
    __HIGH(LCD_ChipSelect);

}

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void commandList(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = (*addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    LCD_IO_WriteMultipleData((*addr++),1); //   Read, issue command
    numArgs  = (*addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
	LCD_IO_WriteData((*addr++),1);  //     Read, issue argument
    }

    if(ms) {
      ms = (*addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      HAL_Delay(ms);
    }
  }
}
// Initialization code common to both 'B' and 'R' type displays
void commonInit(const uint8_t *cmdList) {

  if(cmdList) commandList(cmdList);
}
// Initialization for ST7735R screens (green or red tabs)
void initR() {
    commonInit(Rcmd1);
    commandList(Rcmd2green144);
    commandList(Rcmd3);
    st7735_DisplayOn();
    st7735_DisplayOff();
}
/**
  * @brief  Sets a display window
  * @param  Xpos:   specifies the X bottom left position.
  * @param  Ypos:   specifies the Y bottom left position.
  * @param  Height: display window height.
  * @param  Width:  display window width.
  * @retval None
  */
void st7735_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
}

/**
  * @brief  Draws horizontal line.
  * @param  RGBCode: Specifies the RGB color
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: specifies the line length.
  * @retval None
  */
void st7735_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint8_t counter = 0;

  if(Xpos + Length > ST7735_LCD_PIXEL_WIDTH) return;

  /* Set Cursor */
  st7735_SetCursor(Xpos, Ypos);

  for(counter = 0; counter < Length; counter++)
  {
    ArrayRGB[counter] = RGBCode;
  }
  LCD_IO_WriteMultipleData((uint8_t*)&ArrayRGB[0], Length * 2);
}

/**
  * @brief  Draws vertical line.
  * @param  RGBCode: Specifies the RGB color
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: specifies the line length.
  * @retval None
  */
void st7735_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint8_t counter = 0;

  if(Ypos + Length > ST7735_LCD_PIXEL_HEIGHT) return;
  for(counter = 0; counter < Length; counter++)
  {
    st7735_WritePixel(Xpos, Ypos + counter, RGBCode);
  }
}

/**
  * @brief  Gets the LCD pixel Width.
  * @param  None
  * @retval The Lcd Pixel Width
  */
uint16_t st7735_GetLcdPixelWidth(void)
{
  return ST7735_LCD_PIXEL_WIDTH;
}

/**
  * @brief  Gets the LCD pixel Height.
  * @param  None
  * @retval The Lcd Pixel Height
  */
uint16_t st7735_GetLcdPixelHeight(void)
{
  return ST7735_LCD_PIXEL_HEIGHT;
}

/**
  * @brief  Displays a bitmap picture loaded in the internal Flash.
  * @param  BmpAddress: Bmp picture address in the internal Flash.
  * @retval None
  */
void st7735_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, size = 0;

  /* Read bitmap size */
  size = *(volatile uint16_t *) (pbmp + 2);
  size |= (*(volatile uint16_t *) (pbmp + 4)) << 16;
  /* Get bitmap data address offset */
  index = *(volatile uint16_t *) (pbmp + 10);
  index |= (*(volatile uint16_t *) (pbmp + 12)) << 16;
  size = (size - index)/2;
  pbmp += index;

  /* Set GRAM write direction and BGR = 0 */
  /* Memory access control: MY = 0, MX = 1, MV = 0, ML = 0 */
  st7735_WriteReg(LCD_REG_54, 0x40);

  /* Set Cursor */
  st7735_SetCursor(Xpos, Ypos);

  LCD_IO_WriteMultipleData((uint8_t*)pbmp, size*2);

  /* Set GRAM write direction and BGR = 0 */
  /* Memory access control: MY = 1, MX = 1, MV = 0, ML = 0 */
  st7735_WriteReg(LCD_REG_54, 0xC0);
}
void TFTInit()
{

	TFTWriteCmd(ST7735_SWRESET); // software reset
	Tick(50);
	TFTWriteCmd(ST7735_SLPOUT);  // out of sleep mode
	Tick(500);

	TFTWriteCmd(ST7735_COLMOD);  // set color mode
	TFTWriteData(0x05);          // 16-bit color
	Tick(10);

	TFTWriteCmd(ST7735_FRMCTR1); // frame rate control
	TFTWriteData(0x00);          // fastest refresh
	TFTWriteData(0x06);          // 6 lines front porch
	TFTWriteData(0x03);          // 3 lines backporch
	Tick(10);

	TFTWriteCmd(ST7735_MADCTL);  // memory access control (directions)
	TFTWriteData(0xC8);          // row address/col address, bottom to top refresh

	TFTWriteCmd(ST7735_DISSET5); // display settings #5
	TFTWriteData(0x15);          // 1 clock cycle nonoverlap, 2 cycle gate rise, 3 cycle oscil. equalize
	TFTWriteData(0x02);          // fix on VTL

	TFTWriteCmd(ST7735_INVCTR);  // display inversion control
	TFTWriteData(0x0);           // line inversion

	TFTWriteCmd(ST7735_PWCTR1);  // power control
	TFTWriteData(0x02);          // GVDD = 4.7V
	TFTWriteData(0x03);          // 1.0uA
	Tick(10);
	TFTWriteCmd(ST7735_PWCTR2);  // power control
	TFTWriteData(0x05);          // VGH = 14.7V, VGL = -7.35V
	TFTWriteCmd(ST7735_PWCTR3);  // power control
	TFTWriteData(0x01);          // Opamp current small
	TFTWriteData(0x02);          // Boost frequency


	TFTWriteCmd(ST7735_VMCTR1);  // power control
	TFTWriteData(0x3C);          // VCOMH = 4V
	TFTWriteData(0x38);          // VCOML = -1.1V
	Tick(10);

	TFTWriteCmd(ST7735_PWCTR6);  // power control
	TFTWriteData(0x11);
	TFTWriteData(0x15);

	TFTWriteCmd(ST7735_GMCTRP1);
	TFTWriteData(0x09);
	TFTWriteData(0x16);
	TFTWriteData(0x09);
	TFTWriteData(0x20);
	TFTWriteData(0x21);
	TFTWriteData(0x1B);
	TFTWriteData(0x13);
	TFTWriteData(0x19);
	TFTWriteData(0x17);
	TFTWriteData(0x15);
	TFTWriteData(0x1E);
	TFTWriteData(0x2B);
	TFTWriteData(0x04);
	TFTWriteData(0x05);
	TFTWriteData(0x02);
	TFTWriteData(0x0E);
	TFTWriteCmd(ST7735_GMCTRN1);
	TFTWriteData(0x0B);
	TFTWriteData(0x14);
	TFTWriteData(0x08);
	TFTWriteData(0x1E);
	TFTWriteData(0x22);
	TFTWriteData(0x1D);
	TFTWriteData(0x18);
	TFTWriteData(0x1E);
	TFTWriteData(0x1B);
	TFTWriteData(0x1A);
	TFTWriteData(0x24);
	TFTWriteData(0x2B);
	TFTWriteData(0x06);
	TFTWriteData(0x06);
	TFTWriteData(0x02);
	TFTWriteData(0x0F);
	Tick(10);

	TFTWriteCmd(ST7735_CASET);	/* Set window area X*/
	TFTWriteData(0x00);
	TFTWriteData(0x02);
	TFTWriteData(0x00);
	TFTWriteData(0x81);
	Tick(10);
	TFTWriteCmd(ST7735_RASET);	/* Set window area Y*/
	TFTWriteData(0x00);
	TFTWriteData(0x02);
	TFTWriteData(0x00);
	TFTWriteData(0x81F);
	Tick(10);

	TFTWriteCmd(ST7735_NORON);   // normal display on
	Tick(10);

	TFTWriteCmd(ST7735_DISPON);
	Tick(500);

	TFTWriteCmd(0x00);

	//TFTBackLight(1);

}
void TFTInit2()
{
	__HIGH(LCD_Reset);

	TFTWriteCmd(ST7735_SWRESET); // software reset
	Tick(250);
	TFTWriteCmd(ST7735_SLPOUT);  // out of sleep mode
	Tick(250);
	TFTWriteCmd(0x3A); //Interface pixel format
	//TFTWriteData(0xC6); //18 bit color/ 18bit/pixel
	TFTWriteData(0x55); //16bit
	TFTWriteCmd(0x26); //GAMMA Curve
	TFTWriteData(0x01); //CURVE 1

	TFTWriteCmd(0x13); //normal display mode ON

	TFTWriteCmd(0xB1); //Frame Rate 61,7Hz for 128*160 px
	TFTWriteData(0x0E);  //DIVA 14 decimal
	TFTWriteData(0x14);    //VPA 20 decimal

	TFTWriteCmd(0xB4);   //display invert frame
	TFTWriteData(0x00);  //0x07 old value

	TFTWriteCmd(0xC0);   //PWR CTRL 1
	TFTWriteData(0x0A);   //4.3VC
	TFTWriteData(0x03);   //2.6VRH

	TFTWriteCmd(0xC1);   //PWR CTRL 2
	TFTWriteData(0x02);   //VCL VGH	VGL 2 2xVCI -1xVCI1 2.5xAVDD -2.5xAVDD


	TFTWriteCmd(0xC5);   //VCOM Ctrl 1
	TFTWriteData(0x50);   //VCOML 4.50
	TFTWriteData(0x56);   //VCOML -0.350

	TFTWriteCmd(0xC7);   //VCOM Offset Ctrl
	TFTWriteData(0x00);

	TFTWriteCmd(0x2A);//Set Column Address
	TFTWriteData(0x00);
	TFTWriteData(0x7F); //127

	TFTWriteCmd(0x2B);//Set Page Address
	TFTWriteData(0x00);
	TFTWriteData(0x9F); //159

	TFTWriteCmd(ST7735_DISPON);  // set color mode
	Tick(50);

}
void TFTInit2_4Inch(void) {
    __HIGH(LCD_RD);
    __HIGH(LCD_Reset);
    HAL_Delay(5);
    __LOW(LCD_Reset);
    HAL_Delay(5);
    __HIGH(LCD_Reset);
    __LOW(LCD_RD);
    TFTWriteCmd(0x00);
    TFTWriteData(0x00);
    TFTWriteData(0x00);
    TFTWriteData(0x00);
    TFTWriteData(0x00);
    volatile static uint16_t a, d;
    uint8_t size = sizeof(_regValues_big);
          for (uint8_t i = 0; i < (size / 4); i++) {
              a = (_regValues_big[i*2]);
              d = (_regValues_big[i*2 + 1]);
              if (a == 0x00FF) {
                  HAL_Delay(d);
              } else {
        	  TFTWriteCmd(a>>8);
        	  TFTWriteCmd(a);
        	  TFTWriteData(d>>8);
        	  TFTWriteData(d);
              }
  }
}
void Tick(uint16_t i)
{
	HAL_Delay(i);
}
void TFTWriteCmd(const uint16_t command)
{
	/* wait for busy flag to be false */
	while(spi_TFT_busy_flag);
	/* set busy flag to high*/
	spi_TFT_busy_flag = 1;

	/* set speed of SPI to high */
        set_spi_high_speed(&SpiHandle);

        /* low TFT CS */
	GPIOB->ODR &=  ~((1<<6));

	/* low RS to send command */
	GPIOC->ODR &=  ~((1<<8)&PORTAMSK_RS_W_R);

	spi_send_U16(command);
	/*reset busy flag*/
	spi_TFT_busy_flag = 0;

	//Reset pins, ports etc.
	GPIOC->ODR &=  ~(0xFF&0b0000000100000000); //CLEAR RS
	GPIOB->ODR |=  ((1<<6)); //HIGH CS
}

void TFTWriteData(const uint16_t data)
{
    /* wait for busy flag to be false */
    while(spi_TFT_busy_flag);

    /* set busy flag to high*/
    spi_TFT_busy_flag = 1;

    /* set speed of SPI to high */
    set_spi_high_speed(&SpiHandle);

    /* low TFT CS */
    GPIOB->ODR &=  ~((1<<6));

    /* HIGH RS to send Data */
    GPIOC->ODR |=  ((1<<8)&PORTAMSK_RS_W_R);

    /* send spi stream */
    spi_send_U16(data);

    /*reset busy flag*/
    spi_TFT_busy_flag = 0;

    //Reset pins, ports etc.
    GPIOC->ODR &=  ~(0xFF&0b0000000100000000); //CLEAR RS
    GPIOB->ODR |=  ((1<<6)); //HIGH CS
}

void TFTSetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	TFTWriteCmd(ST7735_CASET);   /* col address command */
	TFTWriteData(0x00);
	TFTWriteData(x0);          /* X Start */
	TFTWriteData(0x00);
	TFTWriteData(x1);          /* X end */

	TFTWriteCmd(ST7735_RASET);   /* row address command */
	TFTWriteData(0x00);
	TFTWriteData(y0);          /* Y Start */
	TFTWriteData(0x00);
	TFTWriteData(y1);          /* Y end */
}
uint16_t TFTReadData()
{
	GPIOB->ODR &=  ~((1<<6)); //LOW CS

	uint8_t temp[2]={0};
	HAL_SPI_Receive(&SpiHandle,temp,2,5);
	uint16_t indata = ((temp[0])<<8)|((temp[1]));
	GPIOB->ODR |=  ((1<<6)); //HIGH CS
	return indata;
}
void TFTPixel(uint16_t x, uint16_t y, uint16_t color)
{
	TFTSetWindow(x, y, x+1 , y+1);
	TFTWriteCmd(ST7735_RAMWR);		/* RAM Access */

	TFTWriteData(color >> 8);
	TFTWriteData(color);
}
void ili9486_ini2(void) {
		TFTWriteCmd(0xB0);
    		TFTWriteData(0x00);
    		TFTWriteCmd(0x11);
    		HAL_Delay(10);

    		TFTWriteCmd(0xB3);
    		TFTWriteData(0x02);
    		TFTWriteData(0b00010100);
    		//TFTWriteData(0x00);
    		//TFTWriteData(0x00);

    		TFTWriteCmd(0xC0);
    		TFTWriteData(0b01110);//13
    		TFTWriteData(0b01110);//480


    		TFTWriteCmd(0xC1);
    		TFTWriteData(0b01100000);//TFTWriteData(0x08);
    		TFTWriteData(0x0);//TFTWriteData(0x16);//CLOCK


    		TFTWriteCmd(0xC4);
    		TFTWriteData(0x0b00110011);

    		TFTWriteCmd(0xC6);
    		TFTWriteData(0b0001000);
    		TFTWriteData(0x0);

    		/*TFTWriteCmd(0xC8);//GAMMA
    		TFTWriteData(0x03);
    		TFTWriteData(0x03);
    		TFTWriteData(0x13);
    		TFTWriteData(0x5C);
    		TFTWriteData(0x03);
    		TFTWriteData(0x07);
    		TFTWriteData(0x14);
    		TFTWriteData(0x08);
    		TFTWriteData(0x00);
    		TFTWriteData(0x21);
    		TFTWriteData(0x08);
    		TFTWriteData(0x14);
    		TFTWriteData(0x07);
    		TFTWriteData(0x53);
    		TFTWriteData(0x0C);
    		TFTWriteData(0x13);
    		TFTWriteData(0x03);
    		TFTWriteData(0x03);
    		TFTWriteData(0x21);
    		TFTWriteData(0x00);*/

    		TFTWriteCmd(0x35);
    		TFTWriteData(0x00);

    		TFTWriteCmd(0x36);
    		TFTWriteData(0x00);

    		TFTWriteCmd(0x3A);
    		TFTWriteData(0b01010101);

    		TFTWriteCmd(0x44);
    		TFTWriteData(0x00);
    		TFTWriteData(0x01);

    		/*TFTWriteCmd(0xB6);
    		TFTWriteData(0x00);
    		TFTWriteData(0x22);//0 GS SS SM ISC[3:0];其中GS SS控制显示方向，同时修改R36
    		TFTWriteData(0x3B);*/

    		/*TFTWriteCmd(0xD0);
    		TFTWriteData(0x07);
    		TFTWriteData(0x07);//VCI1
    		TFTWriteData(0x1D);//VRH*/

    		/*TFTWriteCmd(0xD1);
    		TFTWriteData(0x00);
    		TFTWriteData(0x03);//VCM
    		TFTWriteData(0x00);//VDV*/

    		/*TFTWriteCmd(0xD2);
    		TFTWriteData(0x03);
    		TFTWriteData(0x14);
    		TFTWriteData(0x04);*/

    		TFTWriteCmd(0x29);
    		HAL_Delay(2000);

    		TFTWriteCmd(0xB4);
    		TFTWriteData(0x00);
    		HAL_Delay(2000);
    		//TFTWriteCmd(0x2C);
}
void ili9486_ini(void) {
        TFTWriteCmd(0x11);		// Sleep OUT
    	HAL_Delay(50);

    	TFTWriteCmd(0xF2);		// ?????
    	TFTWriteData(0x1C);
    	TFTWriteData(0xA3);
    	TFTWriteData(0x32);
    	TFTWriteData(0x02);
    	TFTWriteData(0xb2);
    	TFTWriteData(0x12);
    	TFTWriteData(0xFF);
    	TFTWriteData(0x12);
    	TFTWriteData(0x00);

    	TFTWriteCmd(0xF1);		// ?????
    	TFTWriteData(0x36);
    	TFTWriteData(0xA4);

    	TFTWriteCmd(0xF8);		// ?????
    	TFTWriteData(0x21);
    	TFTWriteData(0x04);

    	TFTWriteCmd(0xF9);		// ?????
    	TFTWriteData(0x00);
    	TFTWriteData(0x08);

    	TFTWriteCmd(0xC0);		// Power Control 1
    	TFTWriteData(0x0d);
    	TFTWriteData(0x0d);

    	TFTWriteCmd(0xC1);		// Power Control 2
    	TFTWriteData(0x43);
    	TFTWriteData(0x00);

    	TFTWriteCmd(0xC2);		// Power Control 3
    	TFTWriteData(0x00);

    	TFTWriteCmd(0xC5);		// VCOM Control
    	TFTWriteData(0x00);
    	TFTWriteData(0x48);

    	TFTWriteCmd(0xB6);		// Display Function Control
    	TFTWriteData(0x00);
    	TFTWriteData(0x22);		// 0x42 = Rotate display 180 deg.
    	TFTWriteData(0x3B);

    	TFTWriteCmd(0xE0);		// PGAMCTRL (Positive Gamma Control)
    	TFTWriteData(0x0f);
    	TFTWriteData(0x24);
    	TFTWriteData(0x1c);
    	TFTWriteData(0x0a);
    	TFTWriteData(0x0f);
    	TFTWriteData(0x08);
    	TFTWriteData(0x43);
    	TFTWriteData(0x88);
    	TFTWriteData(0x32);
    	TFTWriteData(0x0f);
    	TFTWriteData(0x10);
    	TFTWriteData(0x06);
    	TFTWriteData(0x0f);
    	TFTWriteData(0x07);
    	TFTWriteData(0x00);

    	TFTWriteCmd(0xE1);		// NGAMCTRL (Negative Gamma Control)
    	TFTWriteData(0x0F);
    	TFTWriteData(0x38);
    	TFTWriteData(0x30);
    	TFTWriteData(0x09);
    	TFTWriteData(0x0f);
    	TFTWriteData(0x0f);
    	TFTWriteData(0x4e);
    	TFTWriteData(0x77);
    	TFTWriteData(0x3c);
    	TFTWriteData(0x07);
    	TFTWriteData(0x10);
    	TFTWriteData(0x05);
    	TFTWriteData(0x23);
    	TFTWriteData(0x1b);
    	TFTWriteData(0x00);

    	TFTWriteCmd(0x20);		// Display Inversion OFF
    	TFTWriteData(0x00);//C8

    	TFTWriteCmd(0x36);		// Memory Access Control
    	TFTWriteData(0x0A);

    	TFTWriteCmd(0x3A);		// Interface Pixel Format
    	TFTWriteData(0x55);

    	TFTWriteCmd(0x2A);		// Column Addess Set
    	TFTWriteData(0x00);
    	TFTWriteData(0x00);
    	TFTWriteData(0x01);
    	TFTWriteData(0xDF);

    	TFTWriteCmd(0x002B);		// Page Address Set
    	TFTWriteData(0x00);
    	TFTWriteData(0x00);
    	TFTWriteData(0x01);
    	TFTWriteData(0x3f);
    	HAL_Delay(50);
    	TFTWriteCmd(0x0029);		// Display ON
    	//TFTWriteCmd(0x002C); // Memory Write
}
void ili9481_ini(void) {
    TFTWriteCmd(0x11); TFTWriteData(0);
    HAL_Delay(50);
    TFTWriteCmd(0xD0);
    TFTWriteData(3);
    TFTWriteData(0x07);
    TFTWriteData(0x42);
    TFTWriteData(0x18);
    TFTWriteCmd(0xD1);
    TFTWriteData(3);
    TFTWriteData(0x00);
    TFTWriteData(0x07);
    TFTWriteData(0x10);
    TFTWriteCmd(0xD2);
    TFTWriteData(2);
    TFTWriteData(0x01);
    TFTWriteData(0x02);
    TFTWriteCmd(0xC0);
    TFTWriteData(5);
    TFTWriteData(0x10);
    TFTWriteData(0x3B);
    TFTWriteData(0x00);
    TFTWriteData(0x02);
    TFTWriteData(0x11);
    TFTWriteCmd(0xC5);
    TFTWriteData(1);
    TFTWriteData(0x03);
    TFTWriteCmd(0x36);
    TFTWriteData(1);
    TFTWriteData(0x0A);
    TFTWriteCmd(0x3A);
    TFTWriteData(1);
    TFTWriteData(0x55);
    TFTWriteCmd(0x2A);
    TFTWriteData(4);
    TFTWriteData(0x00);
    TFTWriteData(0x00);
    TFTWriteData(0x01);
    TFTWriteData(0x3F);
    TFTWriteCmd(0x2B);
    TFTWriteData(4);
    TFTWriteData(0x00);
    TFTWriteData(0x00);
    TFTWriteData(0x01);
    TFTWriteData(0xE0);
    HAL_Delay(50);
    TFTWriteCmd(0x29);
    TFTWriteData(0);
    TFTWriteCmd(0x2C);
    TFTWriteData(0);

       HAL_Delay(500);
       //setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);
}
void ili9341_init_original(){
    //************* Start Initial Sequence **********//
    TFTWriteCmd(0x0);
    HAL_Delay(200);

    TFTWriteCmd(0xCB);
    TFTWriteData (0x39);
    TFTWriteData (0x2C);
    TFTWriteData (0x00);
    TFTWriteData (0x34);
    TFTWriteData (0x02);
    TFTWriteCmd(0xCF);
    TFTWriteData (0x00);
    TFTWriteData (0XC1);
    TFTWriteData (0X30);
    TFTWriteCmd(0xE8);
    TFTWriteData (0x85);
    TFTWriteData (0x00);
    TFTWriteData (0x78);
    TFTWriteCmd(0xEA);
    TFTWriteData (0x00);
    TFTWriteData (0x00);
    TFTWriteCmd(0xED);
    TFTWriteData (0x64);
    TFTWriteData (0x03);
    TFTWriteData (0x12);
    TFTWriteData (0x81);
    TFTWriteCmd(0xF7);
    TFTWriteData (0x20);
    TFTWriteCmd(0xC0); //Power control
    TFTWriteData (0x1b); //VRH[5:0]
    TFTWriteCmd(0xC1); //Power control
    TFTWriteData (0x10); //SAP[2:0];BT[3:0]
    TFTWriteCmd(0xC5); //VCM control
    TFTWriteData (0x2d);
    TFTWriteData (0x33);
    //TFTWriteCmd(0xC7);
    //VCM control2
    //TFTWriteData (0xCf);
    TFTWriteCmd(0x36);
    // Memory Access Control
    TFTWriteData (0x48);
    TFTWriteCmd(0xB1);
    TFTWriteData (0x00);
    TFTWriteData (0x1d);
    TFTWriteCmd(0xB6);
    // Display Function Control
    TFTWriteData (0x0A);
    TFTWriteData (0x02);
    TFTWriteCmd(0xF2);
    // 3Gamma Function Disable
    TFTWriteData (0x00);
    TFTWriteCmd(0x26);
    //Gamma curve selected
    TFTWriteData (0x01);
    TFTWriteCmd(0xE0);
    //Set Gamma
    TFTWriteData (0x0F);
    TFTWriteData (0x3a);
    TFTWriteData (0x36);
    TFTWriteData (0x0b);
    TFTWriteData (0x0d);
    TFTWriteData (0x06);
    TFTWriteData (0x4c);
    TFTWriteData (0x91);
    TFTWriteData (0x31);
    TFTWriteData (0x08);
    TFTWriteData (0x10);
    TFTWriteData (0x04);
    TFTWriteData (0x11);
    TFTWriteData (0x0c);
    TFTWriteData (0x00);
    TFTWriteCmd(0XE1);
    //Set Gamma
    TFTWriteData (0x00);
    TFTWriteData (0x06);
    TFTWriteData (0x0a);
    TFTWriteData (0x05);
    TFTWriteData (0x12);
    TFTWriteData (0x09);
    TFTWriteData (0x2c);
    TFTWriteData (0x92);
    TFTWriteData (0x3f);
    TFTWriteData (0x08);
    TFTWriteData (0x0e);
    TFTWriteData (0x0b);
    TFTWriteData (0x2e);
    TFTWriteData (0x33);
    TFTWriteData (0x0F);
    TFTWriteCmd(0x11);
    //Exit Sleep
    HAL_Delay(120);
    TFTWriteCmd(0x29);
    //Display on
    }

void lcdInitIli9341(void)
    {
    // VCI=2.8V
    //************* Reset LCD Driver ****************//
    //************* Start Initial Sequence **********//
    TFTWriteCmd(0);
    HAL_Delay(250);
    /*TFTWriteCmd(0xCF);
    TFTWriteData (0x00);
    TFTWriteData (0x83);
    TFTWriteData (0X30);
    TFTWriteCmd(0xED);
    TFTWriteData (0x64);
    TFTWriteData (0x03);
    TFTWriteData (0X12);
    TFTWriteData (0X81);
    TFTWriteCmd(0xE8);
    TFTWriteData (0x85);
    TFTWriteData (0x01);
    TFTWriteData (0x79);
    TFTWriteCmd(0xCB);
    TFTWriteData (0x39);
    TFTWriteData (0x2C);
    TFTWriteData (0x00);
    TFTWriteData (0x34);
    TFTWriteData (0x02);
    TFTWriteCmd(0xF7);
    TFTWriteData (0x20);

    TFTWriteCmd(0xEA);
    TFTWriteData (0x00);
    TFTWriteData (0x00);*/
    /*TFTWriteCmd(0xC0); //Power control
    TFTWriteData (0b00000011); //VRH[5:0] 3V
    TFTWriteCmd(0xC1); //Power control
    TFTWriteData (0b00010011); //SAP[2:0];BT[3:0]
    TFTWriteCmd(0xC5); //VCM control
    TFTWriteData (0b00010000);
    TFTWriteData (0b00110010);
    //VCM control2
    TFTWriteCmd(0xC7);
    TFTWriteData (0b10111110);
    // Memory Access Control
    TFTWriteCmd(0x36);
    TFTWriteData (0b00000000);
    TFTWriteCmd(0xB1);
    TFTWriteData (0b01);
    TFTWriteData (0b00011011);
    // Display Function Control
    TFTWriteCmd(0xB6);
    TFTWriteData(0b00001010);
    TFTWriteData(0b00100001);
    TFTWriteData(0b00011100);//248lines
   /* // 3Gamma Function Disable
    TFTWriteCmd(0xF2);
    TFTWriteData (0x00);*/
    /* Gamma curve selected
    TFTWriteCmd(0x26);
    TFTWriteData (0x01);
    //Set Gamma
    TFTWriteCmd(0xE0);
    TFTWriteData (0x0F);
    TFTWriteData (0x23);
    TFTWriteData (0x1F);

    TFTWriteData (0x09);
    TFTWriteData (0x0f);
    TFTWriteData (0x08);
    TFTWriteData (0x4B);
    TFTWriteData (0Xf2);
    TFTWriteData (0x38);
    TFTWriteData (0x09);
    TFTWriteData (0x13);
    TFTWriteData (0x03);
    TFTWriteData (0x12);
    TFTWriteData (0x07);
    TFTWriteData (0x04);
    //Set Gamma
    TFTWriteCmd(0XE1);
    TFTWriteData (0x00);
    TFTWriteData (0x1d);
    TFTWriteData (0x20);
    TFTWriteData (0x02);
    TFTWriteData (0x11);
    TFTWriteData (0x07);
    TFTWriteData (0x34);
    TFTWriteData (0x81);
    TFTWriteData (0x46);
    TFTWriteData (0x06);
    TFTWriteData (0x0e);
    TFTWriteData (0x0c);
    TFTWriteData (0x32);
    TFTWriteData (0x38);
    TFTWriteData (0x0F); */
    //Exit Sleep
    TFTWriteCmd(0x11);
    HAL_Delay(120);
    TFTWriteCmd(0x29);
    //Display on
}
unsigned char lcdBuildMemoryAccessControlConfig(
                        bool rowAddressOrder,
                        bool columnAddressOrder,
                        bool rowColumnExchange,
                        bool verticalRefreshOrder,
                        bool colorOrder,
                        bool horizontalRefreshOrder){
  unsigned char value = 0;
  if(horizontalRefreshOrder) value |= 0x0004;
  if(colorOrder) value |= 0x0008;
  if(verticalRefreshOrder) value |= 0x0010;
  if(rowColumnExchange) value |= 0x0020;
  if(columnAddressOrder) value |= 0x0040;
  if(rowAddressOrder) value |= 0x0080;
  return value;
}
void ili9488_ini() {
	TFTWriteCmd(ILI9488_CMD_SOFTWARE_RESET);
	TFTWriteData(0x0);
    	HAL_Delay(200);

    	TFTWriteCmd(ILI9488_CMD_SLEEP_OUT);
    		TFTWriteData(0x0);
    	HAL_Delay(200);

    	/** make it tRGB and reverse the column order */
    	TFTWriteCmd(ILI9488_CMD_MEMORY_ACCESS_CONTROL);
    	 		TFTWriteData(0x48);
    	HAL_Delay(100);


    	TFTWriteCmd(ILI9488_CMD_CABC_CONTROL_9);
    		TFTWriteData(0x04);
    	HAL_Delay(100);


    	TFTWriteCmd(ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET);
    		TFTWriteData( 0x06);
    	HAL_Delay(100);
    	TFTWriteCmd(ILI9488_CMD_NORMAL_DISP_MODE_ON);
    		TFTWriteData(0);
    	HAL_Delay(100);

    	(ILI9488_CMD_DISPLAY_ON);
    		TFTWriteData(0);
    	HAL_Delay(100);

    	//ili9488_set_display_direction(LANDSCAPE);
    	HAL_Delay(100);

    	//ili9488_set_window(0, 0,p_opt->ul_width,p_opt->ul_height);
    	//ili9488_set_foreground_color(p_opt->foreground_color);
}

void ili9341_ini_adafruit() {
      TFTWriteCmd(0xCB);

      TFTWriteData(0x39);

      TFTWriteData(0x2C);

      TFTWriteData(0x00);

      TFTWriteData(0x34);

      TFTWriteData(0x02);



      TFTWriteCmd(0xCF);

      TFTWriteData(0x00);

      TFTWriteData(0XC1);

      TFTWriteData(0X30);



      TFTWriteCmd(0xE8);

      TFTWriteData(0x85);

      TFTWriteData(0x00);

      TFTWriteData(0x78);



      TFTWriteCmd(0xEA);

      TFTWriteData(0x00);

      TFTWriteData(0x00);



      TFTWriteCmd(0xED);

      TFTWriteData(0x64);

      TFTWriteData(0x03);

      TFTWriteData(0X12);

      TFTWriteData(0X81);



      TFTWriteCmd(0xF7);

      TFTWriteData(0x20);



      TFTWriteCmd(0xC0);    //Power control

      TFTWriteData(0x23);   //VRH[5:0]



      TFTWriteCmd(0xC1);    //Power control

      TFTWriteData(0x10);   //SAP[2:0];BT[3:0]



      TFTWriteCmd(0xC5);    //VCM control

      TFTWriteData(0x3e);

      TFTWriteData(0x28);



      TFTWriteCmd(0xC7);    //VCM control2

      TFTWriteData(0x86);  //--



      TFTWriteCmd(0x36);    // Memory Access Control

      TFTWriteData(0x48);



      TFTWriteCmd(0x3A);

      TFTWriteData(0x55);



      TFTWriteCmd(0xB1);

      TFTWriteData(0x00);

      TFTWriteData(0x18);



      TFTWriteCmd(0xB6);    // Display Function Control

      TFTWriteData(0x08);

      TFTWriteData(0x82);

      TFTWriteData(0x27);



      TFTWriteCmd(0xF2);    // 3Gamma Function Disable

      TFTWriteData(0x00);



      TFTWriteCmd(0x26);    //Gamma curve selected

      TFTWriteData(0x01);



      TFTWriteCmd(0xE0);    //Set Gamma

      TFTWriteData(0x0F);

      TFTWriteData(0x31);

      TFTWriteData(0x2B);

      TFTWriteData(0x0C);

      TFTWriteData(0x0E);

      TFTWriteData(0x08);

      TFTWriteData(0x4E);

      TFTWriteData(0xF1);

      TFTWriteData(0x37);

      TFTWriteData(0x07);

      TFTWriteData(0x10);

      TFTWriteData(0x03);

      TFTWriteData(0x0E);

      TFTWriteData(0x09);

      TFTWriteData(0x00);



      TFTWriteCmd(0XE1);    //Set Gamma

      TFTWriteData(0x00);

      TFTWriteData(0x0E);

      TFTWriteData(0x14);

      TFTWriteData(0x03);

      TFTWriteData(0x11);

      TFTWriteData(0x07);

      TFTWriteData(0x31);

      TFTWriteData(0xC1);

      TFTWriteData(0x48);

      TFTWriteData(0x08);

      TFTWriteData(0x0F);

      TFTWriteData(0x0C);

      TFTWriteData(0x31);

      TFTWriteData(0x36);

      TFTWriteData(0x0F);



      TFTWriteCmd(0x11);    //Exit Sleep

   //   delayms(120);

        HAL_Delay(200);



      TFTWriteCmd(0x29);    //Display on
}

void ili9481_initGit(void) {
    /* SLP_OUT - Sleep out */
    TFTWriteCmd(0x11);
    	HAL_Delay(50);
    	/* Power setting */
    	TFTWriteCmd(0xD0); TFTWriteData(0x07); TFTWriteData(0x42); TFTWriteData(0x18);
    	/* VCOM */
    	TFTWriteCmd( 0xD1);TFTWriteData(0x00);TFTWriteData(0x07);TFTWriteData(0x10);
    	/* Power setting for norm. mode */
    	TFTWriteCmd(0xD2);TFTWriteData(0x01);TFTWriteData(0x02);
    	/* Panel driving setting */
    	TFTWriteCmd(0xC0);TFTWriteData(0x10);TFTWriteData(0x3B);TFTWriteData(0x00);TFTWriteData(0x02);TFTWriteData( 0x11);
    	/* Frame rate & inv. */
    	TFTWriteCmd(0xC5);TFTWriteData(0x03);
    	/* Pixel format */
    	TFTWriteCmd(0x3A);TFTWriteData(0x55);
    	/* Gamma */
    	TFTWriteCmd(0xC8); TFTWriteData(0x00); TFTWriteData(0x32); TFTWriteData(0x36); TFTWriteData(0x45); TFTWriteData(0x06); TFTWriteData(0x16);
    		TFTWriteData(0x37); TFTWriteData(0x75); TFTWriteData(0x77); TFTWriteData(0x54); TFTWriteData(0x0C); TFTWriteData(0x00);
    	/* DISP_ON */
	TFTWriteCmd(0x29);

}
/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

