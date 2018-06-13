#include <Touch/touch.h>

void GUI_TOUCH_X_ActivateX(void) {} //empty
void GUI_TOUCH_X_ActivateY(void) {} //empty
int GUI_TOUCH_X_MeasureX(void) {

    /* Frame of the command for XPT2046
    BIT7(MSB) BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0(LSB)
    S A2 A1 A0 MODE (SER/DFR) PD1 PD0
    S always high, 001=xp read, 1=8bit ADC, 0=DFR, 00=power down mode */
   /* 1 001 0 0 01
    1 101 0 0 01
    1 011 0 0 01
    1 100 0 0 01
    1 001 0 0 00*/
    /* local variables */
    uint8_t frame_read_XP = 0b11011001; //dec:217
    uint8_t power_down_xpt2046 = 0b10011000; //dec:152
    uint8_t zero = 0;
    uint8_t buffer_tmp=0;
    uint8_t uart_buffer[50]={0};

    /* check if spi is busy for LCD */
    if(spi_TFT_busy_flag)
    {

	return -1;

    }

    /* check if touch pin is active */
    else if(__READ(TOUCH_INT))
    {
	//return -1;

    }

    /* set spi busy flag high*/
    spi_TFT_busy_flag = 1;

    /* set speed of SPI to low */
    set_spi_low_speed(&SpiHandle);

    /* set CS to low */
    GPIOC->ODR &=  ~((1<<6));

    /* send the command */
    HAL_SPI_TransmitReceive(&SpiHandle,&frame_read_XP,&buffer_tmp,1,500);
    //spi_send_read_U8(&frame_read_XP,&tmp_buffer);
    /* read the raw value */
    HAL_SPI_TransmitReceive(&SpiHandle,&zero,&buffer_tmp,1,500);
    //spi_send_read_U8(0,&tmp_buffer);
    /* power down xpt2046 */
    HAL_SPI_Transmit(&SpiHandle,&power_down_xpt2046,1,500);
    //spi_send_U8(power_down_xpt2046);

    /* set CS to high */
    GPIOC->ODR |=  ((1<<6));

    /* set spi busy flag low */
    spi_TFT_busy_flag = 0;
    sprintf(uart_buffer,"X: %d\r\n",buffer_tmp);
    HAL_UART_Transmit(&huart2,uart_buffer,sizeof(uart_buffer),5);
    /* return value */
    return buffer_tmp;

}
int GUI_TOUCH_X_MeasureY(void) {

    /* Frame of the command for XPT2046
    BIT7(MSB) BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0(LSB)
    S A2 A1 A0 MODE (SER/DFR) PD1 PD0
    S always high, 101=yp read, 1=8bit ADC, 0=DFR, 00=power down mode */
    /* local variables */
    uint8_t frame_read_YP = 0b10011001; //dec: 153
    uint8_t power_down_xpt2046 = 0b10011000; //dec: 152
    uint8_t zero = 0;
    uint8_t buffer_tmp=0;
    uint8_t uart_buffer[50]={0};


      /* check if spi is busy for LCD */
    if(spi_TFT_busy_flag)
    {

	return -1;

    }

    /* check if touch pin is active (low) */
    else if(__READ(TOUCH_INT))
    {
	//return -1;

    }

    /* set spi busy flag high*/
    spi_TFT_busy_flag = 1;

    /* set speed of SPI to low */
    set_spi_low_speed(&SpiHandle);

    /* set CS to low */
    GPIOC->ODR &=  ~((1<<6));


    /* send the command */
    HAL_SPI_TransmitReceive(&SpiHandle,&frame_read_YP,&buffer_tmp,1,500);
    //spi_send_read_U8(&frame_read_YP,&tmp_buffer);
    /* read the raw value */
    HAL_SPI_TransmitReceive(&SpiHandle,&zero,&buffer_tmp,1,500);
    //spi_send_read_U8(0,&tmp_buffer);
    /* power down xpt2046 */
    HAL_SPI_Transmit(&SpiHandle,&power_down_xpt2046,1,500);
    //spi_send_U8(power_down_xpt2046);

    /* set CS to high */
    GPIOC->ODR |=  ((1<<6));

    /* set spi busy flag low*/
    spi_TFT_busy_flag = 0;
    sprintf(uart_buffer,"Y: %d ",buffer_tmp);
    HAL_UART_Transmit(&huart2,uart_buffer,sizeof(uart_buffer),5);
    /* return value */
    return buffer_tmp;
}
