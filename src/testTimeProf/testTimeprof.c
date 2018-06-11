
#include "hardware_init.h"

void ProfTest() {
    test();test2();
}

void test(void){

static volatile unsigned char CAN[8] = {1,5,8,15,50,56,78,12};
#define __CONVERT_TO_16BIT(HI_B,LOW_B) (( HI_B << 8) | (LOW_B & 0x00FF))
#define __signal1 ((unsigned char)0x30) //0b00110000
#define __signal2 ((unsigned char)0x03) //0b00000011
#define __signal3 ((unsigned char)0x1b) //0b00011011
#define __CAN_BYTE_TO_SIGNAL(pointer,offset,mask) (*(pointer+offset))&(mask)

unsigned char *p = (unsigned char*) CAN;
volatile int signal3 = __CONVERT_TO_16BIT(__CAN_BYTE_TO_SIGNAL(p,7,__signal3),__CAN_BYTE_TO_SIGNAL(p,6,__signal3));
volatile int signal1 = __CAN_BYTE_TO_SIGNAL(p,4,__signal1);
volatile int signal2 = (*(p+6))&(__signal2);

struct test *A = (struct test*) CAN;

volatile int signalStruct1 = A->g;

volatile int tr = signal3+1;
static int num =1;
if(*(char *)&num == 1)
{
    HAL_Delay(100);//("\nLittle-Endian\n");
}
else
{
    HAL_Delay(1000);//("Big-Endian\n");
}

}

void test2(void){
    volatile char i;

    i=1;

    i++;

    if(i<1) { return; }
    else if(i>1) { HAL_Delay(100); }
}
