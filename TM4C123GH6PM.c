#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"

#define CR   0x0D
#define LF   0x0A
#define BS   0x08
#define ESC  0x1B
#define SP   0x20
#define DEL  0x7F

#define UART_FR_TXFF            0x00000020  /* UART Transmit FIFO Full */
#define UART_FR_RXFE            0x00000010  /* UART Receive FIFO Empty */
#define UART_LCRH_WLEN_8        0x00000060  /* 8 bit word length */
#define UART_LCRH_FEN           0x00000010  /* UART Enable FIFOs */
#define UART_CTL_UARTEN         0x00000001  /* UART Enable */
#define SYSCTL_RCGC1_UART0      0x00000001  /* UART0 Clock Gating Control */
#define SYSCTL_RCGC2_GPIOA      0x00000001  /* port A Clock Gating Control */

#define GGA_Buffer_Size 80
#define GGA_Pointers_Size 20

char UART_InChar(void);
void UART_OutChar(char data);
void GPIOPortF_Handler(void);

void DisableInterrupts(void);

void EnableInterrupts(void);

void WaitForInterrupt(void);

void UART_HANDLER(void);

unsigned long int get_gpstime();
float get_latitude(unsigned char);
float get_longitude(unsigned char);
float get_altitude(unsigned char);
void convert_time_to_UTC(unsigned long int);
float convert_to_degrees(float);

void InitI2C(uint32_t BASE, bool mode);

uint32_t IsBMP180Available(void);
uint32_t IsTMP006Available(void);
uint32_t IsISL29023Available(void);
uint32_t IsMPUAvailable(void);

void MPU_init();
void MPU_Receive();
void delayMs(int);

char GGA_Buffer[GGA_Buffer_Size];              /* to store GGA string */
char GGA_CODE[3];

unsigned char N_S, E_W;                        /* for getting direction polarity */
unsigned char GGA_Pointers[GGA_Pointers_Size]; /* to store instances of ',' */
char CommaCounter;
char Data_Buffer[15];
volatile unsigned int GGA_Index;
volatile unsigned char  IsItGGAString   = 0;

uint32_t ReadBMPCalData(int *Cal_Arr); // Look at this function for multi byte read

void LCD_String_xy(int a,int b, char *c)
{
  int  i=0;
while(c[i]!='\0')
{
 printf("%c",(c[i]));
 i++;
}
printf("\n");
}

void LCD_String(char *c)
{
 int   i=0;
while(c[i]!='\0')
{
 printf("%c",((c[i])));
 i++;
}
printf("\n");
}





void DisableInterrupts(void)

{

    __asm ("    CPSID  I\n");

}




void EnableInterrupts(void)

{

    __asm  ("    CPSIE  I\n");

}


void WaitForInterrupt(void)

{

    __asm  ("    WFI\n");

}

char UART_InChar(void)
{


      while( (UART1_FR_R & UART_FR_RXFE) != 0)
          ;
         return((char)(UART1_DR_R & 0xFF));
}

void UART_Init(void)
{
    SYSCTL_RCGCUART_R |= 0x02;            /* activate UART0 */
    SYSCTL_RCGCGPIO_R |= 0x02;            /* activate port A */

    while((SYSCTL_PRGPIO_R&0x0002) == 0){}; /* ready? */

    UART1_CTL_R &= ~UART_CTL_UARTEN;      /* disable UART */
    UART1_IBRD_R = 104;        /* IBRD = int(16,000,000 / (16 * 115,200)) = int(8.680) */
    UART1_FBRD_R = 11;       /* FBRD = round(0.5104 * 64 ) = 44 */
    /* 8 bit word length (no parity bits, one stop bit, FIFOs) */
    UART1_LCRH_R = (UART_LCRH_WLEN_8);
    UART1_CTL_R |= UART_CTL_UARTEN;       /* enable UART */
    GPIO_PORTB_AFSEL_R |= 0x03;           /* enable alt funct on PA1-0 */
    GPIO_PORTB_DEN_R |= 0x03;             /* enable digital I/O on PA1-0 */
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFFFF00)+0x00000011; /* configure PA1-0 as UART */
    GPIO_PORTB_AMSEL_R &= ~0x03;          /* disable analog functionality on PA */

    //----------------------INterrupt

//    UART1_ICR_R = 0x10;        /*  Clear flag4 */

  //  UART1_IM_R |= 0x10;        /*  arm interrupt on PF4 */

    // NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF1FFFFF) | 0x00C00000;  /*  priority 5 */

    //NVIC_EN0_R = 0x00000040;        /*  Enable interrupt 5 in NVIC */

   // EnableInterrupts();             /* Enable global Interrupt flag (I) */

}


void  UART_HANDLER()
{
    printf("in interrupt");
    unsigned char received_char = UART_InChar();
    //c = UART_InChar();
    //UART_OutChar(c);     /* echo received char */
    UART1_ICR_R = 0x01;

                         /* Disable global interrupt */



        if(received_char =='$'){            /* check for '$' */
            GGA_Index = 0;
            IsItGGAString = 0;
            CommaCounter = 0;
        }
        else if(IsItGGAString == 1)
        {        /* if true save GGA info. into buffer */
            if(received_char == ',' )
                GGA_Pointers[CommaCounter++] = GGA_Index;    /* store instances of ',' in buffer */
            GGA_Buffer[GGA_Index++] = received_char;
        }
        else if(GGA_CODE[0] == 'G' && GGA_CODE[1] == 'G' && GGA_CODE[2] == 'A')
        { /* check for GGA string */
            IsItGGAString = 1;
            GGA_CODE[0] = 0; GGA_CODE[1] = 0; GGA_CODE[2] = 0;
        }
        else
        {
            GGA_CODE[0] = GGA_CODE[1];  GGA_CODE[1] = GGA_CODE[2]; GGA_CODE[2] = received_char;
        }
    }



unsigned long int get_gpstime(){
    unsigned char index;
    unsigned char Time_Buffer[15];
    unsigned long int _Time;
    memset(Time_Buffer,0,15);
    /* parse Time in GGA string stored in buffer */
    for(index = 0;GGA_Buffer[index]!=','; index++)
    {
        Time_Buffer[index] = GGA_Buffer[index];
    }
    _Time= atol(Time_Buffer);        /* convert string of Time to integer */
    return _Time;                    /* return integer raw value of Time */
}

float get_latitude(unsigned char lat_pointer)
{
    unsigned char lat_index = lat_pointer+1;    /* index pointing to the latitude */
    unsigned char index = 0;
    char Lat_Buffer[15];
    float _latitude;
    memset(Lat_Buffer,0,15);
    /* parse Latitude in GGA string stored in buffer */
    for(;GGA_Buffer[lat_index]!=',';lat_index++){
        Lat_Buffer[index]= GGA_Buffer[lat_index];
        index++;
    }
    lat_index++;
    N_S = GGA_Buffer[lat_index];
    _latitude = atof(Lat_Buffer);     /* convert string of latitude to float */
    return _latitude;                 /* return float raw value of Latitude */
}

float get_longitude(unsigned char long_pointer)
{
    unsigned char long_index;
    unsigned char index = long_pointer+1;       /* index pointing to the longitude */
    char Long_Buffer[15];
    float _longitude;
    long_index=0;
    memset(Long_Buffer,0,15);
    /* parse Longitude in GGA string stored in buffer */
    for( ; GGA_Buffer[index]!=','; index++){
        Long_Buffer[long_index]= GGA_Buffer[index];
        long_index++;
    }
    long_index++;
    E_W = GGA_Buffer[long_index];
    _longitude = atof(Long_Buffer);    /* convert string of longitude to float */
    return _longitude;                 /* return float raw value of Longitude */
}

float get_altitude(unsigned char alt_pointer)
{
    unsigned char alt_index;
    unsigned char index = alt_pointer+1;        /* index pointing to the altitude */
    char Alt_Buffer[12];
    float _Altitude;
    alt_index=0;
    memset(Alt_Buffer,0,15);
    /* parse Altitude in GGA string stored in buffer */
    for( ; GGA_Buffer[index]!=','; index++){
        Alt_Buffer[alt_index]= GGA_Buffer[index];
        alt_index++;
    }
    _Altitude = atof(Alt_Buffer);   /* convert string of altitude to float */
    return _Altitude;                   /* return float raw value of Altitude */
}

void convert_time_to_UTC(unsigned long int UTC_Time)
{
    unsigned int hour, min, sec;

    hour = (UTC_Time / 10000);                      /* extract hour from integer */
    min = (UTC_Time % 10000) / 100;                 /* extract minute from integer */
    sec = (UTC_Time % 10000) % 100;                 /* extract second from integer*/

    sprintf(Data_Buffer, "%d:%d:%d", hour,min,sec); /* store UTC time in buffer */

}

float convert_to_degrees(float NMEA_lat_long){

    float minutes, dec_deg, decimal;
    int degrees;
    float position;

    degrees = (int)(NMEA_lat_long/100.00);
    minutes = NMEA_lat_long - degrees*100.00;
    dec_deg = minutes / 60.00;
    decimal = degrees + dec_deg;
    if (N_S == 'S' || E_W == 'W') { // return negative
        decimal *= -1;
    }
    /* convert raw latitude/longitude into degree format */
    return decimal;
}


int main(void)
{   unsigned long int Time;
    float Latitude,Longitude,Altitude;
    int intLat,intLong,intAlt;
    char GPS_Buffer[15];
    char RxBuff[100]={};

    volatile uint32_t val;
    int cal_arr[11];

    SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ | SYSCTL_USE_PLL | SYSCTL_SYSDIV_4);

    InitI2C(I2C3_BASE, false);
    MPU_init();
   // UART_Init();


    val = IsMPUAvailable();
    printf("val = %d",val);

     if(val == 1)
    {
        while(1);

    }


    while(1)
    {   NVIC_EN0_R = 0x00000000;        /*  disable interrupt 5 in NVIC */
        DisableInterrupts();

        MPU_Start_Loc();
        MPU_Receive();

        UART_Init();
        UART1_ICR_R = 0x10;        /*  Clear flag4 */

            UART1_IM_R |= 0x10;        /*  arm interrupt on PF4 */

            NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF1FFFFF) | 0x00C00000; /*  priority 5 */
            NVIC_EN0_R = 0x00000040;
        EnableInterrupts();


        //UART_Init();
                //memset(GPS_Buffer,0,15);
                LCD_String_xy(1,0,"UTC Time: ");
                Time = get_gpstime();            /* Extract Time */
                convert_time_to_UTC(Time);       /* convert time to UTC */
                LCD_String(Data_Buffer);
                LCD_String("  ");

                LCD_String_xy(2,0,"Lat: ");
                Latitude = get_latitude(GGA_Pointers[0]);   /* Extract Latitude */
                Latitude = convert_to_degrees(Latitude);    /* convert raw latitude in degree decimal*/
                intLat= (int)(Latitude*1000000);
                printf("%d\n",intLat);
                //LCD_String1(GPS_Buffer);                     /* display latitude in degree */
                //memset(GPS_Buffer,0,15);

                LCD_String_xy(3,0,"Long: ");
                Longitude = get_longitude(GGA_Pointers[2]); /* Extract Latitude */
                Longitude = convert_to_degrees(Longitude);  /* convert raw longitude in degree decimal*/
                intLong= (int)(Longitude*1000000);

                printf("%d\n",intLong);


                LCD_String_xy(4,0,"Alt: ");
                Altitude = get_altitude(GGA_Pointers[7]);   /* Extract Latitude */
                intAlt =(int)(Altitude*1000000);

                printf("%d\n",intAlt);

    }


}

/* Initialize I2C in slow or fast mode
 * uint32_t BASE: I2C0_BASE, I2C1_BASE, I2C2_BASE, I2C3_BASE
 * bool mode: false (100KHz), true (400KHz)
 */
void InitI2C(uint32_t BASE, bool mode)
{
    uint32_t SYSCTL_PERIPH_I2C, SYSCTL_PERIPH_GPIO;
    uint32_t CONF_PIN_SCL, CONF_PIN_SDA;
    uint32_t GPIO_PORT_BASE;
    uint32_t PIN_SCL, PIN_SDA;
    switch(BASE)
    {
        case I2C0_BASE: SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C0;
                        SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOB;
                        CONF_PIN_SCL = GPIO_PB2_I2C0SCL;
                        CONF_PIN_SDA = GPIO_PB3_I2C0SDA;
                        GPIO_PORT_BASE = GPIO_PORTB_BASE;
                        PIN_SCL = GPIO_PIN_2;
                        PIN_SDA = GPIO_PIN_3;
                        break;

        case I2C1_BASE: SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C1;
                        SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOA;
                        CONF_PIN_SCL = GPIO_PA6_I2C1SCL;
                        CONF_PIN_SDA = GPIO_PA7_I2C1SDA;
                        GPIO_PORT_BASE = GPIO_PORTA_BASE;
                        PIN_SCL = GPIO_PIN_6;
                        PIN_SDA = GPIO_PIN_7;
                        break;

        case I2C2_BASE: SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C2;
                        SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOE;
                        CONF_PIN_SCL = GPIO_PE4_I2C2SCL;
                        CONF_PIN_SDA = GPIO_PE5_I2C2SDA;
                        GPIO_PORT_BASE = GPIO_PORTE_BASE;
                        PIN_SCL = GPIO_PIN_4;
                        PIN_SDA = GPIO_PIN_5;
                        break;

        case I2C3_BASE: SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C3;
                        SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOD;
                        CONF_PIN_SCL = GPIO_PD0_I2C3SCL;
                        CONF_PIN_SDA = GPIO_PD1_I2C3SDA;
                        GPIO_PORT_BASE = GPIO_PORTD_BASE;
                        PIN_SCL = GPIO_PIN_0;
                        PIN_SDA = GPIO_PIN_1;
                        break;

        default:        while(1);

    }

    // Enable I2C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C);
    //Wait Until I2C is Ready
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_I2C)));

    //Enable GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO);
    //Wait Until GPIO is Ready
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIO)));


    GPIOPinConfigure(CONF_PIN_SCL);
    GPIOPinConfigure(CONF_PIN_SDA);

    GPIOPinTypeI2C(GPIO_PORT_BASE, PIN_SDA);
    GPIOPinTypeI2CSCL(GPIO_PORT_BASE, PIN_SCL);

    I2CMasterInitExpClk(BASE,SysCtlClockGet(),mode);

}



/* Checks if BMP180 Pressure sensor is present bus or not.
 * Returns 0 if sensor is found else returns 1.
 */


/* Checks if MPU Inertial sensor is present bus or not.
 * Returns 0 if sensor is found else returns 1.
 */
uint32_t IsMPUAvailable(void)
{
    uint32_t val;

    I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
    I2CMasterDataPut(I2C3_BASE, 0x75);

    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C3_BASE));

    I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    while(I2CMasterBusy(I2C3_BASE));

    if(I2CMasterErr(I2C3_BASE)!= 0)
    {
        return 1;
    }

    val = I2CMasterDataGet(I2C3_BASE);

    if(val == 0x68)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void delayMs(int n)
{
int i, j;
for(i = 0 ; i < n; i++)
for(j = 0; j < 3180; j++) {}
     /* do nothing for 1 ms */
}

void MPU_init()
{
    delayMs(150);

    I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    I2CMasterDataPut(I2C3_BASE, 0x6B);
    while(I2CMasterBusy(I2C3_BASE));
    I2CMasterDataPut(I2C3_BASE, 0x01);
    while(I2CMasterBusy(I2C3_BASE));

    I2CMasterDataPut(I2C3_BASE, 0x1A);
    while(I2CMasterBusy(I2C3_BASE));
   I2CMasterDataPut(I2C3_BASE, 0x03);
        while(I2CMasterBusy(I2C3_BASE));

    I2CMasterDataPut(I2C3_BASE, 0x19);
    while(I2CMasterBusy(I2C3_BASE));
    I2CMasterDataPut(I2C3_BASE, 0x00);
    while(I2CMasterBusy(I2C3_BASE));

    I2CMasterDataPut(I2C3_BASE, 0x1B);
        while(I2CMasterBusy(I2C3_BASE));
        I2CMasterDataPut(I2C3_BASE, 0x18);
        while(I2CMasterBusy(I2C3_BASE));

        I2CMasterDataPut(I2C3_BASE, 0x1C);
            while(I2CMasterBusy(I2C3_BASE));
            I2CMasterDataPut(I2C3_BASE, 0x10);
            while(I2CMasterBusy(I2C3_BASE));

            I2CMasterDataPut(I2C3_BASE, 0x1A);
                while(I2CMasterBusy(I2C3_BASE));
                I2CMasterDataPut(I2C3_BASE, 0x00);
                while(I2CMasterBusy(I2C3_BASE));

    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C3_BASE));


 }

void MPU_Start_Loc()
{
    I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
    I2CMasterDataPut(I2C3_BASE, 0x3B);

    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C3_BASE));

}

void MPU_Receive()
{
    int axh,axl,ayh,ayl,azh,azl,th,tl,gxh,gxl,gyh,gyl,gzh,gzl;
    float xa,ya,za,T,xg,yg,zg;

    I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
    I2CMasterDataPut(I2C3_BASE, 0x3B);

    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C3_BASE));


    I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);
            I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
            if(I2CMasterErr(I2C3_BASE)!= 0)
            {
                    return 1;
            }

            axl = I2CMasterDataGet(I2C3_BASE);
            //printf("axl : %d",axl);
            //MPU_Start_Loc();

            I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
            I2CMasterDataPut(I2C3_BASE, 0x3C);

                I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                while(I2CMasterBusy(I2C3_BASE));


                I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);
                        I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
                        if(I2CMasterErr(I2C3_BASE)!= 0)
                        {
                                return 1;
                        }

            axh = I2CMasterDataGet(I2C3_BASE);
            //MPU_Start_Loc();
          //  printf("axh : %d",axh);

            I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
            I2CMasterDataPut(I2C3_BASE, 0x3D);

            I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
            while(I2CMasterBusy(I2C3_BASE));

            I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

            I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

                if(I2CMasterErr(I2C3_BASE)!= 0)
                        {
                            return 1;
                        }

                ayl = I2CMasterDataGet(I2C3_BASE);
                //printf("ayl : %d",ayl);

              //  MPU_Start_Loc();
                I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                         I2CMasterDataPut(I2C3_BASE, 0x3E);

                         I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                         while(I2CMasterBusy(I2C3_BASE));

                         I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                         I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

                    if(I2CMasterErr(I2C3_BASE)!= 0)
                            {
                                return 1;
                            }

                    ayh = I2CMasterDataGet(I2C3_BASE);
                   // printf("ayh : %d",ayh);
                   // I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                 //   MPU_Start_Loc();


                    I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                                I2CMasterDataPut(I2C3_BASE, 0x3F);

                                I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                                while(I2CMasterBusy(I2C3_BASE));

                                I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                    I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
                        if(I2CMasterErr(I2C3_BASE)!= 0)
                                {
                                    return 1;
                                }


                        azl = I2CMasterDataGet(I2C3_BASE);
                       // printf("azl : %d",azl);
                     //   MPU_Start_Loc();

                        I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                                                    I2CMasterDataPut(I2C3_BASE, 0x40);

                                                    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                                                    while(I2CMasterBusy(I2C3_BASE));

                                   I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                               I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

                            if(I2CMasterErr(I2C3_BASE)!= 0)
                                    {
                                        return 1;
                                    }


                            azh = I2CMasterDataGet(I2C3_BASE);
                            //printf("azh : %d",azh);
                            //printf("\n");

                            I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                                                        I2CMasterDataPut(I2C3_BASE, 0x41);

                                                        I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                                                        while(I2CMasterBusy(I2C3_BASE));

                                       I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                                   I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

                                if(I2CMasterErr(I2C3_BASE)!= 0)
                                        {
                                            return 1;
                                        }


                                tl = I2CMasterDataGet(I2C3_BASE);
                                //printf("tl : %d",tl);


                                I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                                                            I2CMasterDataPut(I2C3_BASE, 0x42);

                                                            I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                                                            while(I2CMasterBusy(I2C3_BASE));

                                           I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                                       I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

                                    if(I2CMasterErr(I2C3_BASE)!= 0)
                                            {
                                                return 1;
                                            }


                                    th = I2CMasterDataGet(I2C3_BASE);
                                    //printf("th  : %d",th);
                                    //printf("\n");


                                    I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                                                                I2CMasterDataPut(I2C3_BASE, 0x43);

                                                                I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                                                                while(I2CMasterBusy(I2C3_BASE));

                                               I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                                           I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

                                        if(I2CMasterErr(I2C3_BASE)!= 0)
                                                {
                                                    return 1;
                                                }


                                        gxl = I2CMasterDataGet(I2C3_BASE);
                                       // printf("gxl : %d",gxl);

                                        I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                                                                    I2CMasterDataPut(I2C3_BASE, 0x44);

                                                                    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                                                                    while(I2CMasterBusy(I2C3_BASE));

                                                   I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                                               I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

                                            if(I2CMasterErr(I2C3_BASE)!= 0)
                                                    {
                                                        return 1;
                                                    }


                                            gxh = I2CMasterDataGet(I2C3_BASE);
                                            //printf("gxh :%d",gxh);

                                            I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                                                                        I2CMasterDataPut(I2C3_BASE, 0x45);

                                                                        I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                                                                        while(I2CMasterBusy(I2C3_BASE));

                                                       I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                                                   I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

                                                if(I2CMasterErr(I2C3_BASE)!= 0)
                                                        {
                                                            return 1;
                                                        }


                                                gyl = I2CMasterDataGet(I2C3_BASE);
                                                //printf("gyl: %d",gyl);

                                                I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                                                                            I2CMasterDataPut(I2C3_BASE, 0x46);

                                                                            I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                                                                            while(I2CMasterBusy(I2C3_BASE));

                                                           I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                                                       I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

                                                    if(I2CMasterErr(I2C3_BASE)!= 0)
                                                            {
                                                                return 1;
                                                            }


                                                    gyh = I2CMasterDataGet(I2C3_BASE);
                                                    //printf("gyh : %d",gyh);

                                                    I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                                                                                I2CMasterDataPut(I2C3_BASE, 0x47);

                                                                                I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                                                                                while(I2CMasterBusy(I2C3_BASE));

                                                               I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                                                           I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

                                                        if(I2CMasterErr(I2C3_BASE)!= 0)
                                                                {
                                                                    return 1;
                                                                }


                                                        gzl = I2CMasterDataGet(I2C3_BASE);
                                                        //printf("gzl: %d",gzl);

                                                        I2CMasterSlaveAddrSet(I2C3_BASE,0x68,false);
                                                                                    I2CMasterDataPut(I2C3_BASE, 0x48);

                                                                                    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
                                                                                    while(I2CMasterBusy(I2C3_BASE));

                                                                   I2CMasterSlaveAddrSet(I2C3_BASE, 0x68, true);

                                                               I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

                                                            if(I2CMasterErr(I2C3_BASE)!= 0)
                                                                    {
                                                                        return 1;
                                                                    }


                                                            gzh = I2CMasterDataGet(I2C3_BASE);
                                                          //  printf("gzh: %d",gzh);
                                                            //printf("\n");

                                                            xa = (float)((256*axh +axl)/16384);
                                                            ya = (float)((256*ayh +ayl)/16384);
                                                            za = (float)((256*azh +azl)/16384);

                                                            T= (float)(256*th + tl);
                                                            T = (T/340) + 36.53;

                                                            xg = (float)((256*gxh + gxl)/131);
                                                            yg = (float)((256*gyh + gyl)/131);
                                                            zg = (float)((256*gzh + gzl)/131);

                                                            printf("Ax: %f \t Ay: %f \t Az: %f \t T: %f \t Gx: %f \t Gy: %f \t Gz: %f \t ",xa,ya,za,T,xg,yg,zg);
                                                            printf("\n");


    }

