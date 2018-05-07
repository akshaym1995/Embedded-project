/***********************************************************************************
/ * FIle: main.c
/ * Authors : Raghunath K P,Akshay Muraleedaran,Ashin Antotny
/ * 
/ * *********************************************************************************/

#include<pic18f4550.h>
#include<string.h>
#include<stdio.h>
#include<stdlib.h>
#include <math.h>
#include "uart.h"
#include "i2c.h"
#include "imu.h"
#include "Config.h"

unsigned long int get_gpstime();
float get_latitude_data(unsigned char);
float get_longitude_data(unsigned char);
float Get_altitude_data(unsigned char);
void convert_time_to_UTC(unsigned long int);
float convert_to_degrees(float);


int en_imu =0;
int en_gps =0;

#define GGA_Buffer_Size 80
#define GGA_Pointers_Size 20

char GGA_Buffer[GGA_Buffer_Size];              /* to store GGA string */
char GGA_CODE[3];

unsigned char N_S, E_W;                        /* for getting direction polarity */
unsigned char GGA_Pointers[GGA_Pointers_Size]; /* to store instances of ',' */
char CommaCounter;
char Data_Buffer[15];
volatile unsigned int GGA_Index;
volatile unsigned char	IsItGGAString	= 0;

void imu_init()										/* Gyro initialization function */
{
	MSdelay(150);		
	I2C_Start_Wait(0xD0);	
	I2C_Write(SMPLRT_DIV);	
	I2C_Write(0x07);	
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);	/* Write to power management register */
	I2C_Write(0x01);	
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);	
	I2C_Write(0x00);	
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);	
	I2C_Write(0x18);	
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);	
	I2C_Write(0x01);
	I2C_Stop();

}

void IMU_start()

{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */ 
	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
}

//Start timer for intialisation 
void Timer1_start()
{
    GIE=1;		/* Enable Global Interrupt */
    PEIE=1;  		/* Enable Peripheral Interrupt */
    TMR1IE=1;		/* Enable Timer1 Overflow Interrupt */
    TMR1IF=0;
    
    /* Enable 16-bit TMR1 register,no pre-scale,internal clock, timer OFF */
    T1CON=0x80;		

    TMR1=0xF856;	/* Load Count for generating delay of 1ms */
    TMR1ON=1;		/* Turn ON Timer1 */
}

/*Convert  lat long coordinate to cartesian coordiante system*/

float conv_gps_lat_long(float lat,float lon,float *x,float *y,float *z)
{
    int R= 6371000;
     
    
    *x = R * cos(lat) * cos(lon);

    *y = R * cos(lat) * sin(lon);

    *z = R *sin(lat);

}

//Enable timer 1 for running the gyro& acc data acquisition in 1 ms periodicty
#if 0

void interrupt Timer1_ISR()
{
    
    TMR1=0xF856;
    en_gps=1;  	/* 500 Hz */   
    PIR1bits.TMR1IF=0;  /* Make Timer1 Overflow Flag to '0' */
     count =count+1;
    if(count =100)
    {
        en_gps=1;
    }
}
#endif;

float Dispx=0,Dispy=0,Dispz=0;
void compute_position(float Accx,float Accy,float Accz,float gx,float gy,float gz,float *Disp,float *thetax,float *thetay,float *thetaz)
{

   // float Dispx,Dispy,Dispz;

	Dispx+=Accx*0.01*0.01;
	Dispy+=Accy*0.01*0.01;
	Dispz+=Accz*0.01*0.01;
	
    *Disp=sqrt(Dispx*Dispx+Dispy*Dispy+Dispz*Dispz);
	*thetax+=gx*0.01;
	*thetay+=gy*0.01;
	*thetaz+=gz*0.01;

}



void main(void) {

//GPS
	unsigned long int Time;
	float Latitude,Longitude,Altitude;
    double acc_mag=0;
    float thetax=0,thetay=0,thetaz=0;
	char GPS_Buffer[15];
    
    float GPS_pos_X,GPS_pos_Y,GPS_pos_Z;
//IMU    
    char buffer[20];
	int Ax,Ay,Az,T,Gx,Gy,Gz;
	float Xa,Ya,Za,t,Xg,Yg,Zg;
    float disp;
    
    float pos_x,pos_y,pos_z=0;
 //GPS   
	OSCCON = 0x72;      /* use internal osc. of 8MHz Freq. */
	//LCD_Init();

	I2C_Init();											/* Initialize I2C */	
	imu_init();										/* Initialize Gyro */
	USART_Init(9600);
//IMU
#if 0
    Timer1_start();
#endif;

    while(1)
    {
        INTCONbits.GIE=0;   /* enable Global Interrupt */
        INTCONbits.PEIE=0;  /* enable Peripheral Interrupt */
        PIE1bits.RCIE=0;    /* enable Receive Interrupt */
       
 
        
    	IMU_start();
        

        en_imu =0;
		/* Read Gyro values continuously & send to terminal over UART */
		Ax = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));

		Ay = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));

		Az = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));

		T =  (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));
		Gx = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));

		Gy = (((int)I2C_Read(0)<<8) | (int)I2C_Read(0));

		Gz = (((int)I2C_Read(0)<<8) | (int)I2C_Read(1));

        
		I2C_Stop();
        

        
		
		/* Derive the actual value of data from the parsed data  */
		Xa = (float)Ax/16384.0;	
		Ya = (float)Ay/16384.0;
		Za = (float)Az/16384.0;
		Xg = (float)Gx/131.0;
		Yg = (float)Gy/131.0;
		Zg = (float)Gz/131.0;
		t = ((float)T/340.00)+36.53;/* Convert temperature in °/c */
 

        //compute postion
        compute_position( Xa, Ya, Za, Xg, Yg,Zg,&disp, &thetax,&thetay,&thetaz);
      

  
//Enbale gps after 100 minor cycle         
#if 0        
        if(en_gps='1')

# endif;
         {
        INTCONbits.GIE=1;   /* enable Global Interrupt */
        INTCONbits.PEIE=1;  /* enable Peripheral Interrupt */
        PIE1bits.RCIE=1;    /* enable Receive Interrupt */
       
        /*Extract Time information from GPS CCGA Frame*/
        Time = get_gpstime();            /* Extract Time */
        convert_time_to_UTC(Time);       /* convert time to UTC */
         
        /*Extract Latitude from GPS CCGA Frame*/
         Latitude = get_latitude_data(GGA_Pointers[0]); 	/* Extract Latitude */
         Latitude = convert_to_degrees(Latitude);  	/* convert raw latitude in degree decimal*/

       /*Extract Longitude from CCGA Frame*/
         Longitude = get_longitude_data(GGA_Pointers[2]);	/* Extract Latitude */
         Longitude = convert_to_degrees(Longitude);	/* convert raw longitude in degree decimal*/


         
         Altitude = Get_altitude_data(GGA_Pointers[7]); 	/* Extract Latitude */

        
         conv_gps_lat_long(Latitude,Longitude,&GPS_pos_X,&GPS_pos_Y,&GPS_pos_Z);

         en_gps =0;
         
        memset(GPS_Buffer,0,15);
        sprintf(GPS_Buffer,"UTC Time: ");
        USART_SendString(GPS_Buffer);
        
        USART_SendString(Data_Buffer);
        USART_SendString(" \r\n ");
        
         memset(GPS_Buffer,0,15);
         sprintf(GPS_Buffer,"Latitude: ");
         USART_SendString(GPS_Buffer);   
         
         sprintf(GPS_Buffer,"%.05f\r\n",Latitude);		/* convert float value to string */
         USART_SendString(GPS_Buffer);            			/* display latitude in degree */
       
         //Longitude form GPS
         memset(GPS_Buffer,0,15);		
         sprintf(GPS_Buffer,"Longitude: ");
         USART_SendString(GPS_Buffer); 
         
         sprintf(GPS_Buffer,"%.05f\r\n",Longitude);		/* convert float value to string */
         USART_SendString(GPS_Buffer);            			/* display latitude in degree */
    
         //Latitude from GPS
         memset(GPS_Buffer,0,15);			
         sprintf(GPS_Buffer,"Latitude: ");
         USART_SendString(GPS_Buffer);  
   
         //ALtitutude from GPS
         memset(GPS_Buffer,0,15);
         sprintf(GPS_Buffer,"%.2f\r\n",Altitude);		/* convert float value to string */
         USART_SendString(GPS_Buffer);            			/* display latitude in degree */

         //Derived Position in Y
         pos_x = disp*cos(thetax);
         
         //Derived Position in Y
         pos_y = disp*cos(thetay);
         
         //Derived Position Z
         pos_z = disp*cos(thetaz);
  
#if 0
         
         //Adaptive Kalman filter to be implemented here
         //for GPS  aided Navigation
         if(pos_x-100 <GPS_pos_X >pos_x+100)
         {
             pos_x=GPS_pos_X;
         }
         if(pos_y-100 <GPS_pos_Y >pos_y+100)
         {
             pos_y=GPS_pos_Y;
         }
         if(pos_z-100 <GPS_pos_Z >pos_z+100)
         {
             pos_z=GPS_pos_Z;
         }
#endif;
       
         //Lateral Yaw 
		sprintf(buffer," Ax = %.2f g\r\n",Xa);
		USART_SendString(buffer);

        //Lateral pitch acceleration
		sprintf(buffer," Ay = %.2f g\r\n",Ya);
		USART_SendString(buffer);
		
        // Thrust acceleration
		sprintf(buffer," Az = %.2f g\r\n",Za);
		USART_SendString(buffer);

		//Temeparture in Degree celcius 
		sprintf(buffer," T = %.2f%c\r\n",t,0xF8);
		USART_SendString(buffer);

        //Gyro X output
		sprintf(buffer," Gx = %.2f%c\r\n",Xg,0xF8);
		USART_SendString(buffer);

        //Gyro Y output
		sprintf(buffer," Gy = %.2f%c\r\n",Yg,0xF8);
		USART_SendString(buffer);
		
        //gYRO z output
		sprintf(buffer," Gz = %.2f%c\r\n",Zg,0xF8);
		USART_SendString(buffer);
        
        //Angle turned in X direction
        sprintf(buffer," thetax = %.2f\r\n",thetax);
		USART_SendString(buffer);
        
        //Angle turned in Y direction
        sprintf(buffer," thetay = %.2f\r\n",thetay);
		USART_SendString(buffer);
        
        //ANgle turned in Z direction
        sprintf(buffer," thetaz = %.2f\r\n",thetaz);
		USART_SendString(buffer);
        
        //Displacement
        sprintf(buffer," disp = %.2f\r\n",disp);
		USART_SendString(buffer);
        
         
       
        }
	}
}

unsigned long int get_gpstime(){
	unsigned char index;
	unsigned char Time_Buffer[15];
	unsigned long int _Time;
	
	/* parse Time in GGA string stored in buffer */
	for(index = 0;GGA_Buffer[index]!=','; index++){		
		Time_Buffer[index] = GGA_Buffer[index];
	}
	_Time= atol(Time_Buffer);        /* convert string of Time to integer */
	return _Time;                    /* return integer raw value of Time */        
}

float get_latitude_data(char lat_pointer){
	unsigned char lat_index = lat_pointer+1;	/* index pointing to the latitude */
	unsigned char index = 0;
	char Lat_Buffer[15];
	float _latitude;

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

float get_longitude_data(unsigned char long_pointer){
	unsigned char long_index;
	unsigned char index = long_pointer+1;		/* index pointing to the longitude */
	char Long_Buffer[15];
	float _longitude;
	long_index=0;
	
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

float Get_altitude_data(unsigned char alt_pointer){
	unsigned char alt_index;
	unsigned char index = alt_pointer+1;		/* index pointing to the altitude */
	char Alt_Buffer[12];
	float _Altitude;
	alt_index=0;
	
	/* parse Altitude in GGA string stored in buffer */
	for( ; GGA_Buffer[index]!=','; index++){
		Alt_Buffer[alt_index]= GGA_Buffer[index];
		alt_index++;
	}
	_Altitude = atof(Alt_Buffer);   /* convert string of altitude to float */ 
	return _Altitude;					/* return float raw value of Altitude */
}

void convert_time_to_UTC(unsigned long int UTC_Time)
{
	unsigned int hour, min, sec;
		
	hour = (UTC_Time / 10000);                  	/* extract hour from integer */
	min = (UTC_Time % 10000) / 100;             	/* extract minute from integer */
	sec = (UTC_Time % 10000) % 100;             	/* extract second from integer*/

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

void interrupt Serial_ISR()   
{
	 
	if(RCIF){
		GIE  = 0;							/* Disable global interrupt */
		unsigned char received_char = RCREG;
        if(RCSTAbits.OERR){                 /* check if any overrun occur due to continuous reception */           
            CREN = 0;
            NOP();
            CREN=1;
        }
        
		if(received_char =='$'){     	    /* check for '$' */
			GGA_Index = 0;
			IsItGGAString = 0;
			CommaCounter = 0;
		}
		else if(IsItGGAString == 1){        /* if true save GGA info. into buffer */
			if(received_char == ',' ) GGA_Pointers[CommaCounter++] = GGA_Index;    /* store instances of ',' in buffer */
			GGA_Buffer[GGA_Index++] = received_char;
        }
		else if(GGA_CODE[0] == 'G' && GGA_CODE[1] == 'G' && GGA_CODE[2] == 'A'){ /* check for GGA string */
			IsItGGAString = 1;
			GGA_CODE[0] = 0; GGA_CODE[1] = 0; GGA_CODE[2] = 0;	
		}
		else{
			GGA_CODE[0] = GGA_CODE[1];  GGA_CODE[1] = GGA_CODE[2]; GGA_CODE[2] = received_char; 
		}	
	}
}