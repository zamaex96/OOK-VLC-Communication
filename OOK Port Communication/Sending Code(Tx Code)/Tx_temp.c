#define F_CPU 16000000
#define CHECKBIT(x,y) (x & (y))
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
unsigned char data[32];
volatile long cnt;
//-----------------------------------serial----------------------------------------
void serial_init(void) 
{
  unsigned int baudrate[15] ={416,207,103,68,51,34,25,16,12,8,3,3,1,0}; 

  UBRR1L = baudrate[0];
  UBRR1H = baudrate[0]>>8;      
  
  UBRR0L = baudrate[12];
  UBRR0H = baudrate[12]>>8;         
   
  UCSR1C = (1 << UCSZ1) | (1 << UCSZ0);  // no parity ,  stop bit 1 , data 8
  UCSR1B = (1 << TXEN) | (1 << RXEN);
  //double baudrate 
   
  UCSR0C = (1 << UCSZ1) | (1 << UCSZ0);  //serial 8-bit format. no parity, stop bit 1, data 8
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);    //enable tx rx located in control reg UCSR1B
                                         //When set, these two bits turn on the serial buffers to allow for serial communications
//	UCSR0A |= (1 << U2X0); //double baudrate
//	 UCSR1A |= (1 << U2X1); //double baudrate
}


void Timer1_init(void) 
{

	//unsigned int cycles;
	unsigned long cycles;
  	// Initializing Timer
	TCCR1A = 0;                 // clear control register A 
	TCCR1B |= (1 << WGM13); // set mode 8: phase and frequency correct pwm, stop the timer
 	
	unsigned char oldSREG = SREG;
	cli();
   	//cycles = 10; // 2/(16000000)*10 = 1.25 us,no prescale
//	 cycles = 2; // 2/(16000000/8)*10 = 10us, prescale: clk / 8
	//cycles = 10; // 2/(16000000/8)*10 = 10us, *20 = 20us (50KHz)
	cycles = 200;	// 2/(16000000/64)*125 = 1ms , *25 = 0.2ms
	ICR1 = cycles;
	//sei();
	SREG = oldSREG;

	
	TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12));
	//TCCR1B |= (1 << CS10);	// no prescale, clk / 1
	TCCR1B |= (1 << CS11);	// prescale: clk / 8
	//TCCR1B |= (1 << CS12);	// prescale: clk / 256
	//TCCR1B |= (1 << CS11) | (1 << CS10);	// prescale: clk / 64
	//TCCR1B |= (1 << CS11) | (1 << CS10);	// prescale: clk / 64

	

}

ISR(TIMER1_OVF_vect)
{
	cnt++;
}



void transmit1(unsigned char data)
{
while(!(UCSR1A & (1<<UDRE1)));
UDR1=data;

}

void transmit0(unsigned char data)
{
while(!(UCSR0A & (1<<UDRE0)));
UDR0=data;

}

unsigned int Rxdata (void)
{

	// Wait for empty transmit buffer 
	while ( !(UCSR0A & (1<<RXC0)) );

	// Receive data from buffer, return the data 
	return UDR0;	

}


//Sending Data through port C

unsigned int send_code_portC(char data)
{
	unsigned char bit_to_check=0x80;
	unsigned int flag=0;
	while(flag<8)
	{
			
		if (!(CHECKBIT(data, bit_to_check)))
		{
			PORTC= 0x00;
		}
		else
		{	
			PORTC = 0xFF;
		}
		flag++;
		bit_to_check = bit_to_check >> 1;
		//_delay_us(2000);
		_delay_ms(2000);              // 50ms delay
	}
	return 1;
}

// Sending Data through port D pin 1

unsigned int send_code_portD(char data)
{
	unsigned char bit_to_check=0x80;
	unsigned int flag=0;//,cnt1=0;
	while(flag<8)
	{
			
		if (!(CHECKBIT(data, bit_to_check)))
		{
			PORTD= 0x00;
		}
		else
		{	
			PORTD = 0xFF;
		}
		flag++;
		bit_to_check = bit_to_check >> 1;

/*	cnt1 = 0, cnt=0;
		while(cnt1==0)
		{
			cnt1 = cnt;
		}*/
		_delay_us(1000000);
		//_delay_ms(4000);              // 50ms delay
	}
	return 1;
}

// Sending Data through port D pin 2

unsigned int send_code_portDp2(char data)
{
	unsigned char bit_to_check=0x80;
	unsigned int flag=0;
	while(flag<8)
	{
			
		if (!(CHECKBIT(data, bit_to_check)))
		{
			PORTD= 0x00;
		}
		else
		{	
			PORTD = 0x02;
		}
		flag++;
		bit_to_check = bit_to_check >> 1;
		_delay_us(50);
	}
	return 1;
}

//-------------------------------PM Sensor Data -----------------------------


unsigned int getSensorData(){
		
	unsigned int ctr;
	

	transmit0(0x42);
	transmit0(0x4d);
	transmit0(0xe2);
	transmit0(0x00);
	transmit0(0x00);
	transmit0(0x00);
	transmit0(0x00);

	for(ctr=0; ctr <32; ctr++)
		{
					

			data[ctr] = Rxdata();
		
		}

	//return data;
	return 0;

}

//Main Function----------------------------------------------------------------------------

int main(void)
{   
	serial_init();
	Timer1_init();
	DDRD=0xFF;
	TIMSK |= 0x04;
	SREG |= 0x80;
	DDRA=0xFF;		//sets portA pins as output 
	DDRC=0xFF;
	
	while(1){


//		getSensorData();
	/*	send_code_portDp2(0xFF);
		send_code_portDp2(0x80);
		send_code_portDp2(0xAA);
		send_code_portDp2(0x11);
		send_code_portDp2(0x22);
		send_code_portDp2(0xBB);
		send_code_portDp2(0x00);*/

		//send_code_portD(0xFF);
	//	send_code_portD(0x80);
	//	send_code_portD(0xAA);
	//	send_code_portC(0xAA);
		transmit0(0xAA);
		transmit1(0xAA);
	//	send_code_portD(0x44);
	//	send_code_portD(0x33);
	//	send_code_portD(0xBB);
	//	send_code_portD(0x00);
				

/*		send_code_portC(0xFF);
		send_code_portC(0x80);
		send_code_portC(0xBB);
		send_code_portC(data[7]);
		send_code_portC(data[8]);
		send_code_portC(0xDD);
		send_code_portC(0x00);*/
	}

}
