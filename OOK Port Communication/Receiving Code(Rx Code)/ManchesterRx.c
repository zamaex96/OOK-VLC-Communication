#define F_CPU 16000000
#define CHECKBIT(x,y) (x & (y))
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
volatile long cnt;
//-----------------------------------serial----------------------------------------
void serial_init(void) 
{
  unsigned int baudrate[15] ={416,207,103,68,51,34,25,16,12,8,3,3,1,0}; 

  UBRR1L = baudrate[4];
  UBRR1H = baudrate[4]>>8;      
  
  UBRR0L = baudrate[4];
  UBRR0H = baudrate[4]>>8;         
   
  UCSR1C = (1 << UCSZ1) | (1 << UCSZ0);  // no parity ,  stop bit 1 , data 8
  UCSR1B = (1 << TXEN) | (1 << RXEN);
  //double baudrate 
  UCSR0C = (1 << UCSZ1) | (1 << UCSZ0);  //serial 8-bit format. no parity, stop bit 1, data 8
  UCSR0B = (1 << TXEN) | (1 << RXEN);    //enable tx rx located in control reg UCSR1B
  //UCSR0A |= (1 << U2X0); //double baudrate
//	 UCSR1A |= (1 << U2X1); //double baudrate                                        //When set, these two bits turn on the serial buffers to allow for serial communications
}

void transmit(unsigned char data)
{
while(!(UCSR0A & (1<<UDRE0)));
UDR0=data;

}

unsigned char Rxdata (void)
{

	// Wait for empty transmit buffer 
	while ( !(UCSR1A & (1<<RXC1)) );

	// Receive data from buffer, return the data 
	return UDR1;	

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
	cycles = 200;
	//cycles = 10; // 2/(16000000)*10 = 1.25 us,no prescale
    //cycles = 1; // 2/(16000000/8)*1 = 1us, prescale: clk / 8
	//cycles = 20; // 2/(16000000/8)*10 = 10us, *20 = 20us (50KHz)
	//cycles = 125;	// 2/(16000000/64)*125 = 1ms , *25 = 0.2ms ,prescale: clk / 64
	ICR1 = cycles;
	//sei();
	SREG = oldSREG;

	
	TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12));
	TCCR1B |= (1 << CS10);	// no prescale
	//TCCR1B |= (1 << CS11);	// prescale: clk / 8
	//TCCR1B |= (1 << CS12);	// prescale: clk / 256
	//TCCR1B |= (1 << CS11) | (1 << CS10);	// prescale: clk / 64
	//TCCR1B |= (1 << CS11) | (1 << CS10);	// prescale: clk / 64

	

}

ISR(TIMER1_OVF_vect)
{
	cnt++;
}
//---------------------------------------------------Header Synchronization----------------------------------------------



unsigned int syncHeaderPortC()
{
	unsigned int temp;
	unsigned int byte_placeholder;
	unsigned int byte_placeholder1;
	unsigned int bit_to_check;
	unsigned int bit_to_shift;
	unsigned int predata,cnt1=0;
	bit_to_check 		= 0x01;
	byte_placeholder	= 0x00;
	byte_placeholder1	= 0xFF80;
	predata				= 0x00;

	temp = PINA;
	while(!(CHECKBIT(temp, bit_to_check))){temp = PINA; }//transmit(0x88);}


	while(1){
//		_delay_us(50);
	cnt1 = 0, cnt=0;
		while(cnt1==0)
		{
			cnt1 = cnt;
		}
	
		temp = PINA;	
		if(CHECKBIT(temp, bit_to_check)){
			
				bit_to_shift = 0x01;				
				byte_placeholder = byte_placeholder<<1;
				byte_placeholder = byte_placeholder|bit_to_shift;			
			}	
		else
			{
				bit_to_shift = 0x00;				
				byte_placeholder = byte_placeholder<<1;
				byte_placeholder = byte_placeholder|bit_to_shift;	
			}
		
		if(byte_placeholder == 0xFF80)
		break;
	}

//	transmit(byte_placeholder);
	return 1;
}

// --------------------------- Data Detection ----------------------------------

unsigned int shiftByte(){

	unsigned int temp;
	unsigned int byte_placeholder;
	unsigned int byte_placeholder1;
	unsigned int bit_to_check;
	unsigned int bit_to_shift;
	unsigned int predata,cnt1=0;
	int flag 			= 0;
	bit_to_check 		= 0x01;
	byte_placeholder	= 0x00;
	byte_placeholder1	= 0xFF80;
	predata				= 0x00;
	

	while(flag<8){
//		_delay_us(50);

	cnt1 = 0, cnt=0;
		while(cnt1==0)
		{
			cnt1 = cnt;
		}
	
		temp = PINA;
		
		if(CHECKBIT(temp, bit_to_check)){
			
				bit_to_shift = 0x01;				
				byte_placeholder = byte_placeholder<<1;
				byte_placeholder = byte_placeholder|bit_to_shift;
			
			}					
		else
			{
				bit_to_shift = 0x00;				
				byte_placeholder = byte_placeholder<<1;
				byte_placeholder = byte_placeholder|bit_to_shift;
			
			}


	flag++;
}
return byte_placeholder;
}



//Main Program----------------------------------------------------------------------------
int main(void){

	serial_init(); 
	Timer1_init(); 
	DDRA=0x00;
	PORTA=0xFF;
	TIMSK |= 0x04;
	SREG |= 0x80;
	unsigned int c,j;
	while(1){

	//syncHeaderPortC();
	//transmit(0xFF);
	//transmit(0x80);
	//for(j=0;j<4;j++)
		//{
			c = shiftByte();
			//c = Rxdata();
			transmit(c);
	//	}
	//transmit(0xFF);

	}

}
