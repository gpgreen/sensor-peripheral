/*
 * sensor-peripheral
 *
 * This device will act as an SPI slave. It reads data from the
 * internal ADC These can then be retrieved by a SPI master
 * device. The device interface is register based. The SPI protocol
 * involves the master sending a byte that is the register address,
 * and then 2 more bytes. The slave returns a value in the second byte
 * corresponding to that value
 *
 * REGISTERS
 * 0x01 = adc channels requested
 *   second byte contains flags of which adc channels are requested, ie if bit 0 is set
 *   then adc channel 0 is requested and so on up to 8 channels.
 *   Third byte is zeros
 * 0x10-0x17 = retrieve adc value [address - 16], ie address 0x10 is adc channel 0
 *
 * SPI protocol is implemented using a state machine
 * state 0 = waiting for address byte
 * state 1 = second byte transfer finished
 * state 2 = third byte transfer finished
 * state 3 = transfer finished
 *
 * Device
 * ------
 * ATTiny26
 * signature = 0x1e9109
 * Fuse bits for ATTiny26
 *  Int RC osc 4MHz; Start-up time 6CK+64ms;[CKSEL=0011 SUT=10];
 *  Brown-out detection at VCC=4.0V; [BODLEVEL=0]
 *  Low=0xe3 Hi=0xf5
 *  from http://www.engbedded.com/fusecalc/
 *
 * to set fuses:
 * avrdude -c usbtiny -p attiny26 -U lfuse:w:0xe3:m -U hfuse:w:0xf5:m
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Define pin labels
// these are all portB
#define DI 0
#define DO 1
#define SCK 2
#define OC 3
//#define LED2 4
#define LED1 5
#define SS 6
#define RESET 7

// adc channels
volatile uint8_t adc_finished;
volatile uint8_t adc_channels;
// the channel being measured
int current_channel;

// array of adc values
uint16_t adc_values[8];

// spi
volatile uint8_t addr;          /* register address received via SPI */
volatile uint8_t spi_state;     /* current SPI state machine val */

void
ioinit(void)
{
    // set pullups on unused pins
    // PORTA setup pins for output
    
    // setup PINS for input, RESET is already set as input and pull up on
    // due to fuse setting
    DDRB &= ~(_BV(DI)|_BV(SCK)|_BV(SS));
    // set pullups on input pins
    PORTB |= _BV(DI)|_BV(SCK);
    
    // PORTB setup PINS for output
    DDRB |= _BV(DO)|_BV(LED1)|_BV(OC);
    // light up led
    PORTB |= _BV(LED1);

    // timer 0 set to overflow at fosc/256
    TCCR0 = _BV(CS02);
    TIMSK = _BV(TOIE0);

#if 0
    // timer 1 set to output compare B
    TCCR1A = _BV(COM1B0);                 /* toggle OC1A */
    // clear and prescale 256
    TCCR1B = _BV(CTC1)|_BV(CS13)|_BV(CS10);
    // interrupt at every 16us
    OCR1A = F_CPU / 256 / 100;
    //TIMSK |= _BV(OCIE1B);
#endif

    // start the ADC, and enable interrupt, scale clock by 8
    ADCSR = _BV(ADEN)|_BV(ADIE)|_BV(ADPS1)|_BV(ADPS0);
    // at start, no channels are in use
    adc_channels = 0;           /* the channels enabled mask */
    current_channel = -1;       /* channel currently measured */
    adc_finished = 0;

    // set SS pin change interrupt
    MCUCR |= _BV(ISC00);
    GIMSK |= _BV(INT0);

    // SPI starting state
    spi_state = 0;
}

int
main(void)
{
    ioinit();
    
	// start interrupts
	sei();

    // main loop
    while(1)
    {
        // if we now have a set of channels to look at,
        // start the measuring
        if (adc_channels > 0 && current_channel < 0)
        {
            // find the first channel
            for (uint8_t i=0; i<8; i++)
            {
                if (adc_channels & (1<<i))
                {
                    current_channel = i;
                    break;
                }
            }
            ADMUX = current_channel;
            
            // start a new conversion
            ADCSR |= _BV(ADSC);
        }

        if (adc_finished)
        {
            // if no channels are requested, then finish
            if (adc_channels == 0) {
                current_channel = -1;
            } else {
                // get the next channel
                uint8_t i = current_channel + 1;
            find_channel:
                while (i < 8)
                {
                    if (adc_channels & (1<<i)) {
                        current_channel = i;
                        break;
                    }
                    ++i;
                }
                // roll back to 0 if needed
                if (i == 8) {
                    i = 0;
                    goto find_channel;
                }
                // start new conversion
                ADMUX = current_channel;
                ADCSR |= _BV(ADSC);
            }
            adc_finished = 0;
        }

        // reset SPI if we have just handled a transaction
        if (spi_state == 3)
            spi_state = 0;
    }
    return 0;
}

/*
 * ADC complete interrupt
 * interrupt flag cleared by hardware
 */
ISR(ADC_vect)
{
    uint8_t low, high;
    low = ADCL;
    high = ADCH;
    adc_values[current_channel] = (high << 8) | low;
    adc_finished = 1;
}

/*
 * Timer0 overflow interrupt
 * interrupt flag cleared by hardware
 */
ISR(TIMER0_OVF0_vect)
{
    if (bit_is_clear(PINB, LED1)) PORTB |= _BV(LED1);
    else PORTB &= ~(_BV(LED1));
}

#if 0
/*
 * Timer1 compare B interrupt
 * interrupt flag cleared by hardware
 */
ISR(TIMER1_CMPB_vect)
{
    //if (bit_is_clear(PINB, LED2)) PORTB |= _BV(LED2);
    //else PORTB &= ~(_BV(LED2));
}
#endif

/*
 * INT0 interrupt
 * interrupt flag cleared by hardware
 */
ISR(INT0_vect)
{
    if (bit_is_clear(PINB, SS))
    {
        spi_state = 0;
        // spi transfer started
        // setup the USI as a SPI slave, with counter overflow interrupt
        USICR = _BV(USIOIE)|_BV(USIWM0)|_BV(USICS1);
        PORTB |= _BV(OC);
    } else {
        // turn off USI
        USICR = 0;
        PORTB &= ~(_BV(OC));
        spi_state = 0;
    }
}

/*
 * USI overflow interrupt
 * interrupt flag not cleared by hardware
 */
ISR(USI_OVF_vect)
{
    uint8_t recvd = USIDR;
    static uint8_t send2 = 0;
    
    switch (spi_state)
    {
    case 0: // second byte
        addr = recvd;
        if (addr >= 0x10 && addr < 0x18)
        {
            USIDR = (uint8_t)(adc_values[addr-0x10] & 0xFF);
            send2 = (uint8_t)((adc_values[addr-0x10] & 0xFF00) >> 8);
        }
        else if (addr == 0x01)
        {
            USIDR = 0;
            send2 = 0;
        } else
        {
            USIDR = 0;
            send2 = 0;
        }
        spi_state = 1;
        break;
    case 1: // third byte
        if (addr == 0x1)
            adc_channels = recvd;
        USIDR = send2;
        spi_state = 2;
        break;
    default: // end of transfer
        spi_state = 3;
        USIDR = 0;
        break;
    }

    // clear the interrupt flag
    USISR = _BV(USIOIF);
}
