/* *************************************************************************************************

    Geiger Counter with serial data logging firmware
    ================================================

    This firmware controls the Microchip (AVR) ATtiny2313 microcontroller on board the Geiger
    Counter kit (https://mightyohm.com/geiger).

    Copyright (c) 2011 Jeff Keyzer, MightyOhm Engineering, https://mightyohm.com/, jeff at mightyohm dot com
    Copyright (c) 2019 Philippe Kehl, flipflip industries, https://oinkzwurgl.org/projaeggd/geiger, flipflip at oinkzwurgl dot org

    This program is free software: you can redistribute it and/or modify it under the terms of the
    GNU General Public License as published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
    without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with this program.
    If not, see <http://www.gnu.org/licenses/>.


    Version history
    ---------------

    8/4/11   1.00: Initial release for Chaos Camp 2011!
    06-2019  2.00: Updated version for fun.


    Manual
    ------

    When an impulse from the Geiger-Müller (GM) tube is detected, the firmware flashes the LED and
    produces a short beep on the piezo speaker.  It also outputs an active-high pulse (default
    100us) on the PULSE pin.

    A pushbutton on the PCB can be used to mute the beep.

    A running average of the detected counts per second (CPS), counts per minute (CPM), and
    equivalent dose (uSv/hr) is output on the serial port once per second. The dose is based on
    information collected from the web, and may not be accurate.

    The serial port is configured for BAUD baud, 8-N-1 (default 9600).

    The data is reported in comma separated value (CSV) format:

        CPS, #####, CPM, #####, uSv/hr, ###.##, SLOW|FAST|INST

    Other (non data) output starts with a "#" character, e.g.:

        # hello world

    There are three modes. Normally, the sample period is LONG_PERIOD (default 60 seconds). This is
    SLOW averaging mode.  If the last five measured counts exceed a preset threshold, the sample
    period switches to SHORT_PERIOD seconds (default 5 seconds). This is FAST mode, and is more
    responsive but less accurate. Finally, if CPS > 255, we report CPS*60 and switch to INST mode,
    since we can't store data in the (8-bit) sample buffer. This behavior could be customized to
    suit a particular logging application.

    The largest CPS value that can be displayed is 65535, but the largest value that can be stored
    in the sample buffer is 255.

    ***** WARNING *****

    This Geiger Counter kit is for EDUCATIONAL PURPOSES ONLY.  Don't even think about using it to
    monitor radiation in life-threatening situations, or in any environment where you may expose
    yourself to dangerous levels of radiation.  Don't rely on the collected data to be an accurate
    measure of radiation exposure! Be safe!

************************************************************************************************** */

/* ***** includes ******************************************************************************** */

#include <stdlib.h>                     // libc: general utilities
#include <stdbool.h>                    // libc: boolean types and values
#include <avr/io.h>                     // avr: AVR device-specific IO definitions
#include <avr/interrupt.h>              // avr: interrupt things
#include <avr/pgmspace.h>               // avr: program space utilities
#include <avr/sleep.h>                  // avr: sleep mode utilities
#include <avr/wdt.h>                    // avr: watchdog timer handling
#include <util/delay.h>                 // avr: convenience functions for busy-wait delay loops
#include <util/delay.h>                 // avr: convenience functions for busy-wait delay loops

/* ***** defines (compile time configuration) **************************************************** */

#define VERSION         "2.00-DEV"
#define URL             "https://github.com/phkehl/geiger-counter"

#define F_CPU           8000000         // CPU clock speed in Hz
#define BAUD            9600            // serial BAUD rate
#define THRESHOLD       1000            // CPM threshold for fast avg mode
#define LONG_PERIOD     60              // number of samples to keep in memory in slow avg mode
#define SHORT_PERIOD    5               // number or samples for fast avg mode
#define SCALE_FACTOR    57              // CPM to uSv/hr conversion factor (x10,000 to avoid float)
#define PULSEWIDTH      100             // width of the PULSE output [us] (set to 0 to disable)
#define LINE_END        "\r\n"          // line termination for serial output

/* ***** utility functions *********************************************************************** */

// send a character to the UART
static void uart_putchar(char c)
{
    loop_until_bit_is_set(UCSRA, UDRE); // wait until UART is ready to accept a new character
    UDR = c;                            // send 1 character
}

// send a nul-terminated string in SRAM to the UART
static void uart_putstring(const char *buffer)
{
    // start sending characters over the serial port until we reach the end of the string
    while (*buffer != '\0')             // are we at the end of the string yet?
    {
        uart_putchar(*buffer);          // send the contents
        buffer++;                       // advance to next char in buffer
    }
}

// send a nul-terminated string in PROGMEM to the UART
static void uart_putstring_P(const char *buffer)
{
    // start sending characters over the serial port until we reach the end of the string
    while (pgm_read_byte(buffer) != '\0')         // are we done yet?
    {
        uart_putchar(pgm_read_byte(buffer++));    // read byte from PROGMEM and send it
    }
}

/* ***** global variables ************************************************************************ */

volatile bool     nobeep;               // flag used to mute beeper
volatile uint16_t count;                // number of GM events that have occurred
volatile uint16_t slowcpm;              // GM counts per minute in slow mode
volatile uint16_t fastcpm;              // GM counts per minute in fast mode
volatile uint16_t cps;                  // GM counts per second, updated once a second
volatile bool     overflow;             // overflow flag
volatile uint8_t  buffer[LONG_PERIOD];  // the sample buffer
volatile uint8_t  idx;                  // sample buffer index
volatile bool     eventflag;            // flag for ISR to tell main loop if a GM event has occurred
volatile bool     tick;                 // flag that tells main() when 1 second has passed
volatile uint16_t beepcount;            // flash and beep duration counter

/* ***** flashing and beeping ******************************************************************** */

// count: half period [ms], e.g. count=160 period = 320us, freq= 3.125kHz
// duration: duration of flash/beep TODO: make this [ms]
static void flash_and_beep(const uint8_t count, const uint8_t duration)
{
    PORTB |= _BV(PB4);	                // turn on the LED

    // no beeps in mute mode
    if (!nobeep)
    {
        TCCR0A |= _BV(COM0A0);          // enable OCR0A output on pin PB2
    }

    beepcount = (uint16_t)duration << 4;// countdown to stop the flash/beep
    TCCR0B |= _BV(CS01);                // set prescaler to clk/8 (1Mhz) or 1us/count
    OCR0A   = count;                    // toggle OCR0A every count [ms]
    TIMSK  |= _BV(OCIE0A);              // enable compare match A interrupt
}

ISR(TIMER0_COMPA_vect)
{
    beepcount--;                        // countdown
    if (beepcount == 0)                 // we're done, stop flash and beep
    {
        TIMSK &= ~_BV(OCIE0A);          // disable compare match A interrupt
        TCCR0B = 0;                     // disable clock source, stop Timer0
        TCCR0A &= ~(_BV(COM0A0));       // disconnect OCR0A from Timer0, this avoids occasional HVPS whine after beep
        PORTB &= ~(_BV(PB4));           // turn off the LED
    }
}

/* ***** button handling ************************************************************************* */

//  INT1 pin (pushbutton) interrupt, executed when the user pushes the button
//  We need to be careful about switch bounce, which will make the interrupt execute multiple times
//  if we're not careful.
ISR(INT1_vect)
{
    _delay_ms(25);                      // slow down interrupt calls (crude debounce)

    if ((PIND & _BV(PD3)) == 0)         // is button still pressed?
    {
        nobeep = !nobeep;               // toggle mute mode
    }

    EIFR |= _BV(INTF1);                 // clear interrupt flag to avoid executing ISR again due to switch bounce
}

/* ***** GM events handling ********************************************************************** */

//  INT0 pin is triggered on the falling edge of a GM pulse
ISR(INT0_vect)
{
    if (count < UINT16_MAX)             // check for overflow, if we do overflow just cap the counts at max possible
    {
        count++;                        // increase event counter
    }

    // send a pulse to the PULSE connector
    // a delay of 100us limits the max CPS to about 8000
    // you can set PULSEWIDTH to 0 and increase the max CPS possible (up to 65535!)
#if PULSEWIDTH > 0
    PORTD |= _BV(PD6);                  // set PULSE output high
    _delay_us(PULSEWIDTH);              // wait
    PORTD &= ~(_BV(PD6));               // set pulse output low
#endif

    eventflag = true;                   // tell main program loop that a GM pulse has occurred
}

/* ***** measurement handling ******************************************************************** */

// called once a second
ISR(TIMER1_COMPA_vect)
{
    tick = true;                        // update flag, signal main program loop

    //PORTB ^= _BV(PB4);                // toggle the LED (for debugging purposes)

    cps = count;
    slowcpm -= buffer[idx];             // subtract oldest sample in sample buffer

    if (count > UINT8_MAX)              // watch out for overflowing the sample buffer
    {
        count = UINT8_MAX;
        overflow = true;
    }

    slowcpm += count;                   // add current sample
    buffer[idx] = count;                // save current sample to buffer (replacing old value)

    // compute CPM based on the last SHORT_PERIOD samples
    fastcpm = 0;
    for (uint8_t i = 0; i < SHORT_PERIOD; i++)
    {
        int8_t x = idx - i;
        if (x < 0)
        {
            x = LONG_PERIOD + x;
        }
        fastcpm += buffer[x];   // sum up the last 5 CPS values
    }
    fastcpm = fastcpm * (LONG_PERIOD / SHORT_PERIOD); // convert to CPM

    // Move to the next entry in the sample buffer
    idx++;
    if (idx >= LONG_PERIOD)
    {
        idx = 0;
    }
    count = 0;  // reset counter
}

/* ***** main program loop functionality ********************************************************* */

// flash LED and beep the piezo in case a GM event has happened
static void checkevent(void)
{
    // a GM event has occurred, do something about it!
    if (eventflag)
    {
        // reset flag as soon as possible, in case another ISR is called while we're busy
        eventflag = false;

        // flash LED and beep
        flash_and_beep(160, 3);
    }
}

// log data over the serial port
static void sendreport(void)
{
    // return unless 1 second has passed
    if (!tick)
    {
        return;
    }

    // it's time to report data via UART
    tick = false;                       // reset flag for the next interval

    // format: CPS, #####, CPM, #####, uSv/hr, ###.##, SLOW|FAST|INST

    uint32_t cpm;                      // the CPM value we will report
    static char serbuf[11];            // string buffer
    uint8_t mode;                      // logging mode, 0 = slow, 1 = fast, 2 = inst

    // if cpm is too high, use the short term average instead
    if (overflow)
    {
        cpm = cps * 60UL;
        mode = 2;
        overflow = false;
    }
    else if (fastcpm > THRESHOLD)
    {
        mode = 1;
        cpm = fastcpm;                  // report cpm based on last 5 samples
    }
    else
    {
        mode = 0;
        cpm = slowcpm;                  // report cpm based on last 60 samples
    }

    // send CPS value to the serial port
    uart_putstring_P(PSTR("CPS, "));
    utoa(cps, serbuf, 10);              // radix 10
    uart_putstring(serbuf);

    // send CPM average value to the serial port
    uart_putstring_P(PSTR(", CPM, "));
    ultoa(cpm, serbuf, 10);             // radix 10
    uart_putstring(serbuf);

    // calculate uSv/hr based on scaling factor, and multiply result by 100
    // so we can easily separate the integer and fractional components (2 decimal places)
    uint32_t usv_scaled = (uint32_t)(cpm * SCALE_FACTOR); // scale and truncate the integer part
    // the fractional part
    uint8_t fraction = (usv_scaled/100)%100;

    // this reports the integer part
    uart_putstring_P(PSTR(", uSv/hr, "));
    utoa((uint16_t)(usv_scaled / 10000), serbuf, 10);
    uart_putstring(serbuf);
    uart_putchar('.');
    if (fraction < 10)
    {
        uart_putchar('0');  // zero padding for <0.10
    }
    utoa(fraction, serbuf, 10);
    uart_putstring(serbuf);

    // tell us what averaging method is being used
    if (mode == 2)
    {
        uart_putstring_P(PSTR(", INST"));
    }
    else if (mode == 1)
    {
        uart_putstring_P(PSTR(", FAST"));
    }
    else
    {
        uart_putstring_P(PSTR(", SLOW"));
    }

    // we're done reporting data, output end of line (CRLF)
    uart_putstring_P(PSTR(LINE_END));
}


/* ***** main program **************************************************************************** */

int main(void)
{
    // enable watchdog
    wdt_enable(WDTO_2S);

    // configure the UART
#   include <util/setbaud.h>            // calculate baud rate parameters based on F_CPU
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;
    UCSRB = _BV(RXEN) | _BV(TXEN);      // enable USART transmitter and receiver

    // say hello
    uart_putstring_P(PSTR("# mightyohm.com Geiger Counter " VERSION LINE_END));
    uart_putstring_P(PSTR("# "URL LINE_END));

    // st up AVR IO ports
    DDRB = _BV(PB4) | _BV(PB2);         // set pins connected to LED and piezo as outputs
    DDRD = _BV(PD6);                    // configure PULSE output
    PORTD |= _BV(PD3);                  // enable internal pull up resistor on pin connected to button

    // set up external interrupts
    // INT0 is triggered by a GM impulse
    // INT1 is triggered by pushing the button
    MCUCR |= _BV(ISC01) | _BV(ISC11);   // config interrupts on falling edge of INT0 and INT1
    GIMSK |= _BV(INT0) | _BV(INT1);     // enable external interrupts on pins INT0 and INT1

    // set up 8-bit Timer0 for tone generation
    // toggle OC0A (pin PB2) on compare match and set timer to CTC (clear timer on compare) mode
    TCCR0A = _BV(COM0A0) | _BV(WGM01);
    TCCR0B = 0;                         // disable clock source, stop Timer0 (no sound)

    // set up 16 bit Timer1 for 1 second interrupts
    TCCR1A = 0;                         // normal port operation
    TCCR1B = _BV(WGM12) | _BV(CS12);    // CTC (clear timer on compare) mode, prescaler = 256 (32us ticks)
    OCR1A  = F_CPU / 256;               // 8MHz / 256 = 31250 (32us * 31250 = 1 sec)
    TIMSK |= _BV(OCIE1A);               // Timer1 overflow interrupt enable

    // enable interrupts, start things
    sei();

    // flash and beep when powering on
    flash_and_beep(255, 20);
    _delay_ms(120);
    flash_and_beep(200, 50);

    // keep going forever
    while (true)
    {

        // configure AVR for sleep, this saves a couple mA when idle
        set_sleep_mode(SLEEP_MODE_IDLE);// CPU will go to sleep but peripherals keep running
        sleep_enable();                 // enable sleep
        sleep_cpu();                    // put the core to sleep

        // Zzzzzzz...   CPU is sleeping!
        // Execution will resume here when the CPU wakes up.

        sleep_disable();                // disable sleep so we don't accidentally go to sleep

        checkevent();                   // check if we should signal an event (led + beep)
        sendreport();                   // send a log report over serial
        checkevent();                   // check again before going to sleep

        // assert watchdog
        wdt_reset();
    }

    // never reached
    return 0;
}

/* *********************************************************************************************** */
