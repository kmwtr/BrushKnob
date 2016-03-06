/* Name: main.c
 * Project: BrushKnob
 * Author: Kami Wataru (kmwtr.xyz)
 * Creation Date: 2015-12-28
 * License: GNU GPL v3
 * This project referred to EasyLogger (https://www.obdev.at/products/vusb/easylogger.html) by Christian Starkjohann
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "usbdrv.h"

// bit control macro
#define LOW(addr,bit)   addr &= ~_BV(bit); //レジスタ[addr]のビット[bit]を0(LOW)に。他のビットはそのまま維持。
#define HIGH(addr,bit)  addr |= _BV(bit); //レジスタ[addr]のビット[bit]を1(HIGH)に。他のビットはそのまま維持。

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// [ ATtiny85 ] hardware settings
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// 1: PB5 (PCINT5 / RESET / ADC0 / dW)            8: VCC
// 2: PB3 (PCINT3 / XTAL1 / CLKI / OC18 / ADC3)   7: PB2 (SCK / USCK / SCL / ADC1 / T0 / INT0 / PCINT2)
// 3: PB4 (PCINT4 / XTAL2 / CLKO / OC18 / ADC2)   6: PB1 (MISO / DO / AIN1 / OC0B / OC1A / PCINT1)
// 4: GND                                         5: PB0 (MOSI / DI / SDA / AIN0 / OC0A / OC1A / AREF / PCINT0)
//
// 1: PB5 - None    8: VCC - VCC
// 2: PB3 - RE_A    7: PB2 - D+     // D+はINT0のポートと繋ぐ
// 3: PB4 - RE_B    6: PB1 - D-
// 4: GND - GND     5: PB0 - TSW

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#define KEY_X       0x1B // x
#define KEY_A_US    0x2F // [
#define KEY_B_US    0x30 // ]
//#define KEY_A_JP   0x30 // [
//#define KEY_B_JP   0x31 // ]
volatile static uchar scanCode = 0;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#define TSW     1
#define RE_H    2
#define RE_L    4
volatile static uchar pinFlag = 0; // pin change flag

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

volatile static uchar hotFlag = 0; // report ready flag
volatile static uchar tmpRotateFlag = 0; // rotate direction flag

static uchar    reportBuffer[2];    // buffer for HID reports
static uchar    idleRate;           // in 4 ms units

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// USB report descriptor
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// functions
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static void buildReport(void)
{
    reportBuffer[0] = 0;    /* no modifiers */
    reportBuffer[1] = 0;

    if(hotFlag == 1){
        reportBuffer[1] = scanCode;
        hotFlag = 0;
    }
}

// Pin Change Interrupt
ISR(PCINT0_vect)
{    
    pinFlag = 0; // flag clear

    // TactileSwitch
    if(bit_is_clear(PINB,0)){
        pinFlag |= TSW;
    }

    // RotaryEncoder
    if(bit_is_clear(PINB,3)){ // PB3: HIGH -> LOW
        pinFlag |= RE_L;
    }else if(bit_is_set(PINB, 3)){ // PB3: LOW -> HIGH
        pinFlag |= RE_H;
    }

    TIFR = 0; // count start
}

// timer0 CompareMarch Interrupt
ISR(TIMER0_COMPA_vect)
{
    // TactileSwitch
    if(((pinFlag & TSW) == TSW) && bit_is_clear(PINB,0)){
        hotFlag = 1;
        scanCode = KEY_X;
        //pinFlag &= ~TSW; // flag clear
    }

    // RotaryEncoder
    if(((pinFlag & RE_L) == RE_L) && bit_is_clear(PINB,3)){
        if(bit_is_set(PINB,4)){
            tmpRotateFlag = 'P'; // Positive
        }else{
            tmpRotateFlag = 'N'; // Negative
        }
    }else if(((pinFlag & RE_H) == RE_H) && bit_is_set(PINB,3)){
        if(bit_is_set(PINB,4)){
            if(tmpRotateFlag=='N'){
                hotFlag = 1;
                scanCode = KEY_A_US;
            }
        }else{
            if(tmpRotateFlag=='P'){
                hotFlag = 1;
                scanCode = KEY_B_US;
            }
        }
        tmpRotateFlag = 0;
    }

    pinFlag = 0; // flag clear
}

// Port Initialize
void hardwareInit(void)
{
    // in / out setting (Port B Data Direction Register)
    DDRB = 0b00000000;
    
    // pullup / output setting
    PORTB = _BV(PB0) | _BV(PB3) | _BV(PB4);
    
    // interrupt port type setting
    GIMSK = _BV(PCIE); // enable PCINT port
    
    // authorize interrrupt ports
    PCMSK = _BV(PB0) | _BV(PB3); //PCINT割り込みは出力のポート変化でも発生するため、使うポートを明確にする必要あり。

    // Timer/Counter0 Control Register A & B
    TCCR0A = _BV(WGM01); // timer mode: CTC
    TCCR0B = _BV(CS02); // 16.5MHz/255/129 = 501.596Hz ... nearly 2ms compareMarch
    //TCCR0B = _BV(CS00) | _BV(CS01); // 16.5MHz/64/255 = 1011.029Hz ... nearly 1ms Overflow/CompareMarch

    // timer Interrupt settings
    TIMSK = _BV(OCIE0A); // Timer/Counter0 Output Compare Match A Interrupt Enable (TIMER0_COMPA_vect)
    //TIMSK = _BV(TOIE0); // Timer/Counter0 Overflow Interrupt Enable (TIMER0_OVF_vect)

    // Output Compare Register A
    OCR0A = 129; // 129tick
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// interface to USB driver
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

uchar usbFunctionSetup(uchar data[8])
{
usbRequest_t *rq = (void *)data; // cast to structured data for parsing

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            buildReport();
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// internal Oscillator Calibration
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void usbEventResetReady(void)
{
    /* Disable interrupts during oscillator calibration since
     * usbMeasureFrameLength() counts CPU cycles.
     */
    cli();
    calibrateOscillator();
    sei();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// main
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

int main(void)
{
    uchar i;
    uchar calibrationValue;

    //eeprom_busy_wait(); // waiting for eeprom enable
    calibrationValue = eeprom_read_byte(0); // calibration value from last time
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue; // OSCCAL: Oscillation calibrate register
    }

    // [ disconnect -> delay -> connect ] procedure
    usbDeviceDisconnect(); // enforce re-enumeration
    for(i = 0; i<200; i++){ // wait 400 ms (need more than 250ms)
        wdt_reset(); // keep the watchdog happy
        _delay_ms(2);
    }
    usbDeviceConnect();

    hardwareInit();

    wdt_enable(WDTO_1S); // set 1000 milliseconds watchdog timer

    usbInit(); // initialize the V-USB library

    sei(); // Enable interrupts after usb connect procedure

    // main loop - - - - - - - - - - - - - - - - - - - - - - - - - - -
    while(1){
        wdt_reset(); // keep the watchdog happy
        usbPoll();
		
		if(usbInterruptIsReady()){
        	buildReport();
           	usbSetInterrupt(reportBuffer, sizeof(reportBuffer)); // send the scancode
        }
	}

   	return 0;
}
