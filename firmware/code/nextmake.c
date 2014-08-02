/* USB PCB Business Card
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */
 
 // please see http://www.frank-zhao.com/card/
// note, for ATtiny MCUs, fuses -U lfuse:w:0xE1:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m -U lock:w:0xFF:m
// note, write to ATtiny at a low ISP frequency

// required avr-libc modules, see http://www.nongnu.org/avr-libc/user-manual/modules.html
#include <avr/io.h> // allows access to AVR hardware registers
#include <avr/interrupt.h> // allows enabling/disabling and declaring interrupts
#include <util/delay.h> // includes delay functions
#include <avr/wdt.h> // allows enabling/disabling watchdog timer
#include <avr/pgmspace.h> // descriptor must be stored in flash memory
#include <avr/eeprom.h> // text file and calibration data is stored in EEPROM
#include <stdio.h> // allows streaming strings

// configure settings for V-USB then include the V-USB driver so V-USB uses your settings
#include "usbconfig.h"
#include "usbdrv/usbdrv.h"

// compilation settings check
#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
#if F_CPU != 16500000
#error "ATtiny MCU used but not at 16.5 KHz"
#endif
#else
#if F_CPU != 12000000
#error "Clock speed is not 12 KHz"
#endif
#endif


static inline void message() {
    puts_P(PSTR("Thanks for checking out Next Make!\n\nNext Make is all about:\n* building cool stuff\n* learning from others\n* sharing technical knowledge\n* furthering 'mens et manus' at MIT\n\nFor more info, go to:\nhttp://next-make.mit.edu\n\nEmail us at:\nnext-make-exec@mit.edu"));
}


// USB HID report descriptor for boot protocol keyboard
// see HID1_11.pdf appendix B section 1
// USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH is defined in usbconfig
PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)(224)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)(231)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs) ; Modifier byte
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs) ; Reserved byte
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs) ; LED report
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs) ; LED report padding
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))(0)
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)(101)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

// data structure for boot protocol keyboard report
// see HID1_11.pdf appendix B section 1
typedef struct {
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode[6];
} keyboard_report_t;

// global variables

static keyboard_report_t keyboard_report;
#define keyboard_report_reset() keyboard_report.modifier=0;keyboard_report.reserved=0;keyboard_report.keycode[0]=0;keyboard_report.keycode[1]=0;keyboard_report.keycode[2]=0;keyboard_report.keycode[3]=0;keyboard_report.keycode[4]=0;keyboard_report.keycode[5]=0;
static uint8_t idle_rate = 500 / 4; // see HID1_11.pdf sect 7.2.4
static uint8_t protocol_version = 0; // see HID1_11.pdf sect 7.2.6
static uint8_t LED_state = 0; // see HID1_11.pdf appendix B section 1
static uint8_t blink_count = 0; // keep track of how many times caps lock have toggled

// see http://vusb.wikidot.com/driver-api
// constants are found in usbdrv.h
usbMsgLen_t usbFunctionSetup(uint8_t data[8])
{
	// see HID1_11.pdf sect 7.2 and http://vusb.wikidot.com/driver-api
	usbRequest_t *rq = (void *)data;

	if ((rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_CLASS)
		return 0; // ignore request if it's not a class specific request

	// see HID1_11.pdf sect 7.2
	switch (rq->bRequest)
	{
		case USBRQ_HID_GET_IDLE:
			usbMsgPtr = &idle_rate; // send data starting from this byte
			return 1; // send 1 byte
		case USBRQ_HID_SET_IDLE:
			idle_rate = rq->wValue.bytes[1]; // read in idle rate
			return 0; // send nothing
		case USBRQ_HID_GET_PROTOCOL:
			usbMsgPtr = &protocol_version; // send data starting from this byte
			return 1; // send 1 byte
		case USBRQ_HID_SET_PROTOCOL:
			protocol_version = rq->wValue.bytes[1];
			return 0; // send nothing
		case USBRQ_HID_GET_REPORT:
			usbMsgPtr = &keyboard_report; // send the report data
			return sizeof(keyboard_report);
		case USBRQ_HID_SET_REPORT:
			if (rq->wLength.word == 1) // check data is available
			{
				// 1 byte, we don't check report type (it can only be output or feature)
				// we never implemented "feature" reports so it can't be feature
				// so assume "output" reports
				// this means set LED status
				// since it's the only one in the descriptor
				return USB_NO_MSG; // send nothing but call usbFunctionWrite
			}
			else // no data or do not understand data, ignore
			{
				return 0; // send nothing
			}
		default: // do not understand data, ignore
			return 0; // send nothing
	}
}

// see http://vusb.wikidot.com/driver-api
usbMsgLen_t usbFunctionWrite(uint8_t * data, uchar len)
{
	if (data[0] != LED_state)
	{
		// increment count when LED has toggled
		blink_count = blink_count < 10 ? blink_count + 1 : blink_count;
	}
	
	LED_state = data[0];
	
	#if !defined(__AVR_ATtiny85__) && !defined(__AVR_ATtiny45__) && !defined(__AVR_ATtiny25__)
	 // LED lights for debug
	if (bit_is_set(LED_state, 1))
	{
		DDRB |= _BV(3);
		PORTB |= _BV(3);
	}
	else
	{
		DDRB |= _BV(3);
		PORTB &= ~_BV(3);
	}
	#endif
	
	return 1; // 1 byte read
}

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */
// section copied from EasyLogger
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
    calibrateOscillator();
    eeprom_update_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}
#endif

// translates ASCII to appropriate keyboard report, taking into consideration the status of caps lock
void ASCII_to_keycode(uint8_t ascii)
{
	keyboard_report.keycode[0] = 0x00;
	keyboard_report.modifier = 0x00;
	
	// see scancode.doc appendix C
	
	if (ascii >= 'A' && ascii <= 'Z')
	{
		keyboard_report.keycode[0] = 4 + ascii - 'A'; // set letter
		if (bit_is_set(LED_state, 1)) // if caps is on
		{
			keyboard_report.modifier = 0x00; // no shift
		}
		else
		{
			keyboard_report.modifier = _BV(1); // hold shift // hold shift
		}
	}
	else if (ascii >= 'a' && ascii <= 'z')
	{
		keyboard_report.keycode[0] = 4 + ascii - 'a'; // set letter
		if (bit_is_set(LED_state, 1)) // if caps is on
		{
			keyboard_report.modifier = _BV(1); // hold shift // hold shift
		}
		else
		{
			keyboard_report.modifier = 0x00; // no shift
		}
	}
	else if (ascii >= '0' && ascii <= '9')
	{
		keyboard_report.modifier = 0x00;
		if (ascii == '0')
		{
			keyboard_report.keycode[0] = 0x27;
		}
		else
		{
			keyboard_report.keycode[0] = 30 + ascii - '1'; 
		}
	}
	else
	{
		switch (ascii) // convert ascii to keycode according to documentation
		{
			case '!':
				keyboard_report.modifier = _BV(1); // hold shift
				keyboard_report.keycode[0] = 29 + 1;
				break;
			case '@':
				keyboard_report.modifier = _BV(1); // hold shift
				keyboard_report.keycode[0] = 29 + 2;
				break;
			case '#':
				keyboard_report.modifier = _BV(1); // hold shift
				keyboard_report.keycode[0] = 29 + 3;
				break;
			case '$':
				keyboard_report.modifier = _BV(1); // hold shift
				keyboard_report.keycode[0] = 29 + 4;
				break;
			case '%':
				keyboard_report.modifier = _BV(1); // hold shift
				keyboard_report.keycode[0] = 29 + 5;
				break;
			case '^':
				keyboard_report.modifier = _BV(1); // hold shift
				keyboard_report.keycode[0] = 29 + 6;
				break;
			case '&':
				keyboard_report.modifier = _BV(1); // hold shift
				keyboard_report.keycode[0] = 29 + 7;
				break;
			case '*':
				keyboard_report.modifier = _BV(1); // hold shift
				keyboard_report.keycode[0] = 29 + 8;
				break;
			case '(':
				keyboard_report.modifier = _BV(1); // hold shift
				keyboard_report.keycode[0] = 29 + 9;
				break;
			case ')':
				keyboard_report.modifier = _BV(1); // hold shift
				keyboard_report.keycode[0] = 0x27;
				break;
			case '~':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case '`':
				keyboard_report.keycode[0] = 0x35;
				break;
			case '_':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case '-':
				keyboard_report.keycode[0] = 0x2D;
				break;
			case '+':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case '=':
				keyboard_report.keycode[0] = 0x2E;
				break;
			case '{':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case '[':
				keyboard_report.keycode[0] = 0x2F;
				break;
			case '}':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case ']':
				keyboard_report.keycode[0] = 0x30;
				break;
			case '|':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case '\\':
				keyboard_report.keycode[0] = 0x31;
				break;
			case ':':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case ';':
				keyboard_report.keycode[0] = 0x33;
				break;
			case '"':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case '\'':
				keyboard_report.keycode[0] = 0x34;
				break;
			case '<':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case ',':
				keyboard_report.keycode[0] = 0x36;
				break;
			case '>':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case '.':
				keyboard_report.keycode[0] = 0x37;
				break;
			case '?':
				keyboard_report.modifier = _BV(1); // hold shift
				// fall through
			case '/':
				keyboard_report.keycode[0] = 0x38;
				break;
			case ' ':
				keyboard_report.keycode[0] = 0x2C;
				break;
			case '\t':
				keyboard_report.keycode[0] = 0x2B;
				break;
			case '\n':
				keyboard_report.keycode[0] = 0x28;
				break;
		}
	}
}

void send_report_once()
{
	// perform usb background tasks until the report can be sent, then send it
	while (1)
	{
		usbPoll(); // this needs to be called at least once every 10 ms
		if (usbInterruptIsReady())
		{
			usbSetInterrupt(&keyboard_report, sizeof(keyboard_report)); // send
			break;
			
			// see http://vusb.wikidot.com/driver-api
		}
	}
}

// stdio's stream will use this funct to type out characters in a string
void type_out_char(uint8_t ascii, FILE *stream)
{
	ASCII_to_keycode(ascii);
	send_report_once();
	keyboard_report_reset(); // release keys
	send_report_once();
}

static FILE mystdout = FDEV_SETUP_STREAM(type_out_char, NULL, _FDEV_SETUP_WRITE); // setup writing stream

int main()
{
	#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
	uint8_t calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if (calibrationValue != 0xFF)
	{
        OSCCAL = calibrationValue;
    }
	#endif


    DDRB |= _BV(3);
    
    PORTB &= ~_BV(3);
    _delay_ms(100);
    PORTB |= _BV(3);
    _delay_ms(100);
    PORTB &= ~_BV(3);
    _delay_ms(100);
    PORTB |= _BV(3);
	
	stdout = &mystdout; // set default stream
	
	// initialize report (I never assume it's initialized to 0 automatically)
	keyboard_report_reset();
	
	wdt_disable(); // disable watchdog, good habit if you don't use it
	
	// enforce USB re-enumeration by pretending to disconnect and reconnect
	usbDeviceDisconnect();
	_delay_ms(250);
	usbDeviceConnect();
	
	// initialize various modules
	usbInit();
	
	sei(); // enable interrupts
	

    //XXX: clean this up
    uint16_t count = 0;
    #define MAX_COUNT 2000

	while (1) // main loop, do forever
	{
        if (count < MAX_COUNT) {
            count++;
        }
        if (count == MAX_COUNT) {
            for (int i = 0; i < 8; i++) {
                PORTB &= ~_BV(3);
                _delay_ms(50);
                PORTB |= _BV(3);
                _delay_ms(50);
            }
            PORTB &= ~_BV(3);
            message();
            puts_P(PSTR("\n----------\nBuilt with Next Make\nNext House CPW 2012"));
            PORTB |= _BV(3);

            // prevent this from ever happening again
            count = MAX_COUNT + 1;
        }
/*
		if (blink_count > 2) // activated by blinking lights
		{
			#if !defined(__AVR_ATtiny85__) && !defined(__AVR_ATtiny45__) && !defined(__AVR_ATtiny25__)
			DDRB |= _BV(3); // LED lights for debug
			PORTB |= _BV(3);
			#endif
			
			// PLACE TEXT HERE
			//puts_P(PSTR(" ")); // test size
			puts_P(PSTR("This is only a test!\n* testing 1\n* testing 2\n* testing 3\n\nYou should go here: http://scottbezek.com :) \nThe End."));
			
			blink_count = 0; // reset
		}
*/		
		// perform usb related background tasks
		usbPoll(); // this needs to be called at least once every 10 ms
		// this is also called in send_report_once

        _delay_ms(1);
	}
	
	return 0;
}
