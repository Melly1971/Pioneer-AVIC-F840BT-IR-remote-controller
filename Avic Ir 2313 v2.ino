/*
   IR NEC protocol reader (38kHz, TSOPxxx) and 8 pin out for Pioneer AVIC-F840BT car stereo and Pioneer remote controller QXE1047 CXC8885 CXE3669 QXA3196.
   ATtiny2313, MCU Settings: FUSE_L=0xE4, FUSE_H=0xDF, FUSE_E=0xFF, F_CPU=8MHz internal 14CK+65 ms
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define ShiftPort PD6    // connected to ring of 3.5mm jack and GND
#define Resistor1k2 PB0  // 1k2 resistor connected to tip of 3.5mm jack and GND
#define Resistor3k3 PB1  // 3k3 resistor connected to tip of 3.5mm jack and GND
#define Resistor5k6 PB2  // 5k6 resistor connected to tip of 3.5mm jack and GND
#define Resistor8k2 PB3  // 8k2 resistor connected to tip of 3.5mm jack and GND
#define Resistor12k PB4  // 12k resistor connected to tip of 3.5mm jack and GND
#define Resistor15k PB5  // 15k resistor connected to tip of 3.5mm jack and GND
#define Resistor24k PB6  // 24k resistor connected to tip of 3.5mm jack and GND
#define Resistor68k PB7  // 68k resistor connected to tip of 3.5mm jack and GND

#define IrInPin PD2  // TSOP1338 out to pin PD2

#define IrSuccess (0)
#define IrError (1)

#define Low (0)
#define High (1)

#define IrEventIdle (0)
#define IrEventInit (1)
#define IrEventEnd (2)
#define IrEventProcess (3)

#define IrProtocolEventInit (0)
#define IrProtocolEventData (1)
#define IrProtocolEventEnd (2)
#define IrProtocolEventHook (3)

// NEC IR command and address of Pioneer remote controller
#define CommandRepeat (0x19)
#define AddressRepeat (0xAD)
#define CommandVolumeMinus (0x0B)
#define AddressVolumeMinus (0xAD)
#define CommandVolumePlus (0x0A)
#define AddressVolumePlus (0xAD)
#define CommandBandEsc (0x12)
#define AddressBandEsc (0xAD)
#define CommandMute (0x30)
#define AddressMute (0xAF)
#define CommandFunction (0x67)
#define AddressFunction (0xAF)
#define CommandAudio (0x0D)
#define AddressAudio (0xAD)
#define CommandSrc (0x1A)
#define AddressSrc (0xAD)
#define CommandPause (0x58)
#define AddressPause (0xAD)
#define CommandDisplay (0x6D)
#define AddressDisplay (0xAF)
#define CommandLeft (0x42)
#define AddressLeft (0xAD)
#define CommandRight (0x43)
#define AddressRight (0xAD)
#define CommandUp (0x40)
#define AddressUp (0xAD)
#define CommandDown (0x41)
#define AddressDown (0xAD)
#define CommandEnter (0x20)
#define AddressEnter (0xAF)

// timer to turn out off
volatile uint16_t OffCounter = { 0 };
volatile uint16_t OffInterval = { 10000 };  //delay in 2*38.222kHz pulse, 10000 =131 ms
static boolean OutPortFlag = false;

// timer for NEC protocol
volatile uint16_t IrTimeout{ 0 };
volatile uint16_t IrCounter{ 0 };
volatile uint32_t IrRawdata{ 0 };

// variables for NEC protocol
uint8_t IrEvent{ 0 };
uint8_t IrProtocolEvent{ 0 };
uint8_t IrIndex{ 0 };
uint32_t IrData{ 0 };
uint8_t IrAddress{ 0 };
uint8_t IrCommand{ 0 };

// from raw 32 bit to address 8 bit and command 8 bit
int IrRead(uint8_t *Address, uint8_t *Command) {
  if (!IrRawdata)
    return IrError;
  *Address = IrRawdata;
  *Command = IrRawdata >> 16;
  IrRawdata = 0;
  return IrSuccess;
}


int IrNecProcess(uint16_t counter, uint8_t value) {
  int8_t retval = IrError;

  switch (IrProtocolEvent) {
    case IrProtocolEventInit:
      /* expecting a space */
      if (value == High) {
        if (counter > 330 && counter < 360) {
          /* a 4.5ms space for regular transmition of NEC Code; counter => 0.0045/(1.0/38222.0) * 2 = 344 (+/- 15) */
          IrProtocolEvent = IrProtocolEventData;
          IrData = IrIndex = 0;
          retval = IrSuccess;
        } else if (counter > 155 && counter < 185) {
          /* a 2.25ms space for NEC Code repeat; counter => 0.00225/(1.0/38222.0) * 2 = 172 (+/- 15) */
          IrProtocolEvent = IrProtocolEventEnd;
          retval = IrSuccess;
        }
      }
      break;
    case IrProtocolEventData:
      /* Reading 4 octets (32bits) of data:
           1) the 8-bit address for the receiving device
           2) the 8-bit logical inverse of the address
           3) the 8-bit command
           4) the 8-bit logical inverse of the command
          Logical '0' – a 562.5µs pulse burst followed by a 562.5µs
           (<90 IR counter cycles) space, with a total transmit time of 1.125ms
          Logical '1' – a 562.5µs pulse burst followed by a 1.6875ms
           (>=90 IR counter cycles) space, with a total transmit time of 2.25ms */
      if (IrIndex < 32) {
        if (value == High) {
          IrData |= ((uint32_t)((counter < 90) ? 0 : 1) << IrIndex++);
          if (IrIndex == 32) {
            IrProtocolEvent = IrProtocolEventHook;
          }
        }
        retval = IrSuccess;
      }
      break;
    case IrProtocolEventHook:
      // expecting a final 562.5µs pulse burst to signify the end of message transmission
      if (value == Low) {
        IrProtocolEvent = IrProtocolEventEnd;
        retval = IrSuccess;
      }
      break;
    case IrProtocolEventEnd:
      // copying data to volatile variable; raw data is ready
      IrRawdata = IrData;
      break;
    default:
      break;
  }
  return retval;
}

void IrProcess() {
  uint8_t value;
  uint16_t counter;
  // load IrCounter value to local variable, then reset counter
  counter = IrCounter;
  IrCounter = 0;
  // read IrInPin digital value (NOTE: logical inverse value = value ^ 1 due to sensor used)
  value = (PIND & (1 << IrInPin)) > 0 ? Low : High;  // Low : High;

  switch (IrEvent) {
    case IrEventIdle:  // awaiting for an initial signal
      if (value == High) {
        IrEvent = IrEventInit;
      }
      break;
    case IrEventInit:  // consume leading pulse burst
      if (value == Low) {
        if (counter > 655 && counter < 815) {
          // a 9ms leading pulse burst, NEC Infrared Transmission Protocol detected, counter = 0.009/(1.0/38222.) * 2 = 343.998 * 2 = 686 (+/- 30)
          IrEvent = IrEventProcess;
          IrProtocolEvent = IrProtocolEventInit;
          IrTimeout = 5000;  // 5000 clock at 38kHz > 62.5ms total time NEC protocol - was 7400
        } else {
          IrEvent = IrEventEnd;
        }
      } else {
        IrEvent = IrEventEnd;
      }
      break;
    case IrEventProcess:  // read and decode NEC Protocol data
      if (IrNecProcess(counter, value))
        IrEvent = IrEventEnd;
      break;

    case IrEventEnd:  // clear timeout and set idle mode
      IrEvent = IrEventIdle;
      IrTimeout = 0;
      break;
    default:
      break;
  }
}

void OutPort(uint8_t OutPortB, uint8_t OutPortD)  // OutPortB (resistors pins) to portB, if true OutPortD out ShiftPort, set out flag and time
{
  PORTB |= _BV(OutPortB);
  if (OutPortD == true) {
    PORTD |= _BV(ShiftPort);
  }
  OutPortFlag = true;
  OffCounter = 0;
}

// on signal on PD2 interrupt
ISR(INT0_vect) {
  IrProcess();
}

// on timer interrupt (every 1/38222 s)
ISR(TIMER0_COMPA_vect) {
  if (IrCounter++ > 10000)
    IrEvent = IrEventIdle;

  if (IrTimeout && --IrTimeout == 0)
    IrEvent = IrEventIdle;
   
  OffCounter++;
}

void setup() {
   
  // setup output for portB 0-7 and portD 6  
  DDRB |= _BV(Resistor1k2) | _BV(Resistor3k3) | _BV(Resistor5k6) | _BV(Resistor8k2) | _BV(Resistor12k) | _BV(Resistor15k) | _BV(Resistor24k) | _BV(Resistor68k);
  DDRD |= _BV(ShiftPort);

  DDRD &= ~_BV(IrInPin);   // set IrInPin as INPUT
  PORTD &= ~_BV(IrInPin);  // set LOW level to IrInPin

  // setup timer interrupt 
  TCNT0 = 0;            // Count up from 0
  TCCR0A = 2 << WGM00;  // CTC mode
  TCCR0B = (1 << CS00);  // Set prescaler to /1
  GTCCR |= 1 << PSR10;  // Reset prescaler
  OCR0A = 104;          // set OCR0n to get ~38.222kHz timer frequency, 104 for 8MHz
  TIFR = 1 << OCF0A;    // Clear output compare interrupt flag
  TIMSK |= 1 << OCIE0A;  // Enable output compare interrupt
   
  // setup interrupt for IR sensor
  GIMSK |= _BV(INT0);    // enable INT0 interrupt handler
  MCUCR &= ~_BV(ISC01);  // trigger INTO interrupt on raising
  MCUCR |= _BV(ISC00);   // and falling edge
  sei();                 // enable global interrupts
}

void loop() {

  if ((OutPortFlag == true) && (OffCounter >= OffInterval))  // if any output is high and time (OffInterval) has passed, switch all outputs to low
  {
    PORTB &= ~(_BV(Resistor1k2) | _BV(Resistor3k3) | _BV(Resistor5k6) | _BV(Resistor8k2) | _BV(Resistor12k) | _BV(Resistor15k) | _BV(Resistor24k) | _BV(Resistor68k));
    PORTD &= ~(_BV(ShiftPort));
    OutPortFlag = false;
  }

  // if IR read was a success then outup
  if (IrRead(&IrAddress, &IrCommand) == IrSuccess) {

    if (IrAddress == AddressVolumeMinus && IrCommand == CommandVolumeMinus)  // volume -
    {
      OutPort(Resistor24k, false);
    }

    if (IrAddress == AddressVolumePlus && IrCommand == CommandVolumePlus)  // volume +
    {
      OutPort(Resistor15k, false);
    }

    if (IrAddress == AddressBandEsc && IrCommand == CommandBandEsc)  // band /esc
    {
      OutPort(Resistor68k, false);
    }

    if (IrAddress == AddressMute && IrCommand == CommandMute)  // att sound
    {
      OutPort(Resistor3k3, false);
    }

    if (IrAddress == AddressFunction && IrCommand == CommandFunction)  // phone menu
    {
      OutPort(Resistor1k2, true);
    }

    if (IrAddress == AddressAudio && IrCommand == CommandAudio)  // phone off
    {
      OutPort(Resistor5k6, true);
    }

    if (IrAddress == AddressSrc && IrCommand == CommandSrc)  // src / 2sec OFF
    {
      OutPort(Resistor1k2, false);
    }

    if (IrAddress == AddressPause && IrCommand == CommandPause)  // mute
    {
      OutPort(Resistor68k, true);
    }

    if (IrAddress == AddressDisplay && IrCommand == CommandDisplay)  // display
    {
      OutPort(Resistor5k6, false);
    }

    if (IrAddress == AddressLeft && IrCommand == CommandLeft)  // left
    {
      OutPort(Resistor12k, false);
    }

    if (IrAddress == AddressRight && IrCommand == CommandRight)  // right
    {
      OutPort(Resistor8k2, false);
    }

    if (IrAddress == AddressUp && IrCommand == CommandUp)  // up
    {
      OutPort(Resistor12k, true);
    }

    if (IrAddress == AddressDown && IrCommand == CommandDown)  // down
    {
      OutPort(Resistor8k2, true);
    }

    if (IrAddress == AddressEnter && IrCommand == CommandEnter)  // shift att sound
    {
      OutPort(Resistor24k, true);
    }

    if (IrAddress == AddressRepeat && IrCommand == CommandRepeat)  // ****
    {
    }
  }
}
