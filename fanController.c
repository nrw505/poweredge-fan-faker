/* -*- mode: c++; c-basic-offset: 2; -*- */

#include <alloca.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>

// UNCONNECTED  is pin 1 / PB5 / -RESET
// PIN_SENSE_F  is pin 2 / PB3 / PCINT3
// PIN_PWM_F    is pin 3 / PB4 / OC1B
// PIN_PWM_C    is pin 5 / PB0 / PCINT0
// PIN_SENSE_C  is pin 6 / PB1
// PIN_PWM_M    is pin 7 / PB2 / ADC1 / PCINT2

// PIN_SENSE_F is the open-collector input from the fan: 2 low pulses
//             per revolution

// PIN_PWM_F   is the PWM control to the fan: high duty cycle = fast,
//             low duty cycle = slow

// PIN_PWM_C   is the PWM control from the computer: high duty cycle =
//             wants fast, low duty cycle = wants slow
// PIN_PWM_M   is the PWM control from the computer, run through a low
//             pass filter to get an analog input.
// Note that both these inputs are driven from the same control line,
// but PIN_PWM_C is unfiltered and PIN_PWM_M is filtered

// PIN_SENSE_C is the open-collector sense line to the computer: we use
//             this to lie to the computer about how fast the fan is
//             going (2 low pulses per revolution, 50% duty cycle)


// PWM_COUNTER is used to set the frequency of the PWM signal we send
// to the fan. The spec from Intel says this should be ~25 kHz -
// setting OCR1C to 165, and using the fast peripheral clock /16 to
// drive timer1 gives us 24.5 kHz

#define PWM_COUNTER 165

#define PWM_INPUT_ACCUM_SAMPLES 256
// Ticks here are 1/4000 of a sec, so 16 ticks between samples means 250
// samples a second, or averaging over just over a second
#define PWM_INPUT_TICKS_PER_SAMPLE 16

#define TICKS_PER_FAN_SPEED_CHECK 2000
// Update the fan speed twice a second

volatile uint8_t outSense;
volatile uint8_t lastPwm;
volatile uint8_t lastPwmHighTime;
volatile uint8_t lastPwmLowTime;

volatile uint16_t ticks; // 1 tick per overflow of timer0 == 256 uS
volatile uint32_t secs;  // ideally this is actual seconds.

volatile uint16_t pwmInputAccumulator;

volatile uint8_t fanStartup;
volatile uint8_t checkFanControl;

void setFanPercentage(int pct) {
  OCR1B = (PWM_COUNTER * pct) / 100;
}

void setFanFullSpeed() {
  // Set PB4 Input/HighZ

  // Fan should sense this and switch to full speed / uncontrolled
  // This is used to allow some fans which cannot start when PWM control
  // is asking for too low a speed.

  // Dell BMC also uses it to indicate system fault that requires full
  // speed on all fans
  
  PORTB &= ~(_BV(PB4));
  DDRB &= ~(_BV(DDB4));

  // Disconnect from timer1 PWM generation
  GTCCR &= ~(_BV(COM1B0) | _BV(COM1B1));
}

void setFanPWM() {
  // Set PB4 output
  DDRB |= _BV(DDB4);

  // Connect to timer1 PWM generation
  GTCCR |= _BV(COM1B1);
}

void setupTimer0() {
  ticks = 0;
  secs = 0;

  // CTC Mode: count from 0 to OCR0A and reset to 0
  // clock source is sys clock/8 (or 1MHz)
  // Set OCR0A to 250, so tick is 1MHZ/250 (or 4kHz)
  // Enable interrupt on OCR0A match, so timer interrupt at 4 kHz
  
  // Tick count * 250 + read of TCNT0 gives time in microseconds since ticks
  // wrapped
  
  TCCR0A = _BV(WGM01); 
  TCCR0B = _BV(CS01);
  TCNT0 = 0;
  OCR0A = 250;

  TIMSK |= _BV(OCIE0A);

}

void setupTimer1() {
  // Set PCKE and PLLE

  PLLCSR = _BV(PCKE) | _BV(PLLE);
  
  // Set clock sel = PCK/16, OCR1C to 165 (25kHz ish)
  // PWM1A = off, PWM1B = on
  TCCR1 = _BV(CS12) | _BV(CS10);
  GTCCR = (GTCCR & (_BV(TSM) | _BV(PSR0))) | _BV(PWM1B) | _BV(COM1B1);

  OCR1C = PWM_COUNTER;

  // Set PWM B output to 33%
  setFanPercentage(33);
}

void setupPinInterrupts() {
  PCMSK = _BV(PCINT3) | _BV(PCINT2);
  GIMSK |= _BV(PCIE);
}

void setupADC() {
  // Set left-adjusted (this way we can just read
  // ADCH and get an 8-bit reading)
  // Set ADC source to ADC1 / PB2
  // Set VREF to Vcc
  ADMUX = _BV(ADLAR) | _BV(MUX0);
  
  // Enable ADC
  // Set prescaler to make ADC clock to 125kHz (sysclk / 64)
  // Enable ADC Interrupt on conversion complete
  ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1);

  // Disable digital input buffer for PB2
  DIDR0 = _BV(ADC1D);

}

void setup() {
  // Set PB0, PB4 to output
  // PB1 stays input (HI-Z) because it's supposed to be wired-OR
  DDRB = _BV(DDB0) | _BV(DDB4);

  // disable interrupts
  cli();
  
  setupTimer0();
  setupTimer1();
  setupPinInterrupts();
  setupADC();

  checkFanControl = 0;
  fanStartup = 1;
  setFanFullSpeed();
  
  // now everything's set up, enable interrupts
  sei();
}

void loop() {
  if (fanStartup) {
    if (secs > 0) {
      setFanPWM();
      fanStartup = 0;
      checkFanControl = 1;
    }
  } else {
    if (checkFanControl) {
      uint8_t avgPwm = pwmInputAccumulator / PWM_INPUT_ACCUM_SAMPLES;

      int tmp = avgPwm;
      tmp = tmp * 100 / 256;
      setFanPercentage(tmp);
      checkFanControl = 0;
    }
  }
}

int main(void) {
  setup();
  while (1) {
    loop();
  }
}

ISR(PCINT0_vect) {
  uint8_t val = PINB;

  uint8_t in_fan_s = val & _BV(PB3);
  uint8_t in_pwm_c = val & _BV(PB2);

  if (in_fan_s != 0x00) {
    if (!outSense) {
      // Set PB1 to High-Z and let the pullup do it's job
      val &= ~(_BV(PB1));
      DDRB &= ~(_BV(DDB1));
      outSense = 1;
    }
  } else {
    if (outSense) {
      // Set PB1 to output LOW
      val &= ~(_BV(PB1));
      DDRB |= _BV(DDB1);
      outSense = 0;
    }
  }

  if (in_pwm_c && !lastPwm) {
    lastPwm = in_pwm_c;
  } else if (!in_pwm_c && lastPwm) {
    lastPwm = in_pwm_c;
  }
  
  PORTB = val;
}

ISR(TIMER0_COMPA_vect) {
  uint16_t newTicks = ticks + 1;
  if (newTicks == 4000) {
    secs++;
    newTicks = 0;
  }
  ticks = newTicks;

  if (newTicks % PWM_INPUT_TICKS_PER_SAMPLE == 0) {
    // Start an ADC sample
    ADCSRA |= _BV(ADSC);
  }

  if (newTicks % TICKS_PER_FAN_SPEED_CHECK == 0) {
    checkFanControl = 1;
  }

  PINB |= _BV(PINB0); // Toggle PB0 temporarily so we can verify the ISR is working;
}

ISR(ADC_vect) {
  uint8_t newReading = ADCH;
  pwmInputAccumulator = pwmInputAccumulator - (pwmInputAccumulator / PWM_INPUT_ACCUM_SAMPLES) + newReading;
}
