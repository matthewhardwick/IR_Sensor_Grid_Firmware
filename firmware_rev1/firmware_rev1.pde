

void setup() {
  CLKPR = (1 << CLKPCE);        // enable clock prescaler update
  CLKPR = 0;                    // set clock to maximum (= crystal)

  __watchdog_reset();           // reset watchdog timer
  MCUSR &= ~(1 << WDRF);        // clear the watchdog reset flag
  WDTCSR |= (1<<WDCE)|(1<<WDE); // start timed sequence
  WDTCSR = 0x00;                // disable watchdog timer

  // init PWM values
  int pwm = 0;
  int i;
  for (i=0; i< 8; i++) {
    compare[i] = pwm;
    compbuff[i] = pwm;
  }  

  DDRB  = ComOutputMask; // All B Inputs except for pin PB0, our output.
  PORTB = ComOutputMask | ComInputMask;		// Pull-ups on input lines, output line high to start.

  DDRD  = 255; // All D outputs to LEDs
  PORTD = 0;

  DDRC  = 255; // All C Outputs to IR LEDs
  PORTC = 0;

  DDRA  = 0; // All A Inputs
  PORTA = 0;  

  // Enable ADC, prescale at 128.
  ADCSRA = _BV(ADEN) |_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

  TCCR0A = 2;
  OCR0A = 128;

  TIFR0 = (1 << TOV0);           // clear interrupt flag
  TIMSK0 = (1 << OCIE0A);         // enable overflow interrupt
  TCCR0B = (1 << CS00);         // start timer, no prescale

  //TCCR0A |= _BV(WGM01);

  

  TIFR1 = (1 << TOV1);
  TIMSK1 = (1 << TOIE1);
  //TCCR1B |= (_BV(CS10) | _BV(CS11));
  TCCR1A = 0;
  TCCR1B = _BV(CS11);

  __enable_interrupt();         // enable interrupts

  sensitivity = 0;
  program = 0;
  s_init();
}

void loop() {

  unsigned long clock = 0;
  int16_t bright = 0;

  int ir = 0;

  while(1) {

    while (ir < 3) {

      // Reading IR Levels from Phototransistors
      bright = readIR(ir);
      if (bright >= 0) {
        int act = handle_ir(ir, bright);
        ir++;
      }
      
      // Taking care of the LEDs
      if (clock - lastDim > dimInterval) {
        // step fade routines
        lastDim = clock;
        int j;
        for (j = 0; j< 8; j++) {
          compbuff[j] = dispatch(program, j, last_act[j], compbuff[j]);
        }
      }

      clock++;

    }
  }
}

// For Reading the IR bounce back with the Phototransistors

int16_t readIR(uint8_t channel) {
  static int started = 0;
  static int i = 0;
  static uint16_t sum = 0;

  if (! started) {
    PORTC = 1 << channel; // turn on IR LED

    sum = 0;
    i = 0;
    started++;
  }

  while (i < num_acd_readings) {
    int16_t v = _readIR(channel);

    if (v < 0) {
      return -1;
    }

    sum += v;
    i++;
  }

  PORTC = 0; // turn off IR LED
  started = 0; // reset

  int16_t ret = (sum >> 1);
  return ret;
}

int16_t _readIR(uint8_t channel) {
  static int started = 0;

  if (! started) {
    ADMUX = channel;
    ADCSRA |= _BV(ADSC); // Start initial ADC cycle

    started++;
  }

  if ((ADCSRA & _BV(ADSC)) != 0) {
    // conversion not finished
    return -1;
  }

  int16_t ADIn;
  ADIn = ADCW;

  // reset state
  started = 0;

  return ADIn;
}

int handle_ir(int ir, int16_t bright) {
  int act = activate(ir, bright);
  last_act[ir] = act;
  compbuff[ir] = dispatch(program, ir, act, compbuff[ir]);

  return act;
}

int dispatch(int program, int ir, int activated, int current_val) {
  last_act[ir] = activated;

  if (program < MAX_PROGRAMS) {
    return ledHandlers[program](ir, activated, current_val);
  }

  return 0;
}

#define nsamples 14
int16_t samples[8][nsamples];

enum actstate {neg=1, pos};
int activated[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned int actcount[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int activate(int ir, int16_t reading) {
  int16_t s = filteredReading(ir, reading);

  // tracking steady state
  if (actcount[ir] > 50) {
    activated[ir] = 0;
  }

  int a = _activate(s, activated[ir]);

  if (a) {
    activated[ir] = (s < 0 ? neg : pos);
    actcount[ir] = 0;
  } else {
    actcount[ir]++;
  }
  
  return a;
}

uint8_t cutoffs[] = {100, 50, 35, 20};

int _activate(int16_t slope, int mode) {
  if (mode == pos && slope < 0) {
    return 0;
  }

  if (mode == neg && slope > 0) {
    return 0;
  }

  if (slope < 0) {
    slope *= -1;
  }

  if (slope < cutoffs[sensitivity]) {
    return 0;
  }

  if (slope > 255) {
    return 255;
  }

  return slope;
}

// #define nasamples 4
int16_t asamples[8][8];
unsigned int acounter = 0;
int16_t filteredReading(int ir, int16_t reading) {
  uint8_t nasamples = (sensitivity <= 1 ? 8 : 4);
  int index = acounter++ % nasamples;

  asamples[ir][index] = slope(ir, reading);
  int i;
  int32_t sum = 0;
  for (i = 0; i< nasamples; i++) {
    sum += asamples[ir][i];
  }

  int16_t ret = (sum >> (nasamples == 8 ? 3 : 2));

  return ret;
}

int16_t slope(int ir, int16_t reading) {
  int i;
  // shift 
  for (i=0; i< nsamples - 1; i++) {
    samples[ir][i] = samples[ir][i+1];
  }
  samples[ir][nsamples - 1] = reading;

  int16_t diffs = 0;
  for (i=nsamples-1; i > 0; i--) {
    diffs += (samples[ir][i] - samples[ir][i-1]);
  }

  return diffs;
}

int quick_fade(int ir, int activated, int current_val) {
  if (activated) {
    // return 255;
    return activated;
  }

  if (current_val > 2) {
    return current_val - 3;
  }

  return 0;
}
