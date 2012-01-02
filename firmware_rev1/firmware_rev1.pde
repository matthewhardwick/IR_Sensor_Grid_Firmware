/*
 *
 * These code snippets are from the Evil Mad Scientists Octolively Firmware.
 * So credit goes to them. It is going to be modified to work with an Arduino
 * with no network, and one LED mode. Additionally, it will be reconfigured to
 * a 3x3 grid. 
 *
 * Currently is non functioning!
 *
 */
 
#define sensors 9
#define MAX_PROGRAMS 1

typedef int (*ledHandler)(int, int, int);
 
int8_t irPin[] = {3, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t photoPin[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t ledPin[] = {9, 0, 0, 0, 0, 0, 0, 0, 0};

int last_act[] = {0,0,0,0,0,0,0,0,0};
ledHandler ledHandlers[] = {quick_fade};
int compbuff[9];
int8_t program = 0;

int quick_fade(int ir, int activated, int current_val);

#define DEFAULT_SENSITIVITY 2
#define MAX_SENSITIVITY 4
uint8_t sensitivity = DEFAULT_SENSITIVITY;

int16_t slope(int ir, int16_t reading);
int16_t filteredReading(int ir, int16_t reading);
int _activate(int16_t slope, int mode);
int activate(int ir, int16_t reading);
int dispatch(int program, int ir, int activated, int current_val);
int handle_ir(int ir, int16_t bright);
int16_t readIR(uint8_t channel);
void s_init();

void setup() {

  int pwm = 0;
  int i;
  for (i=0; i < 9; i++) {
    compbuff[i] = pwm;
  }  

  // Setup Pins
  for (int i = 0; i < 1; i++) {
    pinMode(irPin[i],OUTPUT);
    pinMode(8,OUTPUT);
    pinMode(photoPin[i],INPUT);
  }
  
  // Serial for Debug
  Serial.begin(9600); 

  int16_t sensitivity = 0;
  int16_t program = 0;
  s_init();
}

void loop() {

  unsigned long clock = 0;
  int16_t bright = 0;
  int ir = 0;
  unsigned long dimInterval = 75;
  unsigned long lastDim = 0;

  while(1) {
  int ir = 0;
    while (ir < 1) {

      
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

#define num_acd_readings 16

int16_t readIR(uint8_t channel) {
  static int started = 0;
  static int i = 0;
  static uint16_t sum = 0;

  if (! started) {
    digitalWrite(irPin[channel],HIGH);

    sum = 0;
    i = 0;
    started++;
  }

  while (i < num_acd_readings) {
    //int16_t v = _readIR(channel);
    int16_t v = analogRead(photoPin[channel]);

    if (v < 0) {
      return -1;
    }

    sum += v;
    i++;
  }

  digitalWrite(irPin[channel],LOW);
  started = 0; // reset

  int16_t ret = (sum >> 1);
  return ret;
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
//  if (activated) {
//    // return 255;
//    return activated;
//  }

  if (current_val > 2) {
    analogWrite(ledPin[ir],current_val - 3);
    Serial.println(current_val);
    return current_val - 3;
  }

  return 0;
}

void s_init() {
  int i, j;

  for (i=0; i< 8; i++) {
    for (j=0; j< nsamples; j++) {
      samples[i][j] = 0;
    }
    for (j=0; j< 8; j++) {
      asamples[i][j] = 0;
    }
  }
}


