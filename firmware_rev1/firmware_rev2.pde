//Modified by David Tran

//Remember to LENGTHen or SHORTen the arrays as needed, depending on the 
//number of SENSORS used.

#define SENSORS 2
#define DEBUG 0   //Switches the device to DEBUG mode, to place the LEDS correctly
#define SERIAL 1  //Trivial
 
const uint8_t irPin[]    = {2,  4,  7,  8, 12, 13,  0,  0, 0};
const uint8_t photoPin[] = {0,  1,  2,  3,  4,  5,  6,  7, 0}; //Analog pins, otherwise pointless.
const uint8_t ledPin[]   = {9, 10, 11, 12,  0,  0,  0,  0, 0}; //Make sure its a PWM pin

#if(DEBUG==0)

  //Tweak this values depnding on WHERE this is setup.
  #define nsamples 5 //Longer values means more sample but long time between each light
  const uint16_t threshold[] = 
    { 5020, 4700, 0000, 0000,
      0000, 0000, 0000, 0000,
      0000, 0000, 0000, 0000,
      0000, 0000, 0000, 0000
    };
 
  
  int16_t samples[SENSORS][nsamples];

  int readIR(int ir);
  
#endif

void setup() {

  // Setup Pins
  for (uint8_t i = 0; i < SENSORS; i++) { 
    //pinMode(photoPin[i],INPUT); /*Photo ADC pins are default input*/
    pinMode(irPin[i]   ,OUTPUT);
    pinMode(ledPin[i]  ,OUTPUT);
  }
  
  // Serial for Debug
  Serial.begin(9600);
  #if(SERIAL==1)
    Serial.println("Setup done!");
  #endif

}

void loop() {
  
   #if(DEBUG==1)
     //Test if the LEDS are placed correctly
     static uint8_t i,j;
     for( i = 255; i >=0 ; i-=8 ){
       delay(20);
       #if(SERIAL==1)
         Serial.println(i);
       #endif
       for ( j = 0 ; j < SENSORS ; ++j )
         analogWrite(ledPin[j],i);
     }
   #endif
   
   #if(DEBUG==0)
     //Default Mode
     static uint8_t ir;
     static uint16_t sum;
     static int16_t light[SENSORS];
     ir = 0;

     while ( ir < SENSORS ) {
        #if(SERIAL==1)
          Serial.println("Pin is:");
          Serial.println(ir);
        #endif
        
        //Grab next sensors
        digitalWrite(irPin[ir],HIGH);
        sum = readIR(ir);
        #if(SERIAL==1)
          Serial.println("SUM is:");
          Serial.println(sum);
        #endif
        digitalWrite(irPin[ir],LOW);
   
        //calculate fade
        if ( sum < threshold[ir] ){
          #if(SERIAL==1)
            Serial.println("Light on");
          #endif
          //Turn on
          light[ir] = 255;
          analogWrite(ledPin[ir],light[ir]);
        }
        else if (light[ir] > 0 ){
          light[ir] -=16;    // Large value faster fade
          if ( light[ir] < 0 )
            light[ir] = 0;
          analogWrite(ledPin[ir],light[ir]);
        }
        
        ir++;
        #if(SERIAL==1)
          Serial.println("");
        #endif
        delay(20);
      }

   #endif
}

#if(DEBUG==0)
//Insert functions here

  int readIR(int ir) {
    static uint16_t sum;
    static uint8_t i;
    sum = 0;
    
    for (i=0; i< nsamples - 1; i++) {
      sum += samples[ir][i+1];
      samples[ir][i] = samples[ir][i+1];
    }
    
    samples[ir][nsamples - 1] = analogRead(photoPin[ir]);
    sum += samples[ir][nsamples - 1];
    
    return sum;
    
  }
  
#endif
