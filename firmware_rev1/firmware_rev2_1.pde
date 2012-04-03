//Modified by David Tran

//Change this to change the number of sensors for the device
#define SENSORS 4
#define DEBUG 0
 
const uint8_t irPin[]    = {2,  4,  7,  8, 12, 13,  0,  0, 0}; //Input pin
const uint8_t photoPin[] = {0,  1,  2,  3,  4,  5,  6,  7, 0}; //Output pin
const uint8_t ledPin[]   = {9, 10, 11, 5,  0,  0,  0,  0, 0}; //Make sure its a PWM pin

#if(DEBUG==0)

  //Tweak this values depending on WHERE this is setup.
  #define nsamples 5 
  //Longer values means more sample but long time between each light
  const uint16_t threshold[] = 
    { 4850, 5000, 5000, 4800,
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
    Serial.println("Setup done!");

}

void loop() {
  
   #if(DEBUG==1)
     //Test if the LEDS are placed correctly
     static uint8_t i,j;
     for( i = 255; i >=0 ; i-=8 ){
       //delay(20);
       Serial.println(i);
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
          Serial.println("Pin is:");
          Serial.println(ir);
        
        //Grab next n sensors
        digitalWrite(irPin[ir],HIGH); // Turn on sensor
        sum = readIR(ir);
          Serial.println("SUM is:");
          Serial.println(sum);
        digitalWrite(irPin[ir],LOW);  //Turn off sensor
   
        //calculate fade
        if ( sum < threshold[ir] ){
            Serial.println("Light on");
            
          //Turn on
          light[ir] = 255;
          analogWrite(ledPin[ir],light[ir]);
        }
        //Fade here
        else if (light[ir] > 0 ){
          
          if ( light[ir] == 255 ) {
            light[ir] = 192;
          }
          else {
            if ( light[ir] == 192 ){
              light[ir] = 128;
            }
            else if ( light[ir] == 128 ){
              light[ir] = 64;
            }
            else if ( light[ir] == 64 ){
              light[ir] = 48;
            }
            else if ( light[ir] == 48 ){
              light[ir] = 32;
            }
            else if ( light[ir] == 32 ){
              light[ir] = 24;
            }
            else if ( light[ir] == 24 ){
              light[ir] = 16;
            }
            else if ( light[ir] == 16 ){
              light[ir] = 8;
            }
            else if ( light[ir] == 8 ){
              light[ir] = 4;
            }
            else if ( light[ir] == 4 ){
              light[ir] = 2;
            }
            else if ( light[ir] == 2 ){
              light[ir] = 1;
            }
            else
              light[ir] = 0;
          }
          analogWrite(ledPin[ir],light[ir]);
        }
        ir++;
        Serial.println("");
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
