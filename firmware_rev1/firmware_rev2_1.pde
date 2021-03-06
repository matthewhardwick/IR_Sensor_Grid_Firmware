
/* This was the original octolively code forked from
 * https://github.com/oskay/Octolively
 * Modified for the Arduino.
 *
 * Special thanks to Evil Mad Science
 *
 * Edited by Matthew Hardwick, David Tran
 *
 */

/* Change SENSORS to change the number of sensors for the connected device
 * Please make sure that the const arrays contain at least that
 * many elements.
 *
 * For future reference, debugging is very useful BUT DO NOT
 * leave the Serial debug statements on when deploying it!
 */

#define SENSORS 4
#define debug 0
#define DEBUG if (debug)

/* These defines are important. They break the one loop
 * that break the light array into chars, into two different loops
 *
 * This code assumes the size of char is 1 byte.
 */

#define LOOP_FULL (SENSORS>>3)
#define LOOP_MIN  (SENSORS&7)


//Input pin
const uint8_t irPin[]    = {2,  4,  7,  8, 12, 13,  0,  0,  0};

//Output pin
const uint8_t photoPin[] = {0,  1,  2,  3,  4,  5,  6,  7,  0};

//Make sure its a PWM pin
const uint8_t ledPin[]   = {9, 10, 11,  5,  0,  0,  0,  0,  0};

/* This is the number of samples we will have.
 * Higher number means more samples but it will take more time
 * to finish reading all the samples.
 *
 * In total, we will take (nsample*SENSORS) samples.
 */

#define nsamples 5

//Data Structures

/* Tweak this value depending on WHERE this is setup.
 * This is the threshold value that determines if
 * the photo sensor should say it is "ON" or "OFF"
 *
 */

const uint16_t threshold[] =
{
  5000, 5000, 5000, 4900,
  0000, 0000, 0000, 0000,
  0000, 0000, 0000, 0000,
  0000, 0000, 0000, 0000
  };

int16_t samples[SENSORS][nsamples];

//Prototype Function Calls
int readIR(int ir);

void setup() {

  // Setup Pins
  for (uint8_t i = 0; i < SENSORS; i++) {
    //pinMode(photoPin[i],INPUT); /*Photo ADC pins are default input*/
    pinMode(irPin[i]   ,OUTPUT);
    pinMode(ledPin[i]  ,OUTPUT);
    }

  // Serial for Debug
  DEBUG Serial.begin(9600);
  DEBUG Serial.println("Setup done!");

  }

void loop() {

  //Input parsing
  static uint8_t ir;
  static uint16_t sum;
  static int16_t light[SENSORS];

  //Output parsing
  static unsigned char send;
  static uint8_t i,j;

  ir = 0;

  while ( ir < SENSORS ) {
    DEBUG Serial.println("Pin is:");
    DEBUG Serial.println(ir);

    //Grab next nsamples
    digitalWrite(irPin[ir],HIGH); //Turn on sensor
    sum = readIR(ir);
    DEBUG Serial.println("SUM is:");
    DEBUG Serial.println(sum);
    digitalWrite(irPin[ir],LOW);  //Turn off sensor

    //Turning it on
    if ( sum < threshold[ir] ){
      DEBUG Serial.println("Light on");

      light[ir] = 255;
      analogWrite(ledPin[ir],light[ir]); //Turn on sensor
      }

    //Turning it off, aka fade
    else if ( light[ir] > 0){
      if (!debug) light[ir] -= 1; //Fade Amount Here
      DEBUG light[ir]>>=1;
      analogWrite(ledPin[ir],light[ir]); //Write the state in
      }
    ir++; //Go to next sensor
    DEBUG Serial.println("");
    }

  /* At this point we have sampled all the sensors
   * We will send out the information in chars.
   *
   * We will send (SENSORS>>3) + ((SENSORS&7)>0) chars
   */
   
   DEBUG Serial.print("Sending the char:");

  //Here will send all the full characters
  if ( LOOP_FULL != 0 ){
    for( i = 0 ; i < LOOP_FULL ; ++i ){
      send = 0;
      for( j = 0 ; j < 8 ; ++j )
        send |= (((light[(i<<3)+j]==255)<<j));
      Serial.write(send);
      DEBUG Serial.print(send);
      }
    }

  //Here we will send the remaining character
  if ( LOOP_MIN != 0 ){
    send = 0;
    for( i = 0 ; i < LOOP_MIN ; ++i )
      send |= (((light[(LOOP_FULL<<3)+i]==255)<<i));
    Serial.write(send);
    DEBUG Serial.print(send);
    }
    
  DEBUG Serial.println("");

  }

//Function readIR
int readIR(int ir) {
  /* This function will read in the sensor value of the ir th sensor
  * and spit back the sum of the read.
  *
  * Reading a sensor that doesn't exist is not advised.
  */

  static uint16_t sum;
  static uint8_t i;

  sum = 0;

  for (i=0; i< nsamples - 1; i++) {
    sum += samples[ir][i+1];
    samples[ir][i] = samples[ir][i+1];
    }

  samples[ir][nsamples - 1] = analogRead(photoPin[ir]); //Values are from 0-1023
  sum += samples[ir][nsamples - 1];

  return(sum);

  }

