/* This was the original octolively code forked from
 * https://github.com/oskay/Octolively
 * Modified for the Arduino.
 *
 * Special thanks to Evil Mad Science
 *
 * Edited by Matthew Hardwick, David Tran
 *
 * Tested working production code
 * Working on April 8, 2012!
 * HAZZAH!
 */

/* Change SENSORS to change the number of sensors for the connected device
 * Please make sure that the const arrays contain at least that
 * many elements.
 *
 * For future reference, debugging is very useful BUT DO NOT
 * leave the Serial debug statements on when deploying it!
 */

#define SENSORS 9

/* DEBUG/SERIAL statements. Don't edit.
 */
 
#define debug 0
#define DEBUG if (debug) 
#define serial 1
#define SERIAL if (serial) 

/* These defines are important. They break the one loop
 * that break the light array into chars, into two different loops
 *
 * This code assumes the size of char is 1 byte.
 */

#define LOOP_FULL (SENSORS>>3)
#define LOOP_MIN  (SENSORS&7)

//Output pin
const uint8_t irPin[]    = { 30, 32, 34, 36, 38, 40, 42, 44, 46 };

//Input pin
const uint8_t photoPin[] = {  0,  1,  2,  3,  4,  5,  6,  7,  8 };

//Make sure its a PWM pin (Output)
const uint8_t ledPin[]   = {  2,  3,  4,  5,  6,  7,  8,  9, 10 };

/* This is the number of samples we will have.
 * Higher number means more samples but it will take more time
 * to finish reading all the samples.
 *
 * In total, we will take (nsample*SENSORS) samples.
 */

#define nsamples 16

//Null term for the transmission
#define SEND_TERM "z"

char output_buffer[] = "000000000ABCDEF";

//Delay Values (If we need delay)
#define DELAY_CEIL 20

//Data Structures

/* Tweak this value depending on WHERE this is setup.
 * This is the threshold value that determines if
 * the photo sensor should say it is "ON" or "OFF"
 */
 
 #define THE 60

const uint16_t threshold[] = {
  THE, THE, THE, THE, THE, THE, THE, THE, THE };

/* Samples array.
 * Contains all the samples for this point in time.
 */

int16_t samples[SENSORS][nsamples];

//Prototype Function Calls
int readIR(int ir);

#include <SoftwareSerial.h>

SoftwareSerial mySerial(0, 1); // RX, TX

void setup() {

  // Setup Pins
  for (uint8_t i = 0; i < SENSORS; i++) {
    //pinMode(photoPin[i],INPUT); /*Photo ADC pins are default input*/
    pinMode(irPin[i]   ,OUTPUT);
    pinMode(ledPin[i]  ,OUTPUT);
    }

  // Serial for Debug
  if ( (debug) || (serial) ) Serial.begin(9600);
  //mySerial.begin(9600);
  DEBUG Serial.println("Setup done!");

  }

void loop() {

  //Input parsing
  static uint16_t ir;
  static uint16_t sum;
  static int16_t light[SENSORS];
  static uint16_t delay = 0;

  //Output parsing
  static unsigned char send;
  static uint8_t i,j;

  ir = 0;

  while ( ir < SENSORS ) {
    DEBUG Serial.println("Pin is:");
    DEBUG Serial.println(ir);

    //Grab next nsamples
    digitalWrite(irPin[ir],HIGH); //Turn on sensor
    sum = readIR(ir);             //Grab the sample set
    DEBUG Serial.println("SUM is:");
    DEBUG Serial.println(sum);
    digitalWrite(irPin[ir],LOW);  //Turn off sensor

    //Turning it on
    if ( sum > threshold[ir] ){
      DEBUG Serial.println("Light on");

      light[ir] = 252;
      analogWrite(ledPin[ir],light[ir]); //Turn on sensor
      }

    //Turning it off, aka fade
    else if ( light[ir] > 0){
      if (!debug) light[ir] -= 4;        //Fade Amount Here
      DEBUG light[ir]>>=1;               //DEBUG FADE
      analogWrite(ledPin[ir],light[ir]); //Write the state in
      }
    ir++; //Go to next sensor
    DEBUG Serial.println("");
    }

  /* At this point we have sampled all the sensors
   * We will send out the information in chars.
   *
   * We will send out the string
   */
  
  //Delay area for "slowing" down transmission.
  if (serial)
    if ( delay != DELAY_CEIL )
      delay += 1; 
    else{ //Here is where we actually send the char.
      delay = 0;
      DEBUG Serial.print("Sending the bits:");
      //Serial.print("10123123123\n" );
      for( int i = 0 ; i < SENSORS ; i++ ){
          output_buffer[i] = ((char)((light[i]>=240) + '0'));
          //Serial.print( (char)((light[i]>=240) + '0') );
          DEBUG Serial.print( (char)((light[i]>=240) + '0') );
      }
      //mySerial.write(output_buffer);
      Serial.write(output_buffer);
      Serial.write('\n');
      //Serial.write("0010");
      //Serial.print('\n');
      DEBUG Serial.print(SEND_TERM);
      DEBUG Serial.println("");
    }
  
  //END of loop (REPEAT AGAIN!)
  }

//Function readIR
int readIR(int ir) {
  /* This function will read in the sensor value of the ir th sensor
   * and spit back the sum of the read.
   * Possible return values are 0 to (1024*nsamples)-1
   *
   * Remember not to overflow the int!
   *
   * Reading a sensor that doesn't exist is not advised.
   */

  static uint16_t sum;
  static uint8_t i;

  sum = 0;

  for ( i = 0 ; i< nsamples - 1; i++) {
    sum += samples[ir][i+1];
    samples[ir][i] = samples[ir][i+1];
    }

  samples[ir][nsamples - 1] = analogRead(photoPin[ir]); //Values are from 0-1023
 
  sum += samples[ir][nsamples - 1];

  return(sum);

  }
