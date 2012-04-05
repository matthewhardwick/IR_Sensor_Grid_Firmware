/* This is a miniature program that reads the light array
 * and converts it into char pieces to send.
 *
 * Created by David Tran
 */

#define SENSORS 9

#define LOOP_FULL (SENSORS>>3)
#define LOOP_MIN  (SENSORS&7)

#include <stdio.h>
#include <stdint.h>

int main(){

  uint8_t light[SENSORS];

  light[0] = 255;
  light[1] = 255;
  light[2] = 255;
  light[3] = 255;
  light[4] = 255;
  light[5] = 252;
  light[6] = 65;
  light[7] = 32;
  light[8] = 255;
  light[9] = 253;

  char send;
  uint8_t i,j;

  //Here will send all the full characters
  if ( LOOP_FULL != 0 ){
    for( i = 0 ; i < LOOP_FULL ; ++i ){
      send = 0;
      for( j = 0 ; j < 8 ; ++j )
        send |= (((light[(i<<3)+j]==255)<<j));
      //Serial.write(send);
      printf("%x",send);
    }
  }

  //Here we will send the remaining character
  if ( LOOP_MIN != 0 ){
    send = 0;
    for( i = 0 ; i < LOOP_MIN ; ++i )
      send |= (((light[(LOOP_FULL<<3)+i]==255)<<i));
    //Serial.write(send);
    printf("%x",send);
  }

  return(0);
}

