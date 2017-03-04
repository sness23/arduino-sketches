//
// sness_rotary_encoder_7_segment
//

#include <ProTrinketHidCombo.h>
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_7segment matrix = Adafruit_7segment();

#define PIN_ENCODER_A      3
#define PIN_ENCODER_B      5
#define TRINKET_PINx       PIND
#define PIN_ENCODER_SWITCH 4

// rotary encoder
static uint8_t enc_prev_pos   = 0;
static uint8_t enc_flags      = 0;
static char    sw_was_pressed = 0;

// 7 segment
static int val = 1024;
static int counter = 0;
static int MAX_COUNT = 10000;
 
void setup()
{
  Serial.begin(9600);
  // set pins as input with internal pull-up resistors enabled
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SWITCH, INPUT_PULLUP);
 
  // get an initial reading on the encoder pins
  if (digitalRead(PIN_ENCODER_A) == LOW) {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(PIN_ENCODER_B) == LOW) {
    enc_prev_pos |= (1 << 1);
  }

  // 7 segment
  matrix.begin(0x70);
  
  Serial.println("setup");
}

void displayVal() {
  matrix.print(val);
  matrix.writeDisplay();
}
 
void loop()
{
  // 7 segment
  counter++;
  if (counter > MAX_COUNT) {
      displayVal();
      counter = 0;
  }

  //Serial.println("loop");
  int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
 
  // note: for better performance, the code will use
  // direct port access techniques
  // http://www.arduino.cc/en/Reference/PortManipulation
  
  uint8_t enc_cur_pos = 0;
  // read in the encoder state first
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_A)) {
    enc_cur_pos |= (1 << 0);
  }
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_B)) {
    enc_cur_pos |= (1 << 1);
  }
 
  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos)
  {
    if (enc_prev_pos == 0x00)
    {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        enc_flags |= (1 << 0);
      }
      else if (enc_cur_pos == 0x02) {
        enc_flags |= (1 << 1);
      }
    }
 
    if (enc_cur_pos == 0x03)
    {
      // this is when the encoder is in the middle of a "step"
      enc_flags |= (1 << 4);
    }
    else if (enc_cur_pos == 0x00)
    {
      // this is the final edge
      if (enc_prev_pos == 0x02) {
        enc_flags |= (1 << 2);
      }
      else if (enc_prev_pos == 0x01) {
        enc_flags |= (1 << 3);
      }
 
      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
      else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
 
      enc_flags = 0; // reset for next time
    }
  }
 
  enc_prev_pos = enc_cur_pos;
 
  if (enc_action > 0) {
    Serial.println("ccw");
    val--;
    //TrinketHidCombo.pressMultimediaKey(MMKEY_VOL_UP);  // Clockwise, send multimedia volume up
  }
  else if (enc_action < 0) {
    Serial.println("cw");
    val++;
    //TrinketHidCombo.pressMultimediaKey(MMKEY_VOL_DOWN); // Counterclockwise, is multimedia volume down
  }
 
  // remember that the switch is active low 
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_SWITCH)) 
  {
    if (sw_was_pressed == 0) // only on initial press, so the keystroke is not repeated while the button is held down
    {
      Serial.println("sw_was_pressed");
      val = 1024;
      //TrinketHidCombo.pressMultimediaKey(MMKEY_MUTE); // Encoder pushed down, toggle mute or not
      delay(5); // debounce delay
    }
    sw_was_pressed = 1;
  }
  else
  {
    if (sw_was_pressed != 0) {
      delay(5); // debounce delay
    }
    sw_was_pressed = 0;
  }
 
}