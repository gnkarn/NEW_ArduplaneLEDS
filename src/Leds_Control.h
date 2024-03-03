/*
 * led_control.ino part of MavLink_FrSkySPort
 *
 * Copyright (C) 2015 Jochen Kielkopf
 * https://github.com/Clooney82/MavLink_FrSkySPort
 *
 * modified by Alf Pettersson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses>.
 *
 * Additional permission under GNU GPL version 3 section 7
 *
 * If you modify this Program, or any covered work, by linking or
 * combining it with FrSkySportTelemetry library (or a modified
 * version of that library), containing parts covered by the terms
 * of FrSkySportTelemetry library, the licensors of this Program
 * grant you additional permission to convey the resulting work.
 * {Corresponding Source for a non-source form of such a combination
 * shall include the source code for the parts of FrSkySportTelemetry
 * library used as well as that of the covered work.}
 *
 * ====================================================================================================
 *
 * Lighting Controller for ArduCopter based on Teensy 3.1 and
 * APM2.5 Mavlink to FrSky X8R SPort interface using Teensy 3.1
 *
 * Teensy 3.1 PIN 6 is used for controlling WS2812B LEDs
 * Keep in mind teensy only provide 3.3V Output, to control WS2812B LEDs you
 * need to shift this to 5V. possible Solution:  *1:
 * https://www.pjrc.com/teensy/td_libs_OctoWS2811.html *2:
 * http://forums.adafruit.com/viewtopic.php?f=47&t=48908&p=246902#p246902
 *
 * To Use this script in Simulation Mode (#define SIMULATION_MODE)
 * In SIMULATION_MODE script can run stand alone on your development
 * environment, but no input is possible.
 */

// #####################################################################################################
// ### PROGAM BEGIN ###
// #####################################################################################################
// ### PROGRAM SETUP ###
// #####################################################################################################
//  function declarations

#ifdef USE_TEENSY_LED_SUPPORT
//#define SIMULATION_MODE // uncomment to enable simulation mode ( habilitar para probar los diferentes modos)
//#define standalone      // DO NOT UNCOMMENT, just for testing purpose as standalone script without mavlink part.

#ifdef SIMULATION_MODE
int SIM_LOOP;
unsigned long targetmillisfc_sim_mode = 0;
#endif
#ifdef standalone
int ap_base_mode;
int ap_bar_altitude;
int ap_sat_visible;
int ap_fixtype;
int ap_custom_mode;
int ap_throttle;
int ap_chancount;
int ap_chan_raw[18];
int ap_rssi;
#endif

// #####################################################################################################
// ### LED VARIABLES ###
// #####################################################################################################

#define GPS 1
#define ARMED 2
#define FRONT 3
#define BACK 4
#define SIDE 5
#define FLASH 6
/* const byte LED_DEF[NUM_ARMS][NUM_LEDS_PER_STRIP] PROGMEM = {
    {GPS, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, ARMED, FLASH, FLASH},
    {GPS, FRONT, FRONT, FRONT, FRONT, FRONT, FRONT, ARMED, FLASH, FLASH},
    {GPS, BACK, BACK, BACK, BACK, BACK, BACK, ARMED, FLASH, FLASH},
    {GPS, BACK, BACK, BACK, BACK, BACK, BACK, ARMED, FLASH, FLASH} }; */
const byte LED_DEF[NUM_ARMS][NUM_LEDS_PER_STRIP] PROGMEM = {
    {ARMED, GPS, GPS, FRONT, FLASH } };

bool motorsArmed(void);  //
void copter_leds_oscillate(void);
void copter_leds_on(void);  // if motors are armed, battery level OK, all motor leds ON
void copter_leds_slow_blink(void);

void GenericFlash(uint8_t, uint8_t, uint8_t, int, uint8_t);

#endif  // Lled_control_H

// ====================================================================================================
// FIN  variables globales
// ====================================================================================================
#ifdef standalone
void setup() {  // uncomment in SIMULATION_MODE
#else
void Teensy_LED_Init() {  // comment in SIMULATION_MODE
#endif
  // FastLED.addLeds<WS2811, 6,RGBORDER >(leds[0],
  // NUM_LEDS_PER_STRIP);  //Right front arm
  // FastLED.addLeds<WS2812B, DATA_PIN-2,RGBORDER>(leds[0], NUM_LEDS_PER_STRIP);
  // //Left front arm FastLED.addLeds<WS2812B,DATA_PIN+1,RGBORDER>(leds[1],
  // NUM_LEDS_PER_STRIP);  //Left front arm
  Serial.println("Teensy_LED_Init");
  FastLED.addLeds<WS2812B, DATA_PIN, RGBORDER>(
    leds, NUM_LEDS);  //
                      // FastLED.addLeds<APA102, 6, 7,
                      // RGB,DATA_RATE_MHZ(12)>(leds[0], NUM_LEDS_PER_STRIP);
                      // // HABILITAR PARA APA
// FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB, DATA_RATE_MHZ(12)>(leds,NUM_LEDS);  // DOCUMENTADO PARA apa
  FastLED.setBrightness(BRIGHTNESS);
#ifdef standalone
  Serial.begin(9600);
#endif

  Leds_Test();
  Serial.println("efectos..");
 // ***********test de efecto
 // ***************void HeliLed (int bpm , int hue,float dim  )
  for (int i = 0; i < 500; i++) {
    // Call the current pattern function once, updating the 'leds' array
    gPatterns[gCurrentPatternNumber]();  // habilitar esto
    // FastLED.show();                           // display this frame
    FastLED.delay(1000 / FRAMES_PER_SECOND);  // 60 FRAMESPERSECOND
    // do some periodic updates
    EVERY_N_MILLISECONDS(20) { gHue++; }  // slowly cycle the "base color" through the rainbow
      // do some periodic updates
    EVERY_N_SECONDS(10) { nextPattern(); }  // change patterns periodically

    // EVERY_N_MILLISECONDS(60000/g_bpm){g_bpm = (g_bpm*2) % 80 ; } // 60000/bpm
    // es el tiempo  que se tarda en dar una vuelta
    //Serial.print(i);
    //Serial.print(" ");
    };
  delay(500);
  Serial.println("Init END");
  } // END Teensy leds init

  // #####################################################################################################
  // ### PROGRAM LOOP ###
  // #####################################################################################################
#ifdef standalone
void loop() {  // uncomment in SIMULATION_MODE
#else

void Teensy_LED_process() {  // comment in SIMULATION_MODE ( DEJAR NORMAL)
#endif
  // Serial.println("Teensy_LED_process ");
  // float dim = 100; // para probar lo paso a global
  currentmillis = millis();
  ap_base_mode_last = ap_base_mode;

#ifdef SIMULATION_MODE
  fc_simulation_mode();
#endif
  if (LED_MODE_SWITCH == 0) {
    get_mode();
    if (LED_DIMM_CHAN != 0) {
      dim = rc_dimmer_proz();
      }
    }
  else {
  #ifdef SIMULATION_MODE
  #else
      /*
       * from MavLink_FrSkySPort.ino
       * ap_chan_raw[i]  => RC channel i input value, in microseconds. A value of
       * UINT16_MAX (65535U)
       */
      // Serial.print("led mode switch : "); //debug
      // Serial.print(ap_chan_raw[LED_MODE_SWITCH]); // debug
    gbpm = map(ap_chan_raw[LED_BPM_CHAN], 1000, 2000, 2,
      200);  // varia el gbpm en mas o en menos en funcion del valor
             // del canal 17

    if (ap_chan_raw[LED_MODE_SWITCH] <= 1000) {
      get_mode();  // DEFAULT, LED Mode - auto set by FC
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1080) {
      LED_MODE = 101;  // USER_DEFINED LED Mode 1
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1165) {
      LED_MODE = 102;  // USER_DEFINED LED Mode 2
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1250) {
      LED_MODE = 103;  // USER_DEFINED LED Mode 3
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1320) {
      LED_MODE = 104;  // USER_DEFINED LED Mode 4
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1400) {
      LED_MODE = 105;  // USER_DEFINED LED Mode 5
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1480) {
      LED_MODE = 106;  // USER_DEFINED LED Mode 6
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1560) {
      LED_MODE = 107;  // USER_DEFINED LED Mode 7
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1640) {
      LED_MODE = 108;  // USER_DEFINED LED Mode 8
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1730) {
      LED_MODE = 109;  // USER_DEFINED LED Mode 9
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1810) {
      LED_MODE = 110;  // USER_DEFINED LED Mode 10
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1890) {
      LED_MODE = 111;  // USER_DEFINED LED Mode 11
      }
    else if (ap_chan_raw[LED_MODE_SWITCH] <= 1966) {
      LED_MODE = 112;  // USER_DEFINED LED Mode 12
      }
    else {
      get_mode();  // DEFAULT, LED Mode - auto set by FC
      }
    if (LED_DIMM_CHAN != 0) {
      dim = rc_dimmer_proz();
      }
  #endif
    }
  dim = dim / 100;  // ver si esta linea hace falta -- probar.

  switch (LED_MODE) {
      case 1:                   // NO_GPS Flight modes
        default_mode(ON, 250);  // antes ON,dim
        break;
      case 2:  // GPS    Flight modes
        front_arms(ON, dim);
        rear_arms(ON, dim);
        flash_pos_light(ON, dim);
        get_armed_status(ON, dim);
        get_gps_status(ON, dim);
        break;
      case 3:                   // MANUAL Flight modes
        default_mode(ON, 250);  // en lugar de dim pongo 250 pues no tengo knob de
                                // dim con en taranis
        break;
      case 4:  // AUTO   Flight modes
        // LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
        get_armed_status(ON, dim);
        get_gps_status(ON, dim);
        front_arms(ON, dim);
        rear_arms(ON, dim);
        break;
      case 5:  // LAND   Flight mode
        // LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
        flash_pos_light(ON, dim);
        get_armed_status(ON, dim);
        get_gps_status(ON, dim);
        break;
      case 6:  // BRAKE Flight mode
        // flash_pos_light(ON, dim);
        // get_armed_status(ON, dim);
        // get_gps_status(ON, dim);
        break;
      case 7:  // FLIP  Flight mode
        // LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
        break;
      case 8:  // AUTO_TUNE, ...
        // LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
        front_arms(ON, dim);
        rear_arms(ON, dim);

        break;
      case 101:  // USER Mode 1
        // LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
        // discostrobe();
        break;
      case 102:  // USER Mode 2
        // LarsonScanner(0, 0, NUM_LEDS_PER_STRIP, dim);
        get_armed_status(ON, dim);
        get_gps_status(ON, dim);
        break;
      case 103:  // USER Mode 3
        front_arms(ON, dim);
        rear_arms(ON, dim);
        get_armed_status(ON, dim);
        get_gps_status(ON, dim);
        flash_pos_light(ON, dim);
        // longRoll();
        // setBandera();
        break;
      case 104:  // USER Mode 4
        // front_arms(OFF, dim);
        // rear_arms(OFF, dim);
        // get_armed_status(OFF, dim);
        // get_gps_status(OFF, dim);
        flash_pos_light(ON, dim);
        // brazoSenoidal();          //  prueba de efecto
        break;
      case 105:  // USER Mode 5 -
        // rainbowWithGlitter();
        break;
      case 106:  // USER Mode 6 -
                 // paletteWaves();
        flash_pos_light(ON, dim);
        break;
      case 107:  // USER Mode 7 -
        applause();
        break;
      case 108:  // USER Mode 8 -
        // void HeliLed (int bpm , int hue,float dim  )
        // HeliLed(30 ,  gHue, dim  );
        break;
      case 109:  // USER Mode 9 -
        bpm();
        break;
      case 110:  // USER Mode 10 -
        // confetti();
        break;
      case 111:  // USER Mode 11 -
        // paletteWaves();
        break;
      case 112:  // USER Mode 12 -
        // juggle();
        // addGlitter(80);
        break;
      case 999:  // disarmed
        front_arms(OFF, dim);
        rear_arms(OFF, dim);
        flash_pos_light(ON, dim);
        get_armed_status(ON, dim);
        get_gps_status(ON, dim);
        break;
    }
    // ThrotleLed (int brazo , int posicion, int LedsNum, int hue, float dim  )
    // ThrotleLed (0, 2, 13, 200,  dim  );
  get_gps_status(ON, dim); // lo agrego para TEST 
  update_copter_leds();  // update motors and gps leds
  FastLED.show();
  lastmillis = currentmillis;
  }

  // #####################################################################################################
  // ###                             DIMMER FUNCTIONs ###
  // #####################################################################################################
byte rc_dimmer() {  // ppm to byte (min 0 / max 255)
  double factor = (PWM_MAX - PWM_MIN) / 255;
  /*
   * double ppm_val = ap_chan_raw[LED_DIMM_CHAN] - PWM_MIN;
   * float dim = ppm_val / factor;
   */
  byte TARGET_DIM;
  if ((ap_chan_raw[LED_DIMM_CHAN] - PWM_MIN) / factor > 255) {
    TARGET_DIM = 255;
    }
  else if ((ap_chan_raw[LED_DIMM_CHAN] - PWM_MIN) / factor < 0) {
    TARGET_DIM = 0;
    }
  else {
    TARGET_DIM = (ap_chan_raw[LED_DIMM_CHAN] - PWM_MIN) / factor;
    }
  return TARGET_DIM;
  }

byte rc_dimmer_proz() {  // ppm to % ( min 0 % / max 100 % )
  double factor = (PWM_MAX - PWM_MIN) / 100;
  byte TARGET_DIM;
  if (ap_chan_raw[LED_DIMM_CHAN] > 900) {
    TARGET_DIM = (ap_chan_raw[LED_DIMM_CHAN] - PWM_MIN) / factor;
    if (TARGET_DIM > 100) {
      TARGET_DIM = 100;
      }
    else if (TARGET_DIM < 0) {
      TARGET_DIM = 0;
      }
    }
  else {
    TARGET_DIM = 15;
    }
  return TARGET_DIM;
  }

  // #####################################################################################################
  // ### SIMULATION FUNCTION ###
  // #####################################################################################################
#ifdef SIMULATION_MODE
void fc_simulation_mode() {
  int max_loops = 60;

  if (lastmillis >= targetmillisfc_sim_mode) {
    // LED_MODE_SWITCH = 0;// GNK
    if (SIM_LOOP <= 3) {
      ap_fixtype = 0;
      ap_sat_visible = random(0, SIM_LOOP);
      // SIM_LOOP = 20;
      }
    else if (SIM_LOOP <= 5) {
      ap_fixtype = 1;
      ap_sat_visible = random(SIM_LOOP, 3);
      }
    else if (SIM_LOOP > 5) {
      ap_fixtype = 2;
      ap_sat_visible = random(SIM_LOOP, 6);
      }

    if (SIM_LOOP > 8) {
      ap_fixtype = 3;
      ap_sat_visible = random(6, 20);
      }

    if (SIM_LOOP > 10) {
      ap_base_mode = 1;
      }
    else {
      ap_base_mode = 0;
      ap_bar_altitude = 0;
      }

    if (SIM_LOOP > 13) {
      ap_custom_mode = random(0, 17);
      ap_throttle = random(50, 100);
      ap_bar_altitude = random(0, 1000);
      }
    else {
      ap_custom_mode = 0;
      ap_throttle = 0;
      }

    if (SIM_LOOP > 20 && SIM_LOOP < 40) {
      LED_MODE = random(101, 105);
      LED_MODE_SWITCH = 13;
      }
    else {
      LED_MODE_SWITCH = 0;
      }

    if (SIM_LOOP > max_loops - 10) {
      ap_throttle = 0;
      }

    if (SIM_LOOP > max_loops - 5) {
      ap_throttle = 0;
      ap_base_mode = 0;
      ap_custom_mode = 0;
      LED_MODE = 0;
      }

    if (SIM_LOOP > max_loops) {
      SIM_LOOP = 0;
      }
    else {
      SIM_LOOP++;
      }

    if (SIM_LOOP < 9) {
      LED_MODE = SIM_LOOP;
      LED_MODE_SWITCH = 13;
      }

      /*
      LED_MODE = 102;
      LED_MODE = 4;
      LED_MODE_SWITCH = 13;
      */

    Serial.println();
    Serial.print("LED_MODE: ");
    Serial.print(LED_MODE);
    Serial.print(" LastMillis: ");
    Serial.print(lastmillis);
    Serial.print(" CurMillis: ");
    Serial.print(currentmillis);
    Serial.print(", targetMillis: ");
    Serial.print(targetmillisfc_sim_mode);
    Serial.print(", Loop: ");
    Serial.print(SIM_LOOP);
    Serial.print(":");
    Serial.println();
    Serial.print("fixtype: ");
    Serial.print(ap_fixtype);
    Serial.print(" #Sats: ");
    Serial.print(ap_sat_visible);
    Serial.print(" Armed?: ");
    Serial.print(ap_base_mode);
    Serial.print(" FlightModes: ");
    Serial.print(ap_custom_mode);
    Serial.print(" Throttle: ");
    Serial.print(ap_throttle);
    Serial.print(" ALT: ");
    Serial.print(ap_bar_altitude);
    Serial.println();

    targetmillisfc_sim_mode = currentmillis + 3000;
    }
  }
#endif

// #####################################################################################################
// ### GET ArduCopter Flightmode ###
// #####################################################################################################
void get_mode() {
  /* from MavLink_FrSkySPort.ino
   *  ap_custom_mode  => see
   * http://copter.ardupilot.com/wiki/configuration/arducopter-parameters/#flight_mode_1_arducopterfltmode1
   *  LED_MODE  TYPE
   *  1         NO_GPS Flight modes
   *  2         GPS    Flight modes
   *  3         MANUAL Flight modes
   *  4         AUTO   Flight modes
   *  5         LAND
   *  6         BRAKE
   *  7         FLIP
   *  8         AUTO_TUNE, ...
   *  999       DISARMED
   *
   *  VALUE  MEANING    LED_MODE
   *  0      Stabilize  1
   *  1      Acro       3
   *  2      AltHold    1
   *  3      Auto       4
   *  4      Guided     4
   *  5      Loiter     2
   *  6      RTL        4
   *  7      Circle     4
   *  9      Land       5
   *  11     Drift      3
   *  13     Sport      1
   *  14     Flip       7
   *  15     AutoTune   8
   *  16     PosHold    2
   *  17     Brake      6
   *  ...    ...        ...
   *
   *  Mapping of AC Flight modes to LED lighting modes.
   */
  if (ap_base_mode == 1) {
    switch (ap_custom_mode) {
        case 0:
          LED_MODE = 1;
          break;
        case 1:
          LED_MODE = 3;
          break;
        case 2:
          LED_MODE = 1;
          break;
        case 3:
          LED_MODE = 4;
          break;
        case 4:
          LED_MODE = 5;
          break;
        case 5:
          LED_MODE = 2;
          break;
        case 6:
          LED_MODE = 4;
          break;
        case 7:
          LED_MODE = 4;
          break;
        case 9:
          LED_MODE = 5;
          break;
        case 11:
          LED_MODE = 3;
          break;
        case 13:
          LED_MODE = 1;
          break;
        case 14:
          LED_MODE = 7;
          break;
        case 15:
          LED_MODE = 8;
          break;
        case 16:
          LED_MODE = 2;
          break;
        case 17:
          LED_MODE = 6;
          break;
        default:
          LED_MODE = 1;
          break;
      }
    }
  else {
    LED_MODE = 999;
    }
  }

  // #####################################################################################################
  // ### SETUP DEFAULT MODE FOR LED LIGHTNING - SIDEARMS, FRONT-, BREAK-,
  // LANDING-LIGHT always off     ###
  // #####################################################################################################
void default_mode(int STATUS, float dim) {
  front_arms(STATUS, dim);
  rear_arms(STATUS, dim);
  flash_pos_light(ON, dim);
  get_armed_status(STATUS, dim);
  get_gps_status(ON, dim);
  }

  // #####################################################################################################
  // ### GET ARMED STATUS FROM ARDUPILOT AND PRINT CORESPONDING LED(s) ###
  // #####################################################################################################
void get_armed_status(int STATUS, float dim) {
  if (STATUS == 1) {
    switch (ap_base_mode) {
        case 0:
          for (int i = 0; i < NUM_ARMS; i++) {
            for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
              if (pgm_read_byte(&(LED_DEF[i][j])) == ARMED) {
                leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(85, 255, dim);
                }
              }
            }
          break;
        case 1:
          for (int i = 0; i < NUM_ARMS; i++) {
            for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
              if (pgm_read_byte(&(LED_DEF[i][j])) == ARMED) {
                leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(0, 255, dim);
                }
              }
            }
          break;
        default:
          for (int i = 0; i < NUM_ARMS; i++) {
            for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
              if (pgm_read_byte(&(LED_DEF[i][j])) == ARMED) {
                leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(0, 0, 0);
                }
              }
            }
          break;
      }
    }
  else {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (pgm_read_byte(&(LED_DEF[i][j])) == ARMED) {
          leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(0, 0, 0);
          }
        }
      }
    }
  }

  // #####################################################################################################
  // ### GET GPS STATUS FROM ARDUPILOT AND PRINT CORESPONDING LED(s) ###
  // #####################################################################################################
void get_gps_status(int STATUS, float dim) {
  /* from MavLink_FrSkySPort.ino
   *  ap_sat_visible  => numbers of visible satellites
   *  ap_fixtype    => 0 = No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
   */

  CRGB COLOR;
  int FREQ;
  if (STATUS == 1) {
    // Serial.print(ap_fixtype);
    switch (ap_fixtype) {
        case NO_GPS:  // blink red
          COLOR = CHSV(0, 255, dim);
          FREQ = 750;
          break;
        case NO_FIX:  // blink orange
          COLOR = CHSV(39, 255, dim);
          FREQ = 600;
          break;
        case GPS_OK_FIX_2D:  // blue
          COLOR = CHSV(160, 255, dim);
          FREQ = 500;
          break;
        case GPS_OK_FIX_3D:  // const blue
          COLOR = CHSV(160, 255, dim);
          FREQ = 0;
          state_GPS = 0;
          break;
        case GPS_OK_FIX_3D_DGPS:  // const blue
          COLOR = CHSV(160, 255, dim);
          FREQ = 0;
          state_GPS = 0;
          break;
        default:  // off
          COLOR = CHSV(0, 0, 0);
          FREQ = 0;
      }
    if (currentmillis >= targetmillis_GPS) {
      if (state_GPS == 0) {
        for (int i = 0; i < NUM_ARMS; i++) {
          for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
            if (pgm_read_byte(&(LED_DEF[i][j])) == GPS) {
              leds[NUM_LEDS_PER_STRIP * i + j] = COLOR;
              }
            }
          }
        state_GPS = 1;
        }
      else {
        for (int i = 0; i < NUM_ARMS; i++) {
          for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
            if (pgm_read_byte(&(LED_DEF[i][j])) == GPS) {
              leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(0, 0, 0);
              }
            }
          }
        state_GPS = 0;
        }
      targetmillis_GPS = lastmillis + FREQ;
      }
    }
  else {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (pgm_read_byte(&(LED_DEF[i][j])) == GPS) {
          leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(0, 0, 0);
          }
        }
      }
    }
  }

  // #####################################################################################################
  // ### DEFAULT FRONT ARM LED MODE (CONST-Green) ###
  // #####################################################################################################
void front_arms(int STATUS, float dim) {
  if (STATUS == 1) {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (pgm_read_byte(&(LED_DEF[i][j])) == FRONT) {
          leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(96, 255, dim);
          }
        }
      }
    }
  else {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (pgm_read_byte(&(LED_DEF[i][j])) == FRONT) {
          leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(0, 0, 0);
          }
        }
      }
    }
  }

  // #####################################################################################################
  // ### DEFAULT SIDE ARM LED MODE (CONST-YELLOW) ###
  // #####################################################################################################
  /*
  void side_arms(int STATUS, float dim) {
      if (STATUS == 1) {
          for (int i = 0; i < NUM_ARMS; i++) {
              for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
                  if (pgm_read_byte(&(LED_DEF[i][j])) == SIDE) {
                      leds[i][j] = CHSV(60, 255, 255 * dim);
                  }
              }
          }
      } else {
          for (int i = 0; i < NUM_ARMS; i++) {
              for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
                  if (pgm_read_byte(&(LED_DEF[i][j])) == SIDE) {
                      leds[i][j] = CHSV(0, 0, 0);
                  }
              }
          }
      }
  }*/

  // #####################################################################################################
  // ### DEFAULT REAR ARM LED MODE (CONST-RED) ###
  // #####################################################################################################
void rear_arms(int STATUS, float dim) {
  if (STATUS == 1) {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (pgm_read_byte(&(LED_DEF[i][j])) == BACK) {
          leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(0, 255, dim);
          }
        }
      }
    }
  else {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (pgm_read_byte(&(LED_DEF[i][j])) == BACK) {
          leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(0, 0, 0);
          }
        }
      }
    }
  }

  // **** con esta funcion hace un pulso senoidal  *****

void flash_pos_light(int STATUS, float dim) {
  int BPM = 60;
  // uint8_t bright = beatsin8( BPM /*BPM*/, 10 /*dimmest*/, 255 /*brightest*/
  // );
  if (STATUS == ON) {
    uint8_t bright = cubicwave8(beat8(BPM));  // GENERA * 60*1000/bpm MILISECS
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (pgm_read_byte(&(LED_DEF[i][j])) == FLASH) {
          // leds[NUM_LEDS_PER_STRIP*i+j] = CHSV(255, 0, bright * dim);
          leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(255, 0, bright);
          }
        }
      }
    }
  else {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (pgm_read_byte(&(LED_DEF[i][j])) == FLASH) {
          leds[NUM_LEDS_PER_STRIP * i + j] = CHSV(0, 0, 0);
          }
        }
      }
    }
  }

  // ################################################################################
  // ###                                   efectos ###
  // ################################################################################
void nextPattern() {
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE(gPatterns);
  Serial.println(gCurrentPatternNumber); // para debug
  }

  // An animation to play while the crowd goes wild after the big performance
void applause() {
  //Serial.print(" applause : ");
  static uint16_t lastPixel = 0;
  fadeToBlackBy(leds, NUM_LEDS, 32);
  leds[lastPixel] = CHSV(random8(HUE_BLUE, HUE_PURPLE), 255, 255);
  lastPixel = random16(NUM_LEDS);
  leds[lastPixel] = CRGB::White;
  vled_draw();  // draw from virtual to fisical leds
  }

void sinelon() {
  //Serial.print(" sinelon : ");
  // a colored dot sweeping back and forth, with fading trails
  // fadeToBlackBy(Vir_led, NUM_LEDS, 60);
  fadeToBlackBy(leds, NUM_LEDS, 60);
  int pos = beatsin16(13, 0, NUM_LEDS);
  leds[pos] += CHSV(gHue, 255, 192);
  vled_draw();  // draw from virtual to fisical leds
  }

void bpm() {
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = gbpm;  // 62
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
  for (int i = 0; i < NUM_LEDS; i++) {  // 9948

    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
    }
  vled_draw();  // draw from virtual to fisical leds
  //Serial.print(" bpm : ");
  // Serial.print(gbpm);
  }

  // Updated sinelon (no visual gaps)
void sinelon2() {
  Serial.print(".");
  // a colored dot sweeping
  // back and forth, with
  // fading trails
  fadeToBlackBy(leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS);
  static int prevpos = 0;
  if (pos < prevpos) {
    fill_solid(leds + pos, (prevpos - pos) + 1, CHSV(gHue, 220, 255));
    }
  else {
    fill_solid(leds + prevpos, (pos - prevpos) + 1, CHSV(gHue, 220, 255));
    }
  prevpos = pos;
  vled_draw();  // draw from virtual to fisical leds
  }

  // hace el render de la tira virtual a las tiras fisicas en cada brazo
void vled_draw() {
  // Vled_invert(2);
  // memcpy(&leds[1],&Vir_led[NUM_LEDS_PER_ARM], NUM_LEDS_PER_ARM*3);
  // Vled_invert(0);
  FastLED.show();
  }

  // para los brazos que estan conectados en sentido inverso
  /*
  void Vled_invert(int brazo) {
    for (int i = 0; i < NUM_LEDS_PER_ARM; i++)
    {
      leds[brazo][i] = Vir_led[NUM_LEDS_PER_ARM*(brazo+1) - i];

    }
  }*/

  /*
  void setArms(uint8_t h, uint8_t s, uint8_t v, int pos  ) { //        setea todo
  el brazo con las tiras en el mismo patron leds[0][LED_Layout[0][pos]] = CHSV (h,
  s, v ); leds[0][LED_Layout[1][pos]] = CHSV (h, s, v );
    leds[0][LED_Layout[2][pos]] = CHSV (h, s, v);
    memcpy(&leds[1], &leds[0][0], NUM_LEDS_PER_STRIP * 3);
    memcpy(&leds[2], &leds[0][0], NUM_LEDS_PER_STRIP * 3);
  }*/

  // ****  setea un segmento desde el led de inicio al led del final con una barra
  // de color fijo
  /*
  void setSegment (int linit,int  lend, int br ,int pos, CRGB color){
    for (int i =linit; i<lend ;i++){
      leds[br][LED_Layout[pos][i]]= color;
    }
  }*/

  /*******Blink*********************************************************/
  /*  Make a led connected to output pin blink with rectangular form ***/
  // Es el led que esta al frente del arduino nano en el Y6
  // pensado para usa con la funcion everynnmilliseconds pero solo genera una onda
  // cuadrada utilizar mejor las funciones en A3_led_gnk.ino
  /*
  void Blink(boolean LedActivo,byte LedPin , uint8_t Ton){  // Ton puede ser de 1
  a 9 if (LedActivo){ digitalWrite (LedPin,!digitalRead(LedPin));  // togles led
  pin to indicate is alive
    }
    } */

  // test de leds
void Leds_Test(void) {
  // Serial.println("Leds_Test");
  for (int i = 0; i < NUM_LEDS_PER_ARM; i++) {
    leds[i] = CHSV(0, 255, 80 * i);
    FastLED.show();
    FastLED.delay(20);
    }
  for (int i = 0; i < NUM_LEDS_PER_ARM; i++) {
    leds[i] = CHSV(85, 255, 80 * i);
    FastLED.show();
    FastLED.delay(20);
    }
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(254, 0, 0);
    FastLED.show();
    FastLED.delay(200);
    }
  FastLED.clear();
  FastLED.show();
  Serial.println("FIN Leds_Test");
  }
  // fin test de leds

  // motor leds control . GPS and Motors status analog leds
void update_copter_leds(void) {  // static void update_copter_leds(void) {
  // DPN(motorsArmed()); //debug+

  if (motorsArmed()) {  // if (motorsArmed()) {
    if (ap_voltage_battery < minimum_battery_volts) {
      copter_leds_oscillate();  // if motors are armed, but battery level is
                                // low, motor leds fast blink
      }
    else {
      copter_leds_on();  // if motors are armed, battery level OK, all motor
                         // leds ON
      }
    }

  else {
    copter_leds_slow_blink();  // if motors are not armed, blink motor leds
    }
    // DPN(ap_fixtype); //debug+

  if (ap_fixtype == NO_GPS) {
    digitalWrite(GpsLed, OFF);
    }
  else {
    if (ap_fixtype == NO_FIX) {
      GenericFlash(GpsLed, 32, 0, 120, 255);
      }
    else {
      if (ap_fixtype >= GPS_OK_FIX_3D) {
        digitalWrite(GpsLed, ON);
        }
      }
    }
  }

  // Detects if motors are armed
bool motorsArmed(void) {
  return (ap_system_status & MAV_STATE_ACTIVE);  // debug ojo
  }

void copter_leds_oscillate(void) {
  // void GenericFlash(uint8_t pin, uint8_t pw, uint8_t offset, int BPM, uint8_t
  // dim)
  GenericFlash(motorsLedLeft, 32, 0, 32, 255);
  GenericFlash(motorsLedRight, 32, 0, 32, 255);
  }
void copter_leds_fast_blink(void) {
  GenericFlash(motorsLedLeft, 32, 0, 120, 255);
  GenericFlash(motorsLedRight, 32, 0, 120, 255);
  }

void copter_leds_on(void) {
  digitalWrite(motorsLedLeft, ON);
  digitalWrite(motorsLedRight, ON);

  }  // if motors are armed, battery level OK, all motor leds ON
void copter_leds_slow_blink(void) {
  // GenericFlash(motorsLedLeft, 32, 0, 16, 255);
  // GenericFlash(motorsLedRight, 32, 0, 16, 255);
  analogWrite(motorsLedLeft, beat8(16, 0));
  analogWrite(motorsLedRight, beat8(16, 0));
  // beat8 (accum88 beats_per_minute, uint32_t timebase=0)
  }  // if motors are not armed, blink motor leds
  //#endif