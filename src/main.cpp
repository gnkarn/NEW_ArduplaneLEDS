

// ====================================================================================================
// incluir todas las variables globales
// ====================================================================================================
#define USE_TEENSY_LED_SUPPORT
//#ifdef USE_TEENSY_LED_SUPPORT
// #####################################################################################################
// ### MAIN CONFIG PART ###
// #####################################################################################################
// ### INCLUDES ###
// #####################################################################################################
#include <Arduino.h>
#include <stdint.h>
#include <FastLED.h>
#include <avr/pgmspace.h>      // para poder usar progmem
//#include <SoftwareSerial.h> // serial comm para Bluetooth
#define USE_TEENSY_LED_SUPPORT  // Enable LED-Controller functionality
#include <GCS_MAVLink.h>

// serial comm para Bluetooth
//#include <SoftwareSerial.h>
#define debugSerial Serial //btSerial
//#define debugSerialBaud 9600
// SoftwareSerial btSerial(8, 9);  // RX, TX D8 y D9
#define DPN Serial.println //imprime en el puerto del usb, es tambien el de telemetria
// #define DPL btSerial.println // imprime en el puerto para monitorear 

unsigned long targetmillis_LEFT;
unsigned long targetmillis_RIGHT;
unsigned long targetmillis_ARMED;

int state_LEFT = 0;
int state_RIGHT = 0;
int state_ARMED = 0;

float dim = 100;  // variable global para dim
int gbpm = 60;    // variable global para efectos de led que quiero variar desde
                  // el SD del remoto

// *****************************************************************************************

// #####################################################################################################
// ### DEFAULT VARIABLES ###
// #####################################################################################################
// antes en led_control.ino
#define RIGHT 0
#define LEFT 1
#define OFF 0
#define ON 1
#define AUTO 2
#define BLINK 3
#define PWM_MIN 982   // minimum pwm value of your RC-System
#define PWM_MID 1496  // middle  pwm value of your RC-System
#define PWM_MAX 2006  // maximum pwm value of your RC-System

#ifdef SIMULATION_MODE
int LED_MODE_SWITCH;
int LED_DIMM_CHAN;
#else
#define LED_MODE_SWITCH \
  15  // Channel Number of RC Channel used for manual Lightmode
#define LED_DIMM_CHAN 16  // Channel Number of RC Channel used for dimming LEDs
#define LED_BPM_CHAN 14   // Channel Number of RC Channel used for LED BPM
#endif

/*
 * *******************************************************
 * *** Variables needed for Mavlink Connection Status  ***
 * *** and starting FrSkySPort Telemetry               ***
 * *******************************************************
 */
bool MavLink_Connected = 0;  // Connected or Not
// unsigned long start_telemetry       = 30000;  // Start Telemetry after 30s
// (or 5s after init)
bool telemetry_initialized = 0;  // Is FrSkySPort Telemetry initialized

// #####################################################################################################
// ### FASTLED CONFIG ###
// #####################################################################################################
// antes en led_control.ino

#define RGBORDER GRB  // BRG para 2811, GRB para 2812
#define LEDTYPE WS2812B

#define NUM_ARMS 1 //4
#define NUM_LEDS_PER_STRIP 5  // 10
#define BRIGHTNESS 96
#define FRAMES_PER_SECOND 30
#define NUM_LEDS_PER_ARM 5// 10 , cambiar la definicion de  LED_DEF
#define NUM_LEDS NUM_LEDS_PER_ARM *NUM_ARMS  // Number of LED's
// CRGB Vir_led[NUM_LEDS]     ;                      // simula una tira continua
// con todos los leds de cada brazo conectados en serie
#define BRIGHTNESS 96
#define DATA_PIN  5 //  pin D5

// CRGB leds[NUM_ARMS][NUM_LEDS_PER_STRIP]; //
CRGB leds[NUM_LEDS];  //

// CRGBArray<NUM_LEDS_PER_STRIP> leds[NUM_ARMS] ; //

// #define SIMULATION_MODE // uncomment to enable simulation mode ( habilitar
// para probar los diferentes modos) #define standalone      // DO NOT
// UNCOMMENT, just for testing purpose as standalone script without mavlink
// part.

// ###################################################################################################
// ### EFECTOS ###
// ###################################################################################################
void applause();
void sinelon();
void bpm();
void sinelon2();
void Leds_Test(void);
void nextPattern();
void get_mode();
byte rc_dimmer_proz();
void default_mode(int, float);
void front_arms(int, float);
void rear_arms(int, float);
void flash_pos_light(int, float);
void get_armed_status(int, float);
void get_gps_status(int, float);
void update_copter_leds(void);
void vled_draw();
void Teensy_LED_Init();
void LedInit(uint8_t);
void Teensy_LED_process();
void FrontFlash(int, float);

extern void Mavlink_setup();  // Forward declaration
extern void _MavLink_receive();
extern void Mavlink_check_connection();

// List of patterns to cycle through.  Each is defined as a separate function
// below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { bpm, applause, sinelon2 }; // saque bpm de la lista applause, sinelon, sinelon2

uint8_t gCurrentPatternNumber = 0;  // Index number of which pattern is current
uint8_t gHue = 0;    // rotating "base color" used by many of the patterns
uint8_t g_bpm = 20;  // global bpm usado por ejemplo en circling
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void GenericFlash(uint8_t pin, uint8_t pw, uint8_t offset, int BPM, uint8_t dim);

// void addGlitter(fract8 chanceOfGlitter); // incorporada esta declaracion pues
// al tener un tipo dentro de la funcion el compilador de arduino se marea , en
// realidada todas las funciones deberian estar declaradas en un .h aparte

// void setArms(uint8_t h, uint8_t s, uint8_t v, int pos ) ;
// void setSegment (int linit,int  lend, int br ,int pos, CRGB color);
// void HeliLed (int bpm , int hue,float dim  );



// organiza los leds para que luego la secuencia sea siempre ascendente
// independientemente de la forma de cableado de la tira para reducir uso de
// memoria en el nano , no usamos ledLayout, se sigue el orden del acableado
// fisico int  LED_Layout[3][3] = {{0,1,2},
//                              { 3,4,5 }     ,
//                              { 6,7,8}};

int LED_MODE = 999;
int FL_RUN = 0;

uint8_t ap_base_mode_last = 0;
int32_t ground_level = 0;


unsigned long currentmillis;       // current milliseconds
unsigned long lastmillis;          // milliseconds of last loop
unsigned long targetmillis_FLASH;  // target milliseconds
unsigned long targetmillis_GPS;
// unsigned long targetmillis_LS;

int state_GPS;
int state_FLASH;

int pos;
int dir = RIGHT;

/*
 * LED_COLORS
 * CRGB::BLACK   <=> CHSV(0,0,0)
 * CRGB::WHITE   <=> CHSV(0,0,255)
 * CRGB::RED   <=> CHSV(0,255,255)
 * CRGB::GREEN   <=> CHSV(120,255,255)
 * CRGB::YELLOW  <=> CHSV(60,255,255)
 * CRGB::ORANGE  <=> CHSV(39,255,255)
 */
/*
 * Currently used variables from MavLink_FrSkySPort.ino: (
 * Mavlink_arduplane_ledNano.cpp ) ap_base_mode  => ARMED=1, DISARMED=0
 * ap_bar_altitude => ALT from barometer, 100 = 1m
 * ap_sat_visible  => numbers of visible satelites
 * ap_fixtype    => 0 = No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
 * ap_custom_mode  => see
 * http://copter.ardupilot.com/wiki/configuration/arducopter-parameters/#flight_mode_1_arducopterfltmode1
 * ap_throttle   => 0..100
 * ap_chancount  => Number of RC_Channels used
 * ap_chan_raw[i]  => RC channel 1-18 value, in microseconds. A value of
 * UINT16_MAX (65535U) ap_rssi     => Receive signal strength indicator, 0: 0%,
 * 100: 100%, 255: invalid/unknown.
 */

/////////////////////////////////////////////////////////////////////////////////
// arduplane LEDNano.h
//////////////////////////////////////////////////////////////////////////////////
/*
 * MavLink_FrSkySPort
 * https://github.com/Clooney82/MavLink_FrSkySPort
 *
 * Copyright (C) 2014 Rolf Blomgren
 *  http://diydrones.com/forum/topics/amp-to-frsky-x8r-sport-converter
 *  Inspired by https://code.google.com/p/telemetry-convert/
 *    (C) 2013 Jochen Tuchbreiter under (GPL3)
 *
 *  Improved by:
 *    (2014) Christian Swahn
 *    https://github.com/chsw/MavLink_FrSkySPort
 *
 *    (2014) Luis Vale
 *    https://github.com/lvale/MavLink_FrSkySPort
 *
 *    (2015) Michael Wolkstein
 *    https://github.com/wolkstein/MavLink_FrSkySPort
 *
 *    (2015) Jochen Kielkopf
 *    https://github.com/Clooney82/MavLink_FrSkySPort

 (2015) modified by gnk adding front led flash , and many leds effects
 https://github.com/gnkarn/MavLink_FrSkySPort.git
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
 */
/*
 * ====================================================================================================
 *
 * Mavlink to FrSky X8R SPort Interface using Teensy 3.1
 *     http://www.pjrc.com/teensy/index.html
 *  based on ideas found here http://code.google.com/p/telemetry-convert/
 * ========================================================================
 *
 * Cut board on the backside to separate Vin from VUSB
 *
 * Connection on Teensy 3.1:
 * -----------------------------------
 *  SPort S --> TX1
 *  SPort + --> Vin
 *  SPort - --> GND
 *  APM Telemetry DF13-5 Pin 2 --> RX2
 *  APM Telemetry DF13-5 Pin 3 --> TX2
 *  APM Telemetry DF13-5 Pin 5 --> GND
 *
 * Note that when used with other telemetry device (3DR Radio 433 or 3DR
 Bluetooth tested) in parallel
 * on the same port the Teensy should only Receive, so please remove it's TX
 output (RX input on PixHawk or APM)
 *
 * Analog input  --> A0 (pin14) on Teensy 3.1 ( max 3.3 V ) - Not used
 *
 * This is the data we send to FrSky, you can change this to have your own set
 of data
 * ----------------------------------------------------------------------------------------------------
 * Data transmitted to FrSky Taranis:
 * Cell            ( Voltage of Cell=Cells/(Number of cells). [V])
 * Cells           ( Voltage from LiPo [V] )
 * A2              ( HDOP value * 25 - 8 bit resolution)
 * A3              ( Roll angle from -Pi to +Pi radians, converted to a value
 between 0 and 1024)
 * A4              ( Pitch angle from -Pi/2 to +Pi/2 radians, converted to a
 value between 0 and 1024)
 * Alt             ( Altitude from baro  [m] )
 * GAlt            ( Altitude from GPS   [m] )
 * hdg             ( Compass heading  [deg] )
 * Rpm             ( Throttle when ARMED [%] *100 + % battery remaining as
 reported by Mavlink)
 * VSpd            ( Vertical speed [m/s] )
 * Speed           ( Ground speed from GPS,  [km/h] )
 * T1              ( GPS status = ap_sat_visible*10) + ap_fixtype )
 * T2              ( Armed Status and Mavlink Messages :- 16 bit value: bit 1:
 armed - bit 2-5: severity +1 (0 means no message - bit 6-15: number
 representing a specific text)
 * Vfas            ( same as Cells )
 * Longitud        ( Longitud )
 * Latitud         ( Latitud )
 * Dist            ( Will be calculated by FrSky Taranis as the distance from
 first received lat/long = Home Position )
 * Fuel            ( Current Flight Mode reported by Mavlink )
 * AccX            ( X Axis average vibration m/s?)
 * AccY            ( Y Axis average vibration m/s?)
 * AccZ            ( Z Axis average vibration m/s?)
 * ====================================================================================================


 * ====================================================================================================
 *
 * Mavlink Adapted for arduino NANO
 * * Connection on nano GNK
 * -----------------------------------
 *  pin 30 APM telemetry  --> RX0
 *  pin 13 --> MAvlink led interno
 *  pin2  ---> gbled , green arduino front
 *  Pin 9  --> Frontled
 *  Pin 10 --> BAckled
 *  PIN 5  --> 2812 led  Data
 *  PIN 8 -->  Al Rx HB-02 (vde) for debug using arduino monitor on mac@
 *
 * ========================================================================
*/
/*
*===========================================================================
*
* Mavlink Adapted for TEENSY
** Connection GNK
* ---------------------------------- -
* pin 9 y 10 APM telemetry-- > RX2/TX2
* pin 13 -- > MAvlink led interno
* pin2--->gbled, green arduino front
* Pin 23  -- > Frontled
* Pin 11 -- > BAckled
* PIN 6  -- > 2812 led  Data
* PIN 8 -- > Al Rx HB - 02 (vde) for debug using arduino monitor on mac@
*
* ========================================================================
*/

/// GPS status codes
enum GPS_Status {
  NO_GPS = 0,              ///< No GPS connected/detected
  NO_FIX = 1,              ///< Receiving valid GPS messages but no lock
  GPS_OK_FIX_2D = 2,       ///< Receiving valid messages and 2D lock
  GPS_OK_FIX_3D = 3,       ///< Receiving valid messages and 3D lock
  GPS_OK_FIX_3D_DGPS = 4,  ///< Receiving valid messages and 3D lock with
                           ///< differential improvements
  GPS_OK_FIX_3D_RTK = 5,   ///< Receiving valid messages and 3D lock, with
                           ///< relative-positioning improvements
  };



  /*
   * *******************************************************
   * *** Enable Addons:                                  ***
   * *******************************************************
   */
  // #define USE_FAS_SENSOR_INSTEAD_OF_APM_DATA              // Enable  if you use
  // a FrSky FAS   Sensor. #define USE_FLVSS_FAKE_SENSOR_DATA // Enable  if you
  // want send fake cell info calculated from VFAS, please set MAXCELLs according
  // your Number of LiPo Cells #define USE_SINGLE_CELL_MONITOR // Disable if you
  // use a FrSky FLVSS Sensor. - Setup in LSCM Tab #define
  // USE_AP_VOLTAGE_BATTERY_FROM_SINGLE_CELL_MONITOR // Use this only with enabled
  // USE_SINGLE_CELL_MONITOR
#define USE_RC_CHANNELS  // Use of RC_CHANNELS Informations ( RAW Input Valus of
                         // FC ) - enable if you use TEENSY_LED_SUPPORT.
#define USE_TEENSY_LED_SUPPORT  // Enable LED-Controller functionality

/*
 * *******************************************************
 * *** Debug Options:                                  ***
 * *******************************************************
 */
// *** DEBUG MAVLink Messages:
// #define DEBUG_APM_MAVLINK_MSGS              // *Show all messages received
// from APM #define DEBUG_APM_CONNECTION #define DEBUG_APM_CONNECTION1 // *lo
// uso en lugar del anterior para reducir uso de memoria #define
// DEBUG_APM_HEARTBEAT                 // * MSG #0 #define DEBUG_APM_SYS_STATUS
// // *MSG #1   - not used -> use: DEBUG_APM_BAT #define DEBUG_APM_BAT // Debug
// Voltage and Current received from APM 
#define DEBUG_APM_GPS_RAW // *MSG #24
// #define DEBUG_APM_RAW_IMU                   // MSG #27  - not used -> use:
// DEBUG_APM_ACC #define DEBUG_APM_ACC                       // Debug
// Accelerometer #define DEBUG_APM_ATTITUDE                  // *MSG #30 #define
// DEBUG_APM_GLOBAL_POSITION_INT_COV   // MSG #63  - planned - currently not
// implemented - not supported by APM #define DEBUG_APM_RC_CHANNELS // MSG #65
// #define DEBUG_APM_VFR_HUD                   // *MSG #74
// #define DEBUG_APM_STATUSTEXT                // MSG #254 -
// #define DEBUG_APM_PARSE_STATUSTEXT
// #define DEBUG_GIMBAL_HEARTBEAT
// #define DEBUG_OTHER_HEARTBEAT
// #define DEBUG_APM_CONNECTION1
// *** DEBUG FrSkySPort Telemetry:
// #define DEBUG_FrSkySportTelemetry
// #define DEBUG_FrSkySportTelemetry_FAS
// #define DEBUG_FrSkySportTelemetry_FLVSS  *
// #define DEBUG_FrSkySportTelemetry_GPS
// #define DEBUG_FrSkySportTelemetry_RPM
// #define DEBUG_FrSkySportTelemetry_A3A4
// #define DEBUG_FrSkySportTelemetry_VARIO
// #define DEBUG_FrSkySportTelemetry_ACC
// #define DEBUG_FrSkySportTelemetry_FLIGHTMODE

// *** DEBUG other things:
// #define DEBUG_AVERAGE_VOLTAGE
// #define DEBUG_LIPO_SINGLE_CELL_MONITOR // Use this only with enabled
// USE_SINGLE_CELL_MONITOR

#define hbLed  LED_BUILTIN /* Heartbeat LED if any, default arduino board has a LED onboard tied to \
       pin 13. uso el D2 int11 pues el led interno se usa en SPI */
#define frontled A9  // TEENSY digital pin A9 para pwm //NANO digital pin 9 para pwm (puede usarse pin 3,5,6,9,10,11)
#define backled A8  // digital pin 10
#define GpsLed 6    // digital pin 6 GPS status
#define motorsLedLeft 4
#define motorsLedRight 3

// #####################################################################################################
// ### DEFAULT VARIABLES ###
// #####################################################################################################
// antes en led_control.ino
/*
#define RIGHT 0
#define LEFT  1
#define OFF   0
#define ON    1
#define AUTO  2
#define BLINK 3
#define PWM_MIN    982  // minimum pwm value of your RC-System
#define PWM_MID   1496  // middle  pwm value of your RC-System
#define PWM_MAX   2006  // maximum pwm value of your RC-System

#ifdef SIMULATION_MODE
int LED_MODE_SWITCH;
int LED_DIMM_CHAN;
#else
#define LED_MODE_SWITCH 15   // Channel Number of RC Channel used for manual
Lightmode #define LED_DIMM_CHAN   16   // Channel Number of RC Channel used for
dimming LEDs #define LED_BPM_CHAN   14   // Channel Number of RC Channel used
for dimming LEDs #endif
*/

// #####################################################################################################
// ### FASTLED CONFIG ###
// #####################################################################################################
// antes en led_control.ino
/*
#define RGBORDER GRB // BRG para 2811, GRB para 2812
#define LEDTYPE WS2812B

#define NUM_ARMS 4
#define NUM_LEDS_PER_STRIP 10 //
#define BRIGHTNESS 96
#define FRAMES_PER_SECOND 30
#define NUM_LEDS_PER_ARM 10
#define NUM_LEDS NUM_LEDS_PER_ARM *NUM_ARMS // Number of LED's
    // CRGB Vir_led[NUM_LEDS]     ;                      // simula una tira
continua con todos los leds de cada brazo conectados en serie #define BRIGHTNESS
96 #define DATA_PIN 5 // pin D5

    // CRGB leds[NUM_ARMS][NUM_LEDS_PER_STRIP]; //
    CRGB leds[NUM_LEDS]; //

    // CRGBArray<NUM_LEDS_PER_STRIP> leds[NUM_ARMS] ; //

    unsigned long targetmillis_LEFT;
    unsigned long targetmillis_RIGHT;
    unsigned long targetmillis_ARMED;

    int state_LEFT = 0;
    int state_RIGHT = 0;
    int state_ARMED = 0;

    float dim = 100; // variable global para dim
    int gbpm = 60;   // variable global para efectos de led que quiero variar
    desde el SD del remoto
    */

// configure number maximum connected analog inputs(cells)
// if you build an six cell network then MAXCELLS is 6
// #define MAXCELLS 5

// ====================================================================================================
// put function declarations here:

// #####################################################################################################
// ### FUNCIONES MAVLINK FINAL
// #####################################################################################################
/*
 * *******************************************************
 * *** Setup:                                          ***
 * *******************************************************
 */
const int ledPin = LED_BUILTIN;  // Pin del LED incorporado en la placa
void setup() {
  // define pin modes for tx, rx:  for software serial, not sure if needed
  // pinMode(11, INPUT);
  // pinMode(7, OUTPUT);
  pinMode(ledPin, OUTPUT);
  // bluetooth serial
  // debugSerial.begin(9600);// debugSerialBaud

  Serial.begin(115200);// Inicializar la comunicación serie del USB 
  Serial2.begin(57600);// Inicializar la comunicación serie para mavlink

/*   while (!Serial) {
    // Esperar a que la comunicación serie se establezca
    // piscar led
    digitalWrite(ledPin, HIGH);  // Encender el LED
    delay(500);                  // Esperar 0.5 segundos
    digitalWrite(ledPin, LOW);   // Apagar el LED
    delay(500);                  // Esperar 0.5 segundos

    } */


// Enviar el mensaje "listo" al completar el setup
  Serial.println("Setup de arduplane leds iniciado ");
 // Parpadeo del LED durante dos segundos
  for (int i = 0; i < 10; i++) {
    digitalWrite(ledPin, HIGH);  // Encender el LED
    delay(200);                  // Esperar 0.5 segundos
    digitalWrite(ledPin, LOW);   // Apagar el LED
    delay(500);                  // Esperar 0.5 segundos
    }

  Serial.print(("Mav2led SETUP"));
  Serial.println("DEBUG");
    // Realizar cualquier otra configuración necesaria
    // ...

  pinMode(hbLed, OUTPUT);  // pin13 en teensy// nano salida de hbled uso el A3 Pcint 11 pues el 13 es SCK, heart bit
#ifdef USE_TEENSY_LED_SUPPORT
  Teensy_LED_Init();  // Init LED Controller
#endif

  Mavlink_setup();  // Init Mavlink
  // ..DPN(F("MAv2led monitor Nano"));

  LedInit(frontled);  // front led gnk init son salidas analogicas para manejar con ULN2003
  LedInit(backled);
  LedInit(motorsLedLeft);   // motors left Led
  LedInit(motorsLedRight);  // motors right leds
  LedInit(GpsLed);          // 

  Serial.print("Fin de Setup");
  // .. DPN(F("Fin de Setup"));
  }

  /*
   * *******************************************************
   * *** Main Loop:                                      ***
   * *******************************************************
   */
void loop() {
  // Serial.print(".");
  _MavLink_receive();  // receive MavLink communication

  Mavlink_check_connection();  // Check MavLink communication

  if (MavLink_Connected == 1) {  // If we have a valid MavLink Connection
                                 // if ((mavlink_active)) digitalWrite(hbLed,
                                 // ioCounter == 1 ? HIGH : LOW);
    leds[0] = (((millis() % 250) >= 200)
      ? CRGB::White
      : 0);  // El led cero  titila si mavlink_activo era led 1
    vled_draw();
    Teensy_LED_process();  // Process LED-Controller
    }
  else {
    default_mode(0,
      150);  // if mavlink not connected put leds in a default state
    LEDS.show();
    }

  FrontFlash(
    64, 255);  //  hace titilar el led flash de frente GNK cambiar 255 por dim
               // EVERY_N_MILLISECONDS(500)
               // {digitalWrite(hbLed,!digitalRead(hbLed));  } //Blink(boolean
               // LedActivo,byte LedPin , uint8_t Ton)
              // EVERY_N_MILLISECONDS(100) { DPL("."); }  // debug

  digitalWrite(hbLed,
    ((millis() % 1000) >=
      50));  // una formA DE  generar onda rectangula de cualquier %
             // ciclo - en este caso aplicado al led sobre el arduino
  // GenericFlash(uint8_t pin, uint8_t pw, uint8_t offset, int BPM, float dim)
  GenericFlash(backled, 32, 160, 60, 256);  // flash del led  trasero
  // GenericFlash(13, 50, 160, 60, 200); // flash de mavlink para pruebas ,
  // normalmente debe parpadear cuando se establece la coneccion
  }
  // ====================================================================================================
  // put function definitions here:

  /// LEDS CONTROL INO FILE HERE 



#include "A3_Leds.h"
#include "Mavlink_ino.h"
#include "Leds_Control.h"