#include <DacTone.h>
#include <pitches.h>

#include "esp_wifi.h"
// Encoder
#include <Encoder.h>
#define ENCODER_A 19
#define ENCODER_B 23
#define ENCODER_BUTTON 27

// Display
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C


// Receiver IC
#include <SI4735.h>
#include <patch_full.h>    // SSB patch for whole SSBRX full download
#define RESET_PIN 13
#define SI_ADDR 0x11
#define LSB 1
#define USB 2
#define AM_FUNCTION 1
#define RX_ANT_EN 2

//Scheduler
#define MILLIS_PER_DISPLAY_UPDATE 1000
#define MILLIS_PER_RX_UPDATE 500
#define DEBUG_PIN 4

//Encoder
#define POSITIONS_PER_ENCODER_STEP 4

//General constants
#define INCREMENT_STATES 4 //number of items we can increment with the encoder

// TX VFO
#include <si5351_lite.h>
#include <driver/dac.h>
#include <driver/adc.h>
#include <analogWrite.h>
#include "soc/rtc_wdt.h"
#define I2C_SDA 21
#define I2C_SCL 22
#define AD_ADDR 0x60
#define SIDETONE_FREQUENCY 800

// TX other
#define PTT_KEY 35
#define SIDETONE_PIN 26
#define DAC_CH1 25
#define DAC_CH2 26

#define PWM_CHANNEL 0   // ESP32 has 16 channels which can generate 16 independent waveforms
#define PWM_FREQ 200000     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
#define PWM_RESOLUTION 8 // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits 

#define TEST_WAVEFORM_1KHZ false
#define MIC_INPUT ADC2_CHANNEL_7

#define QSK_DELAY_MS 500

// Debouncers
#define PTT_KEY_DEBOUNCE_DELAY 10

// ADCs
#define BATTERY_SENSE_PIN 14

// CW keyer
#define dit_length 1
#define dah_length 3
#define char_space 1
#define letter_space 3
#define word_space 7

#define CW_BUFFER_LENGTH 1000

// Encoder
Encoder myEnc(ENCODER_A, ENCODER_B);

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Receiver IC
const uint16_t size_content = sizeof ssb_patch_content; // see ssb_patch_content in patch_full.h or patch_init.h
SI4735 si4735;

static int frequency = 7100;
uint8_t currentStep = 1;
int16_t currentBFOStep = 25;
int16_t currentBFO = 0;
uint8_t rssi = 0;
uint8_t bandwidthIdx = 2;
const char *bandwidth[] = {"1.2", "2.2", "3.0", "4.0", "0.5", "1.0"};
typedef struct
{
  uint16_t minimumFreq;
  uint16_t maximumFreq;
  uint16_t currentFreq;
  uint16_t currentStep;
  uint8_t currentSSB;
} Band;

Band band[] = {
    {520, 2000, 810, 1, LSB},
    {3500, 4000, 3700, 1, LSB},
    {7000, 7500, 7100, 1, LSB},
    {11700, 12000, 11940, 1, USB},
    {14000, 14300, 14200, 1, USB},
    {18000, 18300, 18100, 1, USB},
    {21000, 21400, 21200, 1, USB},
    {24890, 25000, 24940, 1, USB},
    {27000, 27700, 27300, 1, USB},
    {28000, 28500, 28400, 1, USB}
};

const int lastBand = (sizeof band / sizeof(Band)) - 1;
static int currentFreqIdx = 2;
static uint8_t currentAGCAtt = 0;
static int rx_rssi;

//Scheduler
static int last_RX_update;
static bool update_display_now;
static int last_display_update;
static int morse_element_timer;

//I/O debounce timers and states
static bool frequencyKnobPress = false;
static bool frequencyKnobDoublePress = false;
static int frequencyKnobPressTimer = 0;

static int ptt_key_press_timer = 0;
static bool ptt_key_pressed = false;
static bool key_down = false;
static bool qsk_ing = false;
static int qsk_timer = 0;

// TX VFO
static Si5351 si5351;
static uint32_t tx_frequency = frequency * 100000;
DacTone audio(2);

// CW keyer
static String cw_buffer_string = "abcde f";
static int wpm = 10;
static uint8_t cw_buffer[CW_BUFFER_LENGTH] = {1,1,1,0,1,0,1,1,1,0,1,0,0,0,1,1,1,0,1,1,1,0,1,0,1,1,1};
static int cw_buffer_pointer = 0;
static bool keyer_enable = false;
static int morse_element_length_millis;
static int morse_message_length;

float dBuv_to_S_point() {
  float s;
  if (rx_rssi <= 34) { // S9 or below
    s = (rx_rssi+20) / 6;
  } else {
    s = ((rx_rssi-34) / 100) + 9; // s reading is in format 9.xx where xx is dB over s9
  }
  return s;
}

void update_display() {
  double batt_level = analogRead(BATTERY_SENSE_PIN);
  batt_level = (0.009*batt_level + 2.34);
  float s = dBuv_to_S_point();
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("s");
  display.print(s);
  display.setCursor(90,0);
  display.print(batt_level, 2);
  display.print("V");
  display.setCursor(0,10);
  display.setTextSize(3);
  display.print(frequency);
  display.setTextSize(2);
  display.print("KHz");
  display.setCursor(0,40);
  display.setTextSize(2);
  display.print("BFO:");
  display.print(currentBFO);
  display.print("Hz");
  display.display();
}

void loadSSB()
{
  si4735.setI2CFastModeCustom(500000); // Increase the transfer I2C speed
  si4735.loadPatch(ssb_patch_content, size_content); // It is a legacy function. See loadCompressedPatch 
  si4735.setI2CFastModeCustom(100000); // Set standard transfer I2C speed
}

void setup() {
  esp_wifi_set_mode(WIFI_MODE_NULL);
  Serial.begin(9600);

  pinMode(ENCODER_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON, INPUT);
  pinMode(RX_ANT_EN, OUTPUT);
  digitalWrite(RX_ANT_EN, HIGH);

  pinMode(PTT_KEY, INPUT);
  pinMode(DAC_CH1, OUTPUT);
  digitalWrite(DAC_CH1, 0);

  pinMode(BATTERY_SENSE_PIN, INPUT);
  pinMode(DEBUG_PIN, OUTPUT);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  int16_t si4735Addr = SI_ADDR;
  si4735.setup(RESET_PIN, AM_FUNCTION);
  delay(100);
  si4735.setAudioMute(false);
  delay(10);
  loadSSB();
  
  delay(100);
  si4735.setTuneFrequencyAntennaCapacitor(1); // Set antenna tuning capacitor for SW.
  si4735.setSSB(band[currentFreqIdx].minimumFreq, band[currentFreqIdx].maximumFreq, band[currentFreqIdx].currentFreq, band[currentFreqIdx].currentStep, band[currentFreqIdx].currentSSB);
  delay(100);
  frequency = si4735.getFrequency();
  si4735.setAvcAmMaxGain(90); // Sets the maximum gain for automatic volume control on AM/SSB mode (from 12 to 90dB)
  si4735.setVolume(63);
  si4735.setSSBSidebandCutoffFilter(0);
  si4735.setSSBAudioBandwidth(5);

  update_display_now = true;

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  si5351.set_freq(tx_frequency, SI5351_CLK0);
  //si5351.set_freq(709000000ULL, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK1, 1);

  // Enable DAC2 Channel's Output (sidetone)
  pinMode(DAC_CH2, OUTPUT);

  // Work out the morse element length in milliseconds for the keyer
  morse_element_length_millis = (60000 / (50 * wpm));

  morse_message_length = 27;
  keyer_enable = false;
}

long oldPosition = 0;
int position_change = 0;
int frequency_old = 0;
long newPosition = 0;
bool sending_element = false;
bool auto_tx = false;
void loop() {
  newPosition = myEnc.read();
  position_change = position_change + (oldPosition - newPosition);
  if ((abs(position_change) > 3) and !key_down and !qsk_ing) {
    if (!digitalRead(ENCODER_BUTTON)) {
      currentBFO = currentBFO + (currentBFOStep * (position_change / POSITIONS_PER_ENCODER_STEP));
      si4735.setSSBBfo(currentBFO);
    } else {
      frequency = frequency + (currentStep * (position_change / POSITIONS_PER_ENCODER_STEP));
      tx_frequency = frequency * 100000;
      si5351.set_freq(tx_frequency, SI5351_CLK0);
    }
    position_change = 0;
    update_display_now = true;
  }
  oldPosition = newPosition;
  
  if (update_display_now or ((millis() - last_display_update) >= MILLIS_PER_DISPLAY_UPDATE)) {
    noInterrupts();
    update_display();
    interrupts();
    update_display_now = false;
    last_display_update = millis();
  }

  if (!digitalRead(PTT_KEY) and !ptt_key_pressed) { // If the ptt key has just been pressed, take a timestamp
    ptt_key_pressed = true;
    ptt_key_press_timer = millis();
  } else if (digitalRead(PTT_KEY) and ptt_key_pressed) { // If the PTT key is released at any time, remove the key pressed flag
    ptt_key_pressed = false;
    ptt_key_press_timer = millis();
  }
  
  if ((ptt_key_pressed and !key_down) or auto_tx) { // If the PTT debounce timer hasn't expired, but the key is pressed
    if ((millis() - ptt_key_press_timer) >= PTT_KEY_DEBOUNCE_DELAY) { // Put the thing into 'key down' mode
      digitalWrite(RX_ANT_EN, LOW);
      key_down = true;
      si4735.setAudioMute(true);
      audio.tone(SIDETONE_FREQUENCY);
      si5351.output_enable(SI5351_CLK0, 1);
      digitalWrite(DAC_CH1, 1);
    }
  }
  if ((key_down and digitalRead(PTT_KEY) and ((millis() - ptt_key_press_timer) > PTT_KEY_DEBOUNCE_DELAY)) or (sending_element and !auto_tx)) { // If we're in key down mode, and the key is released, put it int RX mode
    digitalWrite(DAC_CH1, 0);
    key_down = false;
    audio.noTone();
    qsk_ing = true;
    qsk_timer = millis();
  }
  if ((qsk_ing and ((millis() - qsk_timer) >= QSK_DELAY_MS)) and !key_down) {
    si5351.output_enable(SI5351_CLK0, 0);
    si4735.setAudioMute(false);
    qsk_ing = false;
    qsk_timer = 0;
    digitalWrite(RX_ANT_EN, HIGH);
  }

  if (((millis() - last_RX_update) >= MILLIS_PER_RX_UPDATE) and !key_down and !qsk_ing) {
    if (frequency != frequency_old) {
      si4735.setSSB(band[currentFreqIdx].minimumFreq, band[currentFreqIdx].maximumFreq, frequency, band[currentFreqIdx].currentStep, band[currentFreqIdx].currentSSB);
      delay(100);
      frequency_old = si4735.getFrequency();
      rx_rssi = si4735.getReceivedSignalStrengthIndicator();
      si4735.getCurrentReceivedSignalQuality();
    }
  }

  // CW keyer
  if (sending_element and ((millis() - morse_element_timer) > morse_element_length_millis)) {
    sending_element = false;
  }
  
  if (keyer_enable and !sending_element) {
    if ((cw_buffer_pointer >= morse_message_length) or (cw_buffer_pointer >= CW_BUFFER_LENGTH)) { // If we'ver reached the end of the message or CW buffer
      keyer_enable = false; // stop the keyer
      sending_element = false;
    } else { // we move on to the next element in the buffer
      if (cw_buffer[cw_buffer_pointer] == 1) {
        auto_tx = true; // start sending if the next element is a mark
      } else {
        auto_tx = false;
      }
      sending_element = true;
      morse_element_timer = millis();
      cw_buffer_pointer++;
    }
  }
}
