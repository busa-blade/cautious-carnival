#define __STDC_LIMIT_MACROS
#include "sound_analysis.h"

//Number of LEDS total
#define LED_STRIP_SIZE 10
//#define LED_DATA_PIN 6 arduino uno
#define LED_DATA_PIN 13  //ESP32 can be changed to any free digital pin (except the ones that support I2S)

#define SWITCHSTATES 18
// INMP441 I2S pin assignment
//#define I2S_WS GPIO_NUM_25
//#define I2S_SD GPIO_NUM_33
//#define I2S_SCK GPIO_NUM_32

//This supports the INMP441 MEMS microphone. We read 3 pins and are using the microphone in MONO mode. You will need at least one more pin and another INMP441 to get stereo data.
//Stereo may also need additional DMA BUFFERS (I haven't tested the configuration)
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PORT I2S_NUM_0 //We may need to use another port if we are using another I2S device
#define NUM_DMA_BUFFERS 4
#define BYTE_BUFFER_SIZE 1024 //All the code I have seen, seems to indicate this is a good buffer read size. One thing that I initially missed in the documentation is that the buffer is read in bytes
//Whatever buffer you read to should be sized in bytes. I've seen people complain that only half of the buffer was getting filled, but they were passing in buffers that were sized in uint16_t which are 2 bytes large
#define DEFAULT_LOWER_NOISE_THRESHOLD 1100 //This is something you can play with. In my experience, this gave me a good balance of noise rejection
#define DEFAULT_SPEED_FILTER 0.08f
#define GAIN_DAMPEN 1
#define SAMPLE_FREQ 48000 //Remember Nyquist theorem. Sample rate should be 2x highest freq

//#define DEBUG
//#define SHOW_STATISTICS

#define BUTTON_PIN 2
#define BRIGHTNESS 127

#define LED_STRIP_TYPE WS2812B
#define COLOR_ORDER GRB

#define NUM_DMA_BUFFERS 2
#define ANALYZER_MODE 0  //8 band analyzer
#define I2S_READ_TIMEOUT portMAX_DELAY
#define STRIP_FREQ_MULTIPLIER 2 //If we have X bands in the equalizer, we need at least 1X number of LEDs, but we can support it with N number of LEDS, this represents N LEDS which is a multiple of the number of bands
#define EQ_PATTERN 1

#ifdef DEBUG || SHOW_STATISTICS
#pragma message("Console logging will turned on and inforation will be logged to the serial console")
#else
#pragma message("Console loging will not be initialized!")
#endif //DEBUG || SHOW_STATISTICS

#define ISEVEN(n)   ((((int)n)%2 == 0) ? 1 : 0)

//Globals
//Number of segments of LEDs (The number of LEDs divided by
//the number of segments should come out to be a whole number
const uint8_t freq_array_size = GET_SIZE_FOR_BANDMODE_INDEX(ANALYZER_MODE);
const uint8_t strip_frequency_multiplier = STRIP_FREQ_MULTIPLIER;
const uint16_t segment_size = freq_array_size;
const uint16_t led_strip_size = segment_size * strip_frequency_multiplier;
CRGB led_strip[led_strip_size];
uint8_t switchstate = 0;
int loop_counter = 0;
#ifdef SHOW_STATISTICS
uint32_t samples_read_total = 0;
#endif //SHOW_STATISTICS

//int current_band_mode =              4;                     // Default number of bands. change it by pressing the mode button ( 8,16,24,32,64)
//#define MODE_BUTTON_PIN     15                      // this is the IO pin you are using for the button
//#define GAIN_DAMPEN         2                       // Higher values cause auto gain to react more slowly
//int noise_threshold =        2000;                   // this will effect the upper bands most.
//float speed_filter  =          0.08;                   // slowdown factor for columns to look less 'nervous' The higher the quicker
//uint8_t button_press_accumulator = 0;

void setup_microphone() {
  i2s_setup(I2S_PORT, I2S_SCK, I2S_WS, I2S_SD, SAMPLE_FREQ, BYTE_BUFFER_SIZE, NUM_DMA_BUFFERS);
}

void setup() {
  // put your setup code here, to run once:
  //randomSeed(analogRead(0));
  FastLED.addLeds<LED_STRIP_TYPE, LED_DATA_PIN, COLOR_ORDER>(led_strip, led_strip_size);
  FastLED.setBrightness(BRIGHTNESS);
#ifdef DEBUG || SHOW_STATISTICS
  Serial.begin(115200);
#endif //DEBUG || SHOW_STATISTICS
  FastLED.clear();
  setup_microphone();
  delay(100);
}

void loop() {
  size_t samples_read = 0;
  byte samplebuffer[BYTE_BUFFER_SIZE];
  double frequency_bins[freq_array_size];
  #ifdef SHOW_STATISTICS
  long time_to_read = 0l;
  long time_to_process = 0l;
  #endif //SHOW_STATISTICS
  const char **band_cutoff_table_labels = get_cutoff_table_labels(ANALYZER_MODE);
  for (uint8_t i = 0; i < freq_array_size; i++) {
    frequency_bins[i] = 0.0;
  }
  #ifdef SHOW_STATISTICS
  time_to_read = millis();
  #endif //SHOW_STATISTICS
  samples_read = read_i2s_sample(I2S_PORT, &samplebuffer, BYTE_BUFFER_SIZE, I2S_READ_TIMEOUT);
  #ifdef DEBUG
  if (samples_read > BYTE_BUFFER_SIZE) {
    Serial.println(F("Gonna be a bad time with this buffer!"));
  }
  #endif //DEBUG
  #ifdef SHOW_STATISTICS
  time_to_read = millis() - time_to_read;
  samples_read_total+=samples_read;
  time_to_process = millis();
  #endif //SHOW_STATISTICS
  analyze_sample(frequency_bins, ANALYZER_MODE, samplebuffer, samples_read, SAMPLE_FREQ, DEFAULT_LOWER_NOISE_THRESHOLD, GAIN_DAMPEN);
  #ifdef SHOW_STATISTICS
  time_to_process = millis() - time_to_process;
  #endif //SHOW_STATISTICS
  #ifdef DEBUG
  for (uint8_t i = 0; i < freq_array_size; i++) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(band_cutoff_table_labels[i]);
    Serial.print(F(":"));
    Serial.print(frequency_bins[i]);
  }
  Serial.println();
  #endif //DEBUG
  show_on_leds(frequency_bins, freq_array_size, led_strip, segment_size, STRIP_FREQ_MULTIPLIER, EQ_PATTERN);

#ifdef SHOW_STATISTICS
  Serial.print(F("samples_read_total:"));
  Serial.print(samples_read_total);
  Serial.print(F(","));
  Serial.print(F("time_to_read:"));
  Serial.print(time_to_read);
  Serial.print(F(","));
  Serial.print(F("time_to_process:"));
  Serial.print(time_to_process);
  Serial.println();
#endif //STATISTICS
  loop_counter++;
}

void show_on_leds(double *frequencies, uint8_t frequency_array_size, CRGB *led_strip_ptr, uint16_t segment_size, uint8_t strip_segment_multiplier, uint8_t pattern){
  uint16_t led_color_segment_size = segment_size/3;
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;
  #ifdef DEBUG
  if (frequency_array_size != segment_size) {
    Serial.println(F("This is going to suck!"));
  }
  #endif //DEBUG
  for (uint16_t i = 0; i < frequency_array_size; i++) {
    red = 0;
    green = 0;
    blue = 0;
    if (i < led_color_segment_size) {
      red = 255 * (i+1)/led_color_segment_size * frequencies[i];
    } else if (i < round(led_color_segment_size * 2)) {
      green = 255 * (i+1 - led_color_segment_size)/led_color_segment_size * frequencies[i];
    } else {
      blue = 255 * (i + 1 - 2 * led_color_segment_size)/led_color_segment_size * frequencies[i];
    }
      switch(pattern){
        case 0: //Repeat the pattern all the way down the LEDs
            for (uint16_t p=0; p < strip_segment_multiplier; p++) {
              led_strip_ptr[(p*segment_size)+i] = CRGB(red, green, blue);
            }
          break;
        case 1: //Repeat the pattern on every N LEDs and reverse it on N+1, the effect is one side spreading out from the center and going outward
          if (ISEVEN(strip_segment_multiplier)) {
            for (uint16_t p=0; p < strip_segment_multiplier; p+=2) {
              led_strip_ptr[(p*segment_size)+i] = CRGB(red, green, blue);
              led_strip_ptr[((p+2)*segment_size)-i-1] = CRGB(red, green, blue);
            }
          } else {
            for (uint16_t p=0; p < strip_segment_multiplier-1; p+=2) {
              led_strip_ptr[(p*segment_size)+i] = CRGB(red, green, blue);
              led_strip_ptr[((p+2)*segment_size)-i-1] = CRGB(red, green, blue);
            }
            led_strip_ptr[((strip_segment_multiplier-1)*segment_size)+i] = CRGB(red, green, blue);
          }
          break;
        case 2: //Repeat the pattern on every N+1 LEDs and reverse it on N, the effect is one side spreading out from the center and going outward. This reverses the bands in the center and puts the other bands on the outside edges
          if (ISEVEN(strip_segment_multiplier)) {
            for (uint16_t p=0; p < strip_segment_multiplier; p+=2) {
              led_strip_ptr[((p+1)*segment_size)-i-1] = CRGB(red, green, blue);
              led_strip_ptr[((p+1)*segment_size)+i] = CRGB(red, green, blue);
            }
          } else {
            for (uint16_t p=0; p < strip_segment_multiplier-1; p+=2) {
              led_strip_ptr[((p+1)*segment_size)-i-1] = CRGB(red, green, blue);
              led_strip_ptr[((p+1)*segment_size)+i] = CRGB(red, green, blue);
            }
            led_strip_ptr[((strip_segment_multiplier)*segment_size)-i-1] = CRGB(red, green, blue);
          }
          break;
        case 3: //Repeat the pattern on every N+1 LEDs and reverse it on N, the effect is one side spreading out from the center and going outward. This reverses the bands in the center and puts the other bands on the outside edges
          if (ISEVEN(strip_segment_multiplier)) {
            for (uint16_t p=0; p < strip_segment_multiplier; p+=2) {
              led_strip_ptr[(p*segment_size)+i] = CRGB(red, green, blue);
              led_strip_ptr[((p+2)*segment_size)-i-1] = CRGB(red, green, blue);
            }
          } else {
            for (uint16_t p=0; p < strip_segment_multiplier-1; p+=2) {
              led_strip_ptr[(p*segment_size)+i] = CRGB(red, green, blue);
              led_strip_ptr[((p+2)*segment_size)-i-1] = CRGB(red, green, blue);
            }
            led_strip_ptr[((strip_segment_multiplier-1)*segment_size)+i] = CRGB(red, green, blue);
          }
          break;
        case 4: //Repeat the pattern on every N+1 LEDs and reverse it on N, the effect is one side spreading out from the center and going outward. This reverses the bands in the center and puts the other bands on the outside edges
          if (ISEVEN(strip_segment_multiplier)) {
            for (uint16_t p=0; p < strip_segment_multiplier; p+=2) {
              led_strip_ptr[(p*segment_size)+i] = CRGB(red, green, blue);
              led_strip_ptr[((p+2)*segment_size)-i-1] = CRGB(red, green, blue);
            }
          } else {
            for (uint16_t p=0; p < strip_segment_multiplier-1; p+=2) {
              led_strip_ptr[(p*segment_size)+i] = CRGB(red, green, blue);
              led_strip_ptr[((p+2)*segment_size)-i-1] = CRGB(red, green, blue);
            }
            led_strip_ptr[((strip_segment_multiplier-1)*segment_size)+i] = CRGB(red, green, blue);
          }
          break;
      }
    FastLED.show();    
  }
}

bool readButton() {
  bool state_changed = false;
  int buttonState = 0;
  buttonState = digitalRead(BUTTON_PIN);
  #ifdef DEBUG
  log("Reading pushbutton state");
  log("Button state is %d", buttonState);
  #endif //DEBUG
  if (buttonState == HIGH) {
    #ifdef DEBUG
    log("Push button registered press by seeing HIGH state on line (5V), changing animation by increasing switchstate");
    #endif //DEBUG
    switchstate++;
    #ifdef DEBUG
    log("Switchstate is %u", switchstate);
    #endif //DEBUG
    state_changed = true;
    if (switchstate >= SWITCHSTATES) {
      switchstate = 0;
    }
  }
  return state_changed;
}
