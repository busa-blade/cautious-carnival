#include <driver/i2s.h>
#include <driver/adc.h>
#include <arduinoFFT.h>

#ifndef SOUND_ANALYSIS_H
#define SOUND_ANALYSIS_H

//#define ADC_INPUT ADC1_CHANNEL_0
#define BANDMODES 5

#define BANDSIZE_8 8
#define BANDSIZE_16 16
#define BANDSIZE_24 24
#define BANDSIZE_32 32
#define BANDSIZE_64 64

//#define ALL_BANDS_PEAK_UPPER_LIMIT 10000
#define ALL_BANDS_PEAK_LOWER_LIMIT 1

/*********************************************************/
const int bandmode_array[BANDMODES] = {
  BANDSIZE_8,
  BANDSIZE_16,
  BANDSIZE_24,
  BANDSIZE_32,
  BANDSIZE_64
};

// Below are the band frequencies. You can play around with it but be carefull. Also, they have to be sorted from low to high
static uint16_t BandCutoffTable8[BANDSIZE_8] = {
  100, 250, 500, 1000, 2000, 4000, 8000, 16000
};

const char *labels8[BANDSIZE_8] = {
  "100", "250", "500", "1K", "2K", "4K", "8K", "16K"
};

static uint16_t BandCutoffTable16[BANDSIZE_16] = {
  30, 50, 100, 150, 250, 400, 650, 1000, 1600, 2500, 4000, 6000, 12000, 14000, 16000, 17000
};

const char *labels16[BANDSIZE_16] = {
  "30", "50", "100", "150", "250", "400", "650", "1K",
  "1K6", "2K5", "4K", "6K", "12K", "14K", "16K", "17K"
};

static uint16_t BandCutoffTable24[BANDSIZE_24] = {
  40, 80, 150, 220, 270, 320, 380, 440, 540, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150,
  3800, 4200, 4800, 5400, 6200, 7400, 12500
};

const char *labels24[BANDSIZE_24] = {
  "40", "80", "150", "220", "270", "320", "380", "440",
  "540", "630", " 800", "1000", "1250", "1600", "2000", "2500",
  "3150", "3800", "4200", "4800", "5400", "6200", "7400", "12500"
};

static uint16_t BandCutoffTable32[BANDSIZE_32] = {
  45, 90, 130, 180, 220, 260, 310, 350,
  390, 440, 480, 525, 650, 825, 1000, 1300,
  1600, 2050, 2500, 3000, 4000, 5125, 6250, 9125,
  12000, 13000, 14000, 15000, 16000, 16500, 17000, 17500
};

const char *labels32[BANDSIZE_32] = {
  "45", "90", "130", "180", "220", "260", "310", "350",
  "390", "440", "480", "525", "650", "825", "1K", "1K3",
  "1K6", "2K05", "2K5", "3K", "4K", "5125", "6250", "9125",
  "12K", "13K", "14K", "15K", "16K", "16K5", "17K", "17K5"
};

static uint16_t BandCutoffTable64[BANDSIZE_64] = {
  45, 90, 130, 180, 220, 260, 310, 350, 390, 440, 480, 525, 565, 610, 650, 690, 735, 780, 820, 875, 920, 950, 1000, 1050, 1080, 1120, 1170, 1210, 1250, 1300, 1340, 1380, 1430, 1470, 1510, 1560, 1616, 1767, 1932, 2113, 2310, 2526, 2762, 3019, 3301, 3610, 3947, 4315, 4718, 5159, 5640, 6167, 6743, 7372, 8061, 8813, 9636, 10536, 11520, 12595, 13771, 15057, 16463, 18000
};

const char *labels64[BANDSIZE_64] = {
  "45", "90", "130", "180", "220", "260", "310", "350",
  "390", "440", "480", "525", "565", "610", "650", "690",
  "735", "780", "820", "875", "920", "950", "1000", "1050",
  "1080", "1120", "1170", "1210", "1250", "1300", "1340", "1380",
  "1430", "1470", "1510", "1560", "1616", "1767", "1932", "2113",
  "2310", "2526", "2762", "3019", "3301", "3610", "3947", "4315",
  "4718", "5159", "5640", "6167", "6743", "7372", "8061", "8813",
  "9636", "10536", "11520", "12595", "13771", "15057", "16463", "18K"
};

const char **band_cutoff_tables_labels[BANDMODES] = {
  labels8,
  labels16,
  labels24,
  labels32,
  labels64
};

const uint16_t *band_cutoff_tables[BANDMODES] = {
  BandCutoffTable8,
  BandCutoffTable16,
  BandCutoffTable24,
  BandCutoffTable32,
  BandCutoffTable64
};

#define GET_BANDMODE_INDEX(x) (x % BANDMODES)
#define GET_SIZE_FOR_BANDMODE_INDEX(x) bandmode_array[GET_BANDMODE_INDEX(x)]                                      //should always return one of the sizes in the table of modes
#define GET_BANDMODE_CUTOFF_TABLE_FOR_BANDMODE_INDEX(x) band_cutoff_tables[GET_BANDMODE_INDEX(x)]                 //should always return the proper band_cutoff_table for the mode specified
#define GET_BANDMODE_CUTOFF_TABLES_LABELS_FOR_BANDMODE_INDEX(x) band_cutoff_tables_labels[GET_BANDMODE_INDEX(x)]  //should always return the proper band_cutoff_label for the mode specified
static float lastAllBandsPeak = 0.0f;


const char **get_cutoff_table_labels(uint8_t mode) {
  if (mode < BANDMODES) {
    return band_cutoff_tables_labels[mode];
  }
  return NULL;
}

const uint16_t *get_cutoff_table(uint8_t mode) {
  if (mode < BANDMODES) {
    return band_cutoff_tables[mode];
  }
  return NULL;
}

//bandmode_array[GET_CURRENT_BANDMODE_INDEX(current_band_mode);
/*
void increase_bandmode() {                                                                                                                                    //**
  Serial.println("Mode Button has been pressed!");                                                                                                    //**
  //current_band_mode++;                                                                                                              //**
  button_press_accumulator++;
  //SetNumberofBands(bandmode_array[current_band_mode%BANDMODES]);                                                                                                                         //**
  Serial.printf("New number of on next band analysis will be: %d\n", bandmode_array[GET_CURRENT_BANDMODE_INDEX(current_band_mode+button_press_accumulator)]);
}

//Only perform this function between analysis so that the code doesn't crash.
//We accumulate the button presses and give feedback, but don't update anything because we might be in the middle of doing calculations and we don't want pointers going wonky
void perform_actual_mode_update(){
  current_band_mode+=button_press_accumulator;
  button_press_accumulator = 0;
}
*/
esp_err_t i2s_setup(i2s_port_t i2sport, uint8_t sck_pin, uint8_t ws_pin, uint8_t sd_pin, uint16_t sample_frequency, uint16_t sample_size, uint8_t number_of_dma_buffers) {
  esp_err_t err = ESP_OK;
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sample_frequency,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),  //(i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,                              // Interrupt level 1
    .dma_buf_count = number_of_dma_buffers,                                // number of buffers
    .dma_buf_len = sample_size,                                            // samples per buffer
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = sck_pin,
    .ws_io_num = ws_pin,
    .data_out_num = I2S_PIN_NO_CHANGE,  // not used (only for speakers)
    .data_in_num = sd_pin               // Serial Data (SD)
  };

  err = i2s_driver_install(i2sport, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    #ifdef DEBUG
      Serial.printf("Failed installing driver: %d\n", err);
    #endif //DEBUG
    while (true)
      ;
  }
  err = i2s_set_pin(i2sport, &pin_config);
  if (err != ESP_OK) {
    #ifdef DEBUG
    Serial.printf("Failed setting pin: %d\n", err);
    #endif //DEBUG
    while (true)
      ;
  }
  return err;
}

size_t read_i2s_sample(i2s_port_t i2s_port, void *sample_buffer, size_t buffer_size, TickType_t timeout) {
  size_t bytes_read = 0;
  esp_err_t result = i2s_read(i2s_port, sample_buffer, buffer_size, &bytes_read, timeout);  // no timeout
  if (result == ESP_OK && bytes_read > 0) {
    if (bytes_read < buffer_size) {
      #ifdef DEBUG
      Serial.print(F("Buffer Underrun: "));
      Serial.print(F("Could only read "));
      Serial.print(bytes_read);
      Serial.print(F("bytes of "));
      Serial.print(buffer_size);
      Serial.println(F("in read_i2s_sample()"));
      #endif //DEBUG
    }
  }
  return bytes_read;
}


//****************************************************************************************
// Return the frequency corresponding to the Nth sample bucket.  Skips the first two
// buckets which are overall amplitude and something else.
int get_band_cutoff_for_sample(int bucket, double sample_frequency, size_t sample_size) {
  int bucket_offset = 0;
  const int bucket_min = 2;
  const int fft_threshold = 1;
  if (bucket > fft_threshold) {
    //bucket_offset = (bucket - 2) * (sample_frequency / 2) / (sample_size / 2);
    bucket_offset = (bucket  - bucket_min) * (sample_frequency/(sample_size)); /*frequency width of bin*/
  }
  return bucket_offset;
}

void analyze_sample(double *freq_band_outputs, int freq_band_mode_index, byte *samples, uint16_t samples_size, double sampling_frequency, uint32_t noise_threshold, uint8_t gain_dampening) {
  //uint16_t offset = (int)ADC_INPUT * 0x1000 + 0xFFF;      //*
  double *vReal = (double *) malloc(samples_size * sizeof(double));  //*
  double *vImag = (double *) malloc(samples_size * sizeof(double));
  const uint8_t band_size = GET_SIZE_FOR_BANDMODE_INDEX(freq_band_mode_index);
  const uint16_t *band_cutoff_table = get_cutoff_table(freq_band_mode_index);
  uint8_t internal_gain_dampening = gain_dampening;
  if (internal_gain_dampening < 1){
    #ifdef DEBUG
    Serial.println("Gain dampening can't be less than 1 (zero), therefore setting lower ceiling of 1")
    #endif //DEBUG
    internal_gain_dampening = 1;
  }
  int freq = 0;
  float allBandsPeak = 0;
  arduinoFFT FFT;

  //############ Step 2: compensate for Channel number and offset, safe all to vReal Array   ############
  for (uint16_t i = 0; i < samples_size; i++) {
    vReal[i] = (int8_t) samples[i]; //samples are passed to S2I as byte array, but we are re-casting them as a signed integer with the idea that the amplitude can be either positive or negative like a normal sine wave 
    vImag[i] = 0.0;  //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }
  
  FFT = arduinoFFT(vReal, vImag, samples_size, sampling_frequency); // Create FFT object

  //############ Step 3: Do FFT on the VReal array  ############
  // compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
    
  //############ Step 4: Fill the frequency bins with the FFT Samples ############
  for (uint16_t i = 1; i < samples_size / 2; i++) {
    if ((vReal[i] > noise_threshold) ){
      freq = get_band_cutoff_for_sample(i, sampling_frequency, samples_size);

      for(uint8_t freq_band = 0; freq_band < band_size; freq_band++){
        if (freq < band_cutoff_table[freq_band]){
          freq_band_outputs[freq_band] += vReal[i];
          break;
        }
      }
    }
  }
  //Normalizing the data so that it is relative to what we are taking in
  //############ Step 5: Averaging and making it all fit on screen
  for (int i = 0; i < band_size; i++) {
    //get the highest frequency measurement
    if (freq_band_outputs[i] > allBandsPeak) {
      allBandsPeak = freq_band_outputs[i];
    }
  }
  //We should keep a minimum of 1 or else allbands will serve to amplify the signal. What we are looking to do is normalize large sizes.
  if (allBandsPeak < ALL_BANDS_PEAK_LOWER_LIMIT) {
    allBandsPeak = ALL_BANDS_PEAK_LOWER_LIMIT;
  }
  //If we decide to dampen, using a dampening of 1 does nothing (no dampening).
  allBandsPeak = max(allBandsPeak, ((lastAllBandsPeak * (internal_gain_dampening - 1)) + allBandsPeak) / internal_gain_dampening); // Dampen rate of change a little bit on way down
  //Storing the previous value so that we can use it on subsequent calculations
  lastAllBandsPeak = allBandsPeak;
  
  //We should never really use this or have to use this. If we do, something is wrong with the audio and we need to find a way to filter
  /*
  if (allBandsPeak > ALL_BANDS_PEAK_UPPER_LIMIT) {
    allBandsPeak = ALL_BANDS_PEAK_UPPER_LIMIT;
  }
  */
  for (uint8_t i = 0; i < band_size; i++) {
    freq_band_outputs[i] /= (allBandsPeak * 1.0f);  
  }
  
  free(vReal);
  free(vImag);
}
#endif  //SOUND_ANALYSIS_H
