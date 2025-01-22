#include "driver/i2s_std.h"
#include "esp_err.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "soc/gpio_struct.h"
#include <math.h>

// --- PIN Assignments ---
#define I2S_BCLK_PIN GPIO_NUM_14    // Bit Clock Pin (BCLK)
#define I2S_WS_PIN GPIO_NUM_15      // Word Select Pin (LRCK)
#define I2S_DATA_IN_PIN GPIO_NUM_32 // Data Input Pin (DIN)
#define I2S_SAMPLE_RATE 44100
#define I2S_CHANNEL I2S_NUM_0
#define SD_CS_PIN 5

// --- Audio Settings ---
#define SAMPLE_RATE 44100
#define BITS_PER_SAMPLE 16
#define RECORD_DURATION 30 // in seconds

// --- Global Variables ---
i2s_chan_handle_t rx_handle;
bool is_recording = false;

// --- WAV Header Structure ---
struct WAV_HEADER {
    char riff[4] = {'R', 'I', 'F', 'F'};
    uint32_t flength = 0;
    char wave[4] = {'W', 'A', 'V', 'E'};
    char fmt[4] = {'f', 'm', 't', ' '};
    uint32_t chunk_size = 16;
    uint16_t format_tag = 1;
    uint16_t num_chans = 1;
    uint32_t srate = SAMPLE_RATE;
    uint32_t bytes_per_sec = SAMPLE_RATE * (BITS_PER_SAMPLE / 8);
    uint16_t bytes_per_samp = BITS_PER_SAMPLE / 8;
    uint16_t bits_per_samp = BITS_PER_SAMPLE;
    char dat[4] = {'d', 'a', 't', 'a'};
    uint32_t dlength = 0;
} wav_header;

// --- I2S Configuration ---
void i2s_setup() {
    // Configure GPIO pin as input with pull-down resistor to minimize noise
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << I2S_DATA_IN_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gpio_conf);

    // Initialize I2S channel configuration as SLAVE for receiving data
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_CHANNEL, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, NULL, &rx_handle);

    // Configure I2S settings for standard mode
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,      // Bit Clock Pin
            .ws = I2S_WS_PIN,          // Word Select Pin
            .dout = I2S_GPIO_UNUSED,   // Not used for input
            .din = I2S_DATA_IN_PIN,    // Data In Pin
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    // Initialize I2S channel in standard mode
    i2s_channel_init_std_mode(rx_handle, &std_cfg);
    i2s_channel_enable(rx_handle);
}

// --- Write WAV Header ---
void write_wav_header(File &file) {
    file.write((uint8_t *)&wav_header, sizeof(wav_header));
}

// --- Update WAV Header ---
void update_wav_header(File &file, uint32_t data_length) {
    wav_header.dlength = data_length;
    wav_header.flength = data_length + sizeof(wav_header) - 8;
    file.seek(0);
    file.write((uint8_t *)&wav_header, sizeof(wav_header));
}

// --- Record Audio ---
void record_audio(const char *filename) {
    Serial.println("Recording started!");
    if (is_recording) return;

    is_recording = true;
    File audio_file = SD.open(filename, FILE_WRITE);
    if (!audio_file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    write_wav_header(audio_file);
    uint32_t total_bytes = 0;
    uint8_t audio_buffer[4096];
    unsigned long start_time = millis();

    while ((millis() - start_time) < RECORD_DURATION * 1000) {
        size_t bytes_read;
        i2s_channel_read(rx_handle, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
        audio_file.write(audio_buffer, bytes_read);
        total_bytes += bytes_read;
    }

    update_wav_header(audio_file, total_bytes);
    audio_file.close();
    is_recording = false;

    Serial.println("Recording complete!");
}

// --- Task to Record Every Minute ---
void record_task() {
    static unsigned long last_record_time = 0;
    unsigned long current_time = millis();

    if (current_time - last_record_time >= 60000) { // 60 seconds
        last_record_time = current_time;

        char filename[32];
        snprintf(filename, sizeof(filename), "/record_%lu.wav", current_time / 1000);
        record_audio(filename);
    }
}

// List all files in the root directory
void listFiles() {
  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open root directory!");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("DIR: ");
    } else {
      Serial.print("FILE: ");
    }
    Serial.print(file.name());
    Serial.print("  SIZE: ");
    Serial.println(file.size());
    file = root.openNextFile();
  }
  root.close();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
      Serial.println("Failed to initialize SD card");
      return;
  }

  Serial.println("SD card initialized successfully.");
  // Check available files and write a test file
  listFiles();
  i2s_setup();
  Serial.println("I2S and SD card initialized");
}

void loop() {
    record_task();
}