#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "soc/gpio_struct.h"

// Default Access Point credentials
const char *default_ssid = "ESP32-Access-Point";
const char *default_password = "123456789";

// Preferences to store Wi-Fi credentials
Preferences preferences;

// Current Wi-Fi credentials
char ssid[32] = "ESP32-Access-Point";
char password[64] = "123456789";

WebServer server(80);

// I2S Configuration
#define I2S_SAMPLE_RATE 16000
#define I2S_CHANNEL I2S_NUM_0
#define I2S_BCLK_PIN GPIO_NUM_14    // Bit Clock Pin (BCLK)
#define I2S_WS_PIN GPIO_NUM_15      // Word Select Pin (LRCK)
#define I2S_DATA_IN_PIN GPIO_NUM_32 // Data Input Pin (DIN)

#define DEFAULT_NOISE_THRESHOLD 50     // Default noise threshold
#define SPL_READ_DELAY pdMS_TO_TICKS(100) // Delay between readings
#define NUM_AVG_READINGS 10            // Number of readings for averaging

// Declare global handle for I2S
i2s_chan_handle_t rx_handle;

// GPIO for resetting credentials
const int resetPin = 33;
unsigned long buttonPressTime = 0;
bool resetTriggered = false;
bool configMode = false;
unsigned long configStartTime = 0;

// Flags for detecting changes
bool settingsChanged = false;

// Pin definition
const int Built_in_led = 2; //On Chip Led

// Calibration constants
float referenceRMS = 0.01;         // RMS voltage measured for reference SPL

// Lookup table for dB to linear voltage conversion
const int numPoints = 121; // 121 points for 0-120 dB range
const float voltageRange = 3.3; // Adjust to your desired voltage range
float lookupTable[numPoints];
float Calculated_dB = 0.0;

// Save credentials to Preferences
void saveCredentials(const char *new_ssid, const char *new_password) {
  preferences.begin("wifi", false);
  preferences.putString("ssid", new_ssid);
  preferences.putString("password", new_password);
  preferences.end();
}

// Load credentials from Preferences
void loadCredentials() {
  preferences.begin("wifi", true);
  String stored_ssid = preferences.getString("ssid", default_ssid);
  String stored_password = preferences.getString("password", default_password);
  stored_ssid.toCharArray(ssid, sizeof(ssid));
  stored_password.toCharArray(password, sizeof(password));
  preferences.end();
}

// Handle main root request
void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
<html>
<head>
  <title>Noise Level</title>
  <style>
    canvas {
      display: block;
      margin: 20px auto;
    }
  </style>
  <script>
    let frequencyData = []; 
    const maxPoints = 50; // Maximum number of points on the graph

    setInterval(() => {
        fetch('/sound-level')
        .then(response => response.text())
        .then(data => {
        document.getElementById('soundLevel').innerText = data + " dB";
        drawGauge(parseFloat(data));
        updateFrequencyPolygon(parseFloat(data));
    });

    }, 1000);

    function drawGauge(value) {
      const canvas = document.getElementById('gaugeCanvas');
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.beginPath();
      ctx.arc(100, 100, 80, 0.75 * Math.PI, 2.25 * Math.PI);
      ctx.lineWidth = 20;
      ctx.strokeStyle = '#ccc';
      ctx.stroke();
      ctx.beginPath();
      const angle = 0.75 + (value / 120) * 1.5; // Scale to 0-120 dB range
      ctx.arc(100, 100, 80, 0.75 * Math.PI, angle * Math.PI);
      ctx.strokeStyle = '#FF5722';
      ctx.stroke();
    }

    function updateFrequencyPolygon(value) {
      // Add the new value to the frequency data array
      if (frequencyData.length >= maxPoints) {
        frequencyData.shift(); // Remove oldest value if array exceeds maxPoints
      }
      frequencyData.push(value);

      drawFrequencyPolygon();
    }

    function drawFrequencyPolygon() {
      const canvas = document.getElementById('polygonCanvas');
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Set up the graph
      ctx.beginPath();
      ctx.moveTo(0, canvas.height);
      for (let i = 0; i < frequencyData.length; i++) {
        const x = (i / (maxPoints - 1)) * canvas.width;
        const y = canvas.height - (frequencyData[i] / 120) * canvas.height;
        ctx.lineTo(x, y);
      }
      ctx.lineTo(canvas.width, canvas.height); // Close the polygon to the x-axis
      ctx.fillStyle = 'rgba(0, 123, 255, 0.3)';
      ctx.fill();
      ctx.strokeStyle = '#007BFF';
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  </script>
</head>
<body>
  <h1>ESP32 Sound Level Monitoring</h1>
  <p>Current Sound Level: <span id="soundLevel">0</span></p>
  <canvas id="gaugeCanvas" width="200" height="200"></canvas>
  <h2>Frequency Polygon</h2>
  <canvas id="polygonCanvas" width="500" height="200"></canvas>
</body>
</html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleSoundLevel() {
  server.send(200, "text/plain", String(Calculated_dB, 1)); // Send sound level to the webpage
}

// Handle Wi-Fi settings page
void handleWiFiSettings() {
  String html = "<h1>Welcome to ESP32 Wi-Fi Settings!</h1>";
  html += "<form action=\"/config\" method=\"POST\">";
  html += "SSID: <input type=\"text\" name=\"ssid\" value=\"" + String(ssid) + "\"><br>";
  html += "Password: <input type=\"password\" name=\"password\" value=\"" + String(password) + "\"><br>";
  html += "<input type=\"submit\" value=\"Save\">";
  html += "</form>";
  server.send(200, "text/html", html);
}

// Handle form submission for new credentials
void handleSetCredentials() {
  if (server.hasArg("ssid") && server.hasArg("password")) {
    String new_ssid = server.arg("ssid");
    String new_password = server.arg("password");

    // Check if settings are changed
    if (new_ssid != String(ssid) || new_password != String(password)) {
      saveCredentials(new_ssid.c_str(), new_password.c_str());
      settingsChanged = true;
    }
    server.send(200, "text/html", "<h1>Credentials Updated. Restarting after timeout...</h1>");
  } else {
    server.send(400, "text/html", "<h1>Invalid Input</h1>");
  }
}

// Start Access Point for Wi-Fi settings
void startWiFiSettings() {
  server.stop();
  server.close(); // Clear existing handlers

  // Configure the custom AP IP address for configuration mode
  IPAddress local_IP(192, 168, 4, 2);
  IPAddress gateway(192, 168, 4, 2);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP("ESP32-Config", "");
  Serial.println("Wi-Fi Settings Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Set handlers specific to Wi-Fi configuration
  server.on("/config", HTTP_GET, handleWiFiSettings);
  server.on("/config", HTTP_POST, handleSetCredentials);

  server.begin();
  configStartTime = millis(); // Start the 1-minute timer
  configMode = true;
}

// Start normal Access Point
void startAccessPoint(const char *ap_ssid, const char *ap_password) {
  server.stop();
  server.close(); // Clear existing handlers

  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Set handlers specific to normal mode
  server.on("/", HTTP_GET, handleRoot);
  server.on("/sound-level", HTTP_GET, handleSoundLevel);
  server.begin();
}

// Check for reset button press
void checkResetButton() {
  if (digitalRead(resetPin) == LOW) {
    if (buttonPressTime == 0) {
      buttonPressTime = millis();
    } else if (millis() - buttonPressTime >= 5000 && !resetTriggered) {
      resetTriggered = true;
      Serial.println("Entering Wi-Fi settings mode...");
      startWiFiSettings();
    }
  } else {
    buttonPressTime = 0;
    resetTriggered = false;
  }
}

float calculate_spl(int16_t *samples, size_t sample_count, float noise_threshold) {
    double sum_squares = 0.0;
    for (size_t i = 0; i < sample_count; ++i) {
        sum_squares += samples[i] * samples[i]; // Sum of squares of the samples
    }
    double rms = sqrt(sum_squares / sample_count); // Root Mean Square (RMS)

    // If RMS is below the noise threshold, return 0 dB
    if (rms < noise_threshold) {
        return 0.0;
    }

    // SPL in decibels (dB)
    float spl = 20 * log10(rms);
    return spl;
}

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

void spl_task(void *pvParameters) {
    // Allocate buffer for I2S data (1024 samples per read)
    int16_t i2s_read_buff[1024];
    size_t bytes_read;
    float average_spl = 0.0;
    size_t count = 0;

    while (1) {
        // Read data from I2S microphone
        i2s_channel_read(rx_handle, i2s_read_buff, sizeof(i2s_read_buff), &bytes_read, portMAX_DELAY);

        // Calculate SPL for current data
        float spl = calculate_spl(i2s_read_buff, bytes_read / sizeof(int16_t), DEFAULT_NOISE_THRESHOLD);

        // Accumulate SPL readings for averaging
        average_spl += spl;
        count++;

        // Print SPL every NUM_AVG_READINGS readings
        if (count >= NUM_AVG_READINGS) {
            average_spl /= NUM_AVG_READINGS; // Average the SPL over readings
            Calculated_dB = average_spl;
            int index = (int)average_spl;
            float linearVoltage = lookupTable[index];
            printf("Average Sound Level: %.2f dB\n", average_spl);
            // Send the linear voltage to the serial plotter
            Serial.println((linearVoltage-0.03)*float(100));
            average_spl = 0.0; // Reset for next average
            count = 0; // Reset counter
        }

        // Delay between readings for readability
        vTaskDelay(SPL_READ_DELAY);
    }
}

void core_0_loop(void *pvParameters) {
    while (1) {
        server.handleClient();
        if (!configMode) {
            checkResetButton();
        } else if (millis() - configStartTime >= 80000) {
            Serial.println("Configuration mode timed out. Restarting...");
            delay(1000);
            ESP.restart();
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Yield time to other tasks
    }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configure the reset button pin
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(Built_in_led, OUTPUT);

  for (int i = 0; i < numPoints; i++) {
    float db = i;
    float voltage = referenceRMS * pow(10, db / 20);
    lookupTable[i] = voltage * voltageRange; // Scale to desired voltage range
  }
  delay(1000);

  // Load Wi-Fi credentials
  loadCredentials();
  delay(1000);

  // Start the normal Access Point
  startAccessPoint(ssid, password);
  delay(1000);

  Serial.println("Main Web Server Started");

  // Initialize I2S
  i2s_setup();
  delay(1000);

  // Run the wifi like task on core 0
  xTaskCreatePinnedToCore(core_0_loop, "Comm Task", 16384, NULL, 3, NULL, 0);
  // Create the task that reads I2S data and calculates SPL
  xTaskCreatePinnedToCore(spl_task, "SPL Task", 10240, NULL, 5, NULL, 1);
  Serial.println("Sound Intensity Measurement - MAX4466 with ESP32");
}

void loop() {
  //Nothing to do here as everything is running by using the FreeRtos task
}