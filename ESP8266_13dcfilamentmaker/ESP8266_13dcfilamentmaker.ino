// Some libraries used in this project may be licensed under different licenses. 
// Please refer to the LICENSE file for more information about those libraries.

#include <ESP8266WiFi.h>          // Library for managing WiFi functionality on the ESP8266
#include <ESPAsyncWebServer.h>     // Library for asynchronous web server on ESP8266
#include <ESPAsyncTCP.h>           // Library for asynchronous TCP communication
#include <Encoder.h>               // Library to interface with rotary encoders
#include <AccelStepper.h>          // Library for controlling stepper motors
#include <PID_v1.h>                // Library for PID control algorithm
#include <FS.h>                    // Library for SPIFFS (SPI Flash File System)
#include <Ticker.h>                // Library for Ticker, used for timed callbacks




/*
 * The code below is licensed under the MIT License.
 * 
 * MIT License
 * 
 * Copyright (c) 2024 13DCreator
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */




// Define pins for connecting components
#define CLK_PIN D2                 // Clock pin for rotary encoder
#define DT_PIN D3                  // Data pin for rotary encoder
#define BUTTON_PIN D4              // Button pin for rotary encoder
#define SWITCH_PIN D8              // Pin for switch control
#define STEP_PIN D6                // Step pin for stepper motor control
#define DIR_PIN D7                 // Direction pin for stepper motor control
#define ENABLE_PIN D5              // Enable pin for stepper motor
#define MOSFET_PIN D1              // Pin for controlling MOSFET, used for temperature control


// Thermistor parameters

      // Uncomment if using a 10k resistor
const float R1 = 10000; // Pull-up resistor value in ohms, used with the thermistor
      // Uncomment if using a 100k resistor
//const float R1 = 100000; // Pull-up resistor value in ohms, used with the thermistor

const float BETA = 3950; // Beta coefficient of the thermistor for temperature calculations
const float T0 = 298.15; // Reference temperature in Kelvin (25°C)
const float R0 = 100000; // Thermistor resistance at reference temperature (in ohms)

// WiFi credentials for Access Point (AP) mode
const char* apSSID = "13DC";       // SSID for WiFi access point
const char* apPassword = "13dcreator"; // Password for WiFi access point

// Interval for temperature reading
const int temperatureUpdateInterval = 3000; // Update interval of 3 seconds

// Initialize components
Encoder encoder(CLK_PIN, DT_PIN);               // Rotary encoder for position tracking
AsyncWebSocket webSocket("/ws");                // WebSocket server at "/ws" for real-time data
AsyncWebServer server(80);                      // Web server on port 80
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); // Stepper motor driver initialization

// PID variables for temperature control
double Setpoint = 100.0, Input, Output;         // Target temperature, input, and PID output
double Kp = 10, Ki = 0.35, Kd = 12;             // PID tuning constants
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // PID controller setup

// Encoder and stepper motor parameters
long lastEncoderPosition = 0;
long currentSliderValue = 0;
const int sliderMin = -5000;       // Minimum slider value
const int sliderMax = 5000;        // Maximum slider value
const int maxSpeed = 5000;         // Maximum stepper motor speed
const int stepInterval = 25;       // Step interval for encoder readings
const int scaleFactor = 25;        // Scaling factor for encoder readings
const int encoderResolution = 1;   // Encoder resolution

// Ticker for timed temperature updates
Ticker temperatureTicker;

// Low-Pass Filter (LPF) parameters for smoothing PID output
const float alpha = 0.1;           // Smoothing factor (tune as needed)
float lastFilteredOutput = 0;      // Stores last filtered output for LPF

// Function to scale encoder value
long scaleEncoderValue(long value) {
  return value * scaleFactor;      // Scales raw encoder value by a predefined factor
}

// Function to round encoder value to the nearest multiple of scaleFactor
long roundToNearestScale(long value) {
  return (value / scaleFactor) * scaleFactor;   // Rounds value to nearest scale factor multiple
}

// Function to read temperature from the thermistor
float readTemperature() {
    int analogValue = analogRead(A0);                // Read analog value from A0 pin
    if (analogValue == 0) {
        return 0.0;                                  // Prevents division by zero if reading is 0
    }
    float resistance = R1 * (1023.0 / analogValue - 1); // Calculates thermistor resistance
    float temperature = 1.0 / (log(resistance / R0) / BETA + 1.0 / T0) - 273.15; // Converts to Celsius
    return temperature;
}

// HTML content for the web page
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Temperature Control</title>
<style>
  body {
    font-family: Arial, sans-serif;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    min-height: 100vh;
    margin: 0;
    background-color: #f4f4f9;
    text-align: center;
  }
  h1 {
    color: #333;
  }
  h2 {
    color: #555;
  }
  .container {
    width: 90%;
    max-width: 600px;
    padding: 20px;
    background: #fff;
    border-radius: 8px;
    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
  }
  .status {
    font-size: 20px;
    font-weight: bold;
  }
  input[type="range"] {
    margin: 10px 0;
  width: 80%;
  }
  
    input[type="number"] {
    margin: 10px 0;
  width: 10%;
  }
  
  button {
    padding: 10px 20px;
    font-size: 16px;
    border: none;
    border-radius: 5px;
    background-color: #007bff;
    color: #fff;
    cursor: pointer;
    margin: 10px;
  }
  button:hover {
    background-color: #0056b3;
  }
  p {
    margin: 10px 0;
  }
</style>
</head>
<body>
  <div class="container">
    
    <h2>Temperature Control</h2>
    <p>Current Temperature: <span id="temperatureValue">0</span> °C</p>
  <br>
  
  <p>Set Temperature:</p>
    <p> <input type="number" id="setTemp" value="100"> °C</p>
  
      <button id="setTempButton">Set Temperature</button>

    <h2>PID Control</h2>
    <p>Kp: <input type="number" id="kpValue" value="2" step="0.1"></p>
    <p>Ki: <input type="number" id="kiValue" value="5" step="0.1"></p>
    <p>Kd: <input type="number" id="kdValue" value="1" step="0.1"></p>
    <button id="applyPIDButton">Apply PID</button>
  </div>

  <script>
    var socket = new WebSocket('ws://' + location.hostname + '/ws');

    socket.onmessage = function(event) {
      var data = event.data;
      if (data.startsWith("TEMP:")) {
        document.getElementById('temperatureValue').innerText = data.substring(5);
      } else if (data.startsWith("SETPOINT:")) {
        document.getElementById('setTemp').value = data.substring(9);
      } else if (data.startsWith("PID:")) {
        var pidValues = data.substring(4).split(',');
        document.getElementById('kpValue').value = pidValues[0];
        document.getElementById('kiValue').value = pidValues[1];
        document.getElementById('kdValue').value = pidValues[2];
      } 
        
    document.getElementById('setTempButton').onclick = function() {
      var setTemp = document.getElementById('setTemp').value;
      socket.send('SET_TEMP:' + setTemp);
    };

    document.getElementById('applyPIDButton').onclick = function() {
      var kp = document.getElementById('kpValue').value;
      var ki = document.getElementById('kiValue').value;
      var kd = document.getElementById('kdValue').value;
      socket.send('SET_PID:' + kp + ',' + ki + ',' + kd);
    };
  </script>
</body>
</html>

)rawliteral";

// Function to save PID parameters to SPIFFS
void saveParameters() {
  File file = SPIFFS.open("/params.csv", "w"); // Open a file for writing in SPIFFS
  if (!file) {
    Serial.println("Failed to open file for writing"); // Error message if file fails to open
    return;
  }
  file.printf("%.2f,%.2f,%.2f,%.2f\n", Setpoint, Kp, Ki, Kd); // Save PID parameters to file in CSV format
  file.close();
  Serial.println("Parameters saved"); // Confirmation message for successful save
}

// Function to load PID parameters from SPIFFS
void loadParameters() {
  File file = SPIFFS.open("/params.csv", "r"); // Open a file for reading in SPIFFS
  if (!file) {
    Serial.println("No saved parameters found"); // Message if no saved parameters are found
    return;
  }

  String configData = file.readStringUntil('\n'); // Read a line of saved parameters
  file.close();

  // Parse CSV values and assign them to PID variables
  int comma1 = configData.indexOf(',');
  int comma2 = configData.indexOf(',', comma1 + 1);
  int comma3 = configData.indexOf(',', comma2 + 1);

  Setpoint = configData.substring(0, comma1).toDouble(); // Extract Setpoint value
  Kp = configData.substring(comma1 + 1, comma2).toDouble(); // Extract Kp value
  Ki = configData.substring(comma2 + 1, comma3).toDouble(); // Extract Ki value
  Kd = configData.substring(comma3 + 1).toDouble(); // Extract Kd value

  myPID.SetTunings(Kp, Ki, Kd); // Apply loaded PID constants to controller
  Serial.println("Parameters loaded"); // Confirmation message for successful load
}

// Function to apply Low-Pass Filter (LPF) on new value
float applyLowPassFilter(float newValue) {
  lastFilteredOutput = alpha * newValue + (1 - alpha) * lastFilteredOutput; // LPF calculation
  return lastFilteredOutput; // Return smoothed value
}

// Ticker callback for managing temperature control
void temperatureCallback() {
  Input = readTemperature(); // Update PID input with the latest temperature reading

  // Check if temperature is valid (not NaN, zero, or abnormal)
  if (isnan(Input) || Input <= 0.0 || Input > 250.0) { // Assumes maximum valid temperature is 250°C
    analogWrite(MOSFET_PIN, 0); // Turn off MOSFET if temperature is invalid
    resetPosition(); // Reset stepper motor position
    Serial.println("Temperature not detected or abnormal! MOSFET off and stepper motor reset.");
    webSocket.textAll("ERROR: Temperature not detected or abnormal!"); // Notify through WebSocket
    return; // Exit without running PID
  }
  
  myPID.Compute(); // Calculate PID output

  // Apply LPF to PID output
  float smoothedOutput = applyLowPassFilter(Output);

  analogWrite(MOSFET_PIN, constrain(smoothedOutput, 0, 255)); // Send filtered PID output to MOSFET
  webSocket.textAll("TEMP:" + String(Input)); // Send temperature reading via WebSocket
}



void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate

  if (!SPIFFS.begin()) { // Attempt to mount SPIFFS filesystem
    Serial.println("An error has occurred while mounting SPIFFS"); // Error message if SPIFFS fails to mount
    return;
  }

  loadParameters(); // Load saved PID parameters from SPIFFS

  WiFi.mode(WIFI_AP); // Set WiFi mode to Access Point (AP) mode
  WiFi.softAP(apSSID, apPassword); // Create AP with specified SSID and password
  IPAddress IP = WiFi.softAPIP(); // Get IP address of the AP
  Serial.print("AP IP address: "); // Print IP address for the AP
  Serial.println(IP);

  webSocket.onEvent(onWebSocketEvent); // Define WebSocket event handler
  server.addHandler(&webSocket); // Attach WebSocket to server

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", htmlPage); // Serve HTML page for root URL
  });

  server.begin(); // Start the web server

  stepper.setMaxSpeed(maxSpeed); // Set maximum speed for stepper motor
  stepper.setAcceleration(maxSpeed / 2); // Set acceleration for stepper motor

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Configure button pin as input with pull-up resistor
  pinMode(ENABLE_PIN, OUTPUT); // Configure enable pin for stepper motor as output
  digitalWrite(ENABLE_PIN, LOW); // Set enable pin to LOW (activates stepper motor)

  pinMode(SWITCH_PIN, INPUT); // Configure switch pin as input
  pinMode(MOSFET_PIN, OUTPUT); // Configure MOSFET control pin as output
  myPID.SetMode(AUTOMATIC); // Set PID controller mode to automatic

  // Initialize Ticker to call temperatureCallback at regular intervals (every 3 seconds)
  temperatureTicker.attach_ms(temperatureUpdateInterval, temperatureCallback);
}


void loop() {
  webSocket.cleanupClients(); // Remove any disconnected WebSocket clients
  manageStepper(); // Call function to control stepper motor
}

void manageStepper() {
  bool switchState = digitalRead(SWITCH_PIN) == LOW; // Read the switch state
  static bool lastSwitchState = HIGH; // Store last switch state for comparison

  // Check if switch state has changed
  if (switchState != lastSwitchState) {
    lastSwitchState = switchState;
    if (switchState) {
      webSocket.textAll("ENABLE"); // Notify clients that stepper is enabled
      digitalWrite(ENABLE_PIN, LOW); // Enable stepper motor
    } else {
      webSocket.textAll("DISABLE"); // Notify clients that stepper is disabled
      digitalWrite(ENABLE_PIN, HIGH); // Disable stepper motor
    }
  }

  // Reset encoder and stepper position if button is pressed
  if (digitalRead(BUTTON_PIN) == LOW) {
    resetPosition(); // Reset encoder position to zero
    delay(200); // Debounce delay
  }

  long rawEncoderPosition = encoder.read(); // Get raw encoder position
  long scaledEncoderPosition = scaleEncoderValue(rawEncoderPosition); // Scale encoder value
  scaledEncoderPosition = roundToNearestScale(scaledEncoderPosition); // Round to nearest scale factor
  scaledEncoderPosition = constrain(scaledEncoderPosition, sliderMin, sliderMax); // Limit within slider bounds

  // Only update stepper if position change exceeds the step interval
  if (abs(scaledEncoderPosition - lastEncoderPosition) >= stepInterval) {
    lastEncoderPosition = scaledEncoderPosition;

    // Set stepper speed if enabled
    if (digitalRead(ENABLE_PIN) == LOW) {
      stepper.setSpeed(map(scaledEncoderPosition, sliderMin, sliderMax, -maxSpeed, maxSpeed)); // Map scaled position to speed
    }
  }

  stepper.runSpeed(); // Run stepper at the current set speed
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *payload, size_t length) {
  String pidValues; // Variable to store PID values as string

  switch(type) {
    case WS_EVT_DATA: // When data is received from WebSocket client
      if (strncmp((const char*)payload, "RESET", length) == 0) {
        resetPosition(); // Reset encoder and stepper position
      } else if (strncmp((const char*)payload, "SET_TEMP:", 9) == 0) {
        Setpoint = atof((const char*)payload + 9); // Update temperature setpoint
        saveParameters(); // Save updated parameters to SPIFFS
        webSocket.textAll("SETPOINT:" + String(Setpoint)); // Notify all clients of the new setpoint
      } else if (strncmp((const char*)payload, "SET_PID:", 8) == 0) {
        sscanf((const char*)payload + 8, "%lf,%lf,%lf", &Kp, &Ki, &Kd); // Parse PID values
        myPID.SetTunings(Kp, Ki, Kd); // Apply new PID values
        saveParameters(); // Save updated PID values
        pidValues = String(Kp) + "," + String(Ki) + "," + String(Kd); // Convert PID values to string
        webSocket.textAll("PID:" + pidValues); // Notify all clients of the new PID values
      } else if (strncmp((const char*)payload, "ENABLE", length) == 0) {
        digitalWrite(ENABLE_PIN, LOW); // Enable stepper motor
      } else if (strncmp((const char*)payload, "DISABLE", length) == 0) {
        digitalWrite(ENABLE_PIN, HIGH); // Disable stepper motor
      }
      break;

    case WS_EVT_CONNECT: // When a WebSocket client connects
      Serial.println("Client connected");
      client->text("SETPOINT:" + String(Setpoint)); // Send current setpoint to client
      pidValues = String(Kp) + "," + String(Ki) + "," + String(Kd); // Convert PID values to string
      client->text("PID:" + pidValues); // Send current PID values to client
      break;

    case WS_EVT_DISCONNECT: // When a WebSocket client disconnects
      Serial.println("Client disconnected");
      break;

    case WS_EVT_ERROR: // When there is a WebSocket error
      Serial.printf("WebSocket error %u\n", client->id());
      break;

    case WS_EVT_PONG: // When a WebSocket PONG message is received
      Serial.printf("WebSocket PONG received from client %u\n", client->id());
      break;
  }
}

void resetPosition() {
  encoder.write(0); // Reset encoder position to zero
  stepper.setCurrentPosition(0); // Reset stepper motor position to zero
  webSocket.textAll("0"); // Notify all clients of the reset position
}
