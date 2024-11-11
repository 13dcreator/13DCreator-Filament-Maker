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
#define CLK_PIN D2
#define DT_PIN D3
#define BUTTON_PIN D4
#define SWITCH_PIN D8
#define STEP_PIN D6
#define DIR_PIN D7
#define ENABLE_PIN D5
#define MOSFET_PIN D1
 
 
// Thermistor parameters

      // Uncomment if using a 10k resistor
const float R1 = 10000; // Pull-up resistor value in ohms, used with the thermistor
      // Uncomment if using a 100k resistor
//const float R1 = 100000; // Pull-up resistor value in ohms, used with the thermistor

const float BETA = 3950; // Beta coefficient of the thermistor for temperature calculations
const float T0 = 298.15; // Reference temperature in Kelvin (25°C)
const float R0 = 100000; // Thermistor resistance at reference temperature (in ohms)

// WiFi credentials for AP mode
const char* apSSID = "13DC";
const char* apPassword = "13dcreator";

// Interval for temperature reading
const int temperatureUpdateInterval = 3000; // 3 seconds

Encoder encoder(CLK_PIN, DT_PIN);
AsyncWebSocket webSocket("/ws");
AsyncWebServer server(80);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// PID variables
double Setpoint = 175.0, Input, Output; // Set temperature
double Kp = 11, Ki = 0.34, Kd = 11; // PID constants
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

long lastEncoderPosition = 0;
long currentSliderValue = 0;
const int sliderMin = -5000;
const int sliderMax = 5000;
const int maxSpeed = 5000;
const int stepInterval = 25;
const int scaleFactor = 25;
const int encoderResolution = 1;

// Timing variables for Ticker
Ticker temperatureTicker;

// LPF parameters
const float alpha = 0.1; // Smoothing factor (adjust as needed)
float lastFilteredOutput = 0; // Last filtered PID output value

// Function to scale encoder value
long scaleEncoderValue(long value) {
  return value * scaleFactor;
}

// Function to round value to nearest multiple of scaleFactor
long roundToNearestScale(long value) {
  return (value / scaleFactor) * scaleFactor;
}

// Function to read temperature from thermistor
float readTemperature() {
    int analogValue = analogRead(A0); // Read analog value from A0
    if (analogValue == 0) {
        return 0.0; // Prevent division by zero
    }
    float resistance = R1 * (1023.0 / analogValue - 1); // Calculate resistance of thermistor
    float temperature = 1.0 / (log(resistance / R0) / BETA + 1.0 / T0) - 273.15; // Convert to temperature in Celsius
    return temperature;
}

// HTML content for the web page
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Encoder and Slider Sync</title>
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
        
      
        
        //else if (data === "ENABLE") {
        //  document.getElementById('switchStatus').innerText = "Enabled";
       //   document.getElementById('switchStatus').style.color = "green";
      //  } 
        
                
        //else if (data === "DISABLE") {
       //   document.getElementById('switchStatus').innerText = "Disabled";
        //  document.getElementById('switchStatus').style.color = "red";
      //  }
      //}
    };


    //document.getElementById('resetButton').onclick = function() {
    //  socket.send('RESET');
   // };

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

// Function to save parameters to SPIFFS
void saveParameters() {
  File file = SPIFFS.open("/params.csv", "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.printf("%.2f,%.2f,%.2f,%.2f\n", Setpoint, Kp, Ki, Kd);
  file.close();
  Serial.println("Parameters saved");
}

// Function to load parameters from SPIFFS
void loadParameters() {
  File file = SPIFFS.open("/params.csv", "r");
  if (!file) {
    Serial.println("No saved parameters found");
    return;
  }

  String configData = file.readStringUntil('\n');
  file.close();

  int comma1 = configData.indexOf(',');
  int comma2 = configData.indexOf(',', comma1 + 1);
  int comma3 = configData.indexOf(',', comma2 + 1);

  Setpoint = configData.substring(0, comma1).toDouble();
  Kp = configData.substring(comma1 + 1, comma2).toDouble();
  Ki = configData.substring(comma2 + 1, comma3).toDouble();
  Kd = configData.substring(comma3 + 1).toDouble();

  myPID.SetTunings(Kp, Ki, Kd); // Apply loaded PID values
  Serial.println("Parameters loaded");
}

// Function to apply LPF
float applyLowPassFilter(float newValue) {
  lastFilteredOutput = alpha * newValue + (1 - alpha) * lastFilteredOutput;
  return lastFilteredOutput;
}

// Ticker callback for managing temperature
void temperatureCallback() {


  
  Input = readTemperature(); // Update input with the latest temperature reading

  // Check if temperature is valid (not NaN, zero, or abnormal)
  if (isnan(Input) || Input <= 0.0 || Input > 250.0) { // Assumes maximum valid temperature is 250°C
    analogWrite(MOSFET_PIN, 0); // Turn off MOSFET if temperature is invalid
    resetPosition(); // Reset stepper motor position
    Serial.println("Temperature not detected or abnormal! MOSFET off and stepper motor reset.");
    webSocket.textAll("ERROR: Temperature not detected or abnormal!");
    return; // Exit without running PID
  }
  
  myPID.Compute(); // Compute PID output

  // Apply LPF to PID output
  float smoothedOutput = applyLowPassFilter(Output);

  analogWrite(MOSFET_PIN, constrain(smoothedOutput, 0, 255)); // Apply PID output to MOSFET
  webSocket.textAll("TEMP:" + String(Input));
}


void setup() {
  Serial.begin(115200);

  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
    return;
  }

  loadParameters(); // Load parameters from SPIFFS

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID, apPassword);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  webSocket.onEvent(onWebSocketEvent);
  server.addHandler(&webSocket);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", htmlPage);
  });

  server.begin();

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(maxSpeed / 2);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pinMode(SWITCH_PIN, INPUT);
  pinMode(MOSFET_PIN, OUTPUT); 
  myPID.SetMode(AUTOMATIC); // Set PID mode to automatic

  // Initialize Ticker to call temperatureCallback every 2 seconds
  temperatureTicker.attach_ms(temperatureUpdateInterval, temperatureCallback);
}

void loop() {
  webSocket.cleanupClients();
  manageStepper();
}

void manageStepper() {
  bool switchState = digitalRead(SWITCH_PIN) == LOW;
  static bool lastSwitchState = HIGH;

  if (switchState != lastSwitchState) {
    lastSwitchState = switchState;
    if (switchState) {
      webSocket.textAll("ENABLE");
      digitalWrite(ENABLE_PIN, LOW);
    } else {
      webSocket.textAll("DISABLE");
      digitalWrite(ENABLE_PIN, HIGH);
    }
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    resetPosition();
    delay(200);
  }

  long rawEncoderPosition = encoder.read();
  long scaledEncoderPosition = scaleEncoderValue(rawEncoderPosition);
  scaledEncoderPosition = roundToNearestScale(scaledEncoderPosition);
  scaledEncoderPosition = constrain(scaledEncoderPosition, sliderMin, sliderMax);

  if (abs(scaledEncoderPosition - lastEncoderPosition) >= stepInterval) {
    lastEncoderPosition = scaledEncoderPosition;

    if (digitalRead(ENABLE_PIN) == LOW) {
      stepper.setSpeed(map(scaledEncoderPosition, sliderMin, sliderMax, -maxSpeed, maxSpeed));
    }
  }

  stepper.runSpeed();
}



void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *payload, size_t length) 



{

  // Variable declaration moved outside of switch block
  String pidValues;

  switch(type) {
    case WS_EVT_DATA:
      if (strncmp((const char*)payload, "RESET", length) == 0) {
        resetPosition();
      } else if (strncmp((const char*)payload, "SET_TEMP:", 9) == 0) {
        Setpoint = atof((const char*)payload + 9); // Update set temperature
        saveParameters(); // Save parameter after update
        webSocket.textAll("SETPOINT:" + String(Setpoint));
      } else if (strncmp((const char*)payload, "SET_PID:", 8) == 0) {
        sscanf((const char*)payload + 8, "%lf,%lf,%lf", &Kp, &Ki, &Kd); // Update PID values
        myPID.SetTunings(Kp, Ki, Kd); // Apply new PID values
        saveParameters(); // Save parameters after update
        pidValues = String(Kp) + "," + String(Ki) + "," + String(Kd);
        webSocket.textAll("PID:" + pidValues);
      } else if (strncmp((const char*)payload, "ENABLE", length) == 0) {
        digitalWrite(ENABLE_PIN, LOW);
      } else if (strncmp((const char*)payload, "DISABLE", length) == 0) {
        digitalWrite(ENABLE_PIN, HIGH);
      } 
      break;

    case WS_EVT_CONNECT:
      Serial.println("Client connected");
      client->text("SETPOINT:" + String(Setpoint));
      pidValues = String(Kp) + "," + String(Ki) + "," + String(Kd);
      client->text("PID:" + pidValues);
      break;

    case WS_EVT_DISCONNECT:
      Serial.println("Client disconnected");
      break;

    case WS_EVT_ERROR:
      Serial.printf("WebSocket error %u\n", client->id());
      break;

    case WS_EVT_PONG:
      Serial.printf("WebSocket PONG received from client %u\n", client->id());
      break;
  }
}

void resetPosition() {
  encoder.write(0);
  stepper.setCurrentPosition(0);
  webSocket.textAll("0");
}
 
