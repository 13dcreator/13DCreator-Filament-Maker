/**
 * ================================================
 * THIRD-PARTY LIBRARIES (MULTI-LICENSE)
 * ================================================
 * This project uses:
/**
 * Third-party Libraries Attribution:
 * 
 * 1. ESP8266WiFi.h
 *    - Description: Library for managing WiFi functionality on ESP8266
 *    - License: GNU Lesser General Public License v2.1 or later (LGPL-2.1+)
 *    - Source: https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/ESP8266WiFi.h
 * 
 * 2. ESPAsyncWebServer.h
 *    - Description: Asynchronous Web Server library for ESP8266
 *    - License: GNU Lesser General Public License v2.1 or later (LGPL-2.1+)
 *    - Source: https://github.com/lacamera/ESPAsyncWebServer/blob/master/src/ESPAsyncWebServer.h
                or
                https://github.com/ESP32Async/ESPAsyncWebServer/blob/main/src/ESPAsyncWebServer.h
 * 
 * 3. ESPAsyncTCP.h
 *    - Description: Asynchronous TCP library for ESP8266
 *    - License: GNU Lesser General Public License v2.1 or later (LGPL-2.1+)
 *    - Source: https://github.com/dvarrel/ESPAsyncTCP/blob/master/src/ESPAsyncTCP.h
 * 
 * 4. PID_v1.h
 *    - Description: PID control algorithm implementation
 *    - License: MIT License
 *    - Source: https://github.com/br3ttb/Arduino-PID-Library
 *    - Reference: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * 
 * 5. EEPROM.h
 *    - Description: EEPROM emulation library for ESP8266
 *    - License: GNU Lesser General Public License v2.1 or later (LGPL-2.1+)
 *    - Source: https://github.com/esp8266/Arduino/blob/master/libraries/EEPROM/EEPROM.h
 */


#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PID_v1.h>
#include <EEPROM.h> 

/**
 * ================================================
 * ORIGINAL CODE (MIT License)
 * ================================================
 * ESP8266 PID Temperature Controller with Web Interface
 * Copyright (c) 2025 13dcreator 
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */



#define TEMP_OFFSET 0.0  // Ubah nilanya sesuai kalibrasi. Jika suhu terbaca lebih besar maka bisa dikurangi dan jika lebih kecil bisa ditambah. Contoh +15.0 atau -15.0

// WiFi Configuration
const char* ssid = "13DC Temperature V2.0";
const char* password = "13dcreator";

// Hardware Configuration
const int pwmPin = D6;
const int ntcPin = A0;
const float R1 = 950; //resistor
const float BETA = 3950;
const float T0 = 298.15;
const float R0 = 100000;


// PID Variables
double Setpoint, Input, Output;
double Kp, Ki, Kd;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Safety
bool safetyTriggered = false;
String safetyMessage = "";
const float MAX_SAFE_TEMP = 310.0;
const float MIN_SAFE_TEMP = -50.0;


// EEPROM Configuration
struct Settings {
  double setpoint;
  double kp;
  double ki;
  double kd;
};
const int EEPROM_SIZE = sizeof(Settings);

// Web Server
AsyncWebServer server(80);

// ==============================================
// HTML Page with Auto-filled Form Values
// ==============================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Temperature Controller</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background-color: #f5f5f5; margin: 0; padding: 20px; }
    .container { max-width: 500px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
    h1 { color: #333; }
    .data-panel { margin: 20px 0; padding: 15px; background: #f0f0f0; border-radius: 5px; }
    .temp-value { font-size: 36px; font-weight: bold; color: #e74c3c; }
    .pwm-value { font-size: 24px; color: #3498db; }
    form { margin: 20px 0; text-align: left; }
    label { display: block; margin: 10px 0 5px; }
    input { width: 100%; padding: 8px; margin-bottom: 10px; box-sizing: border-box; }
    button { background: #2ecc71; color: white; border: none; padding: 10px 15px; border-radius: 5px; cursor: pointer; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Temperature Controller</h1>
    
    <div class="data-panel">
      <div>Current Temperature:</div>
      <div id="errorMsg" style="color:red; margin:10px 0;"></div>
      <div class="temp-value" id="currentTemp">--</div>
      <div>PWM Output: <span class="pwm-value" id="pwmOutput">0</span>/255</div>
    </div>

    <form id="pidForm">
      <label for="setpoint">Target Temperature (°C):</label>
      <input type="number" id="setpoint" step="0.1" min="0" max="310" required>
      
      <label for="kp">Kp (Proportional):</label>
      <input type="number" id="kp" step="0.1" required>
      
      <label for="ki">Ki (Integral):</label>
      <input type="number" id="ki" step="0.1" required>
      
      <label for="kd">Kd (Derivative):</label>
      <input type="number" id="kd" step="0.1" required>
      
      <button type="submit">Update Settings</button>
    </form>




</div>

  </div>

  <script>

     function updateDisplay() {
      fetch('/data')
        .then(r => r.json())
        .then(data => {
          if(data.error) {
            document.getElementById("errorMsg").textContent = data.error;
            document.getElementById("currentTemp").style.color = "red";
            document.getElementById("pwmOutput").textContent = "0";
          } else {
            document.getElementById("currentTemp").textContent = data.temp.toFixed(1);
            document.getElementById("currentTemp").style.color = "";
            document.getElementById("pwmOutput").textContent = data.pwm;
            document.getElementById("errorMsg").textContent = "";
          }
        });
    }
    setInterval(updateDisplay, 1000);


    // Load saved settings on page load
    function loadSettings() {
      fetch('/getsettings')
        .then(response => response.json())
        .then(settings => {
          document.getElementById('setpoint').value = settings.sp.toFixed(1);
          document.getElementById('kp').value = settings.kp.toFixed(1);
          document.getElementById('ki').value = settings.ki.toFixed(1);
          document.getElementById('kd').value = settings.kd.toFixed(1);
        });
    }

    // Update data every second
    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('currentTemp').textContent = data.temp.toFixed(1);
          document.getElementById('pwmOutput').textContent = data.pwm;
        });
    }

    // Handle form submission
    document.getElementById('pidForm').addEventListener('submit', function(e) {
      e.preventDefault();
      const setpoint = parseFloat(document.getElementById('setpoint').value);
      const kp = parseFloat(document.getElementById('kp').value);
      const ki = parseFloat(document.getElementById('ki').value);
      const kd = parseFloat(document.getElementById('kd').value);
      
      fetch(`/setpid?sp=${setpoint}&kp=${kp}&ki=${ki}&kd=${kd}`)
        .then(response => {
          if (response.ok) {
            alert('Settings updated!');
            // Broadcast to all clients
            updateData();
          }
        });
    });

    // Initial load
    loadSettings();
    setInterval(updateData, 1000);
  </script>
</body>
</html>
)rawliteral";

// ==============================================
// EEPROM Functions
// ==============================================
void saveSettings() {
  Settings settings = { Setpoint, Kp, Ki, Kd };
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, settings);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("Settings saved to EEPROM");
}

void loadSettings() {
  Settings settings;
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, settings);
  EEPROM.end();

  // Default values if EEPROM is empty
  if (isnan(settings.setpoint)) {
    settings = { 60.0, 7.0, 1.0, 7.0 };
    Serial.println("Loaded default settings");
  } else {
    Serial.println("Loaded settings from EEPROM");
  }

  Setpoint = settings.setpoint;
  Kp = settings.kp;
  Ki = settings.ki;
  Kd = settings.kd;
  myPID.SetTunings(Kp, Ki, Kd);
}

// ==============================================
// Temperature Reading
// ==============================================
float readTemperature() {
  int analogValue = analogRead(ntcPin);
  
  // Error handling pembacaan
  if (analogValue == 0 || analogValue == 1023) {
    safetyTriggered = true;
    safetyMessage = "ERROR: Pembacaan sensor tidak valid!";
    return NAN; // Nilai tidak valid
  }

  float resistance = R1 * (1023.0 / analogValue - 1);
  float temperature = 1.0 / (log(resistance / R0) / BETA + 1.0 / T0) - 273.15;

// Koreksi sederhana
  temperature = temperature + TEMP_OFFSET;   // atau: temperature = temperature * TEMP_SCALE;


  // Cek suhu tidak wajar
  if (temperature > MAX_SAFE_TEMP || temperature < MIN_SAFE_TEMP) {
    safetyTriggered = true;
    safetyMessage = "WARNING: Suhu di luar batas aman!";
    return temperature;
  }

  safetyTriggered = false;
  safetyMessage = "";
  return temperature;
}

// ==============================================
// Setup Function
// ==============================================
void setup() {
  Serial.begin(115200);
  
  // Initialize PWM
  pinMode(pwmPin, OUTPUT);
  analogWriteRange(255);
  
  // Load PID settings from EEPROM
  loadSettings();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 200); //Pengaturan Max pwm
  
  // Start Access Point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // Web Server Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String json;
    if(safetyTriggered || isnan(Input)) {
      json = "{\"error\":\"" + safetyMessage + "\"}";
    } else {
      json = String() + "{\"temp\":" + Input + ",\"pwm\":" + (int)Output + "}";
    }
    request->send(200, "application/json", json);
  });

  server.on("/getsettings", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = String() + "{\"sp\":" + Setpoint + ",\"kp\":" + Kp + ",\"ki\":" + Ki + ",\"kd\":" + Kd + "}";
    request->send(200, "application/json", json);
  });

server.on("/setpid", HTTP_GET, [](AsyncWebServerRequest *request) {
  // Validasi Setpoint
  if (request->hasParam("sp")) {
    float newSetpoint = request->getParam("sp")->value().toFloat();
    if (newSetpoint > MAX_SAFE_TEMP) {
      request->send(400, "text/plain", "Error: Setpoint melebihi batas maksimal 310°C");
      return; // Hentikan proses jika invalid
    }
    Setpoint = newSetpoint; // Simpan jika valid
  }

  // Proses parameter PID lainnya
  if (request->hasParam("kp")) Kp = request->getParam("kp")->value().toFloat();
  if (request->hasParam("ki")) Ki = request->getParam("ki")->value().toFloat();
  if (request->hasParam("kd")) Kd = request->getParam("kd")->value().toFloat();
  
  myPID.SetTunings(Kp, Ki, Kd);
  saveSettings();  // Simpan ke EEPROM
  request->send(200, "text/plain", "OK");
});
  server.begin();
}

// ==============================================
// Main Loop
// ==============================================
void loop() {
  Input = readTemperature();
  
  // 1. Cek kondisi bahaya terlebih dahulu
  if (safetyTriggered || isnan(Input)) {
    analogWrite(pwmPin, 0); // Matikan MOSFET
    Serial.println("🔴 SAFETY TRIGGERED: " + safetyMessage);
  } 
  else {
    myPID.Compute();
    analogWrite(pwmPin, Output); // Operasi normal
  }

  // 2. Reset safety status JIKA kondisi sudah normal kembali
  if (!safetyTriggered && !isnan(Input) && Input <= MAX_SAFE_TEMP && Input >= MIN_SAFE_TEMP) {
    safetyTriggered = false;
    safetyMessage = ""; // Clear pesan error
  }

  // 3. Logging data
  Serial.printf("Temp: %.1f°C | Setpoint: %.1f°C | PWM: %d\n", Input, Setpoint, (int)Output);
  delay(1000);
}
