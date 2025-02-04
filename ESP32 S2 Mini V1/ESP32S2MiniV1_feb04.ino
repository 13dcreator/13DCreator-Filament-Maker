// Some libraries used in this project may be licensed under different licenses. 
// Please refer to the LICENSE file for more information about those libraries.

#include <WiFi.h> //LGPL 2.1 or later
#include <ESPAsyncWebServer.h> //LGPL 2.1 or later
#include <AsyncTCP.h> //LGPL-3.0
#include <PID_v1.h> //MIT
#include <EEPROM.h> //LGPL 2.1 or later
#include "FastAccelStepper.h" //MIT
#include <Encoder.h> //MIT
#include <Adafruit_GFX.h> //BSD License
#include <Adafruit_SSD1306.h> ////BSD License


/*
 * The code below is licensed under the MIT License.
 * 
 * MIT License
 * 
 * Copyright (c) 2025 13DCreator
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





// OLED Settings
#define SDA_PIN 33
#define SCL_PIN 35
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Definisi pin untuk motor stepper
#define STEP_PIN 9
#define DIR_PIN 11
#define ENABLE_PIN 12

// Definisi pin untuk rotary encoder dan tombol
#define ENCODER_CLK 18
#define ENCODER_DT 37
#define BUTTON_PIN 39

// Definisi pin untuk switch
#define SWITCH_PIN 21

#define LED_PIN 15

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

Encoder encoder(ENCODER_CLK, ENCODER_DT);

const float defaultEncoderScale = 5.0;
float encoderScale = defaultEncoderScale;

long lastEncoderPosition = 0;
long lastSpeed = 0;
const int maxSpeed = 10000;
const int acceleration = 3000;
const int SpeedInHz = 1000;

#define THERMISTOR_PIN 1
#define MOSFET_PIN 13

const float R1 = 1000; //Resistor 1K
const float BETA = 3950;
const float T0 = 298.15;
const float R0 = 100000;

float currentTemperature = 0.0;
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 2000; //Update Interval

double Setpoint = 60.0;
double Input, Output;
double Kp = 5, Ki = 0.4, Kd = 5;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

AsyncWebServer server(80);

unsigned long motorTimerStart  = 0;       // Timer start time
unsigned long totalRunTime = 0;     // Total run time in milliseconds
bool motorRunning = false;          // Motor movement status

// Variabel untuk mendeteksi durasi tombol encoder
unsigned long buttonPressStartTime = 0;
bool buttonPressed = false;

//variabel pengaturan suhu
bool settingMode = false;      // Apakah dalam mode pengaturan
float tempSetpointDraft = 60;  // Draft setpoint suhu sementara


float readTemperature() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  int analogValue = analogRead(THERMISTOR_PIN);
  if (analogValue <= 0) {
    return -1.0;
  }

  float resistance = R1 * (4095.0 / analogValue - 1);
  float temperature = 1.0 / (log(resistance / R0) / BETA + 1.0 / T0) - 273.15;
  return temperature;
}

void saveSettings() {
  EEPROM.begin(512);
  EEPROM.put(0, Setpoint);
  EEPROM.put(sizeof(Setpoint), Kp);
  EEPROM.put(sizeof(Setpoint) + sizeof(Kp), Ki);
  EEPROM.put(sizeof(Setpoint) + sizeof(Kp) + sizeof(Ki), Kd);
  EEPROM.commit();
  EEPROM.end();
}

void loadSettings() {
  EEPROM.begin(512);
  EEPROM.get(0, Setpoint);
  EEPROM.get(sizeof(Setpoint), Kp);
  EEPROM.get(sizeof(Setpoint) + sizeof(Kp), Ki);
  EEPROM.get(sizeof(Setpoint) + sizeof(Kp) + sizeof(Ki), Kd);
  EEPROM.end();
}

void setup() {
  Serial.begin(115200);

  // Setup WiFi sebagai Access Point
  WiFi.softAP("13DC .....", "13dcreator");
  Serial.println("Access Point Created");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Muat pengaturan dari EEPROM saat boot
  loadSettings();

  // Setup untuk kontrol stepper
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENABLE_PIN);
    stepper->setAutoEnable(true);
    stepper->setSpeedInHz(SpeedInHz);
    stepper->setAcceleration(acceleration);
    Serial.println("Stepper initialized");
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Setup OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED initialization failed"));
    while (true);
  }
  display.clearDisplay();


// Teks "13DC"
  display.setTextSize(3);  // Ukuran font 3
  display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("13DC", 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 3 + 5);  // Center horizontally
  display.print("13DC");

  // Teks "000"
  display.setTextSize(2);  // Ukuran font 2
  display.getTextBounds(".....", 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, ((SCREEN_HEIGHT - h) / 3) + 30);  // Center horizontally, di bawah "13DC"
  display.print(".....");

  display.display();
  delay(5000);  // Delay untuk menampilkan tulisan sebelum melanjutkan



    // Setup server web
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><head><style>";
    html += "body { background-color: #212121; font-family: Arial, sans-serif; color: white; text-align: center; margin: 0; padding: 0; display: flex; justify-content: center; align-items: center; height: 100vh; }";
    html += "h1 { color: #ff9800; font-size: 36px; }";
    html += "p { font-size: 20px; margin-bottom: 20px; }";
    html += "input { padding: 10px; font-size: 16px; margin: 5px 0; }";
    html += "button { background-color: #ff9800; color: white; border: none; padding: 10px 20px; cursor: pointer; font-size: 18px; }";
    html += "button:hover { background-color: #f57c00; }";
    html += "</style></head><body>";
    html += "<div><h1>Temperature Control</h1>";
    html += "<p>Current Temperature: <span id='tempValue'>...</span> &deg;C</p>";
    html += "<p>Setpoint: <input type='number' id='setpoint' value='" + String(Setpoint) + "' /></p>";
    html += "<p>Kp: <input type='number' id='kp' value='" + String(Kp) + "' step='0.1' /></p>";
    html += "<p>Ki: <input type='number' id='ki' value='" + String(Ki) + "' step='0.1' /></p>";
    html += "<p>Kd: <input type='number' id='kd' value='" + String(Kd) + "' step='0.1' /></p>";
    html += "<button onclick='updateSettings()'>Update</button>";

    // Menampilkan informasi status
    html += "<h2>Motor Speed Hz: <span id='speed'>...</span></h2>";
    html += "<h2>Stepper Status: <span id='stepperStatus'>OFF</span></h2>";
    html += "<h2>Timer: <span id='timer'>00:00:00</span></h2>";

    // Tombol untuk reset timer
    html += "<button onclick='resetTimer()'>Reset Timer</button>";
    
    html += "<script>";
    html += "function updateSettings() {";
    html += "  var setpoint = document.getElementById('setpoint').value;";
    html += "  var kp = document.getElementById('kp').value;";
    html += "  var ki = document.getElementById('ki').value;";
    html += "  var kd = document.getElementById('kd').value;";
    html += "  fetch('/setSettings?setpoint=' + setpoint + '&kp=' + kp + '&ki=' + ki + '&kd=' + kd);";
    html += "}";

    html += "function resetTimer() {";
    html += "  fetch('/resetTimer');";
    html += "}";
    html += "setInterval(function(){";
    html += "  fetch('/temperature').then(response => response.text()).then(data => {";
    html += "    document.getElementById('tempValue').innerText = data;";
    html += "  });";
    html += "  fetch('/speed').then(response => response.text()).then(data => {";
    html += "    document.getElementById('speed').innerText = data;";
    html += "  });";
    html += "  fetch('/stepperStatus').then(response => response.text()).then(data => {";
    html += "    document.getElementById('stepperStatus').innerText = data;";
    html += "  });";
    html += "  fetch('/timer').then(response => response.text()).then(data => {";
    html += "    document.getElementById('timer').innerText = data;";
    html += "  });";
    html += "}, 1000);";  // Update every second
    html += "</script>";
    html += "</div></body></html>";
    request->send(200, "text/html", html);
});


server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(currentTemperature));
  });

  server.on("/setSettings", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("setpoint")) {
      Setpoint = request->getParam("setpoint")->value().toDouble();
    }
    if (request->hasParam("kp")) {
      Kp = request->getParam("kp")->value().toDouble();
    }
    if (request->hasParam("ki")) {
      Ki = request->getParam("ki")->value().toDouble();
    }
    if (request->hasParam("kd")) {
      Kd = request->getParam("kd")->value().toDouble();
    }
    saveSettings();
    myPID.SetTunings(Kp, Ki, Kd);
    request->send(200, "text/plain", "Settings Updated");
  });



  server.on("/resetTimer", HTTP_GET, [](AsyncWebServerRequest *request){
    motorTimerStart = 0;  // Reset the motor timer
    totalRunTime = 0;     // Reset total run time
    request->send(200, "text/plain", "Timer Reset");
});

server.on("/speed", HTTP_GET, [](AsyncWebServerRequest *request){
    String speedStr = String(lastSpeed);
    request->send(200, "text/plain", speedStr);
});

server.on("/stepperStatus", HTTP_GET, [](AsyncWebServerRequest *request){
    String stepperStatus = (digitalRead(SWITCH_PIN) == HIGH) ? "OFF" : "ON";
    request->send(200, "text/plain", stepperStatus);
});

server.on("/timer", HTTP_GET, [](AsyncWebServerRequest *request){
    unsigned long elapsedTime = totalRunTime;
    if (motorRunning) {
        elapsedTime += millis() - motorTimerStart;
    }
    unsigned long hours = (elapsedTime / 3600000);
    unsigned long minutes = (elapsedTime / 60000) % 60;
    unsigned long seconds = (elapsedTime / 1000) % 60;
    String timerStr = String(hours) + ":" + String(minutes) + ":" + String(seconds);
    request->send(200, "text/plain", timerStr);
});



  server.begin();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  static bool encoderButtonPressed = false;
  bool buttonState = digitalRead(BUTTON_PIN) == LOW;

  // Reset encoder position when button is pressed
  if (buttonState && !encoderButtonPressed && !settingMode) {
    encoder.write(0);
    if (stepper) {
      stepper->stopMove();
    }
    encoderButtonPressed = true;
  }

  if (!buttonState && encoderButtonPressed) {
    encoderButtonPressed = false;
  }

  bool switchState = digitalRead(SWITCH_PIN);
  static bool lastSwitchState = LOW;  // Track previous switch state


if (!settingMode) {
  if (switchState == HIGH) {
    // Stop motor when switch is high
    if (stepper) {
      stepper->stopMove();
    }
    // Stop the timer when switch is high, but don't reset it
    if (motorRunning) {
      totalRunTime += millis() - motorTimerStart;
      motorTimerStart = 0;
      motorRunning = false;
    }
  } else {
    // If switch was previously HIGH and now is LOW, start or continue the timer
    if (lastSwitchState == HIGH && switchState == LOW) {
      motorTimerStart = millis();  // Restart the timer when switch goes LOW again
      motorRunning = true;

          // Langsung atur kecepatan motor
    if (lastSpeed != 0 && stepper) {
        stepper->setSpeedInHz(abs(lastSpeed));
        if (lastSpeed > 0) {
            stepper->runForward();
        } else {
            stepper->runBackward();
        }
    }



    }
    // Only run motor and timer when switch is low
    long scaledEncoderPosition = encoder.read() * encoderScale;
    if (scaledEncoderPosition != lastEncoderPosition) {
      long speed = map(scaledEncoderPosition, -500, 500, -maxSpeed, maxSpeed);
      speed = constrain(speed, -maxSpeed, maxSpeed);
      if (stepper) {
        if (speed > 0) {
          stepper->setSpeedInHz(speed);
          stepper->runForward();
          motorRunning = true;  // Start timer when motor is moving
          if (motorTimerStart == 0) {
            motorTimerStart = millis();  // Start the timer when the motor begins
          }
        } else if (speed < 0) {
          stepper->setSpeedInHz(-speed);
          stepper->runBackward();
          motorRunning = true;  // Start timer when motor is moving
          if (motorTimerStart == 0) {
            motorTimerStart = millis();  // Start the timer when the motor begins
          }
        } else {
          stepper->stopMove();
          motorRunning = false;  // Stop timer when motor stops
          totalRunTime += millis() - motorTimerStart;
          motorTimerStart = 0;
        }
      }
      lastSpeed = speed;
      lastEncoderPosition = scaledEncoderPosition;
    } else {
      if (stepper && lastSpeed != 0) {
        stepper->setSpeedInHz(lastSpeed);
        if (lastSpeed > 0) {
          stepper->runForward();
        } else if (lastSpeed < 0) {
          stepper->runBackward();
        }
      }
    }
  }
}


// Di dalam loop() saat dalam mode pengaturan
if (settingMode) {
    long encoderPosition = encoder.read();
    tempSetpointDraft = Setpoint + encoderPosition;  // Skala encoder ke setpoint
    tempSetpointDraft = constrain(tempSetpointDraft, 0, 260); // Batas suhu
    //encoder.write(0);  // Reset encoder setelah perubahan

    // Konfirmasi pengaturan suhu jika tombol ditekan
    if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
        Setpoint = tempSetpointDraft; // Update Setpoint dengan nilai draft
        saveSettings();  // Simpan ke EEPROM
        myPID.SetTunings(Kp, Ki, Kd); // Update PID tunings
        settingMode = false;  // Keluar dari mode pengaturan
        encoder.write(0);  // Reset encoder ke posisi awal
        delay(500);  // Hindari input ganda
    }

    // Kondisi keluar dari mode pengaturan jika tombol ditekan lagi
    if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed && millis() - buttonPressStartTime > 1000) {
        settingMode = false;  // Keluar dari mode pengaturan
        delay(500);  // Hindari input ganda
    }

} else {
    // Jika tidak dalam mode pengaturan, cek apakah tombol ditekan selama 1 detik untuk masuk mode pengaturan
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (!buttonPressed) {
            buttonPressStartTime = millis();  // Mulai hitung waktu tombol ditekan
            buttonPressed = true;
        }

        // Masuk ke mode pengaturan jika tombol ditekan selama 1 detik
        if (millis() - buttonPressStartTime >= 1000) {
            settingMode = !settingMode;  // Toggle mode pengaturan
            tempSetpointDraft = Setpoint; // Muat setpoint saat ini ke draft
            delay(500);  // Hindari input ganda
        }
    } else {
        buttonPressed = false;
    }
}




  // Handle long press on button for 2 seconds to reset timer
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!buttonPressed) {
      buttonPressStartTime = millis();  // Start timing button press
      buttonPressed = true;
    }
  } else {
    if (buttonPressed && millis() - buttonPressStartTime >= 5000) {
      motorTimerStart = 0;  // Reset the motor timer when button is held for 2 seconds
      totalRunTime = 0;     // Reset total run time
      Serial.println("Timer Reset");
    }
    buttonPressed = false;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateTime >= updateInterval) {
    currentTemperature = readTemperature();

    if (currentTemperature > 260.0 || isnan(currentTemperature) || currentTemperature <= 0.0) {
      digitalWrite(ENABLE_PIN, HIGH);
      encoder.write(0);
      if (stepper) {
        stepper->stopMove();
      }

      analogWrite(MOSFET_PIN, 0);
    } else {
      Input = currentTemperature;
      myPID.Compute();
      analogWrite(MOSFET_PIN, Output);
      digitalWrite(ENABLE_PIN, LOW);

      if (stepper) {
        stepper->enableOutputs();
      }
    }

    // Update OLED display
if (settingMode) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Adjust Setpoint");

  display.setTextSize(2);
  display.setCursor(20, 20);
  display.print(tempSetpointDraft, 1);
  display.println(" C");

  display.setTextSize(1);
  display.setCursor(0, 50);
  display.println("Press to Confirm");
  display.display();
} else {
  // Lanjutkan dengan tampilan normal




    display.clearDisplay();
    display.setTextSize(0.7);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 8);
    display.print("Set Temp: ");
    display.println(Setpoint);

    display.setTextSize(2.5);
    display.setCursor(32, 20);
    display.print(currentTemperature, 1);
    display.println("C");

    display.setTextSize(1);
    display.setCursor(0, 42);
    display.print("SpeedHz: ");
    display.println(lastSpeed);

    // Display stepper enable status
    display.setTextSize(0.7);
    display.setCursor(0, 0);
    if (switchState == HIGH) {
      display.print("Stepper: OFF");
    } else {
      display.print("Stepper: ON");
    }

    // Display timer
    unsigned long elapsedTime = totalRunTime;
    if (motorRunning) {
      elapsedTime += currentMillis - motorTimerStart;
    }
    unsigned long hours = (elapsedTime / 3600000);
    unsigned long minutes = (elapsedTime / 60000) % 60;
    unsigned long seconds = (elapsedTime / 1000) % 60;
    display.setCursor(0, 55);
    display.print("Time: ");
    display.print(hours);
    display.print(":");
    display.print(minutes);
    display.print(":");
    display.println(seconds);

    display.display();
    lastUpdateTime = currentMillis;
  }

  }

  // Update lastSwitchState for next loop iteration
  lastSwitchState = switchState;
}
