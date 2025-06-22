#include <WiFi.h>
#include <WebServer.h>
#include <Keypad.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/ledc.h> 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// Constants and Defines
#define DEVICE_DISCONNECTED_C -127
#define CURTAIN_CLOSED 0
#define CURTAIN_OPENED 1
#define DOOR_CLOSED 0
#define DOOR_OPENED 1
#define SERVO_LOCK_POS 0
#define SERVO_UNLOCK_POS 90
#define MOTOR_RUN_TIME 2000
#define LDR_DARK_THRESHOLD 2000
#define LDR_LIGHT_THRESHOLD 2000
#define SERVO_MIN_PULSE 500  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSE 2500 // Maximum pulse width in microseconds
#define SERVO_LOCK_POS 90    // Door locked position
#define SERVO_UNLOCK_POS 180 // Door unlocked position
#define CURTAIN_OPENED 0
#define CURTAIN_CLOSED 1
#define CURTAIN_OPENING 2  
#define CURTAIN_CLOSING 3  
#define CURTAIN_STOPPED 0
#define CURTAIN_OPENING 1
#define CURTAIN_CLOSING 2
#define CURTAIN_OPENED 3
#define CURTAIN_CLOSED 4
#define LIGHT_ON HIGH  
#define LIGHT_OFF LOW
#define SAFETY_DISTANCE 15     // Block door if object <12cm
#define SAFETY_TIMEOUT 3000   
// Pin Definitions
bool smokeDetected = false;
const int trigPin = 17;  // GPIO17 for trigger
const int echoPin = 19;  // GPIO19 for echo
const int lightRelayPin = 13;  // Using GPIO 13 for light relay
bool lightState = false;
const int servoPin = 15;
const int motorPin1 = 4;
const int motorPin2 = 16;
const int FanPin = 12;
const int ldrPin = 36;
const int buzzer_pin = 5;
const int oneWireBus = 18;
const int pwmChannel = LEDC_CHANNEL_0;       // LEDC channel (0-15)
const int pwmFrequency = 50;   // 50Hz for standard servos
const int pwmResolution = 16;  // 16-bit resolution (0-65535)
const int minAngle = 0;        // 0 degrees
const int maxAngle = 180;      // 180 degrees
const int minPulseWidth = 500;  // 0° position (500µs)
const int maxPulseWidth = 2500;
bool safetyBlockActive = false;
unsigned long lastSafetyTrigger = 0; // 180° position (2500µs)
// System States
int curtainState = CURTAIN_OPENED;
int doorState = DOOR_CLOSED;
bool fanState = false;
bool autonomyEnabled = true;
bool showTempFanScreen = true;
unsigned long curtainActionStart = 0;
bool curtainMoving = false;
int curtainDirection = 0; // 0=stopped, 1=opening, -1=closing
// WiFi and Server
const char* ssid = "Mostafa Mesbah"; // or use Orange-cocowawa
const char* password = "mostafa010"; // or use #010mDo010#
unsigned long doorActionTime = 0;
bool doorMoving = false;
int targetDoorState = DOOR_CLOSED;
enum BuzzerState { BUZZER_IDLE, PLAYING_TONE, IN_SEQUENCE, IN_LOOP };
BuzzerState buzzerState = BUZZER_IDLE;

unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 0;
unsigned long buzzerPauseDuration = 0;
int buzzerFrequency = 0;
int buzzerLoopCount = 0;
int currentLoop = 0;

struct ToneSequence {
  int frequency;
  int duration;
  int pause;
};
ToneSequence currentSequence[10];
int sequenceLength = 0;
int sequencePosition = 0;
WebServer server(80);

// Keypad Setup
const byte ROWS = 4, COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {14, 27, 26, 25};
byte colPins[COLS] = {33, 32, 23};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
String enteredPassword = "";
const String passwordAtef = "1234";  // Dr. Atef's password
const String passwordAhmed = "5678"; // Dr. Ahmed's password
const String passwordKarim = "9012"; // Dr. Karim's password
unsigned long lastDoorActionTime = 0;
// Components

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd1(0x27, 16, 2);
LiquidCrystal_I2C lcd2(0x26, 16, 2);

// Timing Variables
unsigned long lastKeyPressTime = 0;
unsigned long lastLCDUpdate = 0;
const unsigned long debounceTime = 50;
const int mq2DigitalPin = 35;  // Digital output pin (D0)
unsigned long lastSmokeCheck = 0;
const unsigned long smokeCheckInterval = 500;  // Check every 1 second

void checkSmoke();
void checkDoorAutoClose();
void initializeServo();
void setupServoPWM();
void setServoAngle(int angle);
void playTone(int frequency, int duration, int pause = 0);


void setup() {
  Serial.begin(115200);
  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Chip Cores: %d\n", ESP.getChipCores());
  initializeHardware();
  connectToWiFi();
  setupWebServer();
  initializeSensors();
  initializeDisplay();
  startupBeep();
}

void loop() {
  checkSmoke();
  updateBuzzer();
  updateCurtainMovement();
  updateDoorMovement();
  server.handleClient();
  handleAutonomousCurtains();
  handleKeypadInput();
  checkTemperature();
  updateLCD();
  //checkDoorAutoClose();
}

// ========== INITIALIZATION FUNCTIONS ==========
void initializeHardware() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  pinMode(lightRelayPin, OUTPUT);
  digitalWrite(lightRelayPin, LIGHT_OFF); // Start with light off
  pinMode(buzzer_pin, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(FanPin, OUTPUT);
  stopMotor();
  digitalWrite(FanPin, LOW);
  setupServoPWM(); // Initialize PWM for servo
  setServoAngle(SERVO_LOCK_POS); // Start in locked position
  pinMode(mq2DigitalPin, INPUT);
  delay(1000); // Allow servo to initialize



}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
    successBeep();
  } else {
    Serial.println("\nWiFi connection failed");
    errorBeep();
  }
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/toggle-autonomy", toggleAutonomy);
  server.on("/open-curtains", openCurtainsManually);
  server.on("/close-curtains", closeCurtainsManually);
  server.on("/open-door", openDoorManually);
  server.on("/close-door", closeDoorManually);
  server.on("/fan-on", turnFanOn);
  server.on("/fan-off", turnFanOff);
  server.on("/getTemperature", handleTemperatureRequest);
  server.on("/light-on", turnLightOn);
  server.on("/light-off", turnLightOff);
  server.begin();
  Serial.println("HTTP server started");
}

void initializeSensors() {
  sensors.begin();
}

void initializeDisplay() {
  lcd1.init();
  lcd1.backlight();
  lcd2.init();
  lcd2.backlight();
  
  // Display startup messages
  lcd1.setCursor(0, 0);
  lcd1.print("Smart Curtains");
  lcd1.setCursor(0, 1);
  lcd1.print("System Ready");
  
  lcd2.setCursor(0, 0);
  lcd2.print("Enter Password:");
  lcd2.setCursor(0, 1);
  lcd2.print("");
}

// ========== CORE FUNCTIONALITY ==========
void handleAutonomousCurtains() {
  if (autonomyEnabled) {
    int ldrValue = analogRead(ldrPin);
    Serial.print("LDR Value: ");
    Serial.println(ldrValue);
    
    if (ldrValue < LDR_DARK_THRESHOLD && curtainState == CURTAIN_OPENED) {
      closeCurtains();
      turnLightOn();
      curtainState = CURTAIN_CLOSED;
    } else if (ldrValue > LDR_LIGHT_THRESHOLD && curtainState == CURTAIN_CLOSED) {
      openCurtains();
      turnLightOff();
      curtainState = CURTAIN_OPENED;
    }
  }
}

void handleKeypadInput() {
  char key = keypad.getKey();
  if (key) {
    unsigned long currentTime = millis();
    
    // Skip debounce check for '#' and '*' as they're critical
    if ((key == '#' || key == '*') || (currentTime - lastKeyPressTime > debounceTime)) {
      lastKeyPressTime = currentTime;
      shortBeep();
      
      // Visual feedback on LCD2
      lcd2.setCursor(enteredPassword.length(), 1);
      lcd2.print("*");  // Show immediate feedback
      
      if (key == '#') {
        handleHashKey();
      } else if (key == '*') {
        handleStarKey();
      } else {
        handleNumericKey(key);
      }
    }
  }
}

void handleHashKey() {
  lcd2.clear();
  lcd2.setCursor(0, 0);

  // Check if someone is standing at the door
  if (checkSafety()) {
    lcd2.print("SAFETY ALERT!");
    lcd2.setCursor(0, 1);
    lcd2.print("CLEAR ENTRANCE");
    delay(2000); // Show message for 2 seconds
    lcd2.clear();
    resetPasswordDisplay();
    return; // Exit the function if safety block is active
  }

  // Password validation
  if (enteredPassword == passwordAtef) {
    successBeep();
    toggleDoorState();
    lcd2.clear();
    lcd2.print("DR. ATEF");
    lcd2.setCursor(0, 1);
    lcd2.print("ACCESS GRANTED");
    delay(2000); // Show message for 2 seconds
    lcd2.clear();
    resetPasswordDisplay();
  } 
  else if (enteredPassword == passwordAhmed) {
    successBeep();
    toggleDoorState();
    lcd2.clear();
    lcd2.print("DR. AHMED");
    lcd2.setCursor(0, 1);
    lcd2.print("ACCESS GRANTED");
    delay(2000); // Show message for 2 seconds
    lcd2.clear();
    resetPasswordDisplay();
  }
  else if (enteredPassword == passwordKarim) {
    successBeep();
    toggleDoorState();
    lcd2.clear();
    lcd2.print("DR. KARIM");
    lcd2.setCursor(0, 1);
    lcd2.print("ACCESS GRANTED");
    delay(2000); // Show message for 2 seconds
    lcd2.clear();
    resetPasswordDisplay();
  }
  else {
    errorBeep();
    lcd2.clear();
    lcd2.print("ACCESS DENIED");
    lcd2.setCursor(0, 1);
    lcd2.print("TRY AGAIN");
    delay(2000); // Show message for 2 seconds
    lcd2.clear();
    resetPasswordDisplay();
  }
}

void handleStarKey() {
  enteredPassword = "";
  lcd2.clear();
  Serial.println("Input cleared");
  delay(1000);
  lcd2.clear();
  resetPasswordDisplay();
}

void handleNumericKey(char key) {
  if (enteredPassword.length() < 4) {
    enteredPassword += key;
    // Only update the specific character position
    lcd2.setCursor(enteredPassword.length()-1, 1);
    lcd2.print("*");
    Serial.print("*");
  } else {
    errorBeep();
  }
}

void resetPasswordDisplay() {
  enteredPassword = "";
  lcd2.clear();
  lcd2.setCursor(0, 0);
  lcd2.print("Enter Password:");
  lcd2.setCursor(0, 1);
  lcd2.print("");  // Reset to empty password
}

void updatePasswordDisplay() {
  lcd2.setCursor(0, 1);
  for (int i = 0; i < 4; i++) {
    lcd2.print(i < enteredPassword.length() ? "*" : "_");
  }
}
void checkTemperature() {
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);
  
  if (temp == DEVICE_DISCONNECTED_C) {
    Serial.println("Temperature sensor error!");
    errorBeep();
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" °C");

  if (autonomyEnabled) {
    const float turnOnTemp = 30.0;
    const float turnOffTemp = 30.0;
    if (temp >= turnOnTemp && !fanState) {
      setFanState(true);
    } else if (temp <= turnOffTemp && fanState) {
      setFanState(false);
    }
  }
}

void handleRoot() {
 String html = R"rawliteral(
 <!DOCTYPE html>
 <html>
 <head>
    <title>Smart Room Control</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f5f5f5;
        }
        h1 {
            color: #2c3e50;
            text-align: center;
            margin-bottom: 20px;
        }
        .dashboard {
            display: grid;
            grid-template-columns: 1fr;
            gap: 15px;
        }
        .card {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        .card-title {
            font-size: 1.2em;
            font-weight: bold;
            margin-bottom: 15px;
            color: #34495e;
            display: flex;
            align-items: center;
        }
        .card-title .icon {
            margin-right: 10px;
            font-size: 1.2em;
        }
        .status-badge {
            padding: 5px 10px;
            border-radius: 20px;
            font-size: 0.9em;
            margin-left: auto;
        }
        .status-on {
            background: #27ae60;
            color: white;
        }
        .status-off {
            background: #e74c3c;
            color: white;
        }
        .btn-group {
            display: flex;
            gap: 10px;
            margin-top: 15px;
        }
        .btn {
            flex: 1;
            padding: 10px;
            border: none;
            border-radius: 5px;
            color: white;
            text-align: center;
            text-decoration: none;
            cursor: pointer;
            transition: background 0.3s;
        }
        .btn:hover {
            opacity: 0.9;
        }
        .btn-primary {
            background: #3498db;
        }
        .btn-success {
            background: #2ecc71;
        }
        .btn-danger {
            background: #e74c3c;
        }
        .temp-display {
            font-size: 1.5em;
            text-align: center;
            margin: 20px 0;
            padding: 15px;
            background: #e3f2fd;
            border-radius: 8px;
        }
        .temp-value {
            font-weight: bold;
            color: #2980b9;
        }
    </style>
    <script>
        function updateTemperature() {
            fetch('/getTemperature')
                .then(response => response.text())
                .then(temp => {
                    document.getElementById('temperatureValue').textContent = temp;
                });
        }
        window.onload = function() {
            setInterval(updateTemperature, 1000);
        };
    </script>
 </head>
 <body>
    <h1>Smart Room Control</h1>
    <div class="dashboard">
        <div class="card">
            <div class="temp-display">
                Current Temperature: <span id="temperatureValue" class="temp-value">%TEMPERATURE%</span> °C
            </div>
        </div>
        <div class="card">
            <div class="card-title">
                <span class="icon">↻</span> System Mode
                <span class="status-badge %AUTONOMY_STATUS%">%AUTONOMY_TEXT%</span>
            </div>
            <div class="btn-group">
                <a href="/toggle-autonomy" class="btn btn-primary">Toggle Autonomy</a>
            </div>
        </div>
        <div class="card">
            <div class="card-title">
                <span class="icon">🪟</span> Curtains
                <span class="status-badge %CURTAIN_STATUS%">%CURTAIN_TEXT%</span>
            </div>
            <div class="btn-group">
                <a href="/open-curtains" class="btn btn-success">Open</a>
                <a href="/close-curtains" class="btn btn-danger">Close</a>
            </div>
        </div>
        <div class="card">
            <div class="card-title">
                <span class="icon">🚪</span> Door
                <span class="status-badge %DOOR_STATUS%">%DOOR_TEXT%</span>
            </div>
            <div class="btn-group">
                <a href="/open-door" class="btn btn-success">Open</a>
                <a href="/close-door" class="btn btn-danger">Close</a>
            </div>
        </div>
        <div class="card">
            <div class="card-title">
                <span class="icon">🌀</span> Fan
                <span class="status-badge %FAN_STATUS%">%FAN_TEXT%</span>
            </div>
            <div class="btn-group">
                <a href="/fan-on" class="btn btn-success">ON</a>
                <a href="/fan-off" class="btn btn-danger">OFF</a>
            </div>
        </div>
        <div class="card">
            <div class="card-title">
                <span class="icon">💡</span> Lights
                <span class="status-badge %LIGHT_STATUS%">%LIGHT_TEXT%</span>
            </div>
            <div class="btn-group">
                <a href="/light-on" class="btn btn-success">ON</a>
                <a href="/light-off" class="btn btn-danger">OFF</a>
            </div>
        </div>
    </div>
 </body>
 </html>
 )rawliteral";

    // Update placeholders with current values
    sensors.requestTemperatures();
    html.replace("%TEMPERATURE%", String(sensors.getTempCByIndex(0), 1));
    html.replace("%AUTONOMY_STATUS%", autonomyEnabled ? "on" : "off");
    html.replace("%AUTONOMY_TEXT%", autonomyEnabled ? "ENABLED" : "DISABLED");
    html.replace("%CURTAIN_STATUS%", curtainState == CURTAIN_OPENED ? "on" : "off");
    html.replace("%CURTAIN_TEXT%", curtainState == CURTAIN_OPENED ? "OPEN" : "CLOSED");
    html.replace("%DOOR_STATUS%", doorState == DOOR_OPENED ? "on" : "off");
    html.replace("%DOOR_TEXT%", doorState == DOOR_OPENED ? "OPEN" : "CLOSED");
    html.replace("%FAN_STATUS%", fanState ? "on" : "off");
    html.replace("%FAN_TEXT%", fanState ? "ON" : "OFF");
    html.replace("%LIGHT_STATUS%", lightState ? "on" : "off");
    html.replace("%LIGHT_TEXT%", lightState ? "ON" : "OFF");

    server.send(200, "text/html", html);
}

void handleTemperatureRequest() {
  sensors.requestTemperatures();
  server.send(200, "text/plain", String(sensors.getTempCByIndex(0), 1));
}

// ========== DEVICE CONTROL FUNCTIONS ==========
void toggleAutonomy() {
  autonomyEnabled = !autonomyEnabled;
  server.sendHeader("Location", "/");
  server.send(303);
}

void openCurtains() {
  if (!curtainMoving && curtainState != CURTAIN_OPENED) {
    curtainStartBeep();
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    curtainActionStart = millis();
    curtainMoving = true;
    curtainDirection = 1;
    curtainState = CURTAIN_OPENING;
  }
}

void closeCurtains() {
  if (!curtainMoving && curtainState != CURTAIN_CLOSED) {
    curtainStartBeep();
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    curtainActionStart = millis();
    curtainMoving = true;
    curtainDirection = -1;
    curtainState = CURTAIN_CLOSING;
  }
}

void stopCurtains() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  curtainMoving = false;
  curtainStopBeep();
}



void openCurtainsManually() {
  if(curtainState != CURTAIN_OPENED && curtainState != CURTAIN_OPENING) {
    openCurtains();
    server.send(200, "text/plain", "Opening curtains...");
  } else {
    server.send(200, "text/plain", "Curtains already open or opening");
  }
}

void closeCurtainsManually() {
  if(curtainState != CURTAIN_CLOSED && curtainState != CURTAIN_CLOSING) {
    closeCurtains();
    server.send(200, "text/plain", "Closing curtains...");
  } else {
    server.send(200, "text/plain", "Curtains already closed or closing");
  }
}

void openDoorManually() {
  if(doorState != DOOR_OPENED && !doorMoving) {
    if(checkSafety()) {
      server.send(403, "text/plain", "Safety block active");
      return;
    }
    
    Serial.println("Opening door with safety check...");
    setServoAngle(SERVO_UNLOCK_POS);
    doorState = DOOR_OPENED;
    doorMoving = false;
    doorUnlockBeep();
    lastDoorActionTime = millis();
    
    server.sendHeader("Location", "/");
    server.send(303);
  }
}

void closeDoorManually() {
  if(doorState != DOOR_CLOSED && !doorMoving) {
    if(checkSafety()) {
      server.send(403, "text/plain", "Safety block active");
      return;
    }
    
    Serial.println("Closing door with safety check...");
    setServoAngle(SERVO_LOCK_POS);
    doorState = DOOR_CLOSED;
    doorMoving = false;
    doorLockBeep();
    
    server.sendHeader("Location", "/");
    server.send(303);
  }
}

void turnFanOn() {
  setFanState(true);
  server.sendHeader("Location", "/");
  server.send(303);
}

void turnFanOff() {
  setFanState(false);
  server.sendHeader("Location", "/");
  server.send(303);
}

void setFanState(bool state) {
  digitalWrite(FanPin, state);
  fanState = state;
  if(state) {
    successBeep();
    Serial.println("Fan turned ON");
  } else {
    Serial.println("Fan turned OFF");
  }
}

// ========== LCD FUNCTIONS ==========
void updateLCD() {
  if (millis() - lastLCDUpdate < 1000) return;
  
  lcd1.clear();
  if (showTempFanScreen) {
    displayTempFan();
  } else {
    displayDoorCurtain();
  }
  
  showTempFanScreen = !showTempFanScreen;
  lastLCDUpdate = millis();
}

void displayTempFan() {
  lcd1.setCursor(0, 0);
  lcd1.print("Temp:");
  lcd1.print(sensors.getTempCByIndex(0), 1);
  lcd1.print((char)223);
  lcd1.print("C");
  
  lcd1.setCursor(0, 1);
  lcd1.print("Fan:");
  lcd1.print(fanState ? "ON " : "OFF");
}

void displayDoorCurtain() {
  lcd1.setCursor(0, 0);
  lcd1.print("Door:");
  lcd1.print(doorState == DOOR_OPENED ? "Open " : "Closed");
  
  lcd1.setCursor(0, 1);
  lcd1.print("Curtain:");
  lcd1.print(curtainState == CURTAIN_OPENED ? "Open" : "Closed");
}

// ========== BUZZER FUNCTIONS ==========
// ========== HELPER FUNCTIONS ==========
void stopMotor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  curtainMoving = false;
  curtainDirection = 0;
}



void toggleDoorState() {
  if(doorState == DOOR_CLOSED) {
    openDoorWithSafety();
  } else {
    closeDoorWithSafety();
  }
}

// Add this to your main loop() to handle auto-closing:
void checkDoorAutoClose() {
  if(doorState == DOOR_OPENED && 
     (millis() - lastDoorActionTime >= 4000) && !checkSafety()) {
    
    setServoAngle(SERVO_LOCK_POS);
    doorState = DOOR_CLOSED;
    doorLockBeep();
    
    Serial.println("Door auto-locked (safe)");
    lcd2.setCursor(0, 1);
    lcd2.print("AUTO-LOCKED   ");
    delay(1500);
    resetPasswordDisplay();
    updatePasswordDisplay();
  }
}


void openDoor() {
    Serial.println("[SERVO] Opening door");
    setServoAngle(90);

  }


void closeDoor() {
    Serial.println("[SERVO] Closing door");
    setServoAngle(0);
  
}

void setServoAngle(int angle) {
  // Constrain angle to valid range
  angle = constrain(angle, minAngle, maxAngle);
  
  // Convert angle to pulse width in microseconds
  int pulseWidth = map(angle, minAngle, maxAngle, minPulseWidth, maxPulseWidth);
  
  // Calculate duty cycle (0-65535)
  uint32_t duty = (pulseWidth * (1 << pwmResolution)) / (1000000 / pwmFrequency);
  
  // Set PWM duty cycle
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwmChannel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)pwmChannel);
  
  Serial.printf("Servo set to %d° (Duty: %d)\n", angle, duty);
}

void setupServoPWM() {
  // Configure Timer
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = (ledc_timer_bit_t)pwmResolution,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = pwmFrequency,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  // Configure Channel
  ledc_channel_config_t channel_conf = {
    .gpio_num = servoPin,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = (ledc_channel_t)pwmChannel,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,  // Start with 0% duty cycle
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf);

  Serial.println("Servo PWM initialized");
}



void updateDoorMovement() {
  if(doorMoving && (millis() - doorActionTime >= 500)) {
    doorState = targetDoorState;
    doorMoving = false;
    
    // Optional: Add completion feedback
    Serial.println(doorState == DOOR_OPENED ? "Door opened" : "Door closed");
  }
}

void updateBuzzer() {
  switch(buzzerState) {
    case BUZZER_IDLE:
      break;
      
    case PLAYING_TONE:
      if(millis() - buzzerStartTime >= buzzerDuration) {
        noTone(buzzer_pin);
        if(buzzerPauseDuration > 0) {
          buzzerState = IN_SEQUENCE;
          buzzerStartTime = millis();
        } else {
          buzzerState = BUZZER_IDLE;
        }
      }
      break;
      
    case IN_SEQUENCE:
      if(millis() - buzzerStartTime >= buzzerPauseDuration) {
        sequencePosition++;
        if(sequencePosition < sequenceLength) {
          playTone(currentSequence[sequencePosition].frequency, 
                  currentSequence[sequencePosition].duration,
                  currentSequence[sequencePosition].pause);
        } else {
          buzzerState = BUZZER_IDLE;
        }
      }
      break;
      
    case IN_LOOP:
      if(millis() - buzzerStartTime >= buzzerDuration) {
        noTone(buzzer_pin);
        currentLoop++;
        if(currentLoop < buzzerLoopCount) {
          playTone(buzzerFrequency, buzzerDuration, buzzerPauseDuration);
        } else {
          buzzerState = BUZZER_IDLE;
        }
      }
      break;
  }
}


void playTone(int frequency, int duration, int pause) {
  tone(buzzer_pin, frequency);
  buzzerStartTime = millis();
  buzzerDuration = duration;
  buzzerPauseDuration = pause;
  buzzerFrequency = frequency;
  buzzerState = PLAYING_TONE;
}
void shortBeep() {
  playTone(4000, 20, 30);
}

void errorBeep() {
  playTone(300, 1000);
}

void successBeep() {
  ToneSequence seq[] = {{1000, 80, 120}, {1000, 80, 120}};
  startSequence(seq, 2);
}

void warningBeep() {
  ToneSequence seq[] = {{800, 200, 300}, {400, 200, 300}, 
                       {800, 200, 300}, {400, 200, 300},
                       {800, 200, 300}, {400, 200, 300}};
  startSequence(seq, 6);
}

void doorLockBeep() {
  ToneSequence seq[] = {{1500, 200, 250}, {1500, 200, 250}};
  startSequence(seq, 2);
}

void doorUnlockBeep() {
  playTone(2000, 300);
}

void curtainStartBeep() {
  playTone(1200, 50);
}

void curtainStopBeep() {
  ToneSequence seq[] = {{1200, 30, 50}, {1200, 30, 50}};
  startSequence(seq, 2);
}

void startupBeep() {
  ToneSequence seq[] = {{523, 100, 150}, {654, 100, 150}, {784, 100, 150}};
  startSequence(seq, 3);
}

void startSequence(ToneSequence sequence[], int length) {
  memcpy(currentSequence, sequence, length * sizeof(ToneSequence));
  sequenceLength = length;
  sequencePosition = 0;
  playTone(currentSequence[0].frequency, 
          currentSequence[0].duration,
          currentSequence[0].pause);
  buzzerState = IN_SEQUENCE;
}
// Use just time-based control:
void updateCurtainMovement() {
  if (curtainMoving && (millis() - curtainActionStart >= MOTOR_RUN_TIME)) {
    stopCurtains();
    curtainState = (curtainDirection == 1) ? CURTAIN_OPENED : CURTAIN_CLOSED;
  }
}
int getCurtainPosition() {
  if (!curtainMoving) {
    return (curtainState == CURTAIN_OPENED) ? 100 : 0;
  }
  float progress = (millis() - curtainActionStart) / (float)MOTOR_RUN_TIME;
  return constrain((curtainDirection == 1) ? progress * 100 : 100 - (progress * 100), 0, 100);
}
void turnLightOn() {
  digitalWrite(lightRelayPin, LIGHT_ON);
  lightState = true;
  server.send(200, "text/plain", "Light turned ON");
  Serial.println("Light turned ON via web");
}

void turnLightOff() {
  digitalWrite(lightRelayPin, LIGHT_OFF);
  lightState = false;
  server.send(200, "text/plain", "Light turned OFF");
  Serial.println("Light turned OFF via web");
}
float getUltrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Convert to cm
}

bool checkSafety() {
  // Take 3 readings for reliability
  float readings[3];
  for(int i=0; i<3; i++) {
    readings[i] = getUltrasonicDistance();
    delay(25);
  }
  
  // Check if any reading is below threshold
  for(int i=0; i<3; i++) {
    if(readings[i] > 0 && readings[i] < SAFETY_DISTANCE) {
      lastSafetyTrigger = millis();
      if(!safetyBlockActive) {
        safetyBlockActive = true;
        triggerSafetyAlert();
      }
      return true;
    }
  }
  
  // Clear safety block if timeout elapsed
  if(safetyBlockActive && (millis() - lastSafetyTrigger > SAFETY_TIMEOUT)) {
    safetyBlockActive = false;
    clearSafetyAlert();
  }
  
  return false;
}

void triggerSafetyAlert() {
  Serial.println("Safety triggered! Object too close");
  
  // Audible alert
  for(int i=0; i<3; i++) {
    tone(buzzer_pin, 1200, 200);
    delay(250);
    noTone(buzzer_pin);
    delay(100);
  }
  
  // Visual alert
  lcd2.setCursor(0, 0);
  lcd2.print("SAFETY ALERT!  ");
  lcd2.setCursor(0, 1);
  lcd2.print("CLEAR ENTRANCE ");
  delay(1500); // Show message for 2 seconds
  resetPasswordDisplay();
}

void clearSafetyAlert() {
  Serial.println("Safety block cleared");
  lcd2.setCursor(0, 0);
  lcd2.print("SAFE TO OPERATE");
  delay(1000);
  updatePasswordDisplay(); // Return to normal display
}
void openDoorWithSafety() {
  if(checkSafety()) {
    server.send(403, "text/plain", "Cannot open - safety block active");
    return;
  }

  setServoAngle(SERVO_UNLOCK_POS);
  doorState = DOOR_OPENED;
  doorMoving = false;
  doorUnlockBeep();
  lastDoorActionTime = millis();
  
  Serial.println("Door safely unlocked");
  lcd2.setCursor(0, 1);
  lcd2.print("DOOR OPEN      ");
  
  // Add this delay and reset
  delay(2000); // Show message for 2 seconds
  resetPasswordDisplay(); // Return to password entry screen
}

void closeDoorWithSafety() {
  if(checkSafety()) {
    server.send(403, "text/plain", "Cannot close - safety block active");
    return;
  }

  setServoAngle(SERVO_LOCK_POS);
  doorState = DOOR_CLOSED;
  doorMoving = false;
  doorLockBeep();
  
  Serial.println("Door safely locked");
  lcd2.setCursor(0, 1);
  lcd2.print("DOOR LOCKED    ");
  
  // Add this delay and reset
  delay(1000); // Show message for 2 seconds
  resetPasswordDisplay(); // Return to password entry screen
}
void triggerAlarm() {
  // Play alarm sound with blinking light (runs continuously while smoke is detected)
  while(smokeDetected) {  // Keep repeating as long as smoke is present
    playTone(1000, 500);  // 1kHz tone for 500ms
    digitalWrite(lightRelayPin, !digitalRead(lightRelayPin)); // Toggle LED state
    delay(200);
    setServoAngle(SERVO_UNLOCK_POS);
    doorState = DOOR_OPENED;
    doorMoving = false;
    openCurtains();
    updateCurtainMovement();
    curtainState = CURTAIN_OPENED;           // 200ms off between beeps
    
    // Check smoke status again (in case it cleared during the delay)
    smokeDetected = (digitalRead(mq2DigitalPin) == LOW);
  }
  digitalWrite(lightRelayPin, LOW); // Ensure LED is off when alarm stops
}

void checkSmoke() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSmokeCheck >= smokeCheckInterval) {
    lastSmokeCheck = currentMillis;
    
    // Read digital output (LOW = Smoke detected, HIGH = Clean air)
    bool currentSmokeState = (digitalRead(mq2DigitalPin) == LOW);
    
    // Only trigger alarm if smoke is newly detected
    if (currentSmokeState && !smokeDetected) {
      smokeDetected = true;
      triggerAlarm();

      
      
    }
    // Update smoke state
    smokeDetected = currentSmokeState;
  }
}