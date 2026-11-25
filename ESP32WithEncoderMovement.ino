#include "Arduino.h"
#include <Bluepad32.h>
#include <Servo.h>
#include <WiFi.h>
#include "encoder_motor.h"
#include "encoder_motor_lib.h"
#include "PID.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <Arduino_JSON.h>

//Lidar Server Data
const char* ssid = "Quad_Plus";
const char* password = "19452749";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

HardwareSerial lidarSerial(2);

#define PACKET_LEN 22
uint8_t buf[PACKET_LEN];
uint8_t bufIdx = 0;
bool synced = false;

// Store one full rotation: angle -> distance
float scanDist[360];
bool scanValid[360];
bool scanComplete = false;


namespace {
constexpr uint32_t kPPR = 12;              // Pulses per revolution.
constexpr uint32_t kReductionRation = 90;  // Reduction ratio.

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
em::EncoderMotor g_encoder_motor_0(  // E0
    GPIO_NUM_27,                     // The pin number of the motor's positive pole.
    GPIO_NUM_13,                     // The pin number of the motor's negative pole.
    GPIO_NUM_18,                     // The pin number of the encoder's A phase.
    GPIO_NUM_19,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_1(  // E1
    GPIO_NUM_4,                      // The pin number of the motor's positive pole.
    GPIO_NUM_2,                      // The pin number of the motor's negative pole.
    GPIO_NUM_5,                      // The pin number of the encoder's A phase.
    GPIO_NUM_23,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_2(  // E2
    GPIO_NUM_17,                     // The pin number of the motor's positive pole.
    GPIO_NUM_12,                     // The pin number of the motor's negative pole.
    GPIO_NUM_35,                     // The pin number of the encoder's A phase.
    GPIO_NUM_36,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_3(  // E3
    GPIO_NUM_15,                     // The pin number of the motor's positive pole.
    GPIO_NUM_14,                     // The pin number of the motor's negative pole.
    GPIO_NUM_34,                     // The pin number of the encoder's A phase.
    GPIO_NUM_39,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

#else  // The ESP32 Arduino Core Version is less than 3.0.0

em::EncoderMotor g_encoder_motor_0(  // E0
    GPIO_NUM_27,                     // The pin number of the motor's positive pole.
    0,                               // The positive pole of the motor is attached to LED Control (LEDC) Channel 0.
    GPIO_NUM_13,                     // The pin number of the motor's negative pole.
    1,                               // The negative pole of the motor is attached to LED Control (LEDC) Channel 1.
    GPIO_NUM_18,                     // The pin number of the encoder's A phase.
    GPIO_NUM_19,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_1(  // E1
    GPIO_NUM_4,                      // The pin number of the motor's positive pole.
    2,                               // The positive pole of the motor is attached to LED Control (LEDC) Channel 2.
    GPIO_NUM_2,                      // The pin number of the motor's negative pole.
    3,                               // The negative pole of the motor is attached to LED Control (LEDC) Channel 3.
    GPIO_NUM_5,                      // The pin number of the encoder's A phase.
    GPIO_NUM_23,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_2(  // E2
    GPIO_NUM_17,                     // The pin number of the motor's positive pole.
    12,                               // The positive pole of the motor is attached to LED Control (LEDC) Channel 4.
    GPIO_NUM_12,                     // The pin number of the motor's negative pole.
    13,                               // The negative pole of the motor is attached to LED Control (LEDC) Channel 5.
    GPIO_NUM_35,                     // The pin number of the encoder's A phase.
    GPIO_NUM_36,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);

em::EncoderMotor g_encoder_motor_3(  // E3
    GPIO_NUM_15,                     // The pin number of the motor's positive pole.
    14,                               // The positive pole of the motor is attached to LED Control (LEDC) Channel 6.
    GPIO_NUM_14,                     // The pin number of the motor's negative pole.
    15,                               // The negative pole of the motor is attached to LED Control (LEDC) Channel 7.
    GPIO_NUM_34,                     // The pin number of the encoder's A phase.
    GPIO_NUM_39,                     // The pin number of the encoder's B phase.
    kPPR,                            // Pulses per revolution.
    kReductionRation,                // Reduction ratio.
    em::EncoderMotor::kAPhaseLeads   // Phase relationship (A phase leads or B phase leads, referring to the situation when
                                     // the motor is rotating forward)
);
#endif
}  // namespace
int encoder0 = 0;

// ───  Pin Definitions ───────────────────────────────────────────────────────
#define M0_IN1  27
#define M0_IN2  13

#define M1_IN1  4
#define M1_IN2  2

#define M2_IN1  17
#define M2_IN2  12

#define M3_IN1  14
#define M3_IN2  15



//Motor Construct
struct Motor {
  uint8_t in1;
  uint8_t in2;
  uint8_t ch1;  // LEDC channel for in1
  uint8_t ch2;  // LEDC channel for in2
  float speed;
  uint32_t pwm;
  float rpm;
  int lastTick;
};





// ─── LEDC PWM Configuration ───────────────────────────────────────────────────
#define PWM_FREQ      500   // 20 kHz — above audible range
#define PWM_RES       16       // 8-bit → 0–255 65535

//Defined Variables
Motor mot0 = {M0_IN1, M0_IN2, 0, 1, 0.0f, 0, 0.0f, 0};
Motor mot1 = {M1_IN1, M1_IN2, 2, 3, 0.0f, 0, 0.0f, 0};
Motor mot2 = {M2_IN1, M2_IN2, 12, 13, 0.0f, 0, 0.0f, 0};
Motor mot3 = {M3_IN1, M3_IN2, 14, 15, 0.0f, 0, 0.0f, 0};

//PID
PID pid0, pid1, pid2, pid3;

// CONTROLLER

ControllerPtr myController = nullptr;

// ── Global controller state ──────────────────────────────────────
// Sticks
int lx = 0, ly = 0, rx = 0, ry = 0;
#define STICK_MAX    511
#define DEADZONE     20   

float Lx = 0, Ly = 0, Rx = 0, Ry = 0;
// Triggers
int l2 = 0, r2 = 0;
// Buttons
uint16_t btns     = 0;
bool cross        = false, circle  = false;
bool square       = false, triangle = false;
bool l1           = false, r1      = false;
bool l3           = false, r3      = false;
// D-Pad
uint8_t dpad  = 0;
bool dUp      = false, dDown  = false;
bool dLeft    = false, dRight = false;
// Misc
uint32_t misc = 0;
bool psBtn    = false, touchpad = false;

void processPacket(uint8_t* pkt) {
  int baseAngle = (pkt[1] - 0xA0) * 4;
  //Serial.printf("angle=%d\n", baseAngle);   // TEMP DEBUG

  for (int i = 0; i < 4; i++) {
    int angle = baseAngle + i;
    if (angle < 0 || angle >= 360) continue;

    uint16_t raw = pkt[4 + i*4] | (pkt[5 + i*4] << 8);
    bool invalid = (raw >> 15) & 1;
    uint16_t dist_mm = raw & 0x3FFF;

    scanValid[angle] = !invalid && dist_mm > 0;
    scanDist[angle] = dist_mm;
  }

  // Treat angle wraparound (356-359deg packet) as "scan complete"
  if (baseAngle >= 356) {
    scanComplete = true;
  }
}


void readLidar() {
  while (lidarSerial.available()) {
    uint8_t b = lidarSerial.read();
    if (!synced) {
      if (b == 0xFA) { buf[0] = b; bufIdx = 1; synced = true; }
      continue;
    }
    buf[bufIdx++] = b;
    if (bufIdx == PACKET_LEN) {
      if (buf[0] == 0xFA && buf[1] >= 0xA0 && buf[1] <= 0xF9) {
        processPacket(buf);
      }
      synced = false;
      bufIdx = 0;
    }
  
  }
}

String getScanJSON() {
  JSONVar scan;
  JSONVar angles;
  JSONVar dists;

  int idx = 0;
  for (int a = 0; a < 360; a++) {
    if (scanValid[a]) {
      angles[idx] = a;
      dists[idx] = scanDist[a];
      idx++;
    }
  }
  scan["angles"] = angles;
  scan["distances"] = dists;
  return JSON.stringify(scan);
}

void initLittleFS() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println(WiFi.localIP());
}


void notifyClients(String json) {
  ws.textAll(json);
}


void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("Client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("Client #%u disconnected\n", client->id());
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void onConnectedController(ControllerPtr ctl) {
  myController = ctl;
  Serial.println("DualSense connected!");
  
  // Print some info about the controller
  ControllerProperties props = ctl->getProperties();
  Serial.printf("Model: %s, VID: 0x%04X, PID: 0x%04X\n",
    ctl->getModelName().c_str(), props.vendor_id, props.product_id);
}


void onDisconnectedController(ControllerPtr ctl) {
  myController = nullptr;
  Serial.println("DualSense disconnected");
}
void controller(){// Sticks
  lx = myController->axisX();
  ly = myController->axisY();
  rx = myController->axisRX();
  ry = myController->axisRY();
  // Triggers
  l2 = myController->brake();
  r2 = myController->throttle();
  // Buttons
  btns     = myController->buttons();
  cross    = btns & BUTTON_A;
  circle   = btns & BUTTON_B;
  square   = btns & BUTTON_X;
  triangle = btns & BUTTON_Y;
  l1       = btns & BUTTON_SHOULDER_L;
  r1       = btns & BUTTON_SHOULDER_R;
  l3       = btns & BUTTON_THUMB_L;
  r3       = btns & BUTTON_THUMB_R;
  // D-Pad
  dpad   = myController->dpad();
  dUp    = dpad & DPAD_UP;
  dDown  = dpad & DPAD_DOWN;
  dLeft  = dpad & DPAD_LEFT;
  dRight = dpad & DPAD_RIGHT;
  // Misc
  misc     = myController->miscButtons();
  psBtn    = misc & MISC_BUTTON_SYSTEM;
  touchpad = misc & MISC_BUTTON_HOME;
/*
  Serial.printf("LX=%4d LY=%4d | RX=%4d RY=%4d | L2=%4d R2=%4d | Btns=0x%04X | Dpad=0x%02X\n",
    lx, ly, rx, ry, l2, r2, btns, dpad);*/
}

float applyDeadzone(int raw, int deadzone, int maxVal){
  if (abs(raw)<deadzone){
    return 0.0f;
  }
  if(abs(raw)>maxVal-deadzone){
    if(raw>0){
      return 1.0f;
    }
    else{
      return -1.0f;
    }
  }
  
  // Scale the remaining range back to -1.0 to 1.0
  float sign  = (raw > 0) ? 1.0f : -1.0f;
  float scaled = (float)(abs(raw) - deadzone) / (float)(maxVal - deadzone);
  return sign * constrain(scaled, 0.0f, 1.0f);
}

void setupPWM(Motor &mot) {
  ledcSetup(mot.ch1, PWM_FREQ, PWM_RES);
  ledcSetup(mot.ch2, PWM_FREQ, PWM_RES);
  
  ledcAttachPin(mot.in1, mot.ch1);
  ledcAttachPin(mot.in2, mot.ch2);
}
void setMotor(Motor &mot, float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  mot.speed = speed;
  
  

  uint32_t pwm = abs(speed) * 14400;
  mot.pwm = pwm;
  
  if (speed > 0) {
    ledcWrite(mot.ch1, pwm);
    ledcWrite(mot.ch2, 0);
    
  } else if (speed < 0) {
    ledcWrite(mot.ch1, 0);
    ledcWrite(mot.ch2, pwm);
    
  } else {
    ledcWrite(mot.ch1, 0);
    ledcWrite(mot.ch2, 0);
    
  }
}

void controllerTask (void *pvParameters) {
  while (true){
     bool updated = BP32.update();
  
    if( updated && myController && myController->isConnected() ){
      controller();
      Ly = applyDeadzone(ly, DEADZONE, STICK_MAX);
      Lx = applyDeadzone(lx, DEADZONE, STICK_MAX);
      Ry = applyDeadzone(ry, DEADZONE, STICK_MAX);
      Rx = applyDeadzone(rx, DEADZONE, STICK_MAX);
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


void motorTask(void *pvParameters){
  while(true){

    
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


void encoderSetup(){
  printf("setting up\n");
  printf("Emakefun Encoder Motor Library Version: %s\n", em::esp_encoder_motor_lib::Version().c_str());
  g_encoder_motor_0.Init();
  g_encoder_motor_1.Init();
  g_encoder_motor_2.Init();
  g_encoder_motor_3.Init();
  pinMode(26, INPUT);
  printf("setup completed\n");
}




void encoderTask(void *pvParameters){
  while(true) {
    noInterrupts();
    


    interrupts();
    /*
    Serial.print("0 RPM: ");
    Serial.print(g_encoder_motor_0.SpeedRpm());
    Serial.print(" ");
    Serial.print("0 PWM: ");
    Serial.print(mot0.pwm);
    Serial.print(" | ");
    Serial.print("1 RPM: ");
    Serial.print(g_encoder_motor_1.SpeedRpm());
    Serial.print(" ");
    Serial.print("1 PWM: ");
    Serial.print(mot1.pwm);
    Serial.print(" | ");
    Serial.print("2 RPM: ");
    Serial.print(g_encoder_motor_2.SpeedRpm());
    Serial.print(" ");
    Serial.print("2 PWM: ");
    Serial.print(mot2.pwm);
    Serial.print(" | ");
    Serial.print("3 RPM: ");
    Serial.print(g_encoder_motor_3.SpeedRpm()/10);
    Serial.print("3 PWM: ");
    Serial.print(mot3.pwm);
    Serial.print(" | ");
    Serial.print("pulse counts: [");
    Serial.print(g_encoder_motor_0.EncoderPulseCount());
    Serial.print(", ");
    Serial.print(g_encoder_motor_1.EncoderPulseCount());
    Serial.print(", ");
    Serial.print(g_encoder_motor_2.EncoderPulseCount());
    Serial.print(", ");
    Serial.print(g_encoder_motor_3.EncoderPulseCount());
    Serial.println("]");
    
    */

    
    
    
 
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
float Kp = 0.1;
float Ki = 0.00001;
float Kd = 0.0;

int maxS = 150;

void PIDTask(void* arg) {
  const TickType_t xInterval = pdMS_TO_TICKS(100); // 100ms control loop
  TickType_t xLastWakeTime = xTaskGetTickCount();

  pidInit(&pid0, Kp, Ki, Kd, -1.0, 1.0);
  

  pidInit(&pid1, Kp, Ki, Kd, -1.0, 1.0);
  

  pidInit(&pid2, Kp, Ki, Kd, -1.0, 1.0);
  

  pidInit(&pid3, Kp, Ki, Kd, -1.0, 1.0);
  

  while (true){

    if(circle){
      pid0.setpoint = 0;  
      pid1.setpoint = 0;  
      pid2.setpoint = 0;  
      pid3.setpoint = 0;
    }
    else if (triangle){
      pid0.setpoint = 0.75;  
      pid1.setpoint = 0.75;  
      pid2.setpoint = 0.75;  
      pid3.setpoint = -0.75;
    }
    else if (cross){
      pid0.setpoint = 1;  
      pid1.setpoint = 1;  
      pid2.setpoint = 1;  
      pid3.setpoint = -1;
    }
    else{
      pid0.setpoint = -Ly-Rx-Lx;  
      pid1.setpoint = -Ly-Rx+Lx;  
      pid2.setpoint = Ly-Rx+Lx;
      pid3.setpoint = -Ly+Rx+Lx;

          /*
    setMotor(mot0, Ly);//-Rx+Lx);
    setMotor(mot1, Ly);//-Rx-Lx);
    setMotor(mot2, -Ly);//-Rx-Lx);
    setMotor(mot3, Ly);//+Rx-Lx);*/
    


    }
    

      

    float output0 = pidCompute(&pid0, g_encoder_motor_0.SpeedRpm()/maxS, 0.1f);
    setMotor(mot0, output0);
    

    float output1 = pidCompute(&pid1, g_encoder_motor_1.SpeedRpm()/maxS, 0.1f);
    setMotor(mot1, output1);

    float output2 = pidCompute(&pid2, g_encoder_motor_2.SpeedRpm()/maxS, 0.1f);
    setMotor(mot2, output2);

    float output3 = pidCompute(&pid3, -1*g_encoder_motor_3.SpeedRpm()/maxS, 0.1f);
    setMotor(mot3, output3);

    

    vTaskDelayUntil(&xLastWakeTime, xInterval); // don't forget this!
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);   // wait for Serial Monitor to connect
  lidarSerial.setRxBufferSize(1024);
  lidarSerial.begin(115200, SERIAL_8N1, 21, 22);
  
  initWiFi();
  initLittleFS();
  initWebSocket();

  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });
  server.serveStatic("/", LittleFS, "/");
  server.begin();

  encoderSetup();

  setupPWM(mot0);
  setupPWM(mot1);
  setupPWM(mot2);
  setupPWM(mot3);

  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Forget previously paired controllers so the DualSense can pair fresh
  BP32.forgetBluetoothKeys();

  Serial.println("Ready — hold PS button on DualSense to pair");

  xTaskCreate(
    PIDTask,       // Task function
    "pid_task",     // Task name
    4096,          // Stack size (bytes)
    NULL,           // Parameters
    5,             // Priority (higher = more urgent)
    NULL            // Task handle (not used)
    );         


  // Core 0 — Bluepad32 and controller
  xTaskCreatePinnedToCore(
    controllerTask,
    "ControllerTask",
    4096,           // larger stack for Bluepad32
    NULL,
    1,
    NULL,
    0               // Core 0
  );


  // Core 1 — Motors
  xTaskCreatePinnedToCore(
    motorTask,
    "MotorTask",
    2048,
    NULL,
    2,
    NULL,
    1               // Core 1
  );

  // Core 1 — Encoders
  xTaskCreatePinnedToCore(
    encoderTask,
    "EncoderTask",
    2048,
    NULL,
    3,
    NULL,
    1               // Core 1
  );

}

void loop() {
  readLidar();

  if (scanComplete) {
    //Serial.println("SCAN COMPLETE");   // TEMP DEBUG
    notifyClients(getScanJSON());
    scanComplete = false;
  }

  ws.cleanupClients();
  vTaskDelay(1/10);
}
