#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

//1. CONFIGURATION 
const char* ssid = "WIFI NAME";
const char* password = "WIFI PASS";
WiFiUDP udp;
const unsigned int localUdpPort = 4210;
char packetBuffer[255];

const unsigned long WATCHDOG_TIMEOUT = 2000;
unsigned long lastPacketTime = 0;

int currentTargetSlots = 0;
int botSpeed = 190; 
bool isFiring = false;

// 2. PIN DEFINITIONS 
const int L_Front_Fwd = 26; const int L_Front_Bwd = 27; 
const int L_Rear_Fwd  = 32; const int L_Rear_Bwd  = 33; 
const int R_Front_Fwd = 14; const int R_Front_Bwd = 18; 
const int R_Rear_Fwd  = 25; const int R_Rear_Bwd  = 13; 

const int relayPin = 23; 
const int servoPin = 5; 
const int leftFlamePin = 34; 
const int rightFlamePin = 35; 
const int leftEncoderPin = 19; 
const int rightEncoderPin = 21; 

// 3. PWM CONFIGURATION 
const int PWM_FREQ = 5000;
const int PWM_RES = 8;

// 4. ENCODER VARIABLES
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;
Servo nozzleServo;

void IRAM_ATTR countLeft() { leftEncoderCount++; }
void IRAM_ATTR countRight() { rightEncoderCount++; }

// 5. SETUP 
void setup() {
  Serial.begin(115200);

  nozzleServo.setPeriodHertz(50);
  nozzleServo.attach(servoPin, 500, 2400);
  nozzleServo.write(90);

  ledcAttach(L_Front_Fwd, PWM_FREQ, PWM_RES);
  ledcAttach(L_Front_Bwd, PWM_FREQ, PWM_RES);
  ledcAttach(L_Rear_Fwd, PWM_FREQ, PWM_RES);
  ledcAttach(L_Rear_Bwd, PWM_FREQ, PWM_RES);
  ledcAttach(R_Front_Fwd, PWM_FREQ, PWM_RES);
  ledcAttach(R_Front_Bwd, PWM_FREQ, PWM_RES);
  ledcAttach(R_Rear_Fwd, PWM_FREQ, PWM_RES);
  ledcAttach(R_Rear_Bwd, PWM_FREQ, PWM_RES);

  stopMotors();

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); 
  
  pinMode(leftFlamePin, INPUT);
  pinMode(rightFlamePin, INPUT);
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, RISING);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }

  Serial.println("\n WiFi CONNECTED ");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  udp.begin(localUdpPort);
  lastPacketTime = millis();
}

//  6. MAIN LOOP
void loop() {
  if (millis() - lastPacketTime > WATCHDOG_TIMEOUT) {
    stopMotors();
  }

  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      char cmd;
      int slots;

      if (sscanf(packetBuffer, "%c:%d", &cmd, &slots) == 2) {
        lastPacketTime = millis();
        currentTargetSlots = slots;

        noInterrupts();
        leftEncoderCount = 0;
        rightEncoderCount = 0;
        interrupts();

        if (cmd == 'F') { executeBurstForward(); smartFlush(); }
        else if (cmd == 'L') { executeBurstLeft(); smartFlush(); }
        else if (cmd == 'R') { executeBurstRight(); smartFlush(); }
        else if (cmd == 'S' && !isFiring) {
          isFiring = true;
          extinguishSequence();
          smartFlush();
          isFiring = false;
        }
      }
    }
  }
}

// 7. STOP MOTORS 
void stopMotors() {
  ledcWrite(L_Front_Fwd, 0); ledcWrite(L_Front_Bwd, 0);
  ledcWrite(L_Rear_Fwd, 0);  ledcWrite(L_Rear_Bwd, 0);
  ledcWrite(R_Front_Fwd, 0); ledcWrite(R_Front_Bwd, 0);
  ledcWrite(R_Rear_Fwd, 0);  ledcWrite(R_Rear_Bwd, 0);
}

// 8. SMART FLUSH
void smartFlush() {
  while (udp.parsePacket() > 0) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len <= 0) continue;
    packetBuffer[len] = '\0';
    char cmd; int slots;
    if (sscanf(packetBuffer, "%c:%d", &cmd, &slots) == 2) {
      lastPacketTime = millis(); 
      if (cmd == 'S' && !isFiring) {
        Serial.println("smartFlush: honouring buffered STOP command");
        stopMotors();
        isFiring = true;
        extinguishSequence();
        isFiring = false;
        return; 
      }
    }
  }
}

// 9. MOVEMENT FUNCTIONS
void executeBurstForward() {
  unsigned long burstStart = millis();
  unsigned long lastTickTimeL = millis();
  unsigned long lastTickTimeR = millis();
  int lastL = 0, lastR = 0;

  ledcWrite(L_Front_Fwd, botSpeed); ledcWrite(L_Front_Bwd, 0);
  ledcWrite(R_Front_Fwd, botSpeed); ledcWrite(R_Front_Bwd, 0);
  ledcWrite(L_Rear_Fwd, botSpeed);  ledcWrite(L_Rear_Bwd, 0);
  ledcWrite(R_Rear_Fwd, botSpeed);  ledcWrite(R_Rear_Bwd, 0);

  while (leftEncoderCount < currentTargetSlots || rightEncoderCount < currentTargetSlots) {
    if (leftEncoderCount >= currentTargetSlots) {
      ledcWrite(L_Front_Fwd, 0); ledcWrite(L_Rear_Fwd, 0);
    }
    if (rightEncoderCount >= currentTargetSlots) {
      ledcWrite(R_Front_Fwd, 0); ledcWrite(R_Rear_Fwd, 0);
    }

    // INDEPENDENT STALL DETECTION
    if (leftEncoderCount != lastL) {
      lastL = leftEncoderCount;
      lastTickTimeL = millis();
    } else if (leftEncoderCount < currentTargetSlots && (millis() - lastTickTimeL > 400)) {
      Serial.println("LEFT WHEEL STALL");
      break;
    }

    if (rightEncoderCount != lastR) {
      lastR = rightEncoderCount;
      lastTickTimeR = millis();
    } else if (rightEncoderCount < currentTargetSlots && (millis() - lastTickTimeR > 400)) {
      Serial.println("RIGHT WHEEL STALL");
      break;
    }

    if (millis() - burstStart > 1500) break;
  }
  stopMotors();
}

void executeBurstLeft() {
  unsigned long burstStart = millis();
  unsigned long lastTickTimeL = millis();
  unsigned long lastTickTimeR = millis();
  int lastL = 0, lastR = 0;

  ledcWrite(L_Front_Fwd, 0);        ledcWrite(L_Front_Bwd, botSpeed);
  ledcWrite(R_Front_Fwd, botSpeed); ledcWrite(R_Front_Bwd, 0);
  ledcWrite(L_Rear_Fwd, 0);         ledcWrite(L_Rear_Bwd, botSpeed);
  ledcWrite(R_Rear_Fwd, botSpeed);  ledcWrite(R_Rear_Bwd, 0);

  while (leftEncoderCount < currentTargetSlots || rightEncoderCount < currentTargetSlots) {
    if (leftEncoderCount >= currentTargetSlots) {
      ledcWrite(L_Front_Bwd, 0); ledcWrite(L_Rear_Bwd, 0);
    }
    if (rightEncoderCount >= currentTargetSlots) {
      ledcWrite(R_Front_Fwd, 0); ledcWrite(R_Rear_Fwd, 0);
    }

    // INDEPENDENT STALL DETECTION
    if (leftEncoderCount != lastL) {
      lastL = leftEncoderCount;
      lastTickTimeL = millis();
    } else if (leftEncoderCount < currentTargetSlots && (millis() - lastTickTimeL > 400)) {
      Serial.println("LEFT WHEEL STALL");
      break;
    }

    if (rightEncoderCount != lastR) {
      lastR = rightEncoderCount;
      lastTickTimeR = millis();
    } else if (rightEncoderCount < currentTargetSlots && (millis() - lastTickTimeR > 400)) {
      Serial.println("RIGHT WHEEL STALL");
      break;
    }

    if (millis() - burstStart > 1500) break;
  }
  stopMotors();
}

void executeBurstRight() {
  unsigned long burstStart = millis();
  unsigned long lastTickTimeL = millis();
  unsigned long lastTickTimeR = millis();
  int lastL = 0, lastR = 0;

  ledcWrite(L_Front_Fwd, botSpeed); ledcWrite(L_Front_Bwd, 0);
  ledcWrite(R_Front_Fwd, 0);        ledcWrite(R_Front_Bwd, botSpeed);
  ledcWrite(L_Rear_Fwd, botSpeed);  ledcWrite(L_Rear_Bwd, 0);
  ledcWrite(R_Rear_Fwd, 0);         ledcWrite(R_Rear_Bwd, botSpeed);

  while (leftEncoderCount < currentTargetSlots || rightEncoderCount < currentTargetSlots) {
    if (leftEncoderCount >= currentTargetSlots) {
      ledcWrite(L_Front_Fwd, 0); ledcWrite(L_Rear_Fwd, 0);
    }
    if (rightEncoderCount >= currentTargetSlots) {
      ledcWrite(R_Front_Bwd, 0); ledcWrite(R_Rear_Bwd, 0);
    }

    // INDEPENDENT STALL DETECTION
    if (leftEncoderCount != lastL) {
      lastL = leftEncoderCount;
      lastTickTimeL = millis();
    } else if (leftEncoderCount < currentTargetSlots && (millis() - lastTickTimeL > 400)) {
      Serial.println("LEFT WHEEL STALL");
      break;
    }

    if (rightEncoderCount != lastR) {
      lastR = rightEncoderCount;
      lastTickTimeR = millis();
    } else if (rightEncoderCount < currentTargetSlots && (millis() - lastTickTimeR > 400)) {
      Serial.println("RIGHT WHEEL STALL");
      break;
    }

    if (millis() - burstStart > 1500) break;
  }
  stopMotors();
}

//  10. EXTINGUISH SEQUENCE
void extinguishSequence() {
  stopMotors();
  bool fireLeft = (digitalRead(leftFlamePin) == LOW);
  bool fireRight = (digitalRead(rightFlamePin) == LOW);

  if (fireLeft && !fireRight) {
    nozzleServo.write(45);
  } else if (fireRight && !fireLeft) {
    nozzleServo.write(135);
  } else {
    nozzleServo.write(90);
  }

   delay(600);
   digitalWrite(relayPin, HIGH);    //  = coil ON = pump ON
   delay(3000);
   digitalWrite(relayPin, HIGH);   // HIGH = coil OFF = pump OFF
   nozzleServo.write(90);
}