
#include <Arduino.h>
#include <EEPROM.h>
#include <QTRSensors.h>

#define NUM_SENSORS         8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             2

QTRSensors qtra;
uint16_t sensorValues[NUM_SENSORS];

// Variables para EEPROM
bool calibracion = false;

// Motor Driver
int VA = 3; int DA = 5; int IA = 4;
int IB = 8; int DB = 7; int VB = 9;

// PID Settings
const double KP = 0.07;
const double KD = 0.3;  // Ajustado un poco más alto para estabilidad
const double KI = 0.001;
const int GOAL = 3500; 
const unsigned char MAX_SPEED = 130;
const unsigned char SEARCH_SPEED = 80; // Velocidad de búsqueda cuando se pierde

// Variables PID
double lastError = 0;
double integral = 0;
const unsigned long SAMPLE_TIME = 0;

void setup() {
  Serial.begin(115200);

  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);
  qtra.setEmitterPin(EMITTER_PIN);

  pinMode(VA, OUTPUT); pinMode(DA, OUTPUT); pinMode(IA, OUTPUT);
  pinMode(IB, OUTPUT); pinMode(DB, OUTPUT); pinMode(VB, OUTPUT);
  
  calibrateLineSensor();
}

void loop() {
  static unsigned long lastTime = 0;
  
  if (millis() - lastTime >= SAMPLE_TIME) {
    lastTime = millis();

    // Leemos la posición
    uint16_t position = qtra.readLineBlack(sensorValues);

    // --- LÓGICA DE DETECCIÓN DE PÉRDIDA ---
    bool onLine = false;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > 600) { // Umbral para detectar negro (ajustable)
        onLine = true;
        break;
      }
    }

    if (!onLine) {
      // Si se perdió, girar en el lugar basándose en el último error
      if (lastError > 0) {
        // Estaba hacia la derecha, girar a la izquierda
        motorRun(-SEARCH_SPEED, SEARCH_SPEED);
      } else {
        // Estaba hacia la izquierda, girar a la derecha
        motorRun(SEARCH_SPEED, -SEARCH_SPEED);
      }
      return; // Saltamos el resto del PID en este ciclo
    }

    // --- CÁLCULO PID ---
    int error = GOAL - position;
    integral = constrain(integral + error, -1000, 1000);
    int adjustment = (KP * error) + (KD * (error - lastError)) + (KI * integral);
    lastError = error;

    int setMotorAPower = constrain(MAX_SPEED - adjustment, 0, MAX_SPEED);
    int setMotorBPower = constrain(MAX_SPEED + adjustment, 0, MAX_SPEED);

    motorRun(setMotorAPower, setMotorBPower);
    motorRun(setMotorAPower, setMotorBPower);
  }
}

// Función auxiliar para mover motores de forma sencilla
void motorRun(int speedA, int speedB) {
  if (speedA >= 0) {
    digitalWrite(DA, HIGH); digitalWrite(IA, LOW);
  } else {
    digitalWrite(DA, LOW); digitalWrite(IA, HIGH);
    speedA = -speedA;
  }

  if (speedB >= 0) {
    digitalWrite(IB, HIGH); digitalWrite(DB, LOW);
  } else {
    digitalWrite(IB, LOW); digitalWrite(DB, HIGH);
    speedB = -speedB;
  }

  analogWrite(VA, speedA);
  analogWrite(VB, speedB);
}

// --- FUNCIONES DE CALIBRACIÓN Y EEPROM ---

void calibrateLineSensor() {
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  if(calibracion) {
      for (uint16_t i = 0; i < 400; i++) {
        qtra.calibrate();
      }
      guardarCalibracion();
  } else {
    cargarCalibracion();
  }
  
  digitalWrite(13, LOW);
}

void guardarCalibracion(){
  for(int i = 0; i < NUM_SENSORS; i++){
    EEPROM.put(i * sizeof(uint16_t), qtra.calibrationOn.minimum[i]);
    EEPROM.put((i + 8) * sizeof(uint16_t), qtra.calibrationOn.maximum[i]);
  }
}

void cargarCalibracion(){
  qtra.calibrate(); 
  for(int i = 0; i < NUM_SENSORS; i++){
    EEPROM.get(i * sizeof(uint16_t), qtra.calibrationOn.minimum[i]);
    EEPROM.get((i + 8) * sizeof(uint16_t), qtra.calibrationOn.maximum[i]);
  }
}
