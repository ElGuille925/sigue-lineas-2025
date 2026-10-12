#include <EEPROM.h>
#include <QTRSensors.h>

#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             2

QTRSensors qtra;
uint16_t sensorValues[NUM_SENSORS];

// Variables para EEPROM
uint16_t minValues[8], maxValues[8];
bool calibracion = false;

// Motor Driver
int VA = 3; int DA = 7; int IA = 6;
int IB = 8; int DB = 9; int VB = 5;

// PID Settings
const double KP = 0.05;
const double KD = 0.9;
const double KI = 0.0005;
const int GOAL = 3500; // Centro ideal para 8 sensores (0 a 7000)
const unsigned char MAX_SPEED = 100;

// Variables PID
double lastError = 0;
double integral = 0;
const unsigned long SAMPLE_TIME = 0;

void setup() {
  Serial.begin(115200);

  // Configuración de sensores
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A4, A3, A5, A6, A7}, NUM_SENSORS);
  qtra.setEmitterPin(EMITTER_PIN);

  // Configuración de motores
  pinMode(VA, OUTPUT); pinMode(DA, OUTPUT); pinMode(IA, OUTPUT);
  pinMode(IB, OUTPUT); pinMode(DB, OUTPUT); pinMode(VB, OUTPUT);
  
  calibrateLineSensor();
}

void loop() {
  static unsigned long lastTime = 0;
  
  if (millis() - lastTime >= SAMPLE_TIME) {
    lastTime = millis();

    // 1. Lectura de posición (0 a 7000 para 8 sensores)
    uint16_t position = qtra.readLineBlack(sensorValues);



    // --- LÓGICA PID NORMAL ---

    int error = GOAL - position;
    integral = constrain(integral + error, -1000, 1000);
    int adjustment = (KP * error) + (KD * (error - lastError)) + (KI * integral);
    lastError = error;

    int setMotorAPower = constrain(MAX_SPEED - adjustment, 0, MAX_SPEED);
    int setMotorBPower = constrain(MAX_SPEED + adjustment, 0, MAX_SPEED);

    // Direcciones base (Adelante)
    digitalWrite(DA, HIGH);
    digitalWrite(IB, HIGH);
    digitalWrite(IA, LOW);
    digitalWrite(DB, LOW);
    
    analogWrite(VA, setMotorAPower);
    analogWrite(VB, setMotorBPower);

    // Depuración
    Serial.print("Pos: "); Serial.print(position);
    Serial.print(" | Error: "); Serial.println(error);
    Serial.print("Motor A: "); Serial.print(setMotorAPower);
    Serial.print(" | MOtor B: "); Serial.println(setMotorBPower);
    Serial.print("Ajuste: "); Serial.println(adjustment);
    for(int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(sensorValues[i]); Serial.print(" ");
    }

  }
  delay(200);
}

// --- FUNCIONES DE CALIBRACIÓN Y EEPROM ---

void calibrateLineSensor() {

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