#include <EEPROM.h>
#include <QTRSensors.h>

#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             2

// Objeto qtra sin parámetros en el constructor
QTRSensors qtra;
// La librería moderna utiliza uint16_t para los valores
uint16_t sensorValues[NUM_SENSORS];

// Variables para EEPROM
uint16_t minValues[8], maxValues[8];

bool calibracion = false;
// Motor Driver
int VA = 3; int DA = 5; int IA = 4;
int IB = 8; int DB = 7; int VB = 9;

// PID Settings
const double KP = 0.07;
const double KD = 0.03;
const double KI = 0.02;
const int GOAL = 3530; // Valor medio para 6 sensores (0 a 5000)
const unsigned char MAX_SPEED = 130;

// Variables PID
double lastError = 0;
double integral = 0;
const unsigned long SAMPLE_TIME = 0;

void setup() {
  Serial.begin(115200);

  // --- CORRECCIÓN: Configuración de sensores en setup ---
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);
  qtra.setEmitterPin(EMITTER_PIN);
  // -------------------------------------------------------

  pinMode(VA, OUTPUT); pinMode(DA, OUTPUT); pinMode(IA, OUTPUT);
  pinMode(IB, OUTPUT); pinMode(DB, OUTPUT); pinMode(VB, OUTPUT);
  
  calibrateLineSensor();
}

void loop() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= SAMPLE_TIME) {
    lastTime = millis();

    // --- CORRECCIÓN: readLine ahora es readLineBlack ---
    uint16_t position = qtra.readLineBlack(sensorValues);
    // ---------------------------------------------------
    
    int error = GOAL - position;

    integral = constrain(integral + error, -1000, 1000);
    int adjustment = KP * error + KD * (error - lastError) + KI * integral;
    lastError = error;

    int setMotorAPower = constrain(MAX_SPEED - adjustment, 0, MAX_SPEED);
    int setMotorBPower = constrain(MAX_SPEED + adjustment, 0, MAX_SPEED);

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
  }
  delay(2);
}

void calibrateLineSensor() {
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  if(calibracion) {
      for (uint16_t i = 0; i < 400; i++) {
        qtra.calibrate();
        
        // Movimiento oscilatorio simple
        digitalWrite(DA, i % 100 < 50 ? HIGH : LOW);
        digitalWrite(IA, i % 100 < 50 ? LOW : HIGH);
        analogWrite(VA, 30);
        delay(10);
      }
      analogWrite(VA, 0);
      guardarCalibracion();
  } else {
    cargarCalibracion();
  }
  
  digitalWrite(13, LOW);
}

void guardarCalibracion(){
  for(int i = 0; i < NUM_SENSORS; i++){
    // --- CORRECCIÓN: Acceso a calibrationOn.minimum/maximum ---
    EEPROM.put(i * sizeof(uint16_t), qtra.calibrationOn.minimum[i]);
    EEPROM.put((i + 8) * sizeof(uint16_t), qtra.calibrationOn.maximum[i]);
  }
}

void cargarCalibracion(){
  // Es necesario inicializar la estructura interna antes de cargar datos
  qtra.calibrate(); 
  
  for(int i = 0; i < NUM_SENSORS; i++){
    EEPROM.get(i * sizeof(uint16_t), qtra.calibrationOn.minimum[i]);
    EEPROM.get((i + 8) * sizeof(uint16_t), qtra.calibrationOn.maximum[i]);
  }
}