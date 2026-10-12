
#include <EEPROM.h>
#include <QTRSensors.h>

#define NUM_SENSORS             6
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             2


int vals[6];
QTRSensors qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

unsigned int minValues[6], maxValues[6];

bool calibracion = true;
// Motor Driver
int VA = 3; int DA = 5; int IA = 4;
int IB = 8; int DB = 7; int VB = 9;

// PID Settings - Valores optimizados
const double KP = 0.07;
const double KD = 0.02;
const double KI = 0.01;
const int GOAL = 2600;
const unsigned char MAX_SPEED = 100; // Velocidad reducida para mejor control

// Variables PID
double lastError = 0;
double integral = 0;
const unsigned long SAMPLE_TIME = 20;

void setup() {
  Serial.begin(9600);
  pinMode(VA, OUTPUT); pinMode(DA, OUTPUT); pinMode(IA, OUTPUT);
  pinMode(IB, OUTPUT); pinMode(DB, OUTPUT); pinMode(VB, OUTPUT);
  
  calibrateLineSensor();
}

void loop() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= SAMPLE_TIME) {
    lastTime = millis();

    unsigned int position = qtra.readLine(sensorValues);
    int error = GOAL - position;

    // Cálculo PID mejorado
    integral = constrain(integral + error, -1000, 1000); // Límites más estrictos
    int adjustment = KP * error + KD * (error - lastError) + KI * integral;
    lastError = error;

    // Control de motores simétrico
    int setMotorAPower = constrain(MAX_SPEED + adjustment, 0, MAX_SPEED);
    int setMotorBPower = constrain(MAX_SPEED - adjustment, 0, MAX_SPEED);

    digitalWrite(DA, HIGH);
    digitalWrite(IB, HIGH);
    digitalWrite(IA, LOW);
    digitalWrite(DB, LOW);
    
    analogWrite(VA, setMotorAPower);
    analogWrite(VB, setMotorBPower);

    // Mejor salida de depuración
    Serial.print("Motores: "); 
    Serial.print(setMotorAPower); Serial.print(","); Serial.print(setMotorBPower);
    Serial.print(" | Error: "); Serial.print(error);
    Serial.print(" | Ajuste: "); Serial.print(adjustment);
    Serial.print(" | Sensores: ");
    for(int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(sensorValues[i]); Serial.print(" ");
    }
    Serial.println();
  }
  delay(2);
}

void calibrateLineSensor() {
  delay(500);

  pinMode(11, OUTPUT);
 digitalWrite(11, HIGH);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(12, LOW);
  if(calibracion) {
      for (int i = 0; i < 400; i++) { // Más iteraciones para mejor calibración
      qtra.calibrate();

      // Movimiento oscilatorio durante calibración
      digitalWrite(DA, i % 100 < 50 ? HIGH : LOW);
      digitalWrite(IA, i % 100 < 50 ? LOW : HIGH);
      analogWrite(VA, 30);
      delay(10);
    }

    for(uint8_t i = 0; i < 6; i++){
        minValues[i] = qtra.calibratedMinimumOn[i];
        maxValues[i] = qtra.calibratedMaximumOn[i];
      }
      guardarCalibracion();

  } else {
    cargarCalibracion();
  }
  
  digitalWrite(13, LOW);
  digitalWrite(12, HIGH);
  analogWrite(VA, 0);
}

void guardarCalibracion(){
  for(int i = 0; i < 6; i++){
    EEPROM.put(i* sizeof(int), minValues[i]);
    EEPROM.put((i+6)* sizeof(int), maxValues[i]);
  }
}

void cargarCalibracion(){
  for(int i = 0; i < 6; i++){
    EEPROM.get(i * sizeof(int), minValues[i]);
    EEPROM.get((i+6)* sizeof(int), maxValues[i]);
  }

  qtra.calibratedMinimumOn = minValues;
  qtra.calibratedMaximumOn = maxValues;
}
