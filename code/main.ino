/*
  Water level + DHT11 + Servo + 2 Relays
  - Ultrasonic HC-SR04 measures water level (distance in cm)
  - Relay1 controls water pump:
      distance < 5 cm -> RELAY1 OFF (pump off)
      distance >= 5 cm -> RELAY1 ON  (pump on)
  - DHT11 measures temperature & humidity:
      temp > 40.0 C -> RELAY2 ON (fan on)
      else -> RELAY2 OFF (fan off)
  - Servo rotates to 90 degrees every 5 seconds (toggles 0 <-> 90)
  - Serial prints distance, temperature, humidity and relay states

  Wiring example:
  - HC-SR04:
      VCC -> 5V
      GND -> GND
      TRIG -> pin 9
      ECHO -> pin 8
  - DHT11:
      VCC -> 5V (or 3.3V per module)
      GND -> GND
      DATA -> pin 2
  - Relay1 -> pin 3
  - Relay2 -> pin 4
  - Servo -> pin 5

  Libraries required:
    - #include <DHT.h>
    - #include <Servo.h>
*/

#include <DHT.h>
#include <Servo.h>

/* --------- Pin configuration --------- */
const uint8_t TRIG_PIN = 9;
const uint8_t ECHO_PIN = 8;

const uint8_t DHT_PIN  = 2;
#define DHTTYPE DHT11

const uint8_t RELAY1_PIN = 3; // pump
const uint8_t RELAY2_PIN = 4; // fan

const uint8_t SERVO_PIN = 5;

/* --------- Relay logic constants --------- */
// Some relay modules are active LOW (LOW=ON). If so, change as noted below.
const uint8_t RELAY_ON  = HIGH; // set to LOW if your relay module is active LOW
const uint8_t RELAY_OFF = LOW;  // set to HIGH if active LOW

/* --------- Other constants --------- */
const unsigned long SERVO_INTERVAL_MS = 5000UL; // interval between servo toggles
const unsigned long SENSOR_INTERVAL_MS = 1000UL; // sensor read interval

/* --------- Globals --------- */
DHT dht(DHT_PIN, DHTTYPE);
Servo feeder;

unsigned long lastServoToggle = 0;
unsigned long lastSensorRead = 0;
bool servoAt90 = false;

/* Optional smoothing for ultrasonic (simple moving average) */
const uint8_t ULTRA_SAMPLES = 3;
float ultraSamples[ULTRA_SAMPLES];
uint8_t ultraIndex = 0;
bool ultraFilled = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // wait for serial on some boards (optional)
  Serial.println(F("Starting system..."));

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  // initialize relays as OFF
  digitalWrite(RELAY1_PIN, RELAY_OFF);
  digitalWrite(RELAY2_PIN, RELAY_OFF);

  dht.begin();
  feeder.attach(SERVO_PIN);
  feeder.write(0); // start at 0 degrees

  // init ultrasonic samples
  for (uint8_t i = 0; i < ULTRA_SAMPLES; ++i) ultraSamples[i] = 0.0;
  Serial.println(F("Setup complete."));
}

void loop() {
  unsigned long now = millis();

  // sensor read + relay control (every SENSOR_INTERVAL_MS)
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    float distanceCm = readUltrasonicDistance();
    float avgDistanceCm = movingAverageUltra(distanceCm);

    // read DHT
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature(); // Celsius

    // if DHT failed, returns NAN. We'll print and skip relay2 decision if NaN.
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println(F("DHT11 read failed!"));
    }

    // Relay1 logic — water pump
    // If distance < 5 cm -> turn relay1 OFF (pump off)
    // If distance >= 5 cm -> turn relay1 ON (pump on)
    if (!isnan(avgDistanceCm)) {
      if (avgDistanceCm < 5.0) {
        digitalWrite(RELAY1_PIN, RELAY_OFF);
        Serial.print(F("Distance: "));
        Serial.print(avgDistanceCm, 2);
        Serial.println(F(" cm  -> Water level LOW (<5 cm). Motor is off."));
      } else {
        digitalWrite(RELAY1_PIN, RELAY_ON);
        Serial.print(F("Distance: "));
        Serial.print(avgDistanceCm, 2);
        Serial.println(F(" cm  -> Water level OK (>=5 cm). Motor is on."));
      }
    } else {
      Serial.println(F("Ultrasonic read failed or out of range."));
    }

    // Relay2 logic — fan based on temperature
    if (!isnan(temperature)) {
      if (temperature > 40.0) {
        digitalWrite(RELAY2_PIN, RELAY_ON);
        Serial.print(F("Temp: "));
        Serial.print(temperature, 2);
        Serial.print(F(" C  -> Fan is on. "));
      } else {
        digitalWrite(RELAY2_PIN, RELAY_OFF);
        Serial.print(F("Temp: "));
        Serial.print(temperature, 2);
        Serial.print(F(" C  -> Fan is off. "));
      }
    } else {
      Serial.print(F("Temp: N/A  "));
    }

    // Print humidity too
    if (!isnan(humidity)) {
      Serial.print(F(" Humidity: "));
      Serial.print(humidity, 2);
      Serial.println(F(" %"));
    } else {
      Serial.println(F(" Humidity: N/A"));
    }
  }

  // Servo feed toggling (every SERVO_INTERVAL_MS)
  if (now - lastServoToggle >= SERVO_INTERVAL_MS) {
    lastServoToggle = now;
    servoAt90 = !servoAt90;
    if (servoAt90) {
      feeder.write(90);
      Serial.println(F("Servo moved to 90° (dispense)."));
    } else {
      feeder.write(0);
      Serial.println(F("Servo moved to 0° (reset)."));
    }
  }

  // small delay to avoid hammering CPU (non-blocking behavior kept)
  delay(10);
}

/* ---------- Functions ---------- */

// Reads ultrasonic and returns distance in cm. Returns NAN if out-of-range.
float readUltrasonicDistance() {
  // Send 10us pulse to trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo pulse duration (microseconds)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30 ms timeout (~5 m)
  if (duration == 0) {
    // timeout or no echo
    return NAN;
  }

  // distance in cm = duration / 58.2 (approx)
  float distanceCm = (float)duration / 58.2;
  return distanceCm;
}

// Simple moving average for ultrasonic
float movingAverageUltra(float sample) {
  if (!isnan(sample)) {
    ultraSamples[ ultraIndex ] = sample;
    ultraIndex = (ultraIndex + 1) % ULTRA_SAMPLES;
    if (!ultraFilled && ultraIndex == 0) ultraFilled = true;
  } else {
    // If sample is NAN, skip storing — but return average of existing samples
  }

  uint8_t count = ultraFilled ? ULTRA_SAMPLES : ultraIndex;
  if (count == 0) return NAN;

  float sum = 0.0;
  for (uint8_t i = 0; i < count; ++i) sum += ultraSamples[i];
  return sum / (float)count;
}
