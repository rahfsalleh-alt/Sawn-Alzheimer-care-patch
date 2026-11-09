#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <math.h>

// ========== GPS ==========

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#define GPS_RX_PIN 6
#define GPS_TX_PIN 7

const double HOME_LAT = 21.492331;
const double HOME_LNG = 39.241800;
const double HOME_RADIUS_METERS = 5.0;

bool isOutside = false;
bool firstGPSReading = true;

double currentLat = 0;
double currentLng = 0;
double currentDistance = 0;
bool hasValidLocation = false;
bool locationUpdatedSincePrint = false;
unsigned long lastLocationPrint = 0;

// ========== HEART SENSOR ==========

#define HEART_PIN 4

const int THRESHOLD = 2500;
const int MIN_INTERVAL = 300;
const int MAX_INTERVAL = 2000;

unsigned long lastBeat = 0;
unsigned long lastPrint = 0;

int lastValue = 0;
int bpm = 0;

bool heartAlertSent = false;

// ========== MPU6050 ==========

Adafruit_MPU6050 mpu;

const float FALL_THRESHOLD_G = 1.5;

unsigned long lastMPURead = 0;

bool fallAlertSent = false;

// ========== SETUP ==========

void setup() {

  Serial.begin(115200);

  gpsSerial.begin(9600, SERIAL_8N1,
                  GPS_RX_PIN, GPS_TX_PIN);

  pinMode(HEART_PIN, INPUT);

  Wire.begin(8, 9);

  if (!mpu.begin()) {
    Serial.println("MPU6050 ERROR");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("SON SYSTEM STARTED");
  Serial.println("Waiting for sensors...");
}

// ========== LOOP ==========

void loop() {

  unsigned long now = millis();

  // -------- GPS --------

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated() &&
      gps.location.isValid()) {

    double lat = gps.location.lat();
    double lng = gps.location.lng();

    double distance = TinyGPSPlus::distanceBetween(
      lat, lng, HOME_LAT, HOME_LNG
    );

    if (!hasValidLocation ||
        lat != currentLat ||
        lng != currentLng) {

      currentLat = lat;
      currentLng = lng;
      currentDistance = distance;
      hasValidLocation = true;
      locationUpdatedSincePrint = true;
    }

    if (distance <= HOME_RADIUS_METERS) {

      isOutside = false;

      if (firstGPSReading) {
        Serial.println("المريض داخل المنزل");
      }

    } else {

      if (!isOutside) {
        isOutside = true;
        Serial.println("تنبيه: تم رصد خروج المريض من المنزل");
      }
    }

    firstGPSReading = false;
  }

  if (hasValidLocation &&
      locationUpdatedSincePrint &&
      (lastLocationPrint == 0 ||
       now - lastLocationPrint >= 5000)) {

    Serial.println("موقع المريض:");

    Serial.print("https://maps.google.com/?q=");
    Serial.print(currentLat, 6);
    Serial.print(",");
    Serial.println(currentLng, 6);

    Serial.print("Distance from home: ");
    Serial.print(currentDistance);
    Serial.println(" meters");

    Serial.println("----------------------");

    lastLocationPrint = now;
    locationUpdatedSincePrint = false;
  }

  // -------- HEART RATE --------

  int value = analogRead(HEART_PIN);

  if (value > THRESHOLD &&
      lastValue <= THRESHOLD) {

    if (lastBeat != 0) {

      unsigned long interval = now - lastBeat;

      if (interval >= MIN_INTERVAL &&
          interval <= MAX_INTERVAL) {

        bpm = 60000 / interval;
      }
    }

    lastBeat = now;
  }

  lastValue = value;

  if (now - lastPrint >= 1000) {

    if (bpm > 0) {

      Serial.print("BPM:");
      Serial.println(bpm);

      if (bpm < 60) {

        if (!heartAlertSent) {
          Serial.println("تنبيه: معدل نبض القلب منخفض.");
          heartAlertSent = true;
        }

      }
      else if (bpm > 100) {

        if (!heartAlertSent) {
          Serial.println("تنبيه: معدل نبض القلب مرتفع.");
          heartAlertSent = true;
        }

      }
      else {

        Serial.println("معدل نبض القلب ضمن المعدل الطبيعي.");
        heartAlertSent = false;

      }

    } else {
      Serial.println("بانتظار قراءة النبض...");
    }

    Serial.println("--------------------");

    lastPrint = now;
  }

  // -------- FALL DETECTION --------

  if (now - lastMPURead >= 100) {

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float totalAccelG = sqrt(
      sq(a.acceleration.x) +
      sq(a.acceleration.y) +
      sq(a.acceleration.z)
    ) / 9.81;

    if (totalAccelG > FALL_THRESHOLD_G) {

      if (!fallAlertSent) {

        Serial.println("تنبيه اشتباه سقوط المريض");

        fallAlertSent = true;
      }

    } else {
      fallAlertSent = false;
    }

    lastMPURead = now;
  }

  delay(10);
}
