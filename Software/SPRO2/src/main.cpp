#include <MAX3010x.h>
#include "filters.h"
#include <math.h>

MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

const unsigned long kFingerThreshold = 8000;
const unsigned int kFingerCooldownMs = 500;
const float kEdgeThreshold = -2000.0;

const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

const bool kEnableAveraging = true;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

long last_heartbeat = 0;
long finger_timestamp = 0;
bool finger_detected = false;

float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

//temp
const int lm35Pin0 = A0;
int rawTemp0;
float temp;

//humidity
const int humPin = A1;
unsigned int inputHumidity;
float capVolt=1,opampFactor,offset,z,rh,scalingFactor,resExp;
unsigned char tempSet;

void initializeSensor() {
  Serial.begin(9600);
  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("Sensor initialized");
  } else {
    Serial.println("Sensor not found");
    while (1);
  }
}

void resetFilters() {
  differentiator.reset();
  averager_bpm.reset();
  averager_r.reset();
  averager_spo2.reset();
  low_pass_filter_red.reset();
  low_pass_filter_ir.reset();
  high_pass_filter.reset();
  stat_red.reset();
  stat_ir.reset();
  finger_detected = false;
}

bool detectFinger(unsigned long redValue) {
  if (redValue > kFingerThreshold) {
    if (millis() - finger_timestamp > kFingerCooldownMs) {
      return true;
    }
  } else {
    resetFilters();
    finger_timestamp = millis();
  }
  return false;
}

void processData(float redFiltered, float irFiltered) {
  stat_red.process(redFiltered);
  stat_ir.process(irFiltered);

  float signal = high_pass_filter.process(redFiltered);
  float current_diff = differentiator.process(signal);

  if (!isnan(current_diff) && !isnan(last_diff)) {
    if (last_diff > 0 && current_diff < 0) {
      crossed = true;
      crossed_time = millis();
    }

    if (current_diff > 0) {
      crossed = false;
    }

    if (crossed && current_diff < kEdgeThreshold) {
      if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
        int bpm = 60000 / (crossed_time - last_heartbeat);
        float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
        float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
        float r = rred / rir;
        float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
        spo2 = constrain(spo2, 0.0, 100.0);

        if (bpm > 50 && bpm < 250) {
          if (kEnableAveraging) {
            int average_bpm = averager_bpm.process(bpm);
            int average_spo2 = averager_spo2.process(spo2);

            if (averager_bpm.count() >= kSampleThreshold) {
              Serial.print("HR (avg): "); Serial.println(average_bpm);
              Serial.print("SpO2 (avg): "); Serial.println(average_spo2);
              Serial.print("Temp: "); Serial.println(temp);
              Serial.print("Humidity: "); Serial.println(rh);
            }
          } else {
            Serial.print("HR: "); Serial.println(bpm);
            Serial.print("SpO2: "); Serial.println(spo2);
            Serial.print("Temp: "); Serial.println(temp);
            Serial.print("Humidity: "); Serial.println(rh);
          }
        }
        stat_red.reset();
        stat_ir.reset();
      }
      crossed = false;
      last_heartbeat = crossed_time;
    }
  }

  last_diff = current_diff;
}

void humiditySetup(){
  inputHumidity = analogRead(humPin);
  opampFactor = 24/7;
  offset = 7/30;
  tempSet = 20;
  resExp=(0.0187)*((float)tempSet)-5.68;
  scalingFactor = (1.286e+12)*exp((-0.112)*((float)tempSet));

  //z=(vcp*m*47)/((5/1024)*((double)input)-b)-47;

  z=(197.4*1024)/(5*((float)inputHumidity)-238.93)-47;
  rh=pow((z),(1/resExp))/pow((scalingFactor),(1/resExp));
}

void setup() {
  pinMode(lm35Pin0, INPUT);
  pinMode(humPin, INPUT);
  analogReference(DEFAULT);
  initializeSensor();
}

void loop() {
  auto sample = sensor.readSample(1000);
  float red = sample.red;
  float ir = sample.ir;

  finger_detected = detectFinger(red);

  if (finger_detected) {
    float redFiltered = low_pass_filter_red.process(red);
    float irFiltered = low_pass_filter_ir.process(ir);
    processData(redFiltered, irFiltered);
  }

  humiditySetup();
  rawTemp0 = analogRead(lm35Pin0);
  temp = rawTemp0 * 5*10.0/1023;
}