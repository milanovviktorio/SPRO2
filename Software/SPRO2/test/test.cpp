#include <MAX3010x.h>
#include "filters.h"
#include <math.h>

#define SIZE 25

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

int average_bpm; 
float average_spo2;

float stressScore;

//temp
const int lm35Pin0 = A0;
int rawTemp0;
float temp;

//humidity
const int humPin = A1;
unsigned int inputHumidity;
float capVolt=1,opampFactor,offset,z,rHumidity,scalingFactor,resExp;
unsigned char tempSet;
unsigned int total;
unsigned int humiditySamples[15];

//calibration
int calibrationData[4][SIZE];
float means[4];
float stdDevs[4];

void initializeSensor() {
  Serial.begin(9600);
  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("Sensor initialized");
  } else {
    Serial.println("Sensor not found");
    while (1);
  }
}

void humiditySetup(){
  total=0;
  for(int i=0;i<15;i++)
  {
    humiditySamples[i]=analogRead(humPin);
    total+=humiditySamples[i];
  }
  inputHumidity = total/15;
  opampFactor = 21/4;
  offset = 7/30;
  tempSet = 20;
  capVolt=0.85;
  resExp=(0.0187)*(temp)-5.68;
  scalingFactor = (1.286e+12)*exp((-0.112)*(temp));

  //z=(vcp*m*47)/((5/1024)*((double)input)-b)-47;

  z=(capVolt*opampFactor*1024*47)/((4.8)*((float)inputHumidity)-offset*1024)-47;

  rHumidity=pow((z),(1/resExp))/pow((scalingFactor),(1/resExp));
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

void printValues(int bpm, float spo2) {
  Serial.print("HR: "); Serial.println(bpm);
  Serial.print("SpO2: "); Serial.println(spo2);
  Serial.print("Temp: "); Serial.println(temp);
  Serial.print("Humidity: "); Serial.println(rHumidity);
  Serial.print("Stress Score: "); Serial.println(stressScore);
  delay(500);
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

void computeStats() {
  for (int i = 0; i < 4; i++) {
    float sum = 0;
    for (int j = 0; j < SIZE; j++) {
      sum += calibrationData[i][j];
    }

    means[i] = sum / SIZE;

    float varianceSum = 0;

    for (int j = 0; j < SIZE; j++) {
      varianceSum += pow(calibrationData[i][j] - means[i], 2);
    }
    stdDevs[i] = sqrt(varianceSum / SIZE);
  }
}

float getZScore(int index, float value) {
  if (stdDevs[index] == 0) return 0;
  return (value - means[index]) / stdDevs[index];
}

float computeStressScore(float z[]) {
  // Signed combination: HR (+), Temp (−), Hum (+), SpO2 (−)
  float stressIndex = z[0] - z[1] + z[2] - z[3];
  float stressScore = constrain(50 + 10 * stressIndex, 0, 100);
  return stressScore;
}

float processData(float redFiltered, float irFiltered) {
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
            average_bpm = averager_bpm.process(bpm);
            average_spo2 = averager_spo2.process(spo2);

            if (averager_bpm.count() >= kSampleThreshold) {
              return average_bpm, average_spo2;
              printValues(average_bpm, average_spo2);
            }

          } else {
            return bpm, spo2;
            printValues(bpm, spo2);
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

void calibration()  {
  auto sample = sensor.readSample(1000);
  float red = sample.red;
  float ir = sample.ir;

  humiditySetup();
  rawTemp0 = analogRead(lm35Pin0);
  temp = rawTemp0 * 5*10.0/1023;

  if (detectFinger(red)) {
    float redFiltered = low_pass_filter_red.process(red);
    float irFiltered = low_pass_filter_ir.process(ir);
    processData(redFiltered, irFiltered);

    for (int i = 0; i < SIZE; i++) {
      calibrationData[0][i] = average_bpm;
      calibrationData[1][i] = temp;
      calibrationData[2][i] = rHumidity;
      calibrationData[3][i] = average_spo2;
    }
    computeStats();
  }
}

void setup() {
  pinMode(lm35Pin0, INPUT);
  pinMode(humPin, INPUT);
  analogReference(DEFAULT);
  initializeSensor();
  calibration();
}

void loop() {
  auto sample = sensor.readSample(1000);
  float red = sample.red;
  float ir = sample.ir;
  
  rawTemp0 = analogRead(lm35Pin0);
  temp = rawTemp0 * 5*10.0/1023;

  if (detectFinger(red)) {
    float redFiltered = low_pass_filter_red.process(red);
    float irFiltered = low_pass_filter_ir.process(ir);
    processData(redFiltered, irFiltered);

    float zScores[4];
    zScores[0] = getZScore(0, average_bpm);
    zScores[1] = getZScore(1, temp);
    zScores[2] = getZScore(2, rHumidity);
    zScores[3] = getZScore(3, average_spo2);
    stressScore = computeStressScore(zScores);

    printValues(average_bpm, average_spo2);
  }
}