#include <MAX3010x.h>
#include "filters.h"
#include <math.h>

#define SIZE 15

MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

const unsigned long kFingerThreshold = 9000;
const unsigned int kFingerCooldownMs = 500;
const float kEdgeThreshold = -2000.0;

const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

bool calibrationEnable = true;
bool printMeansAndStddevs = true;
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 2;

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
int average_spo2;
int bpm;
float spo2;
float redFiltered, irFiltered;
float stressScore;
float w_hr, w_temp, w_hum, w_spo2;   

//temp
const int lm35Pin0 = A0;
float rawTemp0;
float temp;
int i = 0;

//humidity
const int humPin = A1;
unsigned int inputHumidity;
float capOffset, capVolt, capFactor,opampFactor,offset,z,rHumidity,scalingFactor,resExp;
unsigned char tempSet;
float total;
float humiditySamples[15];

//calibration
float calibrationData[4][SIZE];
float zScores[4];
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

void humidty(){
  total=0;
  for(int i=0;i<15;i++)
  {
    humiditySamples[i]=analogRead(humPin);
    total+=humiditySamples[i];
  }
  inputHumidity = total/15;
  opampFactor = 4.75;
  offset = 0.2325;
  tempSet = 55;
  capOffset=1.28;
  capFactor=-6e-04;
  capVolt=capFactor*inputHumidity+capOffset;
  resExp=(0.0187)*(tempSet)-5.68;
  scalingFactor = (1.286e+12)*exp((-0.112)*(tempSet));

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

void computeStats() {
  for (int i = 0; i < 4; i++) {
    float sum = 0;
    for (int j = 0; j < SIZE; j++) {
      sum += (float)calibrationData[i][j];
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

  int offset = 50;

  // Signed combination: HR (+), Temp (−), Hum (+), SpO2 (−)
  float stressIndex = w_hr*z[0] - w_temp*z[1] + w_hum*z[2] - w_spo2*z[3];

  float stressScore = constrain(stressIndex + offset, 0, 100);
  return stressScore;
}

void printValues(float bpm, float spo2) {
  if (calibrationEnable)
  {
    if (i == 0)
    {
      Serial.println("Calibration Data: ");
    }
    calibrationData[0][i] = bpm;
    calibrationData[1][i] = temp;
    calibrationData[2][i] = rHumidity;
    calibrationData[3][i] = spo2;
    for (int j = 0; j < 4; j++) {
      Serial.print(calibrationData[j][i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  else
  {
    computeStats();
    if (printMeansAndStddevs)
    {
      Serial.println();
      Serial.println("Means: " + String(means[0]) + " " + String(means[1]) + " " + String(means[2]) + " " + String(means[3]));
      Serial.println("StdDevs: " + String(stdDevs[0]) + " " + String(stdDevs[1]) + " " + String(stdDevs[2]) + " " + String(stdDevs[3]));

      w_hr   =  5 / stdDevs[0];   
      w_temp =  1 / (stdDevs[1] * 10);   
      w_hum  =  1 / (stdDevs[2] * 10);   
      w_spo2 =  5 / stdDevs[3];

      Serial.println("Weights: " + String(w_hr) + " " + String(w_temp) + " " + String(w_hum) + " " + String(w_spo2));
      Serial.println();
      printMeansAndStddevs = false;
    }
    zScores[0] = getZScore(0, bpm);
    zScores[1] = getZScore(1, temp);
    zScores[2] = getZScore(2, rHumidity);
    zScores[3] = getZScore(3, spo2);
    Serial.println("zScores: " + String(zScores[0]) + " " + String(zScores[1]) + " " + String(zScores[2]) + " " + String(zScores[3]));
    stressScore = computeStressScore(zScores);
    Serial.print("HR: "); Serial.println(bpm);
    Serial.print("Temp: "); Serial.println(temp);
    Serial.print("Humidity: "); Serial.println(rHumidity);
    Serial.print("SpO2: "); Serial.println(spo2);
    Serial.print("Stress Score: "); Serial.print(stressScore);
    if (stressScore > 80)
    {
      Serial.println(" Stress Detected");
    }
    else
    {
      Serial.println();
    }
  }
  i++;
  if (i >= SIZE)
  {
    calibrationEnable = false;
  }
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
        bpm = 60000 / (crossed_time - last_heartbeat);
        float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
        float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
        float r = rred / rir;
        spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
        spo2 = constrain(spo2, 0.0, 100.0);

        if (bpm > 50 && bpm < 250) {
          if (kEnableAveraging) {
            average_bpm = averager_bpm.process(bpm);
            average_spo2 = averager_spo2.process(spo2);

            if (averager_bpm.count() >= kSampleThreshold) {
              printValues(average_bpm, average_spo2);
            }
          } else {
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

float calculateAverageStress(float stressArray[]) {
  float sum = 0;
  float averageStress;
  for (int i = 0; i < 5; i++) {
    sum += stressArray[i];
  }
  return averageStress = sum / 5;
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
  
  humidty();
  rawTemp0 = analogRead(lm35Pin0);
  temp = rawTemp0 * 5*10.0/1023;

  if (detectFinger(red)) {
    redFiltered = low_pass_filter_red.process(red);
    irFiltered = low_pass_filter_ir.process(ir);

    processData(redFiltered, irFiltered);
  }
}