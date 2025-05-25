/* 
 * Project myProject
 * Author: Rasmus Gramstrup
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Straight.h"
// #include "Break.h"
// #include "Left.h"
// #include "Right.h"
#include "../../../random_forest_model.h"
#include "Feature_extraction.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(MANUAL);
// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

#define N_FEATURES 6
int16_t features[N_FEATURES];

#define DATA_LENGTH 100

const int BUFFER_SIZE = 50;

Sample buffer1[BUFFER_SIZE];
Sample buffer2[BUFFER_SIZE];
Sample buffer3[BUFFER_SIZE];

// setup() runs once, when the device is first turned on
void setup() {
  // Put initialization like pinMode and begin functions here
  delay(1000);
  Serial.begin(115200);
  Serial.println("Bike light test example");
  

  for(int i = 0; i < BUFFER_SIZE; i++) {
    buffer1[i].timestamp = 0;
    buffer1[i].accX = samples[0][i];
    buffer1[i].accY = samples[1][i];
    buffer1[i].accZ = samples[2][i];
    buffer1[i].gyroX = samples[3][i];
    buffer1[i].gyroY = samples[4][i];
    buffer1[i].gyroZ = samples[5][i];

    buffer2[i].timestamp = 0;
    buffer2[i].accX = samples[0][i+BUFFER_SIZE];
    buffer2[i].accY = samples[1][i+BUFFER_SIZE];
    buffer2[i].accZ = samples[2][i+BUFFER_SIZE];
    buffer2[i].gyroX = samples[3][i+BUFFER_SIZE];
    buffer2[i].gyroY = samples[4][i+BUFFER_SIZE];
    buffer2[i].gyroZ = samples[5][i+BUFFER_SIZE];

  }

}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.

  int timestamp = millis();
  // extract_features(buffer1, buffer2, DATA_LENGTH, features, N_FEATURES);
  Serial.println(features[0]);
  Serial.println(features[1]);
  Serial.println(features[2]);
  Serial.println(features[3]);
  Serial.println(features[4]);
  Serial.println(features[5]);
  
  int prediction = random_forest_model_predict(features, N_FEATURES);
  Serial.print("Prediction time: ");
  Serial.print(millis() - timestamp);
  Serial.println(" ms");
  Serial.print("Prediction: ");
  switch(prediction) {
    case 0:
      Serial.println("BREAK");
      break;
    case 1:
      Serial.println("LEFT");
      break;
    case 2:
      Serial.println("RIGHT");
      break;
    case 3:
      Serial.println("STRAIGHT");
      break;
    default:
      Serial.println("UNKNOWN");
  }
  Serial.println("Done");

  timestamp = millis();

  features[0] = 4439;
  features[1] = 1452;
  features[2] = 640;
  features[3] = 13687;
  features[4] = 5418;
  features[5] = 10496;

  prediction = random_forest_model_predict(features, N_FEATURES);
  Serial.print("Prediction time: ");
  Serial.print(millis() - timestamp);
  Serial.println(" ms");
  Serial.print("Prediction: ");
  switch(prediction) {
    case 0:
      Serial.println("BREAK");
      break;
    case 1:
      Serial.println("LEFT");
      break;
    case 2:
      Serial.println("RIGHT");
      break;
    case 3:
      Serial.println("STRAIGHT");
      break;
    default:
      Serial.println("UNKNOWN");
  }
  Serial.println("Done");


  while(1);
}
