#include <stdio.h>
#include <stdlib.h>
#include <math.h>


 // Buffers for double buffering
 struct Sample
 {
   uint32_t timestamp; // Timestamp in milliseconds
   float accX,accY,accZ,gyroX,gyroY,gyroZ;      // BNO085 data
 }; 

 int compare(const void *a, const void *b);
double calculateQuartile(int16_t *arr, int n, double position);
void extract_features(Sample *data1, Sample *data2, int16_t data_length, int16_t *features, int16_t n_features);

