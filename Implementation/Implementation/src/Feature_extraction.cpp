#include "Feature_extraction.h"

#define accMax 50
#define accMin -50
#define gyroMax 5
#define gyroMin -5

// Function to compare integers for qsort
int compare(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

// Function to calculate Q1 and Q3
double calculateQuartile(int16_t *arr, int n, double position) {
    int index = floor(position);
    double fraction = position - index;

    if (fraction == 0) {
        return arr[index - 1];
    } else {
        return arr[index - 1] * (1 - fraction) + arr[index] * fraction;
    }
}

void extract_features(Sample *data1, Sample *data2, int16_t data_length, int16_t *features, int16_t n_features){

    int16_t data[6][100]; // Assuming 6 data types: accX, accY, accZ, gyroX, gyroY, gyroZ
    float std_accX, std_accY, std_accZ, std_gyroX, std_gyroY, std_gyroZ;

    // This function extracts features from the provided data samples.
    // The implementation details are not provided in the original code snippet.
    // You would typically implement feature extraction logic here based on your requirements.
    
    // Example placeholder logic (to be replaced with actual feature extraction):
    for(int i = 0; i < data_length; i++){
        std_accX = (data1->accX - accMin) / (accMax - accMin);
        data[0][i] = (int16_t)(std_accX * (65535));

        std_accY = (data1->accY - accMin) / (accMax - accMin);
        data[1][i] = (int16_t)(std_accY * (65535));
        
        std_accZ = (data1->accZ - accMin) / (accMax - accMin);
        data[2][i] = (int16_t)(std_accZ * (65535));

        std_gyroX = (data1->gyroX - accMin) / (accMax - accMin);
        data[3][i] = (int16_t)(std_gyroX * (65535));

        std_gyroY = (data1->gyroY - accMin) / (accMax - accMin);
        data[4][i] = (int16_t)(std_gyroY * (65535));
        
        std_gyroZ = (data1->gyroZ - accMin) / (accMax - accMin);
        data[5][i] = (int16_t)(std_gyroZ * (65535));

        std_accX = (data2->accX - accMin) / (accMax - accMin);
        data[0][i+data_length] = (int16_t)(std_accX * (65535));

        std_accY = (data2->accY - accMin) / (accMax - accMin);
        data[1][i+data_length] = (int16_t)(std_accY * (65535));
        
        std_accZ = (data2->accZ - accMin) / (accMax - accMin);
        data[2][i+data_length] = (int16_t)(std_accZ * (65535));

        std_gyroX = (data2->gyroX - accMin) / (accMax - accMin);
        data[3][i+data_length] = (int16_t)(std_gyroX * (65535));

        std_gyroY = (data2->gyroY - accMin) / (accMax - accMin);
        data[4][i+data_length] = (int16_t)(std_gyroY * (65535));
        
        std_gyroZ = (data2->gyroZ - accMin) / (accMax - accMin);
        data[5][i+data_length] = (int16_t)(std_gyroZ * (65535));
    }
    
    //features

    double x_mean = 0, y_mean = 0, z_mean = 0, sma_gyro = 0;
    double q1, q3, iqr;
    int16_t gyroX_max = 0;

    for(int i = 0; i < data_length*2; i++){
        x_mean += data[0][i]; // xa_mean
        y_mean += data[1][i]; // ya_mean
        z_mean += data[5][i]; // zg_mean
    }

    features[0] = (int16_t)(x_mean / (data_length*2));      //xa_mean
    features[1] = (int16_t)(y_mean / (data_length*2));      //ya_mean
    features[2] = (int16_t)(z_mean / (data_length*2));      //zg_mean

    for(int i = 0; i < data_length*2; i++){
        if (data[3][i] > gyroX_max){
            gyroX_max = data[3][i];
        }
    }

    features[3] = gyroX_max;                                 //xg_max
    
    #if 0

    qsort(data[3], data_length*2, sizeof(int16_t), compare);

    // Calculate Q1 and Q3 positions
    double q1_pos = 0.25 * (data_length*2 + 1);
    double q3_pos = 0.75 * (data_length*2 + 1);

    // Calculate Q1 and Q3
    q1 = calculateQuartile(data[3], data_length*2, q1_pos);
    q3 = calculateQuartile(data[3], data_length*2, q3_pos);

    // Calculate IQR
    iqr = q3 - q1;


    features[4] = (int16_t)iqr;

    

    for(int i = 0; i < data_length*2; i++){
        if(data[3][i] < 0){
            sma_gyro += data[3][i]/-100;
        }else{
            sma_gyro += data[3][i]/100;
        }
        if(data[4][i] < 0){
            sma_gyro += data[4][i]/-100;
        }else{
            sma_gyro += data[4][i]/100;
        }
        if(data[5][i] < 0){
            sma_gyro += data[5][i]/-100;
        }else{
            sma_gyro += data[5][i]/100;
        }
    }

    

    features[5] = (int16_t)sma_gyro; //sma_gyro

    #endif
}