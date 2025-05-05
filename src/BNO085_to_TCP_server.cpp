/**
 * @file BNO085_to_TCP_server.cpp
 * @author Rasmus Gramstrup
 * @brief
 * @version 0.1
 * @date 2025-04-25
 * 
 * Modified from original code by Morten Opprud
 */

 #include "Particle.h"
 #include <Wire.h>
 #include "SparkFun_BNO080_Arduino_Library.h"
 

 #define rightButton D4
 #define leftButton D2
 #define breakButton D3

 SYSTEM_THREAD(ENABLED);
 SYSTEM_MODE(SEMI_AUTOMATIC);
 
 SerialLogHandler logHandler;
 
 // Constants
 const unsigned long MAX_RECORDING_LENGTH_MS = 5000; // 5000; max 5 seconds
 const int SAMPLING_INTERVAL_MS = 10;                 // Sampling interval in milliseconds
 const int BUFFER_SIZE = 25;                          // Buffer size
 const int TRANSMIT_THRESHOLD = 20;                   // Transmit buffer after 20 samples
 
 // Server configuration
 IPAddress serverAddr = IPAddress(192, 168, 132, 254);
 int serverPort = 7123;
 
 TCPClient client;
 
 // Buffers for double buffering
 struct Sample
 {
   uint32_t timestamp; // Timestamp in microseconds
   float accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ;      // Accelerometer data
 };
 
 Sample buffer1[BUFFER_SIZE];
 Sample buffer2[BUFFER_SIZE];
 Sample *samplingBuffer = buffer1;
 Sample *transmitBuffer = buffer2;
 volatile int samplingIndex = 0;
 volatile bool bufferReady = false;
 
 // BNO085 accelerometer
 BNO080 IMU;
 
 void sampleIMU();
 void transmitBufferData();
 void buttonHandler(system_event_t event, int data);
 
 // Timer for sampling
 Timer samplingTimer(SAMPLING_INTERVAL_MS, sampleIMU);
 
enum SampleType
{
   RIGHT,
   LEFT,
   BREAK
 };
 SampleType sampleType = RIGHT;

 // State machine
 enum State
 {
   STATE_WAITING,
   STATE_CONNECT,
   STATE_RUNNING,
   STATE_FINISH
 };
 State state = STATE_WAITING;
 
 unsigned long recordingStart = 0;
 unsigned long lastSampleTime = 0;
 
 #define LED_PIN D7
 
 void readData(float *accX, float *accY, float *accZ, float *gyroX, float *gyroY, float *gyroZ, float *magX, float *magY, float *magZ);

 void changeState();
 void rightButtonHandler();
 void leftButtonHandler();
 void breakButtonHandler();

 void transmitBufferData(int samples);
 
 
 void setup()
 {
    pinMode(rightButton, INPUT_PULLUP);
    pinMode(leftButton, INPUT_PULLUP);
    pinMode(breakButton, INPUT_PULLUP);

    attachInterrupt(rightButton, rightButtonHandler, FALLING);
    attachInterrupt(leftButton, leftButtonHandler, FALLING);
    attachInterrupt(breakButton, breakButtonHandler, FALLING);

   Particle.connect();
   System.on(button_click, buttonHandler);

   interrupts();

   Wire.begin();
 
   if (!IMU.begin())
   {
    Log.error("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    System.reset();
   }
 
   Wire.setClock(400000); //Increase I2C data rate to 400kHz
 
   IMU.enableAccelerometer(10); //Send data update every 10ms / 100Hz
   IMU.enableGyro(10);         //Send data update every 10ms / 100Hz
   IMU.enableMagnetometer(10); //Send data update every 10ms / 100Hz
  
   pinMode(LED_PIN, OUTPUT);
   digitalWrite(LED_PIN, LOW);
 
   Log.info("BNO085 initialized. Waiting for button press...");
 }
 
 void loop()
 {
    String data;
   switch (state)
   {
   case STATE_WAITING:
     break;
 
   case STATE_CONNECT:
     if (client.connect(serverAddr, serverPort))
     {
       Log.info("Connected to server. Starting data collection...");
       Log.info("Sample rate: %d Hz", 1000 / SAMPLING_INTERVAL_MS);
        
       switch(sampleType){
          case RIGHT:
            data = String::format("Right\n");
            break;
          case LEFT:
            data = String::format("Left\n");
            break;
          case BREAK:
            data = String::format("Break\n");
            break;
       }
       client.write(((const uint8_t *)data.c_str()), data.length());

       recordingStart = millis();
       samplingIndex = 0;
       samplingTimer.start();
       digitalWrite(LED_PIN, HIGH);
       state = STATE_RUNNING;
     }
     else
     {
       Log.error("Failed to connect to server.");
       state = STATE_WAITING;
     }
     break;
 
   case STATE_RUNNING:
     // Check for buffer ready to transmit
     if (bufferReady)
     {
       // Swap buffers before transmission
       Sample *temp = samplingBuffer;
       samplingBuffer = transmitBuffer;
       transmitBuffer = temp;
 
       int samplesToTransmit = samplingIndex; // Snapshot of current index
       samplingIndex = 0;                     // Reset sampling index
       bufferReady = false;
 
       // Transmit the previous buffer
       transmitBufferData(samplesToTransmit);
     }
 
     // Stop after timeout
     if (millis() - recordingStart >= MAX_RECORDING_LENGTH_MS)
     {
       state = STATE_FINISH;
     }
     break;
 
   case STATE_FINISH:
     samplingTimer.stop();
     if (samplingIndex > 0)
     {
       bufferReady = true;
       transmitBufferData(samplingIndex);
     }
     client.stop();
     digitalWrite(LED_PIN, LOW);
     Log.info("Data collection complete.");
     state = STATE_WAITING;
     break;
   }
 }
 
 void sampleIMU()
 {
   if (samplingIndex < BUFFER_SIZE)
   {
        while(IMU.dataAvailable() != true);
        // Sample accelerometer
        float accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ;

        readData(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ,&magX,&magY,&magZ);
    
        samplingBuffer[samplingIndex].timestamp = micros();
        samplingBuffer[samplingIndex].accX = accX;
        samplingBuffer[samplingIndex].accY = accY;
        samplingBuffer[samplingIndex].accZ = accZ;
        samplingBuffer[samplingIndex].gyroX = gyroX;
        samplingBuffer[samplingIndex].gyroY = gyroY;
        samplingBuffer[samplingIndex].gyroZ = gyroZ;
        samplingBuffer[samplingIndex].magX = magX;
        samplingBuffer[samplingIndex].magY = magY; 
        samplingBuffer[samplingIndex].magZ = magZ;
    
        samplingIndex++;
        lastSampleTime = millis();
    
        // Mark buffer as ready if threshold is met
        if (samplingIndex >= TRANSMIT_THRESHOLD)
        {
        bufferReady = true;
        }
        
   }
 }
 
 void transmitBufferData(int samples)
 {
   if (samples == 0)
   {
     Log.warn("Transmit called with empty buffer. Skipping transmission.");
     return;
   }
   //Log.info("Transmitting %d samples...", samples);
   for (int i = 0; i < samples; i++)
   {
     String data = String::format(
         "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
         transmitBuffer[i].timestamp,
         transmitBuffer[i].accX,
         transmitBuffer[i].accY,
         transmitBuffer[i].accZ,
         transmitBuffer[i].gyroX,
         transmitBuffer[i].gyroY,
         transmitBuffer[i].gyroZ,
         transmitBuffer[i].magX,
         transmitBuffer[i].magY,
         transmitBuffer[i].magZ);
     client.write(((const uint8_t *)data.c_str()), data.length());
   }
   //Log.info("Buffer transmission complete.");
 }
 
void changeState()
{
    switch (state)
    {
    case STATE_WAITING:
      if (WiFi.ready())
      {
        state = STATE_CONNECT;
      }
      else
      {
        Log.warn("Wi-Fi not ready.");
      }
      break;
  
    case STATE_RUNNING:
      state = STATE_FINISH;
      break;
    }
    return;
}
 void buttonHandler(system_event_t event, int data){
   changeState();
   return;
 }

 void rightButtonHandler(){
    while(digitalRead(rightButton) == LOW); // Wait for button release
    sampleType = RIGHT;
    changeState();
    return;
 }

 void leftButtonHandler(){
    while(digitalRead(leftButton) == LOW); // Wait for button release
    sampleType = LEFT;
    changeState();
    return;
 }

 void breakButtonHandler(){
    while(digitalRead(breakButton) == LOW); // Wait for button release
    sampleType = BREAK;
    changeState();
    return;
 }

 void readData(float *accX, float *accY, float *accZ, float *gyroX, float *gyroY, float *gyroZ, float *magX, float *magY, float *magZ){

    *accX = IMU.getAccelX();
    *accY = IMU.getAccelY();
    *accZ = IMU.getAccelZ();

    *gyroX = IMU.getGyroX();
    *gyroY = IMU.getGyroY();
    *gyroZ = IMU.getGyroZ();

    *magX = IMU.getMagX();
    *magY = IMU.getMagY();
    *magZ = IMU.getMagZ();
 }


