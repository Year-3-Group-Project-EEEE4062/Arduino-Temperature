#include <OneWire.h>
#include <DallasTemperature.h>

const uint8_t START_TEMP_MEASURE = 4; // Value for starting temperature measurement
const uint8_t STOP_TEMP_MEASURE = 7;  // Value for stopping temperature measurement

#define TIMER_INTERRUPT_DEBUG         2
#define TIMERINTERRUPT_LOGLEVEL     0
#define USE_TIMER_1     true
#define LED_BUILTIN     13
#define TIMER1_INTERVAL_MS    1
#define TIMER1_FREQUENCY      2000
#define TIMER1_DURATION_MS    0
#include "TimerInterrupt.h"

#define STEP_PIN 3
#define DIR_PIN 2

#define ONE_WIRE_BUS 4

OneWire oneWire(ONE_WIRE_BUS);  
DallasTemperature sensors(&oneWire);

// Variables for depth sensor
const int analogPin = A0; // Analog pin to connect the sensor output
const float voltageReference = 5.0; // Voltage reference used by the Arduino (in volts)
const float maxDepth = 3000.0; // Maximum depth supported by the sensor (in millimeters)

float temp_arr[3];
float depth_arr[3];
float transmit_data_arr[6];

const int num_temp_readings = 7;
float temp_mean_arr[num_temp_readings];

static bool toggle1 = false;  // toggle timer
bool direction = 1;           // 0: UP, 1: DOWN
bool move = 1;                // 1: MOVE, 0: STOP
int flag = 0;
int flag_surface = 0;

int depth_check = 1;      // 1: max_depth, 2: halfway_depth, 3: surface_depth, 0: reached surface
int temp_check = 0;       // 0: no read temperature, 1: read temperature

int counter_max = 0;      // counter for maximum depth
int counter_half = 0;     // counter for halfway depth
int counter_surface = 0;  // counter for surface depth
int counter_const_max = 0;


// Variables for temperature
int counter_temp = 0;     // counter for temperature
int temp_index = 0;       // index for temperature array
float prev_temp = 0;      // store previous temperature
int mean_index = 0;
float sum_temp = 0;

const int median_size = 10;  // size for median array
float desired_max_depth = 0; // global variable to store maximum depth    
float threshold = 7;         // threshold for maximum and surface depth

// Bool from RPi
bool boatBusy = 0;          // 1: take temperature, 0: quit temperature
bool quitTemperature = 0;   // 1: quit operation and turn boatBusy to false 
bool done_Quit = 0;
// bool check_temp = 0;

float cube (float x){
  return x*x*x;
}

float square (float x){
  return x*x;
}

void TimerHandler1()
{
  digitalWrite(DIR_PIN, HIGH*direction*boatBusy); 
  digitalWrite(STEP_PIN, toggle1*move*boatBusy);
  
  toggle1 = !toggle1;
}


// Function to calculate median of an array
float calculateMedian(float readings[], int num) {
  float median;

  for (int i = 0; i < num - 1; i++) {
    for (int j = i + 1; j < num; j++) {
      if (readings[j] < readings[i]) {
        // Swap
        float temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }

  // Calculate the median
  if (num % 2 == 0) {
    // If the number of readings is even
    int middleIndex = num / 2;
    median = (readings[middleIndex - 1] + readings[middleIndex]) / 2.0;
  } else {
    // If the number of readings is odd
    median = readings[num / 2];
  }
  return (median);
}

// Function to read depth and apply median filter
float readDepth() {
  float median_depth_arr[median_size];
  // Collect readings
  for (int i = 0; i < median_size; i++) {
    int sensor_value = analogRead(analogPin);
    float voltage = sensor_value * (voltageReference / 1023.0);
    median_depth_arr[i] = voltage / voltageReference * maxDepth;
  }

  // Calculate the median
  float depth_median = calculateMedian(median_depth_arr, median_size);
  // Correct the depth based on linear model
  float corrected_depth = 0;

  if(depth_median > 15){
    corrected_depth = 2e-8*cube(depth_median)-5e-5*square(depth_median)+1.0692*depth_median+6.4165;
//    corrected_depth = -2e-8*cube(depthMedian)+7e-5*square(depthMedian)+0.9177*depthMedian-2.8906;
  }
  else{
    corrected_depth = 0;
  }

  return (corrected_depth);
}

void checkMaxDepth() {
  if (depth_check==1 && temp_check == 0){
    float cur_depth = readDepth();

    // Serial.print("cur_depth: ");
    // Serial.println(cur_depth);
    // Serial.print("desired_depth: ");
    // Serial.println(desired_max_depth);

    // delay(10);

    if(cur_depth > 10 && abs(cur_depth - desired_max_depth) < threshold){
      counter_max++;
      // Serial.print("Difference: ");
      // Serial.println(abs(cur_depth - desired_max_depth));
    }
    else{
      counter_max = 0;
    }

    if (cur_depth > 0 && cur_depth > desired_max_depth) {
      desired_max_depth = cur_depth;
      move = 1;
      direction = 1;
      counter_const_max = 0;
      counter_max = 0;
    }
    else{
      counter_const_max++;
    }

    if (counter_max == 3){
      depth_arr[0] = cur_depth;
      move = 0;
      direction = 0;
      depth_check = 2;
      temp_check = 1;
    }
    else if (counter_max > 1){
      move = 0;
    }
    

    if(counter_const_max > 8){
      desired_max_depth = cur_depth;
      move = 0;
      counter_const_max = 0;
      // Serial.println("reset");
    }

    if(cur_depth >= 3000.00){
      move = 0;
      direction = 0;
      depth_check = 2;
      temp_check = 1;
    }
    
    // Serial.print(counter_max);
    // Serial.print(depth_check);
    // Serial.print(counter_const_max);
    // Serial.println(move);
  }
}

void checkHalfDepth(){

  float threshold_half = 15;

  if (depth_check == 2 && temp_check == 0){
    move = 1;

    float cur_depth = readDepth();
    float desired_half_depth = depth_arr[0]/2;
    float diff = cur_depth - desired_half_depth;

    // Serial.print("cur_depth: ");
    // Serial.println(cur_depth);
    // Serial.print("desired_depth: ");
    // Serial.println(desired_half_depth);
    // Serial.print("diff: ");
    // Serial.println(diff);

    // delay(50);

    if(abs(diff) < threshold_half){
      move = 0;
      counter_half++;
    }
    else if(abs(diff) > threshold_half && diff > 0){
      move = 1;
      direction = 0;
      counter_half = 0;
    }
    else if(abs(diff) > threshold_half && diff < 0){
      move = 1;
      direction = 1;
      counter_half = 0;
    }

    if (counter_half == 5){
      // check_temp = 1;
      temp_check = 1;
      move = 0;
      direction = 0;
      depth_check = 3;
      depth_arr[1] = cur_depth;
    }
    else if(counter_half > 2){
      move = 0;
    } 

    // Serial.print(counter_half);
    // Serial.print(depth_check);
    // Serial.print(move);
    // Serial.println(direction);

    // delay(50);
  }
}

// ADD PARAM TO SURFACE DEPTH TO GO IN OR NOT GO IN THE TEMPERATURE MEASUREMENT
void checkSurfaceDepth(){
  if (depth_check == 3 && temp_check == 0){
    move = 1;

    float cur_depth = readDepth();
    float desired_surface_depth = 0;
    float diff = cur_depth - desired_surface_depth;

    // Serial.print("cur_depth: ");
    // Serial.println(cur_depth);
    // Serial.print("desired_depth: ");
    // Serial.println(desired_surface_depth);
    // Serial.print("diff: ");
    // Serial.println(diff);

    // delay(50);

    if(abs(diff) < threshold){
      move = 0;
      counter_surface++;
    }
    else if(abs(diff) > threshold && diff > 0){
      move = 1;
      direction = 0;
      counter_surface = 0;
    }
    else if(abs(diff) > threshold && diff < 0){
      move = 1;
      direction = 1;
      counter_surface = 0;
    }
    
    if(counter_surface > 2){
      move = 0;
    }

    if (counter_surface == 5){
      move = 0;
      direction = 0;
      depth_check = 0;
      temp_check = !quitTemperature;
      depth_arr[2] = desired_surface_depth;
      flag_surface = 1;
      flag = 1;
    }
  
    // Serial.print("surface: ");
    // Serial.print(counter_surface);
    // Serial.print(depth_check);
    // Serial.print(move);
    // Serial.println(direction);

    // delay(50);
  }
}

// Read temperature
void readTemp(){
  float threshold_temp = 0.15;

  sensors.requestTemperatures();
  float temp_reading = sensors.getTempCByIndex(0);

  // Serial.println(temp_index);
  // Serial.println(temp_reading);
  // Serial.print("prev: ");
  // Serial.println(temp_reading);

  // Check if the temperature reading has changed significantly
  if (abs(temp_reading - prev_temp) <= threshold_temp) {
    counter_temp++;
    temp_mean_arr[mean_index] = temp_reading;
    mean_index++;
  } 
  else {
    counter_temp= 0;
    mean_index = 0;
  }

  if (counter_temp == num_temp_readings) {
    temp_check = 0;
    temp_arr[temp_index] = temp_reading;

    // Calculating mean of temperature values
    for(int i = 0; i<num_temp_readings; i++){
      sum_temp += temp_mean_arr[i];
    }
    temp_arr[temp_index] = sum_temp/num_temp_readings;
    
    // Reset index and counter
    temp_index++;
    counter_temp = 0;
    mean_index = 0;
    
    // Reset array for storing temperature values
    sum_temp = 0;
    for(int i = 0; i<num_temp_readings; i++){
      temp_mean_arr[i] = 0;
    }
    
    // Serial.print(temp_index);
    // Serial.print(temp_index-1);
    // Serial.print(temp_arr[temp_index-1]);
    // Serial.println("\u00b0C");

    
  }

  if(depth_check == 0 && temp_check == 0){
    flag_surface = 1;
  }
  
  prev_temp = temp_reading;

  return (temp_arr[temp_index-1]);
}

void arrangeData(){
  for(int i = 0; i<3; i++){
    transmit_data_arr[i*2] = depth_arr[i];
    transmit_data_arr[i*2+1] = temp_arr[i];
  }

  // For debugging
  // Make sure to comment in final program
  // Serial.println("Contents of transmit_data_arr array:");
  // for(int j = 0; j<6; j++){
  //   Serial.print(transmit_data_arr[j]);
  //   Serial.print(", ");
  // }
  // Serial.println();
}

// Reset parameters to default state
void resetParams(){

  // Clear depth and temperature array
  for(int i = 0; i<3; i++){
    depth_arr[i] = 0;
    temp_arr[i] = 0;
  }

  direction = 1; 
  move = 1;      // 1: MOVE, 0: STOP

  depth_check = 1;     // 1: max_depth, 2: halfway_depth, 3: surface_depth
  temp_check = 0;

  temp_index = 0;
  desired_max_depth = 0;

  boatBusy = 0;
  flag = 0;

  // Write boolean to indicate temperature reading done.
  // Output.
}

// Function to initiate temperature mechanism
void initTemperature(){

  if(temp_check == 1){
    readTemp();
  }

  else if(temp_check == 0){
    switch (depth_check) {
    case 1:
      checkMaxDepth();
      break;
    case 2:
      checkHalfDepth();
      break;
    case 3:
      checkSurfaceDepth();
      break;
    }
  }

  if(depth_check==0 && temp_check == 0){
    // transmit temperature readings to rpi
    // Serial.println("Depth and Temperature Readings");
    // for(int i=0; i<3; i++){
    //   Serial.print("Depth ");
    //   Serial.print(i+1);
    //   Serial.print(": ");
    //   Serial.println(depth_arr[i]);
    //   Serial.print("Temperature ");
    //   Serial.print(i+1);
    //   Serial.print(": ");
    //   Serial.println(temp_arr[i]);
    // }
    arrangeData();
    // resetParams();
  }

  delay(200);
}

uint8_t start = 1;
uint8_t stop = 0;
bool sent = 0;

byte* floatToByteArray(float* dataToBeSent, int arraySize, int dataLength) {
  // Print contents of dataToBeSent
  // Make sure to comment out when final program
  // Serial.println("Contents of dataToBeSent array:");
  // for (int i = 0; i < dataLength; i++) {
  //   Serial.print(dataToBeSent[i]);
  //   Serial.print(", ");
  // }
  // Serial.println();
  
  byte modeIdentifier = 0x04; // to indicate what mode it is (NOT FIXED)
  byte dataTypeIdentifier = 0x02; // indicate that it is a float

  // Dynamically allocate memory for the byte array
  byte* byteArray = new byte[arraySize];

  // Null satefy
  if (byteArray == nullptr) {
    // Memory allocation failed
    return nullptr;
  }

  // Populate the byte array
  byteArray[0] = modeIdentifier;
  byteArray[1] = dataTypeIdentifier;

  // Populate the byte array with 4 bytes int
  // This contains the mssg length
  byteArray[2] = (dataLength >> 0) & 0xFF;   // Little endian
  byteArray[3] = (dataLength >> 8) & 0xFF;
  byteArray[4] = (dataLength >> 16) & 0xFF;
  byteArray[5] = (dataLength >> 24) & 0xFF;

  // Populating the array with the message
  size_t offset = 6;
  for (int i = 0; i < dataLength; i++) {
    uint32_t temp;
    memcpy(&temp, &dataToBeSent[i], sizeof(temp));
    byteArray[offset++] = (temp >> 0) & 0xFF;
    byteArray[offset++] = (temp >> 8) & 0xFF;
    byteArray[offset++] = (temp >> 16) & 0xFF;
    byteArray[offset++] = (temp >> 24) & 0xFF;
  }

  return byteArray;
}

void sendDataToPi(){
  // Determine the how many floats there are
  int dataLength = sizeof(transmit_data_arr) / sizeof(transmit_data_arr[0]);

  // Calculate the total size required for the byte array
  // 6 bytes for mode identifier, data type identifier, and data length, 4 bytes per float
  int dataSize = 6 + dataLength * 4;

  byte* converted = floatToByteArray(transmit_data_arr, dataSize, dataLength);

  if (converted != nullptr) {
    for (int i = 0; i < dataSize; i++) {
      Serial.write(&converted[i],1);
    }
    Serial.println();
  }

  // Free the dynamically allocated memory
  delete[] converted;
}

void resetPi4booleans(){
  done_Quit = 0;
  quitTemperature = 0;
  sent = 0;
}

void setup(){ 
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Serial.begin(115200);
  Serial.flush();

  ITimer1.init(); // Timer 1-2 for UNO
  ITimer1.attachInterrupt(TIMER1_FREQUENCY, TimerHandler1);
}

void loop(){
  // Check if there is anything to read in Serial
  if (Serial.available() > 0) {
    // Read a line of characters from the serial port
    uint8_t piMssg = Serial.read();
    
    if (piMssg == start) {
      boatBusy = 1;
      resetPi4booleans();
    } else if (piMssg == stop) {
      quitTemperature = 1;
    }
  }
  
  if (boatBusy && quitTemperature) {
    depth_check == 3;
    temp_check == 0;
      while (true){
        checkSurfaceDepth();
        if (flag == 1) {
          resetParams();
          break;
        }
      }
    
    done_Quit = 1;
  }
  else if (boatBusy) {
    // // Return mssg after done temperature operation.
    initTemperature();
    if(flag_surface==1 && temp_check==0){
      flag_surface = 0;
      if(!sent){
        sendDataToPi();
        delay(0.5);
        resetParams();
        sent=1;
      }
    }
  }

  if (done_Quit){
    // Send message to Pi to indicate everything successfully stopped
    Serial.flush();
    Serial.write(&stop,sizeof(stop));
    Serial.println();

    // Reset
    resetPi4booleans();
  }
}