#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "math.h"
#include <Braccio.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>



SoftwareSerial ArduinoMaster(2,3);
char cmd="";
String pit;
int pitch_M2;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// shoulder
MPU6050 mpu_1(0x68);
// hand
MPU6050 mpu_2(0x69);

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

const int flexPin = A0;	
const float VCC = 5;			// voltage at Ardunio 5V line
const float R_DIV = 47000.0;	// resistor used to create a voltage divider
const float flatResistance = 25000.0;	// resistance when flat
const float bendResistance = 100000.0;	// resistance at 90 deg

float flex_angle;

#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_TEAPOT
// shoulder
#define INTERRUPT_PIN_1 2  // use pin 2 on Arduino Uno & most boards
// hand
#define INTERRUPT_PIN_2 3
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

bool dmpReady_1 = false;  // set true if DMP init was successful
bool dmpReady_2 = false;

uint8_t mpuIntStatus_1;   // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus_2; 

uint8_t devStatus_1;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus_2;

uint16_t packetSize_1;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize_2;

uint16_t fifoCount_1;     // count of all bytes currently in FIFO
uint16_t fifoCount_2;
uint8_t fifoBuffer_1[64]; // FIFO storage buffer
uint8_t fifoBuffer_2[64];

float values[3];
float pre_values[3];

float* sh;
float* sh_p;
float* ha;
float* ha_p;
float a,b,c;
StaticJsonDocument<200> doc;
float EPS = 1e-6f;


uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt_1 = false;     // indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt_2 = false;

void dmpDataReady_1() {
  mpuInterrupt_1 = true;
  
}

void dmpDataReady_2() {
 
  mpuInterrupt_2 = true;
}



void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation 
  Wire.setWireTimeout(3000,true);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu_1.initialize();
  mpu_2.initialize();
  pinMode(INTERRUPT_PIN_1, INPUT);
  pinMode(INTERRUPT_PIN_2, INPUT);

   Braccio.begin();

    ArduinoMaster.begin(9600);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu_1.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(mpu_2.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus_2 = mpu_1.dmpInitialize();
  devStatus_2 = mpu_2.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu_1.setXGyroOffset(51);
  mpu_1.setYGyroOffset(8);
  mpu_1.setZGyroOffset(21);
  mpu_1.setXAccelOffset(1150);
  mpu_1.setYAccelOffset(-50);
  mpu_1.setZAccelOffset(1060);

  mpu_2.setXGyroOffset(51);
  mpu_2.setYGyroOffset(8);
  mpu_2.setZGyroOffset(21);
  mpu_2.setXAccelOffset(1150);
  mpu_2.setYAccelOffset(-50);
  mpu_2.setZAccelOffset(1060);

  // make sure it worked (returns 0 if so)
  if (devStatus_1 == 0 && devStatus_2 == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu_1.CalibrateAccel(6);
    mpu_1.CalibrateGyro(6);
    mpu_2.CalibrateAccel(6);
    mpu_2.CalibrateGyro(6);
    Serial.println();
    mpu_1.PrintActiveOffsets();
    mpu_2.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu_1.setDMPEnabled(true);
    mpu_2.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_1));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), dmpDataReady_1, RISING);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), dmpDataReady_2, RISING);
    mpuIntStatus_1 = mpu_1.getIntStatus();
    mpuIntStatus_1 = mpu_2.getIntStatus();

    float values[3];
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady_1 = true;
    dmpReady_2 = true;

    // get expected DMP packet size for later comparison
    packetSize_1 = mpu_1.dmpGetFIFOPacketSize();
    packetSize_2 = mpu_2.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus_1);
    Serial.print(devStatus_2);
    Serial.println(F(")"));
  }

 
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================t6
void loop() {

 if(!dmpReady_1 && !dmpReady_2) return;

  if( mpu_1.dmpGetCurrentFIFOPacket(fifoBuffer_1)){
  sh_p = sh;
  sh = calculate_ypr(fifoBuffer_1, 64);
  if(sh[0] < 90){
        sh[1] = sh[1] - (sh_p[0] - sh[0])*0.1325;
        sh[2] = sh[2] + (sh_p[0] - sh[0])*0.2771;
      }
      if( sh[0] > 90){
        sh[1] = sh[1] - (sh_p[0] - sh[0])*0.06185;
        sh[1] = sh[1] + (sh_p[0] - sh[0])*0.52577;
      }
      if(sh[2] < 90){
        sh[1] = sh[1] + (sh_p[2] - sh[2])*0.22223;
      } 
      sh[2] = map(sh[2], 0, 180 ,180 , 0);
      sh[0] = map(sh[0], 0, 180 , 180, 0);
     // pitch
    Serial.print(sh[0]);
    Serial.print("  ");
    // roll
    Serial.print(sh[1]);
    Serial.print("  ");
    // yaw
    Serial.println(sh[2]);
  }
  a = sh[0];
  b = sh[1];
  c = sh[2];
  
  Serial.println("MPU 2 data");

  if( mpu_2.dmpGetCurrentFIFOPacket(fifoBuffer_2)){
    ha_p = ha;
    ha = calculate_ypr(fifoBuffer_2, 64);
    ha[1] = ha[1] + 90;
    Serial.print(ha[0]);
    Serial.print("  ");
    Serial.print(ha[1]);
    Serial.print("  ");         
    Serial.println(ha[2]);
    if(ha[0] < 90){
      ha[1] = ha[1] + (ha_p[0] - ha[0])*0.5280;
    }
    
    if(ha[1] < 90){
      ha[0] = ha[0] + (ha_p[1] - ha[1])*0.37209;
    }

    ha[0] = map(ha[0], 0, 180, 180, 0);
    
    float flex_angle = get_flex_angle();
    // get_pitch_for();  
    // Braccio.ServoMovement(20,  c, a, P_for_M2, ha[0], ha[1],  flex_angle);
  }
 
  Serial.println("MPU 111 data");
  // Serial.print("SH[0]");
  // Serial.println(a);
  // Serial.print("SH[2]");
  // Serial.println(c);
  Serial.println("MPU 222 data");
  // Serial.print("HA[0]  ");
  // Serial.print(ha[0]);
  // Serial.print(" HA[1] ");
  // Serial.println(ha[1]);
 delay(300);
}


float * calculate_ypr(uint8_t fifoBuffe[], int size_of_array){

    teapotPacket[2] = fifoBuffe[0];
    teapotPacket[3] = fifoBuffe[1];
    teapotPacket[4] = fifoBuffe[4];
    teapotPacket[5] = fifoBuffe[5];
    teapotPacket[6] = fifoBuffe[8];
    teapotPacket[7] = fifoBuffe[9];
    teapotPacket[8] = fifoBuffe[12];
    teapotPacket[9] = fifoBuffe[13];
    teapotPacket[11]++;     
    double a[4];
    double axis[3];
    double b;
    a[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
    a[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
    a[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
    a[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;

    for (int i = 0; i < 4; i++) if (a[i] >= 2) a[i] = -4 + a[i];

    double sa = sqrt(1.0f - a[0] * a[0]);
    if (sa < EPS) {
          sa = 1.0f;
    } else {
          sa = 1.0f / sa;
    }
     values[0] = a[1] * sa * acos(a[0]) * 2.0f * 180/PI + 90;
     values[1] = a[2] * sa * acos(a[0]) * 2.0f * 180/PI + 90;
     values[2] = a[3] * sa * acos(a[0]) * 2.0f* 180/PI +  90;  

      if(values[0] > 180){
        values[0] = 180;
      }
      else if(values[0] < 0){
        values[0] = 0;
      }
       if(values[1] > 180){
        values[1] = 180;
      }
      else if(values[1] < 0){
        values[1] = 0;
      }
      
      if(values[2] > 180){
        values[2] = 180;
      }
      else if(values[2] < 0){
        values[2] = 0;
      }

     
      
  return values;
}

float get_flex_angle(){
  int ADCflex = analogRead(flexPin);
	float Vflex = ADCflex * VCC / 1023.0;
	float Rflex = R_DIV * (VCC / Vflex - 1.0);
	float angle = map(Rflex, 12727, 37307, 5, 90);
	Serial.println("Bend: " + String(angle) + " degrees");
  return angle;
}

// void get_pitch_for(){
//    while (ArduinoMaster.available()){
//     cmd=ArduinoMaster.read();
//     pit = pit + cmd; 
//     Serial.println(cmd);
//   }
//   // deserializeJson(doc, pit);
//   // int p = doc["pitch_value"];
//   // Serial.println(p);
//   // pit = "";
// }
