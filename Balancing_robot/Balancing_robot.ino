#include <Streaming.h>  // only for simple print
#include "TB6612.h"
#include "motorPID.h"
#include "QEncoder.h"
#include "MPU9250_SPI.h"
#include "eeprom_utils.h"
#include "parameters.h"
#include <SoftwareSerial.h>     // 소프트웨어 시리얼 라이브러리 불러옴

#include "filterlib.h"
#include <MedianFilterLib.h>
LPFilter<float&> LPF(10.,0.005);
HPFilter<float> HPF(500, 0.005);
MedianFilter<float> medianFilter(3);

//#define RAW_DATA
//#define CALIBRATION_MODE
MPU9250_SPI mpu(SPI, MPU_CS, INT_PIN);
EEPROM_BACKUP eeprom;
TB6612 motorA(APWM, AIN1, AIN2);  //create a motor instance
TB6612 motorB(BPWM, BIN1, BIN2);  //create a motor instance
rPID  pidA(0.045, 0.10, 0.001, 255);     // kp, ki, kd,imax
rPID  pidB(0.045, 0.10, 0.001,  255);     // kp, ki, kd,imax
rPID  bala(130, 2040, 5, 5000);   // 1차 성공 200, 2040,5,5000 840 ok
rPID  movePID(0.0015, 0.001, 0.00, 2);
Encoders encoderA(ENA_A,ENA_B), encoderB(ENB_A,ENB_B);      // Create an Encoder instance (2,3) (1,0)
uint32_t prevTime=0,intVal=2000, pT=0;
int32_t ref=0, desiredPosA=0,desiredPosB=0,cnt=0;
int32_t desiredVelA=0,desiredVelB=0;
String inStr = "";

int BT_RXD = 8; // 아두이노측 수신부 RXD는 8번핀(HC-06측 송신부 TXD와 연결)
int BT_TXD = 10; // 아두이노측 송신부 TXD는 7번핀(HC-06측 수신부 RXD와 연결)
int input=0;
SoftwareSerial bluetooth(BT_RXD, BT_TXD);   // 소프트웨어 시리얼 bluetooth 객체 선언

void setup() {
  Serial.begin(115200);     // 시리얼 통신 시작
  bluetooth.begin(115200);  // 블루투스 통신 시작
  //while (!Serial) {}
  inStr.reserve(20);
  pinMode(LED_PIN, OUTPUT);
  motorA.stop(); motorB.stop();
  encoderA.setEncoderCount(0);
  encoderB.setEncoderCount(0);
  mpu.setup();
  mpu.setMagneticDeclination(8.5);
  mpu.setSampleRate(SR_100HZ);
  mpu.setGyroRange(GYRO_RANGE_2000DPS);
  mpu.setAccelRange(ACCEL_RANGE_16G);
  mpu.setDlpfBandwidth( DLPF_BANDWIDTH_184HZ);
  mpu.enableDataReadyInterrupt();
#ifdef CALIBRATION_MODE
  calibrationProcess();
#endif
  eeprom.loadCalibration();  // calibration data
  prevTime = micros(); pT = micros();
  
}
float velA=0, velB=0,yaw=0, dt=DT;
int Desired_vel_A=0,Desired_vel_B=0;
bool second_num=false;
char cmd = bluetooth.read();
void loop() {
  if (mpu.isDataReady()) {if (bluetooth.available()){
    getDt();    
#ifndef RAW_DATA
    mpu.update(COMPLEMENTARY); //  MAGDWICK  /COMPLEMENTARY
#else
    Vect3  a, g, m;  // acc/gyro/mag vectors
    mpu.update(a, g, m);
#endif
      Vect3 gyro = mpu.getGyroVect();
      float pitchDeg = mpu.getPitch() * RAD_TO_DEG;
      if (cmd == "for"){pitchDeg = pitchDeg + 3; Serial.print("for");}
      else if (cmd == "back"){pitchDeg = pitchDeg - 3;Serial.print("back"); }
      
      int32_t curPosA = encoderA.getEncoderCount();
      int32_t curPosB = encoderB.getEncoderCount();
      float curPos = (curPosA + curPosB) / 2;
      float pitchRef = movePID.wheelControl(500, curPos, dt);
      float mCommand = bala.balanceControl(-pitchRef, pitchDeg, gyro.x, dt);
      mCommand = constrain(mCommand, -5000, 5000);
      desiredVelA = mCommand; desiredVelB = mCommand;  
    int32_t uA = pidA.speedControl(desiredVelA, curPosA, dt, velA);
    int32_t uB = pidB.speedControl(desiredVelB, curPosB, dt, velB);
    
    if ((pitchDeg < ANGLE_LIMIT)&&(pitchDeg > -ANGLE_LIMIT)){  // fail
      motorA.run(uA); motorB.run(uB);  }
      else{ motorA.run(0);motorB.run(0); bala.setIntegrator(0);}  
         
    ///uint32_t curTime=micros();
    //if (curTime-prevTime>20000){
        //prevTime=curTime;
      //  Serial <<bala.getP()<<","<<bala.getI() <<","<<bala.getD()<<endl;
       // Serial <<mpu.getPitch()*RAD_TO_DEG<<","<<gyro.x <<","<<mCommand/10.<<endl;
      //Serial << "YPR "<<mpu.getYaw()*RAD_TO_DEG<<" "<<mpu.getPitch()*RAD_TO_DEG<<" "<<mpu.getRoll()*RAD_TO_DEG<<endl;
     bluetooth.print(pitchDeg); bluetooth.print(" ");bluetooth.print(velA); bluetooth.print(" "); bluetooth.println(uA); 
}
}}

void makeDt(float dt){
  uint32_t cT=micros();
  while (uint32_t(cT-pT) < dt*1000000.){ cT=micros();}
  pT=cT;
}
void getDt(){
  uint32_t cT=micros();
  dt=uint32_t(cT - pT)/1000000.0;
  pT=cT;
}
void serialEvents(){
  while (Serial.available()){
    char c= (char)Serial.read();
    if (c=='\n'){
       bala.setD(inStr.toFloat());
       inStr="";
    }
    else if (c==','){
      if (second_num){ bala.setI(inStr.toFloat());second_num=false;}
      else { bala.setP(inStr.toFloat());second_num=true;}
      inStr="";
    }
    else
     inStr+=c;   
  }
}

int compensateDZ(int input,int epsilon,int minusStart, int plusStart){
    if (abs(input)<epsilon)
      return 0;
   else
      return (input>=0)? input+epsilon: input-epsilon; 
}
float map(float value, float istart, float istop, float ostart, float ostop){
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}
void calibrationProcess(){
  mpu.calibrateGyro();   
  mpu.calibrateMag();
  eeprom.saveCalibration();
  Serial.println("Calibration Completed !!!!!!!!");  
    while(1);
}


void go(){
/*for(int i=0; i<6; i++){
  motorA.forward();
  motorB.forward();
  delay(500);
  motorA.stop();
  motorB.stop();
}*/
}
