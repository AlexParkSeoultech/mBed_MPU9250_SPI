#include "mbed.h"
#include "MPU9250_SPI.h"
//#define CALIBRATION_MODE
//#define RAW_DATA
MPU9250_SPI mpu(D11, D12, D13, D2,D3); // mosi, miso, sclk, cs, intr
//DigitalOut led1(LED1);  // be careful it is D13 too
Timer tmr;
Serial pc(USBTX,USBRX, 115200);
float dt;
void getDt();
void calibrationProcess();
void applyCalbratedValue();
int main(){
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
    applyCalbratedValue();   
    tmr.start(); tmr.reset();
    while(true){
        if (mpu.isDataReady()){
            getDt(); 
        #ifndef RAW_DATA
            mpu.update(MADGWICK ); //  MADGWICK  /COMPLEMENTARY
            pc.printf("YPR,%5.2f,%5.2f,%5.2f\n", mpu.getYaw()*RAD_TO_DEG, mpu.getPitch()*RAD_TO_DEG, mpu.getRoll()*RAD_TO_DEG );
        #else
            Vect3  a, g, m;  // acc/gyro/mag vectors
            mpu.update(a,g,m);
            pc.printf("%5.2f, %5.2f, %5.2f,  %5.2f, %5.2f, %5.2f,  %5.2f, %5.2f, %5.2f\n", a.x, a.y, a.z,g.x, g.y, g.z,m.x, m.y, m.z);
        #endif            
        }
    }
}

void getDt(){
  dt=tmr.read_us()/1000000.0;
  tmr.reset();
}
void calibrationProcess(){
  mpu.calibrateGyro();   
  mpu.calibrateMag();
  pc.printf("Calibration Completed !!!!!!!!\n");  
    while(true);   //stop here
}
void applyCalbratedValue(){
  Vect3 gBias ={ 1.881,   -2.630  ,   0.226};
  mpu.setGyroBias(gBias); 
  Vect3 mBias ={12.885,  314.122  , -594.508};
  mpu.setMagBias(mBias);
  Vect3 mScale={ 1.852,    1.176  ,   0.621}; 
  mpu.setMagScale(mScale); 
}
  
