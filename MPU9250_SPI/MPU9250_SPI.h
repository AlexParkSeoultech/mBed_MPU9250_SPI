#ifndef MPU9250_SPI_
#define MPU9250_SPI_
// MPU9250 with SPI interface library Ver. 0.98 for mBed
// Made by HeeJae Park 
// 2019.08.06
#include "mbed.h"
#include <cmath>
#include "MPU9250RegisterMap.h"
#include "QuaternionFilter.h"
//#define PRINT_DETAILS
#define DT 0.01f
#define fc 0.5f
#define Tc (1./(2.*M_PI*fc))
#define ALPHA (Tc/(Tc+DT))
#define BETA (1-ALPHA)
enum GyroRange { GYRO_RANGE_250DPS,  GYRO_RANGE_500DPS,  GYRO_RANGE_1000DPS,  GYRO_RANGE_2000DPS };
enum AccelRange  { ACCEL_RANGE_2G,  ACCEL_RANGE_4G,  ACCEL_RANGE_8G,  ACCEL_RANGE_16G  };
enum DlpfBandwidth  { DLPF_BANDWIDTH_250HZ, DLPF_BANDWIDTH_184HZ,  DLPF_BANDWIDTH_92HZ,  DLPF_BANDWIDTH_41HZ,
                       DLPF_BANDWIDTH_20HZ,  DLPF_BANDWIDTH_10HZ,  DLPF_BANDWIDTH_5HZ };
enum MagnBits { MGN_14BITS, MGN_16BITS };  // CNTL1 offset 4   0:14bits, 1:16bits
enum MagnMode { MGN_POWER_DN=0, MGN_SNGL_MEAS=1, MGN_CONT_MEAS1=2,MGN_CONT_MEAS2=6,MGN_EX_TRIG=4,
                MGN_SELF_TEST=8, MGN_FUSE_ROM=15}; // CNTL1 offset 0
enum SampleRate {SR_1000HZ=0, SR_200HZ=4, SR_100HZ=9 };  // 1kHz/(1+SRD) 
enum FusionMethod { COMPLEMENTARY, MADGWICK};             
struct Vect3 {float x,y,z;};
class MPU9250_SPI {
    float aRes ;      // 가속도 (LSB 당의 값)
    float gRes ;      // 자이로 (LSB 당의 값)
    float mRes ;      // 지자기 (LSB 당의 값)
    AccelRange _accelRange;     
    GyroRange _gyroRange;     
    DlpfBandwidth _bandwidth;
    MagnMode _mMode; 
    MagnBits _mBits;
    SampleRate _srd;
    Vect3 magCalibration ;   // factory mag calibration
    Vect3 magBias  ;     //지자기 바이어스
    Vect3 magScale   ;   // 지자기 스케일
    Vect3 gyroBias  ;  // 자이로 바이어스
    Vect3 accelBias  ;   // 가속도 바이어스
    int16_t tempCount;            // 원시 온도값
    float temperature;          //실제 온도값 (도C)
    float SelfTestResult[6];      // 자이로 가속도 실험결과
    Vect3 a, g, m;
    float q[4];      // 사원수 배열
    float roll, pitch, yaw;
    float a12, a22, a31, a32, a33;            // 회전행렬의 계수 
    float magnetic_declination;       // Seoul 2019.1.2
    SPI _spi;
    DigitalOut _csPin;
    InterruptIn _intPin;
    Timer _tmr;
    volatile static bool _dataReady;  
    QuaternionFilter qFilter;
public:
   // MPU9250_SPI(SPIClass& bus,uint8_t csPin,uint8_t intPin);
    MPU9250_SPI(PinName mosi,PinName miso,PinName sclk, PinName cs, PinName intpin );
    void setup() ;
    void update(FusionMethod)  ;
    void update(Vect3& _a,Vect3& _g,Vect3& _m)   ;
    Vect3 getAccBias() const { return accelBias; }
    Vect3 getGyroBias() const { return gyroBias; }
    Vect3 getMagBias() const { return magBias; }
    Vect3 getMagScale() const { return magScale; }
    void setAccBias(Vect3 v) { accelBias = v; }
    void setGyroBias(Vect3 v) { gyroBias = v; }
    void setMagBias(Vect3 v) { magBias = v; }
    void setMagScale(Vect3 v) { magScale = v; }
    void setMagneticDeclination(const float d) { magnetic_declination = d; }
    void setAccelRange(AccelRange range) ;
    void setGyroRange(GyroRange range);
    void setDlpfBandwidth(DlpfBandwidth bandwidth);
    void setSampleRate(SampleRate srd);
    void calibrateMag();
    void calibrateGyro() ;
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    Vect3 getAccelVect() {return a;}
    Vect3 getGyroVect() {return g;}
    Vect3 getMagVect() {return m;}
    void enableDataReadyInterrupt();
    bool isDataReady(){return _dataReady;}
  private:
    uint8_t isConnectedMPU9250() ;
    uint8_t isConnectedAK8963() ;  
    void initMPU9250()   ;
    void initAK8963()   ;
    void intService(){ _dataReady=true;   }
    void updateSensors();
    void updateAccelGyro()   ;
    void readMPU9250Data(int16_t * destination)   ;
    void updateMag()   ;
    void readMagData(int16_t * destination)  ;   
    void magCalMPU9250();
    void updateRPY();
    void compFilter(float dt);
    void writeByte(uint8_t subAddress, uint8_t data);
    uint8_t readByte(uint8_t subAddress);
    void readBytes(uint8_t subAddress, uint8_t count, uint8_t* dest);
    void replaceBlock(uint8_t address, uint8_t block, uint8_t at, uint8_t sz);
    void replaceBlockAK(uint8_t address, uint8_t block, uint8_t at, uint8_t sz);
    void writeAK8963Byte(uint8_t subAddress, uint8_t data);
    void readAK8963Bytes(uint8_t subAddress, uint8_t count, uint8_t* dest);   
    uint8_t readAK8963Byte(uint8_t subAddress); 
};
#endif

