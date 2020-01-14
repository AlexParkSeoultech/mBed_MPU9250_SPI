#include "mbed.h"
#include  "MPU9250_SPI.h"
#include  "MPU9250RegisterMap.h"
#include <cmath>
// MPU9250 with SPI interface library Ver. 0.98
// Made by HeeJae Park 
// 2019.05.27
extern Serial pc;
volatile bool MPU9250_SPI::_dataReady=false;
MPU9250_SPI::MPU9250_SPI(PinName mosi,PinName miso,PinName sclk, PinName cs, PinName intpin)
: _spi(mosi,miso,sclk), _csPin(cs), _intPin(intpin),_mMode(MGN_CONT_MEAS2),_mBits( MGN_16BITS),_srd(SR_100HZ)  {
      magCalibration.x=0;magCalibration.y=0;magCalibration.z=0;   
      magBias.x=0; magBias.y=0; magBias.z=0;
      magScale.x=1;magScale.y=1;magScale.z=1;
      gyroBias.x =0; gyroBias.y =0; gyroBias.z =0; 
      accelBias.x=0; accelBias.y=0; accelBias.z=0; 
      magnetic_declination = 8.5;
      q[0]=1.0f; q[1]=0.0f;  q[2]=0.0f;  q[3]=0.0f;
      _csPin=1;
}
void  MPU9250_SPI::setup() {      
   _csPin=1;      // setting CS pin high  
  _spi.format(8,3); // SPI mode 3
  _spi.frequency(SPI_HS_CLOCK); // 1Mega
  uint8_t m_whoami = 0x00;
  uint8_t a_whoami = 0x00;
//  while(true){
//  // 
//         _csPin = 0;              // select device
//        _spi.write(0x75 | 0x80);    // to read slave data
//        int whoami = _spi.write(0x33);      // send any byte to read
//        _csPin = 1; // Deselect the device
//        pc.printf("WHOAMI register = 0x%X\n", whoami);
//     wait(0.1);
//  }
   m_whoami = isConnectedMPU9250();
 if (m_whoami==MPU9250_WHOAMI_DEFAULT_VALUE) {
    #ifdef  PRINT_DETAILS
      pc.printf("MPU9250 is online...\n");
    #endif
      initMPU9250();
      a_whoami = isConnectedAK8963();
      if (a_whoami == AK8963_WHOAMI_DEFAULT_VALUE)             {
          initAK8963();
      }
      else {
          pc.printf("Could not connect to AK8963: 0x%x\n",a_whoami);
          while(1);
      }
  }
  else {
      pc.printf("Could not connect to MPU9250: 0x%x\n",m_whoami);
      while(1);
  }  
  _intPin.rise(callback(this, &MPU9250_SPI::intService)); 
  _tmr.start();
} 
void MPU9250_SPI::update(FusionMethod method)    {
  if (_dataReady){
    updateSensors();
    float deltat = (_tmr.read_us()/ 1000000.0f); // set integration time by time elapsed since last filter update
    _tmr.reset();
    if (method == MADGWICK){
      qFilter.update(-a.y, -a.x, a.z, g.y, g.x, -g.z, m.x, m.y, m.z, q,deltat);  // dot의 방향이 전진인 경우
      updateRPY();
    }
    else
        compFilter(deltat);
  }
}
void MPU9250_SPI::compFilter(float dt){
  Vect3 _a, _g, _m;
  _a.x= a.y; _a.y= a.x; _a.z= -a.z;
  _g.x= DEG_TO_RAD*g.y; _g.y= DEG_TO_RAD*g.x; _g.z= -DEG_TO_RAD*g.z; 
  float ayz= sqrt(_a.y*_a.y+_a.z*_a.z);
  float rollAcc= atan2(-_a.y,-_a.z);
  float pitchAcc= atan2(_a.x,ayz);
  roll= ALPHA*(roll+_g.x*dt)+BETA*rollAcc;
  pitch= ALPHA*(pitch+_g.y*dt)+BETA*pitchAcc;
  float c_th=cos(pitch), s_th=sin(pitch), c_pi=cos(roll), s_pi=sin(roll); 
  _m.x= m.x*c_th+m.y*s_pi*c_th+m.z*c_pi*s_th;
  _m.y= m.y*c_pi-m.z*s_pi;
  float heading=-atan2(_m.y, _m.x);
  if ((yaw-heading)>M_PI) heading+=TWO_PI;
  else if ((yaw-heading)<-M_PI) yaw+=TWO_PI; 
  yaw= ALPHA*(yaw+_g.z*dt)+BETA*heading;
  yaw= (yaw> M_PI) ? (yaw - TWO_PI) : ((yaw < -M_PI) ? (yaw +TWO_PI) : yaw);
}
void MPU9250_SPI::update(Vect3& _a,Vect3& _g,Vect3& _m)    {
  if (_dataReady){  // On interrupt, check if data ready interrupt
      updateSensors();
      _a=a;_g=g;_m=m;
   }
}

uint8_t MPU9250_SPI::isConnectedMPU9250() {
    uint8_t c = readByte(WHO_AM_I_MPU9250);
    #ifdef  PRINT_DETAILS
    pc.printf("MPU9250 WHO AM I = 0x%x\n",c);
    #endif
    return c; // (c == MPU9250_WHOAMI_DEFAULT_VALUE);
}
uint8_t MPU9250_SPI::isConnectedAK8963() {
    uint8_t c = readAK8963Byte(AK8963_WHO_AM_I);
    #ifdef  PRINT_DETAILS
    pc.printf("AK8963  WHO AM I = 0x%x\n ",c);
    #endif
    return c; // (c == AK8963_WHOAMI_DEFAULT_VALUE);
}

void MPU9250_SPI::initMPU9250()    {
    wait_ms(100);
    writeByte(PWR_MGMT_1, CLOCK_SEL_PLL);
    writeByte(USER_CTRL,I2C_MST_EN);       // Master enable
    writeByte(I2C_MST_CTRL,I2C_MST_CLK);   // I2C master clock =400HZ
    replaceBlockAK(AK8963_CNTL,MGN_POWER_DN,0,4); // Power down
    writeByte(PWR_MGMT_1, PWR_RESET); // Clear sleep mode bit (6), enable all sensors  
    wait_ms(100);
    writeByte(PWR_MGMT_1, CLOCK_SEL_PLL);
    setDlpfBandwidth( DLPF_BANDWIDTH_5HZ);
    writeByte(SMPLRT_DIV, SR_100HZ);  //{SR_1000HZ=0, SR_200HZ=4, SR_100HZ=9 }
    setGyroRange(GYRO_RANGE_2000DPS);             
    writeByte(PWR_MGMT_2,SEN_ENABLE);      
    setAccelRange(ACCEL_RANGE_16G);//{ _2G, _4G,  _8G,  _16G  }
    setDlpfBandwidth(DLPF_BANDWIDTH_184HZ);  // [250HZ, 184HZ,  92HZ,  41HZ, 20HZ,  10HZ,  5HZ]    
    writeByte(INT_PIN_CFG, 0x20);  // LATCH_INT_EN=1,  BYPASS_EN=1-->0 (0x22)
    writeByte(INT_ENABLE, 0x01);  // Enable raw data ready (bit 0) interrupt
    writeByte(USER_CTRL,I2C_MST_EN);
    wait_ms(100);
    writeByte(I2C_MST_CTRL,I2C_MST_CLK);         
    wait_ms(100);
}

void MPU9250_SPI::initAK8963()    {
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    replaceBlockAK(AK8963_CNTL,MGN_POWER_DN,0,4); // Power down magnetometer
    wait_ms(50);
    replaceBlockAK(AK8963_CNTL,MGN_FUSE_ROM,0,4);
    wait_ms(50);
    readAK8963Bytes( AK8963_ASAX, 3, rawData);  // Read the x-, y-, and z-axis calibration values
    magCalibration.x =  (float)(rawData[0] - 128)/256.f + 1.f;   // Return x-axis sensitivity adjustment values, etc.
    magCalibration.y =  (float)(rawData[1] - 128)/256.f + 1.f;
    magCalibration.z =  (float)(rawData[2] - 128)/256.f + 1.f;
    replaceBlockAK(AK8963_CNTL,MGN_POWER_DN,0,4); // Power down magnetometer
    wait_ms(50);
    replaceBlockAK(AK8963_CNTL,((_mBits << 4 )| _mMode),0,5); // Set measurment mode, mMode[0:3]
    writeByte(PWR_MGMT_1,CLOCK_SEL_PLL);
    wait_ms(50);   
    mRes=10. * 4912. / 32760.0;  // for Magenetometer 16BITS
#ifdef  PRINT_DETAILS
    pc.printf("Calibration values:\n");
    pc.printf("X-Axis sensitivity adjustment value %7.2f \n",magCalibration.x); 
    pc.printf("Y-Axis sensitivity adjustment value %7.2f \n",magCalibration.y);
    pc.printf("Z-Axis sensitivity adjustment value %7.2f \n",magCalibration.z); 
    pc.printf("X-Axis sensitivity offset value %7.2f \n",magBias.x); 
    pc.printf("Y-Axis sensitivity offset value %7.2f \n",magBias.y);
    pc.printf("Z-Axis sensitivity offset value %7.2f \n",magBias.z); 
#endif
}
void MPU9250_SPI::setAccelRange(AccelRange range) {
   switch(range) {
     case ACCEL_RANGE_2G: 
     aRes =  2.0f/32767.5f;  break;     
    case ACCEL_RANGE_4G: 
     aRes =  4.0f/32767.5f;   break;    
    case ACCEL_RANGE_8G: 
      aRes =  8.0f/32767.5f;  break;    
    case ACCEL_RANGE_16G: 
      aRes = 16.0f/32767.5f; // setting the accel scale to 16G
      break;    
   }
   replaceBlock(ACCEL_CONFIG,range,3,2); // addr, value, at, size 
   _accelRange = range;
}
void MPU9250_SPI::setGyroRange(GyroRange range) {
  switch(range) {
    case GYRO_RANGE_250DPS: 
      gRes =  250.0f/32767.5f;  break;   
    case GYRO_RANGE_500DPS: 
      gRes =  500.0f/32767.5f; break;      
    case GYRO_RANGE_1000DPS:
      gRes =  1000.0f/32767.5f; break; 
    case GYRO_RANGE_2000DPS:   
     gRes =  2000.0f/32767.5f ; break; 
  }
  replaceBlock(GYRO_CONFIG,range,3,2);
  _gyroRange = range;
}
void MPU9250_SPI::setDlpfBandwidth(DlpfBandwidth bandwidth) {
  replaceBlock(ACCEL_CONFIG2,bandwidth,0,4);     //Accel DLPF [0:2]
  replaceBlock(MPU_CONFIG,bandwidth,0,3);        //Gyro DLPF [0:2]
  _bandwidth = bandwidth;
}

void MPU9250_SPI::setSampleRate(SampleRate srd){
   writeByte(SMPLRT_DIV, srd);   // sampling rate set
   _srd = srd;
}

void MPU9250_SPI::calibrateMag() {
    magCalMPU9250();
}

void MPU9250_SPI::enableDataReadyInterrupt() {
  writeByte(INT_PIN_CFG,0x00);  // setup interrupt, 50 us pulse
  writeByte(INT_ENABLE,0x01) ; // set to data ready
}

void MPU9250_SPI::updateSensors(){
  int16_t MPU9250Data[10]; // MPU9250 accel/gyro 에서 16비트 정수로 7개 저장
  uint8_t rawData[21];  // 가속도 자이로 원시 데이터 보관
  writeByte(I2C_SLV0_ADDR,AK8963_I2C_ADDR|SPI_READ); // Set the I2C slave addres of AK8963 and set for read.
  writeByte(I2C_SLV0_REG,AK8963_XOUT_L);   // I2C slave 0 register address from where to begin data transfer
  writeByte(I2C_SLV0_CTRL, 0x87);                     // Read 7 bytes from the magnetometer
  readBytes(ACCEL_XOUT_H, 21, rawData);  // 16비트 정수로 7개 저장--> 14byte
  MPU9250Data[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // signed 16-bit  (MSB + LSB)
  MPU9250Data[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  MPU9250Data[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  MPU9250Data[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  MPU9250Data[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  MPU9250Data[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  MPU9250Data[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 
  MPU9250Data[7] = (((int16_t)rawData[15]) << 8) |rawData[14];
  MPU9250Data[8] = (((int16_t)rawData[17]) << 8) |rawData[16];
  MPU9250Data[9] = (((int16_t)rawData[19]) << 8) |rawData[18];                
  a.x = (float)MPU9250Data[0] * aRes - accelBias.x;  // 가속도 해상도와 바이어스 보정 
  a.y = (float)MPU9250Data[1] * aRes - accelBias.y;
  a.z = (float)MPU9250Data[2] * aRes - accelBias.z;
  g.x = (float)MPU9250Data[4] * gRes- gyroBias.x;  // 자이로 해상도 보정
  g.y = (float)MPU9250Data[5] * gRes- gyroBias.y;  // 자이로 바이어스는 칩내부에서 보정함!!!
  g.z = (float)MPU9250Data[6] * gRes- gyroBias.z;  
  m.x = (float)(MPU9250Data[7] * mRes * magCalibration.x - magBias.x) * magScale.x;  
  m.y = (float)(MPU9250Data[8] * mRes * magCalibration.y - magBias.y) * magScale.y;
  m.z = (float)(MPU9250Data[9] * mRes * magCalibration.z - magBias.z) * magScale.z;               
}
void MPU9250_SPI::updateAccelGyro()    {
    int16_t MPU9250Data[7]; // MPU9250 accel/gyro 에서 16비트 정수로 7개 저장
    readMPU9250Data(MPU9250Data); // 읽으면 INT 핀 해제 
    a.x = (float)MPU9250Data[0] * aRes - accelBias.x;  // 가속도 해상도와 바이어스 보정 
    a.y = (float)MPU9250Data[1] * aRes - accelBias.y;
    a.z = (float)MPU9250Data[2] * aRes - accelBias.z;
    g.x = (float)MPU9250Data[4] * gRes - gyroBias.x;  // 자이로 해상도 보정
    g.y = (float)MPU9250Data[5] * gRes - gyroBias.y;  // 
    g.z = (float)MPU9250Data[6] * gRes - gyroBias.z;
}

void MPU9250_SPI::readMPU9250Data(int16_t * destination)     {
    uint8_t rawData[14];  // 가속도 자이로 원시 데이터 보관
    readBytes(ACCEL_XOUT_H, 14, rawData);  // 16비트 정수로 7개 저장--> 14byte
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // signed 16-bit  (MSB + LSB)
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
    destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
    destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
    destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
    destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

void MPU9250_SPI::updateMag()    {
    int16_t magCount[3] = {0, 0, 0};    // 16-bit 지자기 데이터
    readMagData(magCount);  // 지자기 데이터 읽기
    // 지자기 해상도, 검정값, 바이어스 보정,  검정값 (magCalibration[] )은 칩의 ROM에서 
    m.x = (float)(magCount[0] * mRes * magCalibration.x - magBias.x) * magScale.x;  
    m.y = (float)(magCount[1] * mRes * magCalibration.y - magBias.y) * magScale.y;
    m.z = (float)(magCount[2] * mRes * magCalibration.z - magBias.z) * magScale.z;
}
void MPU9250_SPI::readMagData(int16_t * destination)    {
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    if(readAK8963Byte(AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
        readAK8963Bytes(AK8963_XOUT_L, 7,rawData);  // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
            destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
        }
    }
}  

void MPU9250_SPI::calibrateGyro() { 
  size_t  _numSamples=100;
  Vect3 gyroSum;gyroSum.x=0;gyroSum.y=0;gyroSum.z=0;
  pc.printf("Please, don't move!\n");
  wait_ms(100);
  for (size_t i=0; i < _numSamples; i++) {
    updateAccelGyro();
    gyroSum.x+= (g.x + gyroBias.x)/(( float)_numSamples);
    gyroSum.y+= (g.y  + gyroBias.y)/(( float)_numSamples);
    gyroSum.z+= (g.z  + gyroBias.z)/(( float)_numSamples);
    wait_ms(20);
  }
  gyroBias = gyroSum;
    pc.printf("Gyro Calibration done!\n");
    pc.printf("MPU9250 Gyro biases (deg/s)\n");
    pc.printf("%7.3f,  %7.3f  , %7.3f\n", gyroBias.x, gyroBias.y, gyroBias.z);
} 
void MPU9250_SPI::magCalMPU9250()    {
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
    pc.printf("Mag Calibration: Wave device in a figure eight until done!\n");
    wait_ms(3000);
    if      (_mMode == MGN_CONT_MEAS1) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    else if (_mMode == MGN_CONT_MEAS2) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
    for(ii = 0; ii < sample_count; ii++)   {
        readMagData(mag_temp);  // Read the mag data
        for (int jj = 0; jj < 3; jj++)            {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        if(_mMode == MGN_CONT_MEAS1) wait_ms(135);  // at 8 Hz ODR, new mag data is available every 125 ms
        if(_mMode == MGN_CONT_MEAS2) wait_ms(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }
    pc.printf("mag x min/max: %d /%d \n",mag_min[0],mag_max[0]);
    pc.printf("mag y min/max: %d /%d \n",mag_min[1],mag_max[1]);    
    pc.printf("mag z min/max: %d /%d \n",mag_min[2],mag_max[2]);
    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    magBias.x = (float) mag_bias[0]*mRes*magCalibration.x;  // save mag biases in G for main program
    magBias.y = (float) mag_bias[1]*mRes*magCalibration.y;
    magBias.z = (float) mag_bias[2]*mRes*magCalibration.z;

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    magScale.x = avg_rad/((float)mag_scale[0]);
    magScale.y = avg_rad/((float)mag_scale[1]);
    magScale.z = avg_rad/((float)mag_scale[2]);
    pc.printf("Mag Calibration done!\n");
    pc.printf("AK8963 mag biases (mG)\_numSamples\n");
    pc.printf("%7.3f,  %7.3f  , %7.3f\n", magBias.x, magBias.y, magBias.z); 
    pc.printf("AK8963 mag scale (mG)\n");
    pc.printf("%7.3f,  %7.3f  , %7.3f\n", magScale.x, magScale.y,magScale.z);
}     
void MPU9250_SPI::writeByte(uint8_t subAddress, uint8_t data){   /* write data to device */
    // _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
    // digitalWrite(_csPin,LOW); // select the MPU9250 chip
    // _spi->transfer(subAddress); // write the register address
    // _spi->transfer(data); // write the data
    // digitalWrite(_csPin,HIGH); // deselect the MPU9250 chip
    // _spi->endTransaction(); // end the transaction
    _spi.frequency(SPI_LS_CLOCK); // setup clock
    _csPin=0; // select the MPU9250 chip
    _spi.write(subAddress); // write the register address
    _spi.write(data); // write the data
    _csPin=1; // deselect the MPU9250 chip
}
uint8_t MPU9250_SPI::readByte(uint8_t subAddress){
    // _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
    // digitalWrite(_csPin,LOW); // select the MPU9250 chip
    // _spi->transfer(subAddress | SPI_READ); // specify the starting register address
    // uint8_t data = _spi->transfer(0x00); // read the data
    // digitalWrite(_csPin,HIGH); // deselect the MPU9250 chip
    // _spi->endTransaction(); // end the transaction

    _spi.frequency(SPI_LS_CLOCK); // setup clock
    _csPin=0; // select the MPU9250 chip
    _spi.write(subAddress| SPI_READ); // use READ MASK
    uint8_t data =_spi.write(0);   // write any to get data
    _csPin=1; // deselect the MPU9250 chip 
    return data;
}

void MPU9250_SPI::readBytes(uint8_t subAddress, uint8_t cnt, uint8_t* dest){
    // _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
    // digitalWrite(_csPin,LOW); // select the MPU9250 chip
    // _spi->transfer(subAddress | SPI_READ); // specify the starting register address
    // for(uint8_t i = 0; i < count; i++){
    //   dest[i] = _spi->transfer(0x00); // read the data
    // }
    // digitalWrite(_csPin,HIGH); // deselect the MPU9250 chip
    // _spi->endTransaction(); // end the transaction
    _spi.frequency(SPI_HS_CLOCK); // setup clock
     _csPin=0; // select the MPU9250 chip
    _spi.write(subAddress | SPI_READ); // specify the starting register address
    for(uint8_t i = 0; i < cnt; i++){
      dest[i] = _spi.write(0x00); // read the data
    }
    _csPin=1; // deselect the MPU9250 chip
}

void MPU9250_SPI::writeAK8963Byte(uint8_t subAddress, uint8_t data){   
    writeByte(I2C_SLV0_ADDR,AK8963_I2C_ADDR) ; // set slave 0 to the AK8963 and set for write
    writeByte(I2C_SLV0_REG,subAddress) ; // set the register to the desired AK8963 sub address 
    writeByte(I2C_SLV0_DO,data) ; // store the data for write
    writeByte(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1); // enable I2C and send 1 byte
}

void MPU9250_SPI::readAK8963Bytes(uint8_t subAddress, uint8_t count, uint8_t* dest){
   writeByte(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) ; // set slave 0 to the AK8963 and set for read
   writeByte(I2C_SLV0_REG,subAddress) ; // set the register to the desired AK8963 sub address
   writeByte(I2C_SLV0_CTRL,I2C_SLV0_EN | count); // enable I2C and request the bytes
   wait_ms(1); // takes some time for these registers to fill
   readBytes(EXT_SENS_DATA_00,count,dest);  // read the bytes off the MPU9250 EXT_SENS_DATA registers
}

uint8_t MPU9250_SPI::readAK8963Byte(uint8_t subAddress){
  writeByte(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) ; // set slave 0 to the AK8963 and set for read
  writeByte(I2C_SLV0_REG,subAddress) ;  // set the register to the desired AK8963 sub address
  writeByte(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);   // enable I2C and request the bytes
  wait_ms(11); // takes some time for these registers to fill
  return  readByte(EXT_SENS_DATA_00);  // read the bytes off the MPU9250 EXT_SENS_DATA registers 
}
void MPU9250_SPI::replaceBlock(uint8_t address, uint8_t block, uint8_t at, uint8_t sz){
  uint8_t data=readByte(address);
  data &= ~(((1<<sz)-1)<<at);
  data |= block<<at;
  writeByte(address, data );
}
void MPU9250_SPI::replaceBlockAK(uint8_t address, uint8_t block, uint8_t at, uint8_t sz){
  uint8_t data=readByte(address);
  data &= ~(((1<<sz)-1)<<at);
  data |= block<<at;
  writeAK8963Byte(address, data );
}    
void MPU9250_SPI::updateRPY()  {
    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);
    yaw   += magnetic_declination*DEG_TO_RAD;
    yaw= (yaw> M_PI) ? (yaw - TWO_PI) : ((yaw < - M_PI) ? (yaw +TWO_PI) : yaw);
}
    

