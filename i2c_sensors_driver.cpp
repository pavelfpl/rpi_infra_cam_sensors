/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: i2c_sensors_driver.cpp (c++)
*********************************************************************************/
/* Pavel Fiala notes
 * ------------------------
!!! NOTE: add selected user to i2c group [to run driver without root permission] !!!
e.g. sudo adduser odroid i2c
*
*/

/* Richard Linhart notes:
 * -------------------------------
sudo modprobe i2c-dev
sudo modprobe i2c-tiny-usb
sudo usermod -a -G i2c riki
http://elinux.org/Interfacing_with_I2C_Devices
http://raspberrypi.stackexchange.com/questions/3627/is-there-an-i2c-library
*
*/

// GLOBAL constants ...
// -----------------------------
#define CONST_CYCLE_REPEAT 10000000
#define CONST_ACC_GYRO_SIZE 128
#define CONST_GRID_EYE_SIZE 128

#define STATUS_OK 1
#define STATUS_ERROR -1
#define STATUS_EXIT -2

#define I2C_OPEN_FAILED -1
#define I2C_IOCTL_FAILED -2
#define I2C_WRITE_FAILED -3
#define I2C_READ_FAILED -4
#define I2C_CLOSE_FAILED -5

// Use configuration file - option ...
// ---------------------------------------------
// #define CONST_USE_CONFIGURATION_FILE

// I2C Addresses - GRIDEYE and ACCGYRO and  BME_200 ...
// ----------------------------------------------------------------------------------
#define I2C_GRIDEYE 0x69
#define I2C_ACCGYRO 0x68
#define I2C_BME_200 0x77

// MPU-6000 Settings ...
// -------------------------------
#define GYRO_SENS  0x10  // Reg. 0x1B without shifting
#define ACCL_SENS  0x10  // Reg. 0x1C without shifting
#define RATE_DIV    100  // Reg. 0x19
#define DLPF_SET      6  // Reg. 0x1A only DLPF
#define FIFO_VALS  0x78  // Reg. 0x23 values to transfer via FIFO
#define CYCLES       10  // Number of measured cycles

#include <i2c_sensors_driver.h>

#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// #define I2C_MSG_FMT char
#ifndef I2C_FUNC_I2C
#include <linux/i2c.h>
#define I2C_MSG_FMT __u8
#endif

#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// toString helper function / template ...
// ---------------------------------------------------
template <typename T> std::string toString(const T& t){
      
    std::ostringstream os;
    os << t;
    return os.str();
}

static std::string NumberToStringDec(int8_t number){
    
    std::ostringstream ss;
    ss << (int8_t)number;
    return ss.str(); 
}

// i2c_sensors_driver constructor ...
// ----------------------------------------------
i2c_sensors_driver::i2c_sensors_driver(std::string file_name_,std::string i2c_device0_,std::string i2c_device1_){

    file_name = file_name_;
    i2c_device0 = i2c_device0_;
    i2c_device1 = i2c_device1_;
    
    accGyroStatusRdy = false;
    gridEyeStatusRdy = false;
    bme200StatusRdy = false;
    mlx90621StatusRdy = false;
    mlx90640StatusRdy = false;
    
    stopAcquistionI2C = false;
    
    accGyroCount = 0;
    gridEyeCount = 0;
    bmeCount = 0;
    mlx90621Count = 0;
    mlx90640Count = 0;
    
    boost::posix_time::ptime my_posix_time =  boost::posix_time::second_clock::local_time();
    std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);   
    
    accGyroFile_="/home/pi/sensors/i2c_sensors_store/accGyroLog_"+iso_time_string+".log";  
    gridEyeFile_="/home/pi/sensors/i2c_sensors_store/gridEyeLog_"+iso_time_string+".log";    
    bmeFile_="/home/pi/sensors/i2c_sensors_store/bmeLog_"+iso_time_string+".log";    
    mlx90621File_="/home/pi/sensors/i2c_sensors_store/mlx90621Log_"+iso_time_string+".log";   
    mlx90640File_="/home/pi/sensors/i2c_sensors_store/mlx90640Log_"+iso_time_string+".log"; 
    mlx90640RawFile_="/home/pi/sensors/i2c_sensors_store/mlx90640RawLog_"+iso_time_string+".log"; 
    
    acc.ok=false; 
    for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){acc.x_data[i]= 0;acc.y_data[i]= 0;acc.z_data[i]= 0;}
    gyro.ok=false;
    for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){gyro.x_data[i]= 0;gyro.y_data[i]= 0;gyro.z_data[i]= 0;}
    
    gridEye.ok = false; gridEye.thermistor = 0.0;
    
    for(int i=0;i<CONST_GRID_EYE_DOUBLE_SIZE;i++){ gridEye.data[i] = (double)0.0; }
    
    dig_T1=0,dig_T2=0,dig_T3=0;
    dig_P1=0,dig_P2=0,dig_P3=0,dig_P4=0,dig_P5=0;
    dig_P6=0,dig_P7=0,dig_P8=0,dig_P9=0;
    dig_H1=0,dig_H2=0,dig_H3=0,dig_H4=0,dig_H5=0,dig_H6=0;
    
    bme.ok = false; bme.tempCelsius = 0.0; bme.tempFahrenheit = 0.0;
    bme.pressureData = 0.0; bme.humidityData = 0.0;

    mlx16x4.ok = false;
    mlx32x24.ok = false;
    
#ifdef CONST_USE_CONFIGURATION_FILE
    readParamsFromFile(file_name);
#endif
    
}

// initSensorBme200 - public function  ...
// ------------------------------------------------------
int32_t i2c_sensors_driver::initSensorBme200(){

    uint8_t buf[24];
    
    bme200StatusRdy = false;
    
    // Read 24 bytes of data from register(0x88) ...
    // -------------------------------------------------------------- 
    if (i2c_read_buf(i2c_device1.c_str(), I2C_BME_200, 0x88, 24, buf)!=STATUS_OK){
        return STATUS_ERROR;
    }
    
    // Convert the data - temp coefficents ...
    // -----------------------------------------------------
	dig_T1 = (buf[0] + buf[1] * 256);
    dig_T2 = (buf[2] + buf[3] * 256);
	
    if(dig_T2 > 32767) dig_T2 -= 65536;
	
    dig_T3 = (buf[4] + buf[5] * 256);
	if(dig_T3 > 32767) dig_T3 -= 65536;

	//  and pressure coefficents ...
    // --------------------------------------
	dig_P1 = (buf[6] + buf[7] * 256);
	dig_P2 = (buf[8] + buf[9] * 256);
	
    if(dig_P2 > 32767) dig_P2 -= 65536;

    dig_P3 = (buf[10] + buf[11] * 256);
	
    if(dig_P3 > 32767) dig_P3 -= 65536;
	
    dig_P4 = (buf[12] + buf[13] * 256);
	if(dig_P4 > 32767) dig_P4 -= 65536;
	
	dig_P5 = (buf[14] + buf[15] * 256);
	if(dig_P5 > 32767) dig_P5 -= 65536;

	dig_P6 = (buf[16] + buf[17] * 256);
	if(dig_P6 > 32767) dig_P6 -= 65536;

    dig_P7 = (buf[18] + buf[19] * 256);
	if(dig_P7 > 32767) dig_P7 -= 65536;
	dig_P8 = (buf[20] + buf[21] * 256);
	if(dig_P8 > 32767) dig_P8 -= 65536;
	
	dig_P9 = (buf[22] + buf[23] * 256);
	if(dig_P9 > 32767) dig_P9 -= 65536;
       
    // Read 1 byte of data from register(0xA1) ...
    // -----------------------------------------------------------
    if (i2c_read_buf(i2c_device1.c_str(), I2C_BME_200, 0xA1, 1, buf)!=STATUS_OK){
        return STATUS_ERROR;
    }
    
    int dig_H1 = buf[0];
    
    // Read 7 bytes of data from register(0xE1) ...
    // ------------------------------------------------------------ 
    if (i2c_read_buf(i2c_device1.c_str(), I2C_BME_200, 0xE1, 7, buf)!=STATUS_OK){
        return STATUS_ERROR;
    }
    
    // Convert the data - humidity coefficents ...
    // ----------------------------------------------------------
    dig_H2 = (buf[0] + buf[1] * 256);
	if(dig_H2 > 32767) dig_H2 -= 65536;
	
    dig_H3 = buf[2] & 0xFF ;
	dig_H4 = (buf[3] * 16 + (buf[4] & 0xF));
	
    if(dig_H4 > 32767) dig_H4 -= 65536;

	dig_H5 = (buf[4] / 16) + (buf[5] * 16);
	if(dig_H5 > 32767) dig_H5 -= 65536;
	
    dig_H6 = buf[6];
    if(dig_H6 > 127) dig_H6 -= 256;
    
    // Select control humidity register(0xF2)  - humidity over sampling rate = 1(0x01) ...
    // -----------------------------------------------------------------------------------------------------------------
    buf[0] = 0xF2;
    buf[1] = 0x01;
    
    if(i2c_write_buf(i2c_device0.c_str(), I2C_BME_200, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;
    }

	// Select control measurement register(0xF4) - normal mode, temp and pressure over sampling rate = 1(0x27) ...
	// ---------------------------------------------------------------------------------------------------------------------------------------------------------
	buf[0] = 0xF4;
	buf[1] = 0x27;
    
    if(i2c_write_buf(i2c_device0.c_str(), I2C_BME_200, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;
    }

	// Select config register(0xF5) - stand_by time = 1000 ms(0xA0) ...
	// -----------------------------------------------------------------------------------------
	buf[0] = 0xF5;
	buf[1] = 0xA0;
    
    if(i2c_write_buf(i2c_device0.c_str(), I2C_BME_200, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;
    }

    bme200StatusRdy = true;
    
    return STATUS_OK; // configuration was properly set - OK ...
}

// initGridEyeSensor - public function / return status ...
// -------------------------------------------------------
int32_t i2c_sensors_driver::initGridEyeSensor(){
    
    uint8_t buf[2];

    gridEyeStatusRdy = false;

    // Grid Eye - Normal Mode ...
    // --------------------------
    buf[0] = 0x00;
    buf[1] = 0x00;
    
    if(i2c_write_buf(i2c_device0.c_str(), I2C_GRIDEYE, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;
    }

    // Grid Eye - Full Software Reset ...
    // ----------------------------------
    buf[0] = 0x01;
    buf[1] = 0x3F;
    
    if(i2c_write_buf(i2c_device0.c_str(), I2C_GRIDEYE, 2, buf)!=STATUS_OK){
       return STATUS_ERROR; 
    }
    
    // Wait 500 000 micro seconds after reset ...
    // ------------------------------------------
    usleep(500000);
    
    // Grid Eye - Frame Rate - 1 FPS  or 10 FPS ...
    // --------------------------------------------
    buf[0] = 0x02;
    buf[1] = 0x01; // 0x00 -> for 10 FPS ...
    
    if (i2c_write_buf(i2c_device0.c_str(), I2C_GRIDEYE, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;         
    }
	
    // Grid Eye - Average Register - Double Moving Average ...
    // -------------------------------------------------------
    buf[0] = 0x07;
    buf[1] = 0x20;
    
    if (i2c_write_buf(i2c_device0.c_str(), I2C_GRIDEYE, 2, buf)!=STATUS_OK){
        return STATUS_ERROR; 
    }

    gridEyeStatusRdy = true;
    
    return STATUS_OK; // configuration was properly set - OK ...
}

// initAccGyroSensor - public function / return status ...
// -------------------------------------------------------
int32_t i2c_sensors_driver::initAccGyroSensor(){
    
    uint8_t buf[3];
    
    accGyroStatusRdy = false;

    // Acc+Gyro - Reset ...
    // --------------------
    buf[0] = 0x6B;
    buf[1] = 0x80;
    
    if(i2c_write_buf(i2c_device1.c_str(), I2C_ACCGYRO, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;         
    }

    // Wait 500 000 micro seconds after reset ...
    // ------------------------------------------
    usleep(500000);
	
    // Acc+Gyro - Sleep Disable ...
    // ----------------------------
    buf[0] = 0x6B;
    buf[1] = 0x00;
    
    if(i2c_write_buf(i2c_device1.c_str(), I2C_ACCGYRO, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;    
    }
	
    // WhoAmI Readout ...
    // ------------------
    if (i2c_read_buf(i2c_device1.c_str(), I2C_ACCGYRO, 0x75, 1, buf)!=STATUS_OK){
        return STATUS_ERROR;
    }
    
    // Debug - disable in production ...
    // ---------------------------------
    // std::cout << "Whoami - debug:"<< std::hex << buf[0] << std::endl;
    
    // Option - selftest ...
    // ---------------------    
    
    // Measurement Setup - rate and bW ...
    // -----------------------------------
    buf[0] = 0x19;
    buf[1] = RATE_DIV;
    buf[2] = DLPF_SET;
    
    if(i2c_write_buf(i2c_device1.c_str(), I2C_ACCGYRO, 3, buf)!=STATUS_OK){
       return STATUS_ERROR;
    }

    // Measurement Setup - sensitivity ...
    // -----------------------------------
    buf[0] = 0x1B;
    buf[1] = GYRO_SENS;
    buf[2] = ACCL_SENS;
    if(i2c_write_buf(i2c_device1.c_str(), I2C_ACCGYRO, 3, buf)!=STATUS_OK){
       return STATUS_ERROR;  
    }

    // Measurement Setup - FIFO ...
    // ----------------------------
    buf[0] = 0x23;
    buf[1] = FIFO_VALS;
    if(i2c_write_buf(i2c_device1.c_str(), I2C_ACCGYRO, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;   
    }
	
    buf[0] = 0x6A;
    buf[1] = 0x40;
    if(i2c_write_buf(i2c_device1.c_str(), I2C_ACCGYRO, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;     
    }
    
    accGyroStatusRdy = true;

    return STATUS_OK; // Configuration was properly set - OK ...    
}

void i2c_sensors_driver::acquireDataBme(){
    
    uint8_t data[8];
    
   	// Read 8 bytes of data from register(0xF7) - pressure msb1, pressure msb, pressure lsb, temp msb1, temp msb, temp lsb, humidity lsb, humidity msb ...
    // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if (i2c_read_buf(i2c_device1.c_str(), I2C_BME_200, 0xF7, 8, data)!=STATUS_OK){
        return;
    }
    
    // Convert pressure and temperature data to 19-bits ...
	// -------------------------------------------------------------------------
	int64_t adc_p = ((int64_t)(data[0] * 65536 + ((int64_t)(data[1] * 256) + (int64_t)(data[2] & 0xF0)))) / 16;
	int64_t adc_t = ((int64_t)(data[3] * 65536 + ((int64_t)(data[4] * 256) + (int64_t)(data[5] & 0xF0)))) / 16;
	// Convert the humidity data
	int64_t adc_h = (data[6] * 256 + data[7]);

	// Temperature offset calculations ...
    // ------------------------------------------------
	float var1 = (((float)adc_t) / 16384.0 - ((float)dig_T1) / 1024.0) * ((float)dig_T2);
	float var2 = ((((float)adc_t) / 131072.0 - ((float)dig_T1) / 8192.0) *
					(((float)adc_t)/131072.0 - ((float)dig_T1)/8192.0)) * ((float)dig_T3);
	float t_fine = (int64_t)(var1 + var2);
	float cTemp = (var1 + var2) / 5120.0;
	float fTemp = cTemp * 1.8 + 32;

	// Pressure offset calculations ...
    // ------------------------------------------
	var1 = ((float)t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((float)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((float)dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((float)dig_P4) * 65536.0);
	var1 = (((float) dig_P3) * var1 * var1 / 524288.0 + ((float) dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((float)dig_P1);
	float p = 1048576.0 - (float)adc_p;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((float) dig_P9) * p * p / 2147483648.0;
	var2 = p * ((float) dig_P8) / 32768.0;
	float pressure = (p + (var1 + var2 + ((float)dig_P7)) / 16.0) / 100;

	// Humidity offset calculations ...
    // -------------------------------------------
	float var_H = (((float)t_fine) - 76800.0);
	var_H = (adc_h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)));
	float humidity = var_H * (1.0 -  dig_H1 * var_H / 524288.0);
	
    if(humidity > 100.0) humidity = 100.0;
	if(humidity < 0.0)  humidity = 0.0;
	
    bme.ok = true;
    
    bme.tempCelsius = cTemp; 
    bme.tempFahrenheit = fTemp;
    bme.pressureData = pressure; 
    bme.humidityData = humidity;

   std::cout <<"Read Bme200 Data OK"<<std::endl;
}

// acquireSensorAccGyroData - public function ...
// ----------------------------------------------
void i2c_sensors_driver::acquireSensorAccGyroData(){
  
  uint32_t pfifo = 0;  
  uint8_t buf[CONST_ACC_GYRO_SIZE];  // I2C Buffer - max size 128 / real 120...
  
  for(int i=0;i<CONST_ACC_GYRO_SIZE;i++){
      buf[i] = 0x00;
  }
    
  for(int i=0;i<CONST_CYCLE_REPEAT;i++){
      pfifo = read_output_acc_gyro(i2c_device1.c_str(),0x72); 
      if(pfifo >= (CYCLES*12)) break;
      usleep(10);   // Sleep for 10 us ...
  }
  
  // Reading the FIFO ...
  // --------------------  
  if(i2c_read_buf(i2c_device1.c_str(), I2C_ACCGYRO, 0x74, CYCLES * 12, buf)!=STATUS_OK){
     acc.ok=false; 
     for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){acc.x_data[i]= 0;acc.y_data[i]= 0;acc.z_data[i]= 0;}
     gyro.ok=false;
     for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){gyro.x_data[i]= 0;gyro.y_data[i]= 0;gyro.z_data[i]= 0;}
     return;
  }
  
  acc.ok = true;
  gyro.ok = true;	

  for(int i = 0; i < CYCLES; i++) {
      acc.x_data[i] = (int16_t) buf[12 * i + 0] << 8 | buf[12 * i + 1];
      acc.y_data[i] = (int16_t) buf[12 * i + 2] << 8 | buf[12 * i + 3];
      acc.z_data[i] = (int16_t) buf[12 * i + 4] << 8 | buf[12 * i + 5];

      gyro.x_data[i] = (int16_t) buf[12 * i + 6] << 8 | buf[12 * i + 7];
      gyro.y_data[i] = (int16_t) buf[12 * i + 8] << 8 | buf[12 * i + 9];
      gyro.z_data[i] = (int16_t) buf[12 * i + 10] << 8 | buf[12 * i + 11];
  }

  // FIFO Check after reading ...
  // ----------------------------
  pfifo = read_output_acc_gyro(i2c_device1.c_str(),0x72);
  // Debug - disable in production ...
  // std::cout << "Rest in FIFO after reading:"<< pfifo << std::endl;
  
  // Stop fifo and reset ...
  // -----------------------
  buf[0] = 0x6A;
  buf[1] = 0x44;  // 0x04 - reset and stop FIFO !!! 
  
  i2c_write_buf(i2c_device1.c_str(), I2C_ACCGYRO, 2, buf); // Do nothing here ...
  
  accGyroCount++;
  
  std::cout <<"Read AccGyroData OK"<<std::endl;
  
}

// acquireSensorGridEyeData - public function ...
// ----------------------------------------------
void i2c_sensors_driver::acquireSensorGridEyeData(){
   
   int count = 0; 
   int16_t x = 0;
   uint8_t buf[CONST_GRID_EYE_SIZE];  // I2C Buffer - max size 128 ...
   
   for(int i=0;i<CONST_GRID_EYE_SIZE;i++){
       buf[i] = 0x00;
   }
   
   // Thermistor read ...
   // -------------------
   if(i2c_read_buf(i2c_device0.c_str(), I2C_GRIDEYE, 0x0E, 2, buf)!=STATUS_OK){
      gridEye.ok = false; gridEye.thermistor = 0.0;
      for(int i=0;i<CONST_GRID_EYE_DOUBLE_SIZE;i++){gridEye.data[i] = (double)0.0;} 
      return;  
   }
    
   x = (int16_t) buf[1];
   x |= (x & 0x08) ? 0xF8 : 0;
   x = (x << 8) | (int16_t) buf[0];				
   gridEye.ok = true;
   gridEye.thermistor = (double)0.0625*x; // Conversion ...    
   
   // Raw data readout ...
   // --------------------
   if(i2c_read_buf(i2c_device0.c_str(), I2C_GRIDEYE, 0x80, 128, buf)!=STATUS_OK){
      gridEye.ok = false; gridEye.thermistor = 0.0;
      for(int i=0;i<CONST_GRID_EYE_DOUBLE_SIZE;i++){gridEye.data[i] = (double)0.0;} 
      return;    
   }
   
   // Raw sensor data ...
   // ---------------------------
   memcpy(gridEye.data_raw,buf,128);
   
   gridEye.ok = true;
   
   for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
             x = (int16_t) buf[1 + 2 * j + 16 * i];
             x |= (x & 0x08) ? 0xF8 : 0;
             x = (x << 8) | (int16_t) buf[2 * j + 16 * i];
             gridEye.data[count++] = (double)0.25*x; // Conversion ...
        }
    }
    std::cout <<"Read GridEyeData OK"<<std::endl;
}

// readParamsFromFile - private function ...
// -----------------------------------------
int32_t i2c_sensors_driver::readParamsFromFile(std::string file_name_){
  
  std::ifstream fin(file_name_.c_str());
  
  if(fin.fail()){
     std::cout << "Could not open configuration file:" << file_name_.c_str() << std::endl;
     return STATUS_ERROR;
  }

  YAML::Node doc = YAML::LoadFile(file_name_);
  
  try{
    i2c_device0 =  doc["i2c_dev0"].as<std::string>();
    i2c_device1 =  doc["i2c_dev1"].as<std::string>();
  } catch (std::runtime_error) {}
  
  fin.close(); 
  
  return STATUS_OK;
}

// Write  buffer content to the I2C slave - private function ...
// -------------------------------------------------------------
int32_t i2c_sensors_driver::i2c_write_buf(const char *dev, uint8_t addr, uint8_t nData, uint8_t *data){
	
        int fd = -1;
        int error = STATUS_OK;
        
	if((fd = open(dev, O_RDWR)) < 0) {
            std::cout << "Failed to open the I2C bus - status(" << fd <<")"<< std::endl;
            return I2C_OPEN_FAILED;
	}

	if(stopAcquistionI2C) {  close(fd);  return STATUS_EXIT;}
	
	if(ioctl(fd, I2C_SLAVE, addr) < 0) {
           std::cout << "IOCTL failed - I2C_SLAVE"<< std::endl;
           return I2C_IOCTL_FAILED;
	}
	
	if(stopAcquistionI2C) {  close(fd);  return STATUS_EXIT;}

	if(write(fd, data, nData) != nData) {
           std::cout << "Failed to write to the I2C bus"<< std::endl;
           error = I2C_WRITE_FAILED;
    
    }
        
    if(stopAcquistionI2C) {  close(fd);  return STATUS_EXIT;}

	if(close(fd) != 0) {
           std::cout << "Failed to close the I2C bus"<< std::endl;
           error = I2C_CLOSE_FAILED;
	}

	return error;
}

// Read the word result from given register (H) and next register (L) ...
// ---------------------------------------------------------------------- 
int16_t i2c_sensors_driver::read_output_acc_gyro(const char *dev, uint8_t addr){
    
    uint8_t buf[2];
    
    if(i2c_read_buf(dev, I2C_ACCGYRO, addr, 2, buf)!=STATUS_OK){
       return STATUS_ERROR;
    }
    
    return(((int16_t) buf[0]) << 8 | buf[1]);
}

// Read  buffer content from  I2C slave - private function ...
// -----------------------------------------------------------
int32_t i2c_sensors_driver::i2c_read_buf(const char *dev, uint8_t addr, uint8_t subaddr, int nData, uint8_t *data){
        
        int fd = -1;
        int error = STATUS_OK;
        
	if((fd = open(dev, O_RDWR)) < 0) {
            std::cout << "Failed to open the I2C bus - status(" << fd <<")"<< std::endl;
            return I2C_OPEN_FAILED;
	}
	
	if(stopAcquistionI2C){ close(fd);  return STATUS_EXIT;}

	if(ioctl(fd, I2C_SLAVE, addr) < 0) {
           std::cout << "IOCTL failed - I2C_SLAVE"<< std::endl;
           return I2C_IOCTL_FAILED;
	}
	
	if(stopAcquistionI2C) { close(fd);  return STATUS_EXIT;}
    
    if(subaddr!=0x00){
       if(write(fd, &subaddr, 1) != 1) {
          std::cout << "Failed to write to the I2C bus"<< std::endl;
          error = I2C_WRITE_FAILED;
       } 
    }

	if(stopAcquistionI2C) {  close(fd);  return STATUS_EXIT;}

	
	int n_data_read = read(fd, data, nData);
	
    if(n_data_read != nData) {
           std::cout << "Failed to read from I2C bus - n_data: "<<nData<<", "<<n_data_read<<", "<<strerror(errno)<< std::endl;
           error = I2C_READ_FAILED;
	}
    
    
	/*
	if(read(fd, data, nData) != nData) {
       std::cout << "Failed to read from I2C bus - n_data: "<<nData<< std::endl;
       error = I2C_READ_FAILED;
	}
	*/
    
	if(stopAcquistionI2C) {  close(fd);  return STATUS_EXIT;}

	if(close(fd) != 0) {
           std::cout << "Failed to close the I2C bus"<< std::endl;
           error = I2C_CLOSE_FAILED;
	}

	return error;
}

// accGyroCallback function ...
// ---------------------------------------
void i2c_sensors_driver::accGyroCallback(){
  
  // Timestamp --> to String conversion ... 
  // -----------------------------------------------------
  boost::posix_time::ptime my_posix_time = boost::posix_time::second_clock::local_time();
  std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);   
  
  std::ofstream accGyroFile(accGyroFile_.c_str(),std::ios::out | std::ios::app);
  
  if(accGyroFile.is_open()){
     accGyroFile << "-----------------"<< std::endl;    
     accGyroFile << "[Cycle: "<< accGyroCount <<"]"<< " timestamp: "<< iso_time_string << std::endl; accGyroCount++;    
     
     // 1] Gyro ...
     // -----------   
     accGyroFile << "dataGyroX: ";   
     for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){
         if((CONST_ACC_GYRO_INT_SIZE-1) == i)  accGyroFile << gyro.x_data[i]; else accGyroFile << gyro.x_data[i]<< ",";              
     }
     accGyroFile << std::endl;
     
     accGyroFile << "dataGyroY: ";   
     for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){
         if((CONST_ACC_GYRO_INT_SIZE-1) == i) accGyroFile << gyro.y_data[i]; else accGyroFile << gyro.y_data[i]<< ","; 
     }
     accGyroFile << std::endl;
     
     accGyroFile << "dataGyroZ: ";   
     for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){
         if((CONST_ACC_GYRO_INT_SIZE-1) == i) accGyroFile << gyro.z_data[i]; else accGyroFile << gyro.z_data[i]<< ",";
     }
     accGyroFile << std::endl;
     
     // 2] Acc ...
     // ----------
     accGyroFile << "dataAccX: ";   
     for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){
         if((CONST_ACC_GYRO_INT_SIZE-1) == i) accGyroFile << acc.x_data[i]; else accGyroFile << acc.x_data[i]<< ","; 
     }
     accGyroFile << std::endl;
     
     accGyroFile << "dataAccY: ";   
     for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){
          if((CONST_ACC_GYRO_INT_SIZE-1) == i) accGyroFile << acc.y_data[i]; else accGyroFile << acc.y_data[i]<< ","; 
     }
     accGyroFile << std::endl;
     
     accGyroFile << "dataAccZ: ";   
     for(int i=0;i<CONST_ACC_GYRO_INT_SIZE;i++){
          if((CONST_ACC_GYRO_INT_SIZE-1) == i) accGyroFile << acc.z_data[i]; else accGyroFile << acc.z_data[i]<< ","; 
     }
     
     accGyroFile << std::endl;
     
     accGyroFile.close();
  }
}

// gridEyeCallback function ...
// --------------------------------------
void i2c_sensors_driver::gridEyeCallback(){
  
  // Timestamp --> to String conversion ... 
  // --------------------------------------
  boost::posix_time::ptime my_posix_time = boost::posix_time::second_clock::local_time();
  std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);   
  
  std::ofstream gridEyeFile(gridEyeFile_.c_str(),std::ios::out | std::ios::app);
  
  if(gridEyeFile.is_open()){
     gridEyeFile << "-----------------"<< std::endl;    
     gridEyeFile << "[Cycle: "<< gridEyeCount <<"]"<<" timestamp: "<< iso_time_string << std::endl; gridEyeCount++;    
     
     // GridEye ...
     // -----------   
     gridEyeFile << "GridEye thermistor: "<< gridEye.thermistor<< std::endl;   
     gridEyeFile << "GridEye data: ";
     
     for(int i=0;i<CONST_GRID_EYE_DOUBLE_SIZE;i++){
         if((CONST_GRID_EYE_DOUBLE_SIZE-1) == i) gridEyeFile << gridEye.data[i]; else gridEyeFile << gridEye.data[i]<< ","; 
     }
     
     gridEyeFile << std::endl;
     
    // GridEye / RAW ...
     // ----------------------   
     gridEyeFile << "GridEye RAW data: ";

     gridEyeFile << std::hex;
     for(int i=0;i<CONST_GRID_EYE_RAW_SIZE;i++){
          if((CONST_GRID_EYE_RAW_SIZE-1) == i)  gridEyeFile <<"0x"<<(static_cast<int>(gridEye.data_raw[i]) & 0xFF); else  gridEyeFile<<"0x"<<(static_cast<int>(gridEye.data_raw[i]) & 0xFF)<< ","; 
     }
     
     gridEyeFile << std::endl;
     gridEyeFile.close();
  }
}

// bmeCallback function ...
// ----------------------------------
void i2c_sensors_driver::bmeCallback(){
  
  // Timestamp --> to String conversion ... 
  // --------------------------------------
  boost::posix_time::ptime my_posix_time = boost::posix_time::second_clock::local_time();
  std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);   
  
  std::ofstream bmeFile(bmeFile_.c_str(),std::ios::out | std::ios::app);
  
  if(bmeFile.is_open()){
     bmeFile << "-----------------"<< std::endl;    
     bmeFile << "[Cycle: "<< bmeCount <<"]"<<" timestamp: "<< iso_time_string << std::endl; bmeCount++;    
     
     // ----------------
     // Bme 200 ...
     // ---------------   
     bmeFile << "Temperature [C]: "<<  bme.tempCelsius<< std::endl;   
     bmeFile << "Temperature [F]: "<<  bme.tempFahrenheit<< std::endl;   
     bmeFile << "Pressure [hPa]: "<<  bme.pressureData<< std::endl;   
     bmeFile << "Relative humidity  [RH]: "<<  bme.humidityData<< std::endl;  
     
     bmeFile << std::endl;
     bmeFile.close();
  }
}

// ------------------------------------------------------------------------------------------------------------------
// --  MLX90640 driver -- START section ... 
// ------------------------------------------------------------------------------------------------------------------

// initSensorMlx90640 - public function  ...
// -----------------------------------------
int32_t i2c_sensors_driver::initSensorMlx90640(){
     
    // Init Melexis 90640 buffers / eeprom ...
    // ---------------------------------------
    for(int i = 0;i<MLX90640_EEPROM_DATA_SIZE;i++){
        mlx32x24.eeMLX90640[i] = 0x0000;
    }
    
    for(int i = 0;i<MLX90640_FRAME_DATA_SIZE;i++){
        mlx32x24.frame[i] = 0x0000;
    }
    
    for(int i = 0;i<MLX90640_IR_DATA_SIZE;i++){
        mlx32x24.mlx90640To[i] = float(0.0);
    }
    
    mlx32x24.emissivity = 1;
    mlx32x24.eTa = 0;
    
    // Melexis 90640 configuration ...
    // -------------------------------
    if(MLX90640_SetDeviceMode(MLX90640_I2C_ADDR, 0)!=0) return STATUS_ERROR;
    if(MLX90640_SetSubPageRepeat(MLX90640_I2C_ADDR, 0)!=0) return STATUS_ERROR;
    if(MLX90640_SetRefreshRate(MLX90640_I2C_ADDR, 0b010)!=0) return STATUS_ERROR; 
    if(MLX90640_SetChessMode(MLX90640_I2C_ADDR)!=0) return STATUS_ERROR;
    
    if(MLX90640_DumpEE(MLX90640_I2C_ADDR, mlx32x24.eeMLX90640)!=0) return STATUS_ERROR; 
    if(MLX90640_ExtractParameters(mlx32x24.eeMLX90640, &mlx90640)!=0) return STATUS_ERROR;

    mlx32x24.refresh_rate_mlx = MLX90640_GetRefreshRate(MLX90640_I2C_ADDR);
    
    mlx90640StatusRdy = true;
    
    // Write configuration ...
    // -----------------------
    writeConfigurationMlx90640(mlx90640);
    
    std::cout << "MLX90640 succesfully initialized ..."<< std::endl;;
    
    return STATUS_OK; // configuration was properly set - OK ...
    
}

void i2c_sensors_driver::writeConfigurationMlx90640(paramsMLX90640 mlx90640){
  
  // Timestamp --> to String conversion ... 
  // --------------------------------------
  boost::posix_time::ptime my_posix_time = boost::posix_time::second_clock::local_time();
  std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);  
    
  // Timestamp --> to String conversion ... 
  // --------------------------------------
   std::ofstream mlx90640ConfigFile("/home/pi/sensors/i2c_sensors_store/MLX90640_config.txt",std::ios::out | std::ios::app);

   if(mlx90640ConfigFile.is_open()){
     mlx90640ConfigFile << "-----------------"<< std::endl;    
     mlx90640ConfigFile <<"Config of MLX90640 - timestamp: "<< iso_time_string << std::endl;
     mlx90640ConfigFile << "MLX90640 kVdd: "<< mlx90640.kVdd<< std::endl;
     mlx90640ConfigFile << "MLX90640 vdd25: "<< mlx90640.vdd25<< std::endl;
     mlx90640ConfigFile << "MLX90640 KvPTAT: "<< std::fixed <<mlx90640.KvPTAT<< std::endl;
     mlx90640ConfigFile << "MLX90640 KtPTAT: "<< std::fixed <<mlx90640.KtPTAT<< std::endl;
     mlx90640ConfigFile << "MLX90640 vPTAT25: "<< mlx90640.vPTAT25<< std::endl;
     mlx90640ConfigFile << "MLX90640 alphaPTAT: "<< std::fixed <<mlx90640.alphaPTAT<< std::endl;
     mlx90640ConfigFile << "MLX90640 gainEE: "<< mlx90640.gainEE<< std::endl;
     mlx90640ConfigFile << "MLX90640 tgc: "<< std::fixed <<mlx90640.tgc<< std::endl;
     mlx90640ConfigFile << "MLX90640 cpKv: "<< std::fixed <<mlx90640.cpKv<< std::endl;
     mlx90640ConfigFile << "MLX90640 cpKta: "<< std::fixed <<mlx90640.cpKta<< std::endl;
     mlx90640ConfigFile << "MLX90640 resolutionEE: "<< mlx90640.resolutionEE<< std::endl;
     mlx90640ConfigFile << "MLX90640 calibrationModeEE: "<< mlx90640.calibrationModeEE<< std::endl;
     mlx90640ConfigFile << "MLX90640 KsTa: "<< std::fixed <<mlx90640.KsTa<< std::endl;
     mlx90640ConfigFile << "MLX90640 ksTo: "<< std::endl;
     //mlx90640ConfigFile << std::endl;
     
     for(int i=0;i<4;i++){
         if((4-1) == i) mlx90640ConfigFile << std::fixed << mlx90640.ksTo[i]; else mlx90640ConfigFile << std::fixed << mlx90640.ksTo[i]<< ","; 
     }
     mlx90640ConfigFile << std::endl;
     
     mlx90640ConfigFile << "MLX90640 ct: "<< std::endl;
     //mlx90640ConfigFile << std::endl;
     
     for(int i=0;i<4;i++){
         if((4-1) == i) mlx90640ConfigFile << mlx90640.ct[i]; else mlx90640ConfigFile << mlx90640.ct[i]<< ","; 
     }
     mlx90640ConfigFile << std::endl;
     
     mlx90640ConfigFile << "MLX90640 alpha: "<< std::endl;
     //mlx90640ConfigFile << std::endl;
     
     for(int i=0;i<768;i++){
         if((768-1) == i) mlx90640ConfigFile << std::fixed << mlx90640.alpha[i]; else mlx90640ConfigFile << std::fixed << mlx90640.alpha[i]<< ","; 
         if(i!=0 && i%48==0) mlx90640ConfigFile << std::endl;
     }
     mlx90640ConfigFile << std::endl;
     
     mlx90640ConfigFile << "MLX90640 offset: ";
     //mlx90640ConfigFile << std::endl;
     
     for(int i=0;i<768;i++){
         if((768-1) == i) mlx90640ConfigFile << mlx90640.offset[i]; else mlx90640ConfigFile <<  mlx90640.offset[i]<< ","; 
         if(i!=0 && i%48==0) mlx90640ConfigFile << std::endl;
     }
     mlx90640ConfigFile << std::endl;   
        
     mlx90640ConfigFile << "MLX90640 kta: ";
     //mlx90640ConfigFile << std::endl;
     
     for(int i=0;i<768;i++){
         if((768-1) == i) mlx90640ConfigFile << std::fixed << mlx90640.kta[i]; else mlx90640ConfigFile << std::fixed << mlx90640.kta[i]<< ","; 
         if(i!=0 && i%48==0) mlx90640ConfigFile << std::endl;
     }
     mlx90640ConfigFile << std::endl;
     
     mlx90640ConfigFile << "MLX90640 kv: ";
     //mlx90640ConfigFile << std::endl;
     
     for(int i=0;i<768;i++){
         if((768-1) == i) mlx90640ConfigFile << std::fixed << mlx90640.kv[i]; else mlx90640ConfigFile << std::fixed << mlx90640.kv[i]<< ","; 
         if(i!=0 && i%48==0) mlx90640ConfigFile << std::endl;
     }
     mlx90640ConfigFile << std::endl;
     
     mlx90640ConfigFile << "MLX90640 cpAlpha: "<< std::endl;
     //mlx90640ConfigFile << std::endl;
     
     for(int i=0;i<2;i++){
         if((2-1) == i) mlx90640ConfigFile << std::fixed << mlx90640.cpAlpha[i]; else mlx90640ConfigFile << std::fixed << mlx90640.cpAlpha[i]<< ","; 
     }
     mlx90640ConfigFile << std::endl;
     
     mlx90640ConfigFile << "MLX90640 cpOffset: "<< std::endl;
     //mlx90640ConfigFile << std::endl;
     
     for(int i=0;i<2;i++){
         if((2-1) == i) mlx90640ConfigFile << mlx90640.cpOffset[i]; else mlx90640ConfigFile << mlx90640.cpOffset[i]<< ","; 
     }
     mlx90640ConfigFile << std::endl;
     
     mlx90640ConfigFile << "MLX90640 ilChessC: "<< std::endl;
     //mlx90640ConfigFile << std::endl;
     
     for(int i=0;i<3;i++){
         if((3-1) == i) mlx90640ConfigFile << std::fixed << mlx90640.ilChessC[i]; else mlx90640ConfigFile << std::fixed << mlx90640.ilChessC[i]<< ","; 
     }
     mlx90640ConfigFile << std::endl;
     
     
     mlx90640ConfigFile << "MLX90640 brokenPixels: "<< std::endl;
     // mlx90640ConfigFile << std::endl;
     for(int i=0;i<5;i++){
         if((5-1) == i) mlx90640ConfigFile << mlx90640.brokenPixels[i]; else mlx90640ConfigFile << mlx90640.brokenPixels[i]<< ","; 
     }
     mlx90640ConfigFile << std::endl;
     
     mlx90640ConfigFile << "MLX90640 outlierPixels: "<< std::endl;
     // mlx90640ConfigFile << std::endl;
     for(int i=0;i<5;i++){
         if((5-1) == i) mlx90640ConfigFile << mlx90640.outlierPixels[i]; else mlx90640ConfigFile << mlx90640.outlierPixels[i]<< ","; 
     }
     
     mlx90640ConfigFile << std::endl;
     
     mlx90640ConfigFile.close();
   }
     
}

// acquireSensorMlx90640Data - public function ...
// -----------------------------------------------
void i2c_sensors_driver::acquireSensorMlx90640Data(){

    MLX90640_GetFrameData(MLX90640_I2C_ADDR, mlx32x24.frame);
    // MLX90640_InterpolateOutliers(frame, eeMLX90640);
    mlx32x24.eTa = MLX90640_GetTa(mlx32x24.frame, &mlx90640);
    MLX90640_CalculateTo(mlx32x24.frame, &mlx90640, mlx32x24.emissivity, mlx32x24.eTa, mlx32x24.mlx90640To);

    MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, mlx32x24.mlx90640To, 1, &mlx90640);
    MLX90640_BadPixelsCorrection((&mlx90640)->outlierPixels, mlx32x24.mlx90640To, 1, &mlx90640);
    
    std::cout << "MLX90640 read OK ..."<< std::endl;
    
    mlx32x24.ok = true;
}

// Mlx90621Callback function ...
// --------------------------------------
void i2c_sensors_driver::Mlx90640Callback(){
  
  // Timestamp --> to String conversion ... 
  // --------------------------------------
  boost::posix_time::ptime my_posix_time = boost::posix_time::second_clock::local_time();
  std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);   
  
  std::ofstream mlx90640File(mlx90640File_.c_str(),std::ios::out | std::ios::app);
  
  if(mlx90640File.is_open()){
     mlx90640File << "-----------------"<< std::endl;    
     mlx90640File << "[Cycle: "<< mlx90640Count <<"]"<<" timestamp: "<< iso_time_string << std::endl;    
     
     // MLX90640 ...
     // ------------   
     mlx90640File << "MLX90640 resolution: "<< mlx32x24.refresh_rate_mlx<< std::endl;   
     mlx90640File << "MLX90640 emissivity: "<< mlx32x24.emissivity<< std::endl;  
     mlx90640File << "MLX90640 eTa: "<< mlx32x24.eTa<< std::endl;  
     
     mlx90640File << "MLX90640 data [deg C]: ";
     
     for(int i=0;i<MLX90640_IR_DATA_SIZE;i++){
         if((MLX90640_IR_DATA_SIZE-1) == i) mlx90640File << std::fixed << mlx32x24.mlx90640To[i]; else mlx90640File << std::fixed << mlx32x24.mlx90640To[i]<< ","; 
         if(i!=0 && i%48==0) mlx90640File << std::endl;
     }
     
     mlx90640File << std::endl;
     
     mlx90640File.close();
  }
  
  std::ofstream mlx90640FileRaw(mlx90640RawFile_.c_str(),std::ios::out | std::ios::app);
  
  if(mlx90640FileRaw.is_open()){
     mlx90640FileRaw << "-----------------"<< std::endl;    
     mlx90640FileRaw << "[Cycle: "<< mlx90640Count <<"]"<<" timestamp: "<< iso_time_string << std::endl;  
     
     // MLX90640 ...
     // ------------   
     mlx90640FileRaw << "MLX90640 resolution: "<< mlx32x24.refresh_rate_mlx<< std::endl;   
     mlx90640FileRaw << "MLX90640 emissivity: "<< mlx32x24.emissivity<< std::endl;  
     mlx90640FileRaw << "MLX90640 eTa: "<< mlx32x24.eTa<< std::endl;  
 
     // MLX90640 / RAW ...
     // ------------------   
     mlx90640FileRaw << "MLX90640 RAW data: ";

     // mlx90640File << std::hex;
     for(int i=0;i<MLX90640_FRAME_DATA_SIZE;i++){
          if((MLX90640_FRAME_DATA_SIZE-1) == i)  mlx90640FileRaw << mlx32x24.frame[i]; else mlx90640FileRaw << mlx32x24.frame[i]<< ",";  
          if(i!=0 && i%52==0) mlx90640FileRaw << std::endl;
     }
     
     mlx90640FileRaw << std::endl;
     mlx90640FileRaw.close();
  }
  
  mlx90640Count++; 
}

// ------------------------------------------------------------------------------------------------------------------
// --  MLX90621 driver -- START section ... 
// ------------------------------------------------------------------------------------------------------------------

// initSensorMlx90621 - public function  ...
// -----------------------------------------
int32_t i2c_sensors_driver::initSensorMlx90621(){
    
    // Init Melexis 90621 buffers ...
    // ------------------------------
    for(int i = 0;i<32;i++){
        mlx16x4.melexis_cmd[i] = 0x00;
    }
    
    for(int i = 0;i<MLX90621_EEPROM_SIZE;i++){
        mlx16x4.melexis_eeprom[i] = 0x00;
    }

    for(int i = 0;i<MLX90621_IR_DATA_SIZE;i++){
        mlx16x4.dataIrMlx[i] = 0;
        mlx16x4.tempMlx[i] = float(0.0);
    }
    
    // Set / init variables ...
    // ------------------------
    mlx16x4.resolutionMlx = 0;
    mlx16x4.ptatMlx = 0;
    mlx16x4.k_t1_scale = 0;
    mlx16x4.k_t2_scale = 0;
    mlx16x4.cpixMlx = 0;
    
    mlx16x4.refresh_rate_mlx = Hz_LSB_1;
    
    // Sleep for 5000us ...
    // --------------------
    usleep(5000);
    
    if(readEepromMlx90621()!=STATUS_OK) return STATUS_ERROR;
    if(writeTrimmingMlx90621()!=STATUS_OK) return STATUS_ERROR;
    if(setConfigurationMlx906201()!=STATUS_OK) return STATUS_ERROR;
    if(readResMlx906201()!=STATUS_OK) return STATUS_ERROR;
    if(readPtatMlx906201()!=STATUS_OK) return STATUS_ERROR;
    
    // Calculate TA  ...
    // -----------------
    calculateTaMlx90621();
    
    mlx90621StatusRdy = true;
    
    std::cout << "MLX90621 succesfully initialized ..."<< std::endl;;
    
    return STATUS_OK; // configuration was properly set - OK ...
}

// acquireSensorMlx90621Data - public function ...
// -----------------------------------------------
void i2c_sensors_driver::acquireSensorMlx90621Data(){

    // Check for possible resets ...
    // -----------------------------
    if(mlx90621Count % 16 == 0){
       if(checkConfigMlx906201()){
          readEepromMlx90621();
          writeTrimmingMlx90621();
          setConfigurationMlx906201();
          readResMlx906201();
       }
    }
    
    readPtatMlx906201();
    calculateTaMlx90621();
    readCpixMlx906201();
    readIrMlx906201();
    calculateToMlx90621(); 
    
    mlx16x4.ok = true;
}

int32_t i2c_sensors_driver::mlx90640_I2CRead(const char *dev,uint8_t slaveAddr,int cmdLength,int dataLength,uint8_t *data){
    
    int fd = -1;
    int error = STATUS_OK;
    
    struct i2c_msg i2c_messages[2];
    struct i2c_rdwr_ioctl_data i2c_messageset[1];
        
	if((fd = open(dev, O_RDWR)) < 0) {
        std::cout << "Failed to open the I2C bus - status(" << fd <<")"<< std::endl;
        return I2C_OPEN_FAILED;
	}

    i2c_messages[0].addr = slaveAddr;
    i2c_messages[0].flags = 0;
    i2c_messages[0].len = cmdLength;
    i2c_messages[0].buf = (I2C_MSG_FMT*)mlx16x4.melexis_cmd;

    i2c_messages[1].addr = slaveAddr;
    i2c_messages[1].flags = I2C_M_RD | I2C_M_NOSTART;
    i2c_messages[1].len = dataLength;
    i2c_messages[1].buf = (I2C_MSG_FMT*)data;

    i2c_messageset[0].msgs = i2c_messages;
    i2c_messageset[0].nmsgs = 2;


    if(ioctl(fd, I2C_RDWR, &i2c_messageset) < 0) {
        std::cout << "IOCTL failed - I2C_SLAVE"<< std::endl;
        return I2C_IOCTL_FAILED;
    }

    if(close(fd) != 0) {
       std::cout << "Failed to close the I2C bus"<< std::endl;
      error = I2C_CLOSE_FAILED;
	}
    
    return error;
} 

// Mlx90621Callback function ...
// --------------------------------------
void i2c_sensors_driver::Mlx90621Callback(){
  
  // Timestamp --> to String conversion ... 
  // --------------------------------------
  boost::posix_time::ptime my_posix_time = boost::posix_time::second_clock::local_time();
  std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time);   
  
  std::ofstream mlx90621File(mlx90621File_.c_str(),std::ios::out | std::ios::app);
  
  if(mlx90621File.is_open()){
     mlx90621File << "-----------------"<< std::endl;    
     mlx90621File << "[Cycle: "<< mlx90621Count <<"]"<<" timestamp: "<< iso_time_string << std::endl; mlx90621Count++;    
     
     // MLX90621 ...
     // ------------   
     mlx90621File << "MLX90621 resolution: "<< mlx16x4.resolutionMlx<< std::endl;   
     mlx90621File << "MLX90621 PtaT: "<< mlx16x4.ptatMlx<< std::endl;  
     mlx90621File << "MLX90621 k_t1_scale: "<< mlx16x4.k_t1_scale<< std::endl;  
     mlx90621File << "MLX90621 k_t2_scale: "<< mlx16x4.k_t2_scale<< std::endl;
     mlx90621File << "MLX90621 CpiX: "<< mlx16x4.cpixMlx<< std::endl;
     
     mlx90621File << "MLX90621 data [deg C]: ";
     
     for(int i=0;i<MLX90621_IR_DATA_SIZE;i++){
         if((MLX90621_IR_DATA_SIZE-1) == i) mlx90621File << std::fixed << mlx16x4.tempMlx[i]; else mlx90621File << std::fixed << mlx16x4.tempMlx[i]<< ","; 
     }
     
     mlx90621File << std::endl;
     
     // MLX90621 / RAW ...
     // ----------------------   
     mlx90621File << "MLX90621 RAW data: ";

     // mlx90621File << std::hex;
     for(int i=0;i<MLX90621_IR_DATA_SIZE;i++){
          if((MLX90621_IR_DATA_SIZE-1) == i)  mlx90621File << mlx16x4.dataIrMlx[i]; else mlx90621File << mlx16x4.dataIrMlx[i]<< ",";  
     }
     
     mlx90621File << std::endl;
     mlx90621File.close();
  }
}

// readEepromMelexis90621 - read Melexis 90621 eeprom - private function ...
// -------------------------------------------------------------------------
int i2c_sensors_driver::readEepromMlx90621(){
    
    mlx16x4.melexis_cmd[0] = 0x00; // Initial cmd - 0x00 ...
    
    /*
    if(i2c_write_buf(i2c_device0.c_str(), I2C_MEL90621_EEPROM, 1, mlx16x4.melexis_cmd)!=STATUS_OK){
       return STATUS_ERROR;
    }
    
    if (i2c_read_buf(i2c_device0.c_str(), I2C_MEL90621_EEPROM, 0x00, MLX90621_EEPROM_SIZE, mlx16x4.melexis_eeprom)!=STATUS_OK){
        return STATUS_ERROR;
    }
    */
    
    // Read 256 bytes of eeprom data / no subaddress - using 0x00 instead ...
    // ---------------------------------------------------------------------- 
    if(mlx90640_I2CRead(i2c_device0.c_str(),I2C_MEL90621_EEPROM,1,MLX90621_EEPROM_SIZE,mlx16x4.melexis_eeprom)!=STATUS_OK){
       return STATUS_ERROR;
    }
    
    std::cout << "MLX90621 eeprom OK ..."<< std::endl;
    
    // Return status OK ...
    // --------------------
    return STATUS_OK;
}

// writeTrimmingMelexis90621 - write Melexis 90621 trimming value - private function ...
// -------------------------------------------------------------------------------------
int i2c_sensors_driver::writeTrimmingMlx90621(){
  
    mlx16x4.melexis_cmd[0] = 0x04; // Initial cmd - 0x04 ...
    mlx16x4.melexis_cmd[1] = (uint8_t)mlx16x4.melexis_eeprom[OSC_TRIM_VALUE] - 0xAA;
    mlx16x4.melexis_cmd[2] = mlx16x4.melexis_eeprom[OSC_TRIM_VALUE];
    mlx16x4.melexis_cmd[3] = 0x56; // MSByte – 0xAA = 0x00 – 0xAA = 0x56
    mlx16x4.melexis_cmd[4] = 0x00; // 0x00    
    
    if(i2c_write_buf(i2c_device0.c_str(), I2C_MEL90621_MAIN, 5, mlx16x4.melexis_cmd)!=STATUS_OK){
       return STATUS_ERROR;
    }
    
    std::cout << "MLX90621 write trimming OK ..."<< std::endl;;
    
    // Return status OK ...
    // --------------------
    return STATUS_OK;
}

// setConfigurationMlx906201 - set Melexis 90621 configuration - private function ...
// ----------------------------------------------------------------------------------
int i2c_sensors_driver::setConfigurationMlx906201(){
  
    uint8_t defaultConfig_H = 0x44; // 0x54 ...
  
    mlx16x4.melexis_cmd[0] = 0x03; // Initial cmd - 0x03 ...
    mlx16x4.melexis_cmd[1] = (uint8_t)mlx16x4.refresh_rate_mlx - (uint8_t)0x55;
    mlx16x4.melexis_cmd[2] = mlx16x4.refresh_rate_mlx;
    mlx16x4.melexis_cmd[3] = (uint8_t)defaultConfig_H - (uint8_t)0x55; 
    mlx16x4.melexis_cmd[4] = defaultConfig_H;    
    
    if(i2c_write_buf(i2c_device0.c_str(), I2C_MEL90621_MAIN, 5, mlx16x4.melexis_cmd)!=STATUS_OK){
       return STATUS_ERROR;
    }
    
    std::cout << "MLX90621 set configuration OK ..."<< std::endl;;
    
    // Return status OK ...
    // --------------------
    return STATUS_OK;
  
}

// readConfigMlx906201 - read Melexis 90621 configuration - private function ...
// -----------------------------------------------------------------------------
int i2c_sensors_driver::readConfigMlx906201(uint16_t *configuration){
  
  uint8_t buff_tmp[2];
  
  // Set default values ...
  // ----------------------
  buff_tmp[0]=0x00; buff_tmp[1]=0x00;
  
  mlx16x4.melexis_cmd[0] = 0x02; // Initial cmd - 0x02 ...
  mlx16x4.melexis_cmd[1] = 0x92;
  mlx16x4.melexis_cmd[2] = 0x00;
  mlx16x4.melexis_cmd[3] = 0x01; 
    

  /*
  if(i2c_write_buf(i2c_device0.c_str(), I2C_MEL90621_MAIN, 4, mlx16x4.melexis_cmd)!=STATUS_OK){
     return STATUS_ERROR;
  }

  if(i2c_read_buf(i2c_device0.c_str(), I2C_MEL90621_MAIN, 0x00,2 , buff_tmp)!=STATUS_OK){
     return STATUS_ERROR;
  }
  */

  if(mlx90640_I2CRead(i2c_device0.c_str(),I2C_MEL90621_MAIN,4,2,buff_tmp)!=STATUS_OK){
       return STATUS_ERROR;
  }
  
  // Set configuration ...
  // ---------------------
  *configuration = ((uint16_t) (buff_tmp[1] << 8) | buff_tmp[0]);
  
  std::cout << "MLX90621 read configuration OK ..."<< std::endl;;
  
  // Return status OK ...
  // --------------------
  return STATUS_OK;
}

// readResMlx906201 - read Melexis 90621 resolution - private function ...
// -----------------------------------------------------------------------
int i2c_sensors_driver::readResMlx906201(){
  
  uint16_t configuration = 0x0000;
  
  // Read the resolution from the config register ...
  // ------------------------------------------------
  if(readConfigMlx906201(&configuration)!=STATUS_OK){
     return STATUS_ERROR;
  }
  
  // Set resolution to global variable ...
  // -------------------------------------
  mlx16x4.resolutionMlx = (configuration & 0x30) >> 4; 
  
  // Return status OK ...
  // --------------------
  return STATUS_OK;
}

// checkConfigMlx906201 - check Melexis 90621 configuration - private function ...
// -------------------------------------------------------------------------------
bool i2c_sensors_driver::checkConfigMlx906201(){
  
   /*
   Poll the MLX90621 for its current status. 
   Returns true if the POR/Brown out bit is set
   */
   
   uint16_t configuration = 0x0000;
  
   // Read the resolution from the config register ...
   // ------------------------------------------------
   if(readConfigMlx906201(&configuration)!=STATUS_OK){
      return false;
   }
  
   mlx16x4.configurationMlx = configuration;
  
   // Return result ...
   // -----------------
   return !((configuration & 0x0400) >> 10);    
}

// readPtatMlx906201 - read Melexis 90621 ambient temperature PTAT - private function ...
// -------------------------------------------------------------------------------------
int i2c_sensors_driver::readPtatMlx906201(){
  
  /*
  Absolute ambient temperature data of the device 
  can be read by using the following function. 
  */
  
  uint8_t buff_tmp[2];
  
  // Set default values ...
  // ----------------------
  buff_tmp[0]=0x00; buff_tmp[1]=0x00;
  
  mlx16x4.melexis_cmd[0] = 0x02; // Initial cmd - 0x02 ...
  mlx16x4.melexis_cmd[1] = 0x40;
  mlx16x4.melexis_cmd[2] = 0x00;
  mlx16x4.melexis_cmd[3] = 0x01; 
    
  /*
  if(i2c_write_buf(i2c_device0.c_str(), I2C_MEL90621_MAIN, 4, mlx16x4.melexis_cmd)!=STATUS_OK){
     return STATUS_ERROR;
  }
    
  if(i2c_read_buf(i2c_device0.c_str(), I2C_MEL90621_MAIN, 0x00, 2, buff_tmp)!=STATUS_OK){
     return STATUS_ERROR;
  }
  */
  
  if(mlx90640_I2CRead(i2c_device0.c_str(),I2C_MEL90621_MAIN,4,2,buff_tmp)!=STATUS_OK){
     return STATUS_ERROR;
  }
  
  mlx16x4.ptatMlx = ((int16_t) (buff_tmp[1] << 8) | buff_tmp[0]);
  
  std::cout << "MLX90621 PtaT OK ..."<< std::endl;
  
  // Return status OK ...
  // --------------------
  return STATUS_OK;
}

// readCpixMlx906201 - read Melexis 90621 compensation pixel data  - private function ...
// --------------------------------------------------------------------------------------
int i2c_sensors_driver::readCpixMlx906201(){
  
  /*
  Compensation pixel data of the device can 
  be read by using the following function.
  */
  
  uint8_t buff_tmp[2];
  
  // Set default values ...
  // ----------------------
  buff_tmp[0]=0x00; buff_tmp[1]=0x00;
  
  mlx16x4.melexis_cmd[0] = 0x02; // Initial cmd - 0x02 ...
  mlx16x4.melexis_cmd[1] = 0x41;
  mlx16x4.melexis_cmd[2] = 0x00;
  mlx16x4.melexis_cmd[3] = 0x01; 

  /*
  if(i2c_write_buf(i2c_device0.c_str(), I2C_MEL90621_MAIN, 4, mlx16x4.melexis_cmd)!=STATUS_OK){
     return STATUS_ERROR;
  }
    
  if(i2c_read_buf(i2c_device0.c_str(), I2C_MEL90621_MAIN, 0x00, 2, buff_tmp)!=STATUS_OK){
     return STATUS_ERROR;
  }
  */
  
  if(mlx90640_I2CRead(i2c_device0.c_str(),I2C_MEL90621_MAIN,4,2,buff_tmp)!=STATUS_OK){
     return STATUS_ERROR;
  }
  
  mlx16x4.cpixMlx = ((int16_t) (buff_tmp[1] << 8) | buff_tmp[0]);
  if (mlx16x4.cpixMlx >= 32768)  mlx16x4.cpixMlx -= 65536;
  
  std::cout << "MLX90621 Cpix OK ..."<< std::endl;
  
  // Return status OK ...
  // --------------------
  return STATUS_OK;
}

// readIrMlx906201 - read Melexis 90621 IR data  - private function ...
// --------------------------------------------------------------------
int i2c_sensors_driver::readIrMlx906201(){
  
  /*
  IR data of the device that it 
  is read as a whole.
  */
  
  int c=0;
  uint8_t buff_tmp[128];
  
  // Set default values ...
  // ----------------------
  buff_tmp[0]=0x00; buff_tmp[1]=0x00;
  
  mlx16x4.melexis_cmd[0] = 0x02;    // Initial cmd - 0x02 ...
  mlx16x4.melexis_cmd[1] = 0x00;
  mlx16x4.melexis_cmd[2] = 0x01;
  mlx16x4.melexis_cmd[3] = 0x40; 

  /*
  if(i2c_write_buf(i2c_device0.c_str(), I2C_MEL90621_MAIN, 4, mlx16x4.melexis_cmd)!=STATUS_OK){
     return STATUS_ERROR;
  }
    
  if(i2c_read_buf(i2c_device0.c_str(), I2C_MEL90621_MAIN, 0x00, MLX90621_IR_DATA_SIZE*2, buff_tmp)!=STATUS_OK){
     return STATUS_ERROR;
  }
  */
  
  if(mlx90640_I2CRead(i2c_device0.c_str(),I2C_MEL90621_MAIN,4,MLX90621_IR_DATA_SIZE*2,buff_tmp)!=STATUS_OK){
     return STATUS_ERROR;
  }
  
  for(int j=0;j<64;j++){
      // Each pixel value takes up two bytes thus NUM_PIXELS * 2 ...
      // -----------------------------------------------------------
      mlx16x4.dataIrMlx[j] = ((int16_t) (buff_tmp[c+1] << 8) | buff_tmp[c]);
      c+=2;
  }
  
  std::cout << "MLX90621 IR OK ..."<< std::endl;
  
  // Return status OK ...
  // --------------------
  return STATUS_OK;
}

// calculateTaMlx90621 - calculate TA using eeprom values - private function ...
// -----------------------------------------------------------------------------
void i2c_sensors_driver::calculateTaMlx90621(){
  
  // See Datasheet MLX90621 ...
  // --------------------------
  float v_th = 0, k_t1 = 0, k_t2 = 0;

  mlx16x4.k_t1_scale = (int16_t) (mlx16x4.melexis_eeprom[KT_SCALE] & 0xF0) >> 4;    // KT_SCALE=0xD2[7:4]
  mlx16x4.k_t2_scale = (int16_t) (mlx16x4.melexis_eeprom[KT_SCALE] & 0x0F) + 10;    // KT_SCALE=0xD2[3:0]+10
  
  v_th = (float) 256 * mlx16x4.melexis_eeprom[VTH_H] + mlx16x4.melexis_eeprom[VTH_L];
  
  if (v_th >= 32768.0)   v_th -= 65536.0;
  
  v_th = v_th / pow(2, (3 - mlx16x4.resolutionMlx));
  k_t1 = (float) 256 * mlx16x4.melexis_eeprom[KT1_H] + mlx16x4.melexis_eeprom[KT1_L];
  
  if (k_t1 >= 32768.0)   k_t1 -= 65536.0;
  
  k_t1 /= (pow(2, mlx16x4.k_t1_scale) * pow(2, (3 - mlx16x4.resolutionMlx)));
  k_t2 = (float) 256 * mlx16x4.melexis_eeprom[KT2_H] + mlx16x4.melexis_eeprom[KT2_L];
  
  if (k_t2 >= 32768.0)   k_t2 -= 65536.0;
  
  k_t2 /= (pow(2, mlx16x4.k_t2_scale) * pow(2, (3 - mlx16x4.resolutionMlx)));      // 0.000768
  
  mlx16x4.tambientMlx = ((-k_t1 + sqrt((k_t1*k_t1) - (4 * k_t2 * (v_th - (float) mlx16x4.ptatMlx)))) / (2 * k_t2)) + 25.0;
}

// calculateToMlx90621 - calculate TO using measurement values - private function ...
// ----------------------------------------------------------------------------------
void i2c_sensors_driver::calculateToMlx90621(){                                              
  
  // See Datasheet MLX90621 ...
  // --------------------------
  float a_ij[64], b_ij[64], alpha_ij[64];
  float emissivity, tgc, alpha_cp, a_cp, b_cp;
  int16_t a_common, a_i_scale, b_i_scale;

  emissivity = (256 * mlx16x4.melexis_eeprom[CAL_EMIS_H] + mlx16x4.melexis_eeprom[CAL_EMIS_L]) / 32768.0;
  a_common = (int16_t) 256 * mlx16x4.melexis_eeprom[CAL_ACOMMON_H] + mlx16x4.melexis_eeprom[CAL_ACOMMON_L];
  
  if (a_common >= 32768) a_common -= 65536;
  
  alpha_cp = (256 * mlx16x4.melexis_eeprom[CAL_alphaCP_H] + mlx16x4.melexis_eeprom[CAL_alphaCP_L]) / (pow(2, CAL_A0_SCALE) * pow(2, (3 - mlx16x4.resolutionMlx)));
  a_i_scale = (int16_t) (mlx16x4.melexis_eeprom[CAL_AI_SCALE] & 0xF0) >> 4;
  b_i_scale = (int16_t) mlx16x4.melexis_eeprom[CAL_BI_SCALE] & 0x0F;
  a_cp = (float) 256 * mlx16x4.melexis_eeprom[CAL_ACP_H] + mlx16x4.melexis_eeprom[CAL_ACP_L];
  
  if (a_cp >= 32768.0) a_cp -= 65536.0;
  
  a_cp /= pow(2, (3 - mlx16x4.resolutionMlx));
  b_cp = (float) mlx16x4.melexis_eeprom[CAL_BCP];
  
  if (b_cp > 127.0)     b_cp -= 256.0;
  
  b_cp /= (pow(2, b_i_scale) * pow(2, (3 - mlx16x4.resolutionMlx)));
  tgc = (float) mlx16x4.melexis_eeprom[CAL_TGC];
  
  if (tgc > 127.0)  tgc -= 256.0;
  
  tgc /= 32.0;
  
  float v_cp_off_comp = (float) mlx16x4.cpixMlx - (a_cp + b_cp * (mlx16x4.tambientMlx - 25.0));
  float v_ir_off_comp, v_ir_tgc_comp, v_ir_norm, v_ir_comp;
  
  mlx16x4.tminMlx = 250;
  mlx16x4.tmaxMlx = -250;
  
  for (int i = 0; i < 64; i++){
    
      a_ij[i] = ((float) a_common + mlx16x4.melexis_eeprom[i] * pow(2, a_i_scale)) / pow(2, (3 - mlx16x4.resolutionMlx));
      b_ij[i] = mlx16x4.melexis_eeprom[0x40 + i];
      
      if (b_ij[i] > 127)        b_ij[i] -= 256;
      
      b_ij[i] /= (pow(2, b_i_scale) * pow(2, (3 - mlx16x4.resolutionMlx)));
      v_ir_off_comp = mlx16x4.dataIrMlx[i] - (a_ij[i] + b_ij[i] * (mlx16x4.tambientMlx - 25.0));
      v_ir_tgc_comp = v_ir_off_comp - tgc * v_cp_off_comp;
      
      alpha_ij[i] = ((256 * mlx16x4.melexis_eeprom[CAL_A0_H] + mlx16x4.melexis_eeprom[CAL_A0_L]) / pow(2, mlx16x4.melexis_eeprom[CAL_A0_SCALE]));
      alpha_ij[i] += (mlx16x4.melexis_eeprom[0x80 + i] / pow(2, mlx16x4.melexis_eeprom[CAL_DELTA_A_SCALE]));
      alpha_ij[i] /= pow(2, 3 - mlx16x4.resolutionMlx);
      
      v_ir_norm = v_ir_tgc_comp / (alpha_ij[i] - tgc * alpha_cp);
      v_ir_comp = v_ir_norm / emissivity;
      mlx16x4.tempMlx[i] = sqrt(sqrt((v_ir_comp + pow((mlx16x4.tambientMlx + 273.15), 4)))) - 273.15;
      
      if (mlx16x4.tempMlx[i] < mlx16x4.tminMlx) mlx16x4.tminMlx = mlx16x4.tempMlx[i];
      if (mlx16x4.tempMlx[i] > mlx16x4.tmaxMlx) mlx16x4.tmaxMlx = mlx16x4.tempMlx[i];
    }
}

// ---------------------------------------------------------------------------------------------------------------
// --  MLX90621 driver -- STOP section ... 
// ---------------------------------------------------------------------------------------------------------------
