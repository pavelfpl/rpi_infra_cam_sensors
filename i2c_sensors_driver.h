/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: i2c_sensors_driver.h
*********************************************************************************/

#ifndef I2C_SENSORS_DRIVER_H
#define I2C_SENSORS_DRIVER_H

#define CONST_GRID_EYE_DOUBLE_SIZE 64
#define CONST_GRID_EYE_RAW_SIZE 128
#define CONST_ACC_GYRO_INT_SIZE 10

// -- MLX90621 drivers -- 
// refreshRate Possible values: Hz_LSB_0=0.5Hz, Hz_LSB_1=1Hz, Hz_LSB_2=2Hz, Hz_LSB_4=4Hz, Hz_LSB_8=8Hz, Hz_LSB_16=16Hz, Hz_LSB_32=32Hz.
// ------------------------------------------------------------------------------------------------------------------------------------
#define Hz_LSB_0  0x3F      //  0b00111111
#define Hz_LSB_1  0x3E      //  0b00111110
#define Hz_LSB_2  0x3D      //  0b00111101
#define Hz_LSB_4  0x3C      //  0b00111100
#define Hz_LSB_8  0x3B      //  0b00111011
#define Hz_LSB_16 0x3A 	    //  0b00111010;
#define Hz_LSB_32 0x39;	    //  0b00111001 
#define Hz_LSB_default 0x3E //  0b00111110;

#define CAL_ACOMMON_L 0xD0
#define CAL_ACOMMON_H 0xD1
#define CAL_ACP_L 0xD3
#define CAL_ACP_H 0xD4
#define CAL_BCP 0xD5
#define CAL_alphaCP_L 0xD6
#define CAL_alphaCP_H 0xD7
#define CAL_TGC 0xD8
#define CAL_AI_SCALE 0xD9
#define CAL_BI_SCALE 0xD9

#define VTH_L 0xDA
#define VTH_H 0xDB
#define KT1_L 0xDC
#define KT1_H 0xDD
#define KT2_L 0xDE
#define KT2_H 0xDF
#define KT_SCALE 0xD2

// Common sensitivity coefficients
// -------------------------------
#define CAL_A0_L 0xE0
#define CAL_A0_H 0xE1
#define CAL_A0_SCALE 0xE2
#define CAL_DELTA_A_SCALE 0xE3
#define CAL_EMIS_L 0xE4
#define CAL_EMIS_H 0xE5

// Config register = 0xF5-F6
// -------------------------
#define OSC_TRIM_VALUE 0xF7
#define CHECK_COUNTER_VALUE_POR 16

#define I2C_MEL90621_EEPROM 0x50 
#define I2C_MEL90621_MAIN 0x60

#define MLX90621_EEPROM_SIZE 256
#define MLX90621_IR_DATA_SIZE 64   // 16x4 - uint16_t

#define MLX90640_I2C_ADDR 0x33
#define MLX90640_EEPROM_DATA_SIZE 832
#define MLX90640_FRAME_DATA_SIZE 834
#define MLX90640_IR_DATA_SIZE 768

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "MLX90640_API.h"

// Class AccGyroCoordinates / helper class ...
// -------------------------------------------
class AccGyroCoordinates{
public:
    bool ok;
    int16_t x_data[CONST_ACC_GYRO_INT_SIZE];
    int16_t y_data[CONST_ACC_GYRO_INT_SIZE];
    int16_t z_data[CONST_ACC_GYRO_INT_SIZE];
};

// Class GridEyeData / helper class ...
// ------------------------------------
class GridEyeData{
public:
    bool ok;
    double thermistor;
    double data[CONST_GRID_EYE_DOUBLE_SIZE];
    uint8_t data_raw[CONST_GRID_EYE_RAW_SIZE];
};

// Class BmeData / helper class ...
// --------------------------------
class BmeData{
public:
    bool ok;
    float tempCelsius;
    float tempFahrenheit;
    float pressureData;
    float humidityData;
};

// Class MLX90621 / helper class ...
// ---------------------------------
class MLX90621Data{
public:
    bool ok;
    uint8_t refresh_rate_mlx;
    int16_t resolutionMlx;
    int16_t configurationMlx;
    int16_t k_t1_scale;
    int16_t k_t2_scale;
    int16_t ptatMlx;
    int16_t cpixMlx; 
    float tambientMlx;
    float tminMlx;
    float tmaxMlx;
    uint8_t melexis_cmd[32];
    uint8_t melexis_eeprom[256];
    int16_t dataIrMlx[64];
    float tempMlx[64];
};

// Class MLX90640 / helper class ...
// ---------------------------------
class MLX90640Data{
public:
    bool ok;
    int32_t refresh_rate_mlx;
    float emissivity;
    float eTa;
    uint16_t eeMLX90640[832];
    uint16_t frame[834];
    float mlx90640To[768];
};

// Class i2c_sensors_driver / main class ...
// -----------------------------------------
class i2c_sensors_driver{
    
public:
    i2c_sensors_driver(std::string file_name_,std::string i2c_device0_,std::string i2c_device1_);
    int32_t initGridEyeSensor();
    int32_t initAccGyroSensor();
    int32_t initSensorBme200();
    int32_t initSensorMlx90621();
    int32_t initSensorMlx90640();
    void acquireSensorAccGyroData();
    void acquireSensorGridEyeData();
    void acquireDataBme();
    void acquireSensorMlx90621Data();
    void acquireSensorMlx90640Data();
    void stopAcquistion(){stopAcquistionI2C = true;}
    bool isGridEyeSensorReady() const {return gridEyeStatusRdy;}
    bool isAccGyroSensorReady() const {return accGyroStatusRdy;}
    GridEyeData getGridEyeSensorData() const {return gridEye;}
    AccGyroCoordinates getAccSensorData() const {return acc;}
    AccGyroCoordinates getGyroSensorData() const {return gyro;}
    void accGyroCallback();
    void gridEyeCallback();
    void bmeCallback();
    void Mlx90621Callback();
    void Mlx90640Callback();
protected:
    bool accGyroStatusRdy;
    bool gridEyeStatusRdy;
    bool bme200StatusRdy;
    bool mlx90621StatusRdy;
    bool mlx90640StatusRdy;
    bool stopAcquistionI2C;
    std::string file_name;
    std::string i2c_device0;
    std::string i2c_device1;
    std::string accGyroFile_;   
    std::string gridEyeFile_;
    std::string bmeFile_;
    std::string mlx90621File_;
    std::string mlx90640File_;
    std::string mlx90640RawFile_;
    AccGyroCoordinates acc;
    AccGyroCoordinates gyro;
    GridEyeData gridEye;
    MLX90621Data mlx16x4;
    MLX90640Data mlx32x24;
    BmeData bme;
private:
    // MLX90621 private functions ...
    // ------------------------------
    int readEepromMlx90621();
    int writeTrimmingMlx90621();
    int setConfigurationMlx906201();
    int readConfigMlx906201(uint16_t *configuration);
    int readResMlx906201();
    bool checkConfigMlx906201();
    int readPtatMlx906201();
    int readCpixMlx906201();
    int readIrMlx906201();
    void calculateTaMlx90621();
    void calculateToMlx90621();  
    void writeConfigurationMlx90640(paramsMLX90640 mlx90640);

    int32_t readParamsFromFile(std::string file_name_);
    int32_t i2c_write_buf(const char *dev, uint8_t addr, uint8_t nData, uint8_t *data);
    int16_t read_output_acc_gyro(const char *dev,uint8_t addr);
    int32_t i2c_read_buf(const char *dev, uint8_t addr, uint8_t subaddr, int nData, uint8_t *data);
    int32_t mlx90640_I2CRead(const char *dev,uint8_t slaveAddr,int cmdLength,int dataLength,uint8_t *data);
    uint64_t accGyroCount;
    uint64_t gridEyeCount;
    uint64_t bmeCount;
    uint64_t mlx90621Count;
    uint64_t mlx90640Count;
    
    paramsMLX90640 mlx90640;
    
    int32_t dig_T1, dig_T2, dig_T3;
    int32_t dig_P1, dig_P2, dig_P3, dig_P4, dig_P5;
    int32_t dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t dig_H1, dig_H2, dig_H3, dig_H4 ,dig_H5, dig_H6;
};

#endif  // I2C_SENSORS_DRIVER_H
