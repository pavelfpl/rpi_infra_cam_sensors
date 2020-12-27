/********************************************************************************

Copyright 2017  Pavel Fiala (University of West bohemia - pavelf@rice.zcu.cz)
                Richard Linhart (University of West bohemia - rlinhart@kae.zcu.cz
                
                File: i2c_driver.cpp (c++, main)
*********************************************************************************/
#include "i2c_sensors_driver.h"

#include <string>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#define SLEEP_TIME 1

// Default is sync disabled ...
// ----------------------------
// #define CONST_SYNC

sig_atomic_t volatile g_running = 1;

// signal_handler function ...
// ------------------------------------
void signal_handler(int signum){
  if (signum == SIGINT || signum == SIGTERM ) 
      g_running = 0;
}

int main(int argc, char ** argv){

   signal(SIGINT, &signal_handler); 
   signal(SIGTERM, &signal_handler); 
   
  int statusGridEye = -1;
  int statusAccGyro = -1;
  int statusBme = -1;
  int statusMlx90621 = -1;
  int statusMlx90640 = -1;
  
  std::string file_name = "i2c_driver.yaml";
  std::string i2c_dev_0 = "/dev/i2c-0";
  std::string i2c_dev_1 = "/dev/i2c-0";
  
#ifdef CONST_SYNC 
    int fd_sync_0 = -1;
    char dummy_sync_arr[2];  dummy_sync_arr[1] = 0x00; dummy_sync_arr[1] = 0x00;
    
    fd_sync_0 = open("/home/pi/sync_fifo_0",O_RDONLY);
#endif
  
  i2c_sensors_driver driver(file_name,i2c_dev_0,i2c_dev_1);
  statusGridEye = driver.initGridEyeSensor();
  statusAccGyro = driver.initAccGyroSensor();
  statusBme = driver.initSensorBme200();
  statusMlx90621 = driver.initSensorMlx90621();
  statusMlx90640 = driver.initSensorMlx90640();
  
  if(statusGridEye == -1 && statusAccGyro == -1  && statusBme == -1 && statusMlx90621 == -1 && statusMlx90640 == -1){       // No i2c drivers properly initialized ...
     std::cout << "Could not open any i2c device !!! "<<std::endl;
     return 0;
  }

#ifdef CONST_SYNC 
    while(g_running){
        if(fd_sync_0 < 0) break;
        // -------------------------------------------------------
        // Wait - blocking call - synchronization ...
        // -------------------------------------------------------
        read(fd_sync_0,dummy_sync_arr,2);
        if(statusGridEye !=-1){ driver.acquireSensorGridEyeData(); driver.gridEyeCallback();}
        if(statusAccGyro !=-1){ driver.acquireSensorAccGyroData(); driver.accGyroCallback();}
        if(statusBme!=-1){ driver.acquireDataBme(); driver.bmeCallback();}
        if(statusMlx90621!=-1){ driver.acquireSensorMlx90621Data(); driver.Mlx90621Callback();}
        if(statusMlx90640!=-1){ driver.acquireSensorMlx90640Data(); driver.Mlx90640Callback();}
        // ------------------------
        // Optional sleep ...
        // -----------------------
        // sleep(SLEEP_TIME);
    }
#endif  
  
  // Event loop ...
  // ------------------
  while (g_running){
    if(statusGridEye !=-1){ driver.acquireSensorGridEyeData(); driver.gridEyeCallback();}
    if(statusAccGyro !=-1){ driver.acquireSensorAccGyroData(); driver.accGyroCallback();}
    if(statusBme!=-1){ driver.acquireDataBme(); driver.bmeCallback();}
    if(statusMlx90621!=-1){ driver.acquireSensorMlx90621Data(); driver.Mlx90621Callback();}
    if(statusMlx90640!=-1){ driver.acquireSensorMlx90640Data(); driver.Mlx90640Callback();}
    sleep(SLEEP_TIME);
  }
  
#ifdef CONST_SYNC 
       close(fd_sync_0);
#endif
  
  return 1;
}
