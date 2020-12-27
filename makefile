SOURCE  = i2c_sensors_driver.cpp i2c_sensors_driver.h i2c_driver.cpp
MY_PROGRAM = i2c_driver
MY_INCLUDES = /home/pi/i2c_sensors
MY_LIBRARIES = yaml-cpp 
CC = g++

all: $(MY_PROGRAM)

$(MY_PROGRAM):  $(SOURCE)
	$(CC)  -I$(MY_INCLUDES) $(SOURCE) -o$(MY_PROGRAM) -l$(MY_LIBRARIES) -lboost_date_time -lpthread -lMLX90640_API
	
clean:
	rm -f $(MY_PROGRAM)
