###############  Example Makefile and example application  ######################################
#Build Revision : 2 
#Build Date :  04-04-17  
#Change Log : 
#Build By   :  ganesh
#Revised By :
#Build Note : builds a Simple SerialDevice library the Interfaces are accesible thru SerialDevice.h and libSerialDevice.a
############################################################################################################################

ARLIB = libs/libSerialDevice.a
SRCS = src/SerialDevice.cpp 
OBJ = obj/SerialDevice.o
LOCALINCLUDES = includes/

INCLUDES = -I$(LOCALINCLUDES) 
CC = gcc
CXX = g++ 
STRIP = strip
CFLAGS = $(include_dirs) -c -Wall -D_GCC=1 -O2 -std=c++11
AR = ar -cru 
 
LIBS= 

$(ARLIB):
	$(CXX) $(CFLAGS) $(SRCS) $(INCLUDES) $(LIBS) -o $(OBJ)  
	$(AR) $(ARLIB) obj/*.o

.PHONY : clean
clean :
	rm -f  $(ARLIB) obj/*.o
