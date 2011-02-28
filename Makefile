CC     = g++
CFLAGS = -Wall -O3 
LIB    = -L/home/ens/biri/Lib/openkn-1.2/lib/ -L/home/ens/biri/ARToolKit/lib -lglut -lGL -lm -lARgsub -lAR -lOpenKN-controller -lOpenKN-vision -lOpenKN-image -lOpenKN-math -ljpeg -lpng
OBJ    = main.o DrawCircle.o ProjectiveCamera.o CameraCalibrationZhang.o
RM     = rm -f
BIN    = arTeapot
INCLUDES = -I/home/ens/biri/Lib/openkn-1.2/include  -I/home/ens/biri/ARToolKit/include/

all : $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) $(LIB) $(INCLUDES) -o $(BIN)
	@echo "--------------------------------------------------------------"
	@echo "----------   to execute type: $(BIN) &   -----------"
	@echo "--------------------------------------------------------------"


ProjectiveCamera.o  : ProjectiveCamera.cpp ProjectiveCamera.hpp
	@echo "compile ProjectiveCamera"
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  
	@echo "done..."

CameraCalibrationZhang.o : CameraCalibrationZhang.cpp CameraCalibrationZhang.hpp ProjectiveCamera.o
	@echo "compile ProjectiveCamera"
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  
	@echo "done..."

DrawCircle.o : DrawCircle.cpp DrawCircle.hpp
	@echo "compile DrawCircle"
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  
	@echo "done..."

main.o : main.cpp ProjectiveCamera.o CameraCalibrationZhang.o 
	@echo "compile main"
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  
	@echo "done..."

clean :	
	@echo "**************************"
	@echo "CLEAN"
	@echo "**************************"
	$(RM) *~ $(OBJ) $(BIN)    

