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

Obj.o  : Obj.cpp Obj.hpp vector3d.h
	@echo "compile Obj"
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  
	@echo "done..."

ObjLoader.o  : ObjLoader.cpp ObjLoader.hpp
	@echo "compile ObjLoader"
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  
	@echo "done..."

Texture.o  : Texture.cpp Texture.hpp
	@echo "compile Texture"
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  
	@echo "done..."


TextureLoader.o  : TextureLoader.cpp TextureLoader.hpp
	@echo "compile TextureLoader"
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  
	@echo "done..."

ppm.o  : ppm.cpp ppm.hpp
	@echo "compile PPM"
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  
	@echo "done..."

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

main.o : main.cpp ProjectiveCamera.o CameraCalibrationZhang.o ObjLoader.o Obj.o Texture.o TextureLoader.o ppm.o
	@echo "compile main"
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  
	@echo "done..."

clean :	
	@echo "**************************"
	@echo "CLEAN"
	@echo "**************************"
	$(RM) *~ $(OBJ) $(BIN)    

