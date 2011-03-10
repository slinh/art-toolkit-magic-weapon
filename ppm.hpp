#ifndef __PPM_H__
#define __PPM_H__

#ifdef __APPLE__
	#include <openGL/gl.h>
	#include <GLUT/glut.h>
    #include <OpenGL/glu.h>
#else
	#include <GL/gl.h>
	#include <GL/glut.h>
	#include <GL/glu.h>
#endif

#include <fstream>
#include <cstring>
#include "vector3d.h"

GLubyte * loadPPM(const char * const fn,
			unsigned int& w,
			unsigned int& h);
unsigned char * loadPPMRGBA(char *fn, int *w, int *h, vector3di c);
#endif
