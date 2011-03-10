#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>

#include "Obj.hpp"
//#include "Bbox.hpp"
#include "Texture.hpp"
//#include "utils.hpp"

void checkGLError(int line)
{
  #ifdef CHECK_ERRORS
  GLenum err = glGetError();
  if(err!=GL_NO_ERROR){
    std::cerr << "Erreur GL "<<line<<" :" << std::endl;
    std::cerr << gluErrorString(err) << std::endl;
  }
  #endif
}

Obj::Obj(ObjLoader* obj): objLoader(obj)
{
//	bbox = NULL;
}


Obj::~Obj()
{

}

void Obj::setScale(float x, float y, float z)
{
	scale.X = x;
	scale.Y = y;
	scale.Z = z;
}

void Obj::setTranslate(float x, float y, float z)
{
	translate.X = x;
	translate.Y = y;
	translate.Z = z;
}


void Obj::setAngleRotation(float angle)
{
	angleRotation = angle;
}

void Obj::setAxeRotate(float x, float y, float z)
{
	axeRotate.X = x;
	axeRotate.Y = y;
	axeRotate.Z = z;
}


void Obj::init()
{ 
/*	bbox = new Bbox(objLoader->getBbox());
	bbox->initialize(this);
	bbox->initTextures();
*/
}

void Obj::display()
{
  checkGLError(473);
  //glEnable(GL_TEXTURE_2D);
  for(unsigned int i=0; i<textures.size(); ++i)
  {
    textures[i]->bind();
  }
  glPushMatrix();

    checkGLError(481);

    glTranslatef(translate.X, translate.Y, translate.Z);
    glScalef(scale.X, scale.Y, scale.Z);
    glRotatef(angleRotation, axeRotate.X, axeRotate.Y, axeRotate.Z);

    glCallList(objLoader->getDisplayListId());

  glPopMatrix();



  for(unsigned int i=0; i<textures.size(); ++i)
  {
    textures[i]->unbind();
  }
  //glDisable(GL_TEXTURE_2D);


}

void Obj::displayList()
{
  //glEnable(GL_TEXTURE_2D);
  for(unsigned int i=0; i<textures.size(); ++i)
  {
    textures[i]->bind();
  }
  glPushMatrix();

    glCallList(objLoader->getDisplayListId());

  glPopMatrix();



  for(unsigned int i=0; i<textures.size(); ++i)
  {
    textures[i]->unbind();
  }
  //glDisable(GL_TEXTURE_2D);


}
