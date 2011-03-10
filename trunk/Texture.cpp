#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include <GL/glew.h>
	#include <GL/gl.h>
	#include <GL/glut.h>
	#include <GL/glu.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>

#include "Texture.hpp"
#include "ppm.hpp"


void Texture::bind()const
{
  glEnable(GL_TEXTURE_2D);
  switch(id)
  {
	  case 0:
	  	glActiveTexture(GL_TEXTURE0);
	    break;
	  
	  case 1:
	  	glActiveTexture(GL_TEXTURE1);
	    break;
	  
	  case 2:
	  	glActiveTexture(GL_TEXTURE2);
	    break;
		  
	  case 3:
		  glActiveTexture(GL_TEXTURE3);
	    break;
      
    case 4:
		  glActiveTexture(GL_TEXTURE4);
	    break;
      
    case 5:
		  glActiveTexture(GL_TEXTURE5);
	    break;
  }	
  
	glBindTexture(GL_TEXTURE_2D,textureLoader->getIdTex());
	
  glTexEnvf(GL_TEXTURE_ENV, GL_SRC0_RGB, GL_TEXTURE0);
  glTexEnvf(GL_TEXTURE_ENV, GL_SRC1_RGB, GL_PRIMARY_COLOR);
  
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE);
  glTexEnvf(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_MODULATE);

}


void Texture::unbind()const
{
	
  switch(id)
  {
	  case 0:
	  	glActiveTexture(GL_TEXTURE0);
	    break;
	  
	  case 1:
	  	glActiveTexture(GL_TEXTURE1);
	    break;
	  
	  case 2:
	  	glActiveTexture(GL_TEXTURE2);
	    break;
		  
	  case 3:
		  glActiveTexture(GL_TEXTURE3);
	    break;
      
    case 4:
		  glActiveTexture(GL_TEXTURE4);
	    break;
      
    case 5:
		  glActiveTexture(GL_TEXTURE5);
	    break;
  }	
	
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,0);
//  glDisable(GL_TEXTURE_2D);
}


void Texture::tryCube(){

	
	//application de la texture
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureLoader->getIdTex());
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
    glTranslatef(-10,-10,0.0f);
	glColor3ub(0,255,255);
	glBegin(GL_QUADS);
		glTexCoord2f(0.f,0.0f);	
		glVertex3f(0.,0.,0.);
		
		glTexCoord2f(1.f,0.0f);
		glVertex3f(20,0.,0.);

		glTexCoord2f(1.f,1.0f);
		glVertex3f(20,20,0.);

		glTexCoord2f(0.f,1.0f);
		glVertex3f(0.,20,0.);
	glEnd();	
  glDisable(GL_TEXTURE_2D);

}
