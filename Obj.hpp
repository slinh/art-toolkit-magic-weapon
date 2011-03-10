#ifndef __OBJ_H__
#define __OBJ_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "Texture.hpp"
//#include "Bbox.hpp"
#include "vector3d.h"
#include "ObjLoader.hpp"


class Obj {
	
protected:

	ObjLoader*  									objLoader;
			int                           id;

// textures
	std::vector<Texture *> 			  textures;
	
  Bbox *                        bbox;
  
  int                           idShader;
  vector3df                     scale;
  vector3df											translate;
  vector3df                     axeRotate;
  float                         angleRotation;
  TypeObj						typeObj;
  int							associatedScene;
	
public:
  Obj(ObjLoader* objLoader);
	~Obj();
	


/* getter - setters */
public:

	inline const int & getId() const { return id;}
	inline int & setId() { return id;}

  inline const ObjLoader & getObjLoader() const { return *objLoader; }
  inline ObjLoader & setObjLoader() { return *objLoader; }
  
  inline const Bbox & getBbox() const { return *bbox; }
  inline Bbox & setBbox() { return *bbox; }

  inline const std::vector<Texture *> & getTextures() const { return textures;}
  inline std::vector<Texture *> & setTextures() { return textures;}

  inline const vector3df & getScale() const { return scale;}
  inline vector3df & setScale() { return scale;}
  
  void setScale(float x, float y, float z);	
	
  inline const vector3df & getTranslate() const { return translate;}
  inline vector3df & setTranslate() { return translate;}

  void setTranslate(float x, float y, float z);		
	
  inline const vector3df & getAxeRotate() const { return axeRotate;}
  inline vector3df & setAxeRotate() { return axeRotate;}
	
  void setAxeRotate(float x, float y, float z);	

  inline const float & getAngleRotation() const { return angleRotation;}
  inline float  & setAngleRotation() { return angleRotation;}
  
  void setAngleRotation(float angle); 

  inline const TypeObj & getType() const { return typeObj;}
  inline TypeObj & setType() { return typeObj;}
  
  inline const int & getAssociatedScene() const { return associatedScene;}
  inline int & setAssociatedScene() { return associatedScene;}

  void init();
  void display();
  void displayList();

};





#endif
