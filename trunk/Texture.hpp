#ifndef __TEXTURE_HPP_
#define __TEXTURE_HPP_

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
#include "TextureLoader.hpp"

class Texture 
{
protected:	
	int				id;
	TextureLoader * textureLoader;

public:
	
	Texture(int i, TextureLoader * texture) : id(i), textureLoader(texture){} ;
	~Texture(){};

	const TextureLoader & getTextureLoader() const { return *textureLoader; }
	TextureLoader & setTextureLoader() { return *textureLoader; }

	void bind() const;
	void unbind() const;
	void tryCube();
};

#endif
