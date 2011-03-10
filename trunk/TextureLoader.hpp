#ifndef __TEXTURE_LOADER_HPP__
#define __TEXTURE_LOADER_HPP__

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif

#include <iostream>
#include <string>
#include <vector>

class TextureLoader {
	
protected:
	GLuint 				idTex;
	std::string 	file;
	bool					repeat;
	
public:

	TextureLoader(std::string s, bool r = false) : file(s), repeat(r){} ;
	~TextureLoader(){};
	
	const std::string & getFile() const { return file; }
	std::string & setFile() { return file; }
	
	const GLuint & getIdTex() const { return idTex; }
	GLuint & setIdTex() { return idTex; }

	void create();

	
	
};

#endif
