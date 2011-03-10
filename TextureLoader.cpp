#include "TextureLoader.hpp"
#include "ppm.hpp"


void TextureLoader::create()
{
	glGenTextures(1,&idTex);

	// Texture init ok
	if(idTex == 0){
		std::cerr << "Identifiant de texture incorrect" << std::endl;
		return;
	}

	// bind for params
    glBindTexture(GL_TEXTURE_2D,idTex);

	unsigned int tmpwidth, tmpheight;
	unsigned char * image = loadPPM(file.c_str(), tmpwidth,tmpheight);
	if(image==0){
		std::cerr << "Erreur au chargement le l'image" << std::endl;
		return;
	}
  
	if( repeat )
	{
    std::cout << "texture avec repeat" << std::endl;
    
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tmpwidth, tmpheight, 0, GL_RGB, GL_UNSIGNED_BYTE,image );
	}

	else
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tmpwidth, tmpheight, 0, GL_RGB, GL_UNSIGNED_BYTE,image );
	}

	delete[] image;
	glBindTexture(GL_TEXTURE_2D,0);
}
