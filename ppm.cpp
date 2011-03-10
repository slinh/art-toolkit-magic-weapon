#include "ppm.hpp"


GLubyte * loadPPM(const char * const fn,
			unsigned int& w,
			unsigned int& h)
{

char head[70];
  int i,j;
  int d;
  GLubyte * img = NULL;

  FILE * f = fopen(fn, "r");

  if(f==NULL){
    fprintf(stderr,"Error in function readPPM : %s doesn't exist\n",fn);
    return 0;
  }
  fgets(head,70,f);

  if(!strncmp(head, "P6", 2)){
    i=0;
    j=0;
    while(i<3){
      fgets(head,70,f);

      if(head[0] == '#'){
	continue;
      }
      if(i==0)
	i += sscanf(head, "%d %d %d", &w, &h, &d);
      else
	if(i==1)
	  i += sscanf(head, "%d %d", &h, &d);
	else
	  if(i==2)
	    i += sscanf(head, "%d", &d);
    }

    img = new GLubyte[(size_t)(w) * (size_t)(h) * 3];
    if(img==NULL){
      fclose(f);
      return 0;
    }

    fread(img, sizeof(GLubyte), (size_t)w*(size_t)h*3,f);
    fclose(f);
  }
  else{
    fclose(f);
    fprintf(stderr,"Error in function readPPM : %s isn't a PPM file\n",fn);
  }
  return img;

}

unsigned char * loadPPMRGBA(char *fn, int *width, int *height, vector3di c) {
    int i, w, h, d;
    unsigned char* image;
    char head[70];  
    
    FILE * fp = fopen(fn, "r");
	
	if(fp==NULL){
		fprintf(stderr,"Error in function readPPM : %s doesn't exist\n",fn);


	}
	
    fgets(head, 70, fp);
    if(!strncmp(head, "P6", 2)) { 
    	i = 0;
    	while(i < 3) {
			fgets(head, 70, fp);
			if (head[0] == '#') 
            	continue;
			if (i == 0)
            	i += sscanf(head, "%d %d %d", &w, &h, &d);
			else if (i == 1)
            	i += sscanf(head, "%d %d", &h, &d);
			else if (i == 2)
            	i += sscanf(head, "%d", &d);
    	} 
    	image = new unsigned char[sizeof(unsigned char) * w * h * 4];
		for(int i =0; i < w * h * 4; i+=4){
			fread(image + i, sizeof(unsigned char), 1, fp);
			fread(image + i + 1, sizeof(unsigned char), 1, fp);
			fread(image + i + 2, sizeof(unsigned char), 1, fp);
			if(image[i] == c.X && image[i+1] == c.Y && image[i+2] == c.Z){
				image[i + 3] = 0; // Transparence
			}
			else
				image[i + 3] = 255;
		}
    	fclose(fp);
    	*width = w;
    	*height = h;
    	return image;
    }
    return 0;
}
