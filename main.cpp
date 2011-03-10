/*************************************************************************** \
 * created in 03 2010.
 * Mail : vnozick (at) univ-mlv.fr
 *
\***************************************************************************/

// external include
#include <iostream>
#include <cassert>
#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glext.h>
#include <vector>


// general
#include <OpenKN/image/ImageRGB.hpp>
#include <OpenKN/image/ioPPM.hpp>
#include <OpenKN/math/Matrix.hpp>
#include <OpenKN/math/Matrix3x3.hpp>
#include <OpenKN/math/Vector.hpp>
#include <OpenKN/math/Vector3.hpp>
#include <OpenKN/math/MathIO.hpp>
//#include <OpenKN/vision/ProjectiveCamera.hpp>
#include "ProjectiveCamera.hpp"
#include "CameraCalibrationZhang.hpp"
#include "ObjLoader.hpp"
#include <OpenKN/vision/Homography.hpp>
#include <OpenKN/math/InverseMatrix.hpp>

#include "DrawCircle.hpp"

// webcam
#include <OpenKN/controller/LIN_V4L2Webcam.hpp>

// ARtollkit
#include <AR/gsub.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/video.h>
#include <AR/gsub_lite.h>
#include <AR/config.h>

////////////////////////////////////////////////////////////////
// structure
typedef struct pattern{
  int patternId;
  unsigned char color[3];
}pattern;

////////////////////////////////////////////////////////////////
// global variables

// camera calibration
ProjectiveCamera projectiveCamera;// projective camera matrices
const double markerSize = 15.0;   // size of the border of a marker, in your unit (i.e. cm)
const double near = 10.0;         // openGL frustum near parameter
const double far  = 1000.0;       // openGL frustum far parameter
float projectionMatrix[16];       // GL projection matrix
float modelviewMatrix[16];        // GL modelview matrix

// webcam 
kn::V4L2Webcam webcam;            // webcam controller
GLuint         textureId;         // texture id to display the webcam image
kn::ImageRGB8u webcamImage;       // webcam image

// ARtoolkit 
std::vector<pattern> patternList; // pattern list


////////////////////////////////////////////////////////////////
// prototypes
void initGL();
void initTextures(const kn::ImageRGB8u &image, int &texId);
void updateTextures(const GLint &textureId, const unsigned char *imageData,
		    const int &width, const int &height);
void initCamera();
void initAR();
void frameRate();
void drawCorners(kn::ImageRGB8u &image, int markerId, ARMarkerInfo *marker_info);
void drawFunc(void);
void reshapeFunc(int, int);
void idleFunc(void);
void kbdFunc(unsigned char key, int x, int y);
void draw2D();
void draw3D();
bool cameraCalibration(ProjectiveCamera &Ptmp, 
		       const int markerNumber,
		       const int patternId, 
		       const ARMarkerInfo *marker_info);
kn::Matrix3x3d computeHomography(const ARMarkerInfo &marker_info);
void setGL2dParameters(const int width, const int height);
void setGL3dParameters(const ProjectiveCamera &Ptmp);
void setLightPosition();
void drawCube(kn::ImageRGB8u &image, const ProjectiveCamera &Ptmp);



/////////////////////////////////////////////////////////////////////////
/// camera initialisation
void initCamera()
{
  // webcam setup
  kn::V4L2WebcamParams V4L2params("/dev/video0",   // device
				  320,240,         // resolution
				  kn::FMT_YUYV,    // fmt
				  kn::MMAP_METHOD, // MMAP or READ
				  60);             // fps);

  webcam.openDevice(V4L2params);
  webcam.start();
  webcamImage = kn::ImageRGB8u(webcam.width(),webcam.height());  
  
  std::cout << "webcam resolution  = " << webcamImage.width()
	    << " x " << webcamImage.height()
	    << std::endl;

  // load camera intrinsic parameters
  kn::Matrix<double> K(3,3);
  K.setIdentity();
  K[0][0] = K[1][1] = 270.0;//sqrt(pow(webcamImage.width(),2)+pow(webcamImage.height(),2));
  K[0][2] = webcamImage.width()/2.0;
  K[1][2] = webcamImage.height()/2.0;

  //kn::loadMatrix(K,"inputData/Kfusion320x240.mat");
  projectiveCamera.updateK(K);  
  //projectiveCamera.defaultK(webcamImage.width(),webcamImage.height());
  projectiveCamera.getGLProjectionMatrix(webcamImage.width(),webcamImage.height(),near,far,projectionMatrix);
}


///////////////////////////////////////////////////////////
void initAR()
{
  // camera parameters
  ARParam cparam;
  cparam.xsize = webcamImage.width();
  cparam.ysize = webcamImage.height();
  cparam.dist_factor[0] = 0.0;
  cparam.dist_factor[1] = 0.0;
  cparam.dist_factor[2] = 0.0;
  cparam.dist_factor[3] = 1.0;
  arInitCparam( &cparam );

#if 0
  std::cout << "AR camera parameters" << std::endl;
  arParamDisp( &cparam );
#endif

  // load your patterns
  pattern patt;

  // patate de tomate !
  if( (patt.patternId = arLoadPatt("markerData/patate2tomate.patt")) < 0 ) {
    std::cerr << "pattern load error (markerData/patate2tomate.patt)" << std::endl;
    exit(0);
  }
  patt.color[0] = 255;
  patt.color[1] = 0;
  patt.color[2] = 0;

  patternList.push_back(patt);

  // lapin
  if( (patt.patternId = arLoadPatt("markerData/lapin.patt")) < 0 ) {
    std::cerr << "pattern load error (markerData/lapin.patt)" << std::endl;
    exit(0);
  }
  patt.color[0] = 0;
  patt.color[1] = 0;
  patt.color[2] = 255;

  patternList.push_back(patt);

  // far
  if( (patt.patternId = arLoadPatt("markerData/far.patt")) < 0 ) {
    std::cerr << "pattern load error (markerData/far.patt)" << std::endl;
    exit(0);
  }
  patt.color[0] = 0;
  patt.color[1] = 255;
  patt.color[2] = 0;

  patternList.push_back(patt);


  // half
  if( (patt.patternId = arLoadPatt("markerData/half.patt")) < 0 ) {
    std::cerr << "pattern load error (markerData/half.patt)" << std::endl;
    exit(0);
  }
  patt.color[0] = 0;
  patt.color[1] = 255;
  patt.color[2] = 0;

  patternList.push_back(patt);
}


/////////////////////////////////////////////////////////////////////////
/// GL parameters initialisation
void initGL()
{
  glutInitDisplayMode(GLUT_RGB|GLUT_DEPTH|GLUT_DOUBLE);

  glutInitWindowPosition(50,50);

  glutInitWindowSize(webcamImage.width(),webcamImage.height());
  if (glutCreateWindow("ar toolkit") == GL_FALSE){
    std::cerr << "ERROR : glutCreateWindow" << std::endl;
    exit(0);
  }

  glutReshapeFunc(reshapeFunc);
  glutDisplayFunc(drawFunc);
  glutIdleFunc(idleFunc);
  glutKeyboardFunc(kbdFunc);

  glClearColor(0.2,0.0,0.0,1.0);
  glEnable(GL_DEPTH_TEST);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  glEnable(GL_LIGHTING);
}


///////////////////////////////////////////////////////////
void initTextures(const kn::ImageRGB8u &myImage, GLuint &texId)
{
  glGenTextures(1, &texId);
  glBindTexture(GL_TEXTURE_2D, texId);

  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 
	       myImage.width(), myImage.height(),
	       0, GL_RGB, GL_UNSIGNED_BYTE, myImage.begin());
}


///////////////////////////////////////////////////////////
void updateTextures(const GLint &texId, unsigned char *image, 
		    const int width, const int height)
{
  assert(image != NULL);

  glBindTexture(GL_TEXTURE_2D, texId);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 
	       width, height,
	       0, GL_RGB, GL_UNSIGNED_BYTE, image);
}


////////////////////////////////////////////////////
/// set light position
void setLightPosition()
{
  // light 0 : on
  glEnable(GL_LIGHT0);

  // light position
  GLfloat lightPos[4] = {30.0, 30.0, 3.0 ,1.0};
  glLightfv(GL_LIGHT0,GL_POSITION, lightPos);
}


///////////////////////////////////////////////////////////
void setGL2dParameters(const int width, const int height)
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,width, height,0, -1.0, 1.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}


///////////////////////////////////////////////////////////
void setGL3dParameters(const ProjectiveCamera &Ptmp)
{
  // Computes the new GL projection matrix
  Ptmp.getGLProjectionMatrix(webcamImage.width(),webcamImage.height(),near,far,projectionMatrix);
 
  // Computes the new GL modelview matrix
  Ptmp.getGLModelviewMatrix(modelviewMatrix);

  // Applying the computed projection matrix
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(projectionMatrix);
   
  // Applying the computed modelview matrix
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(modelviewMatrix);
}


///////////////////////////////////////////////////////////
void draw2D()
{
  glBindTexture(GL_TEXTURE_2D, textureId);
  glEnable(GL_TEXTURE_2D);
  glBegin(GL_QUADS);
   glTexCoord2f(0.0f, 0.0f);
   glVertex2d(0.0, 0.0);

   glTexCoord2f(1.0f, 0.0f);
   glVertex2d(webcamImage.width(), 0.0);

   glTexCoord2f(1.0f, 1.0f);
   glVertex2d(webcamImage.width(), webcamImage.height());

   glTexCoord2f(0.0f, 1.0f);
   glVertex2d(0.0, webcamImage.height());
  glEnd();
  glDisable(GL_TEXTURE_2D);
}


///////////////////////////////////////////////////////////
void draw3D()
{
  setLightPosition();

/*
  // teapot
  GLfloat colorTeapot[4]  = { 0.6f, 0.6f, 0.4f, 1.0f};
  glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,colorTeapot);
  glPushMatrix();  
   glTranslatef( markerSize/2, markerSize/2, markerSize/4);
   glRotated(90,1.0,0.0,0.0); 
   glutSolidTeapot(markerSize/2);     
  glPopMatrix();
  // end teapot
*/

  // box
  glDisable(GL_LIGHTING);
  glColor3f(1.0,0.0,0.0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(0,0,0); 
  glVertex3f(markerSize,0,0); 
  glVertex3f(markerSize,markerSize,0); 
  glVertex3f(0,markerSize,0);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3f(0,0,markerSize);
  glVertex3f(markerSize,0,markerSize);
  glVertex3f(markerSize,markerSize,markerSize); 
  glVertex3f(0,markerSize,markerSize);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(0,0,0); glVertex3f(0,0,markerSize);
  glVertex3f(markerSize,0,0); glVertex3f(markerSize,0,markerSize);
  glVertex3f(markerSize,markerSize,0); glVertex3f(markerSize,markerSize,markerSize); 
  glVertex3f(0,markerSize,0); glVertex3f(0,markerSize,markerSize);
  glEnd();
  glEnable(GL_LIGHTING);
  // end box
}


///////////////////////////////////////////////////////////
void drawFunc(void)
{
  // 2D
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  setGL2dParameters(webcamImage.width(), webcamImage.height());
  draw2D();

  // 3D
  glClear(GL_DEPTH_BUFFER_BIT);
  setGL3dParameters(projectiveCamera);
  draw3D();

  // finish
  glutSwapBuffers();
  frameRate();
}


///////////////////////////////////////////////////////////
void reshapeFunc(int w,int h)
{
  glViewport(0, 0, (GLint)w, (GLint)h);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, webcamImage.width(), webcamImage.height(), 0, -1.0, 1.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}


///////////////////////////////////////////////////////////
// draw marker corners on the images
void drawCorners(kn::ImageRGB8u &image, int markerNumber, ARMarkerInfo *marker_info)
{
  for(int m=0; m<markerNumber; m++)
    {
      if(marker_info[m].cf<0.2) continue;

      // identify the pattern
      int k = -1; // pattern id
      for(int i=0; i<(int)patternList.size(); i++)
	if( patternList[i].patternId == marker_info[m].id ) {
	  if( k == -1 ) k = i;
	  else if( marker_info[m].cf < marker_info[i].cf ) k = i;
	}    

      // if the marker is not identifyed, quit
      if(k == -1) return;

      // direction color
      unsigned char white[3] = {255,255,255};

      // draw the 4 corners
      for(int i=0; i<4; i++)
      {
	// one of the corner in white
	unsigned char *color = patternList[i].color;

	if(i==marker_info[m].dir) color = white; // default color for corner
	else color = patternList[k].color;
	
	drawDisc(webcamImage,
	        (int)marker_info[m].vertex[i][0],
		(int)marker_info[m].vertex[i][1],
		3, 
		color[0],color[1],color[2]);
      }
    }
}


///////////////////////////////////////////////////////////
void drawCube(kn::ImageRGB8u &image, const ProjectiveCamera &Ptmp)
{
  for(int i=0; i<8; i++)
    {
      kn::Vector<double> pt(4);
      pt[3] = 1.0;
      for(int j=0; j<3; j++)
	pt[j] = markerSize * ((i>>j)%2);

      kn::Vector<double> px(Ptmp.project(pt));
      drawDisc(image,(int)px[0],(int)px[1], 3, 255,255,0);
    }
}


///////////////////////////////////////////////////////////
// computes an homography between the detected pixels and the marker referencial
kn::Matrix3x3d computeHomography(const ARMarkerInfo &marker_info)
{
  std::vector< std::pair< kn::Vector3d, kn::Vector3d > > correspondance2d2d;

  kn::Vector3d pt;
  kn::Vector3d px(3);
  pt[2] = px[2] = 1.0;

  // corner 0
  px[0] = marker_info.vertex[(0+marker_info.dir)%4][0];
  px[1] = marker_info.vertex[(0+marker_info.dir)%4][1];
  pt[0] = 0.0;
  pt[1] = 0.0;
  correspondance2d2d.push_back(std::pair<kn::Vector3d, kn::Vector3d>(pt,px));

  // corner 1
  px[0] = marker_info.vertex[(1+marker_info.dir)%4][0];
  px[1] = marker_info.vertex[(1+marker_info.dir)%4][1];
  pt[0] = 0.0;
  pt[1] = markerSize;
  correspondance2d2d.push_back(std::pair<kn::Vector3d, kn::Vector3d>(pt,px));

  // corner 2
  px[0] = marker_info.vertex[(2+marker_info.dir)%4][0];
  px[1] = marker_info.vertex[(2+marker_info.dir)%4][1];
  pt[0] = markerSize;
  pt[1] = markerSize;
  correspondance2d2d.push_back(std::pair<kn::Vector3d, kn::Vector3d>(pt,px));

  // corner 3
  px[0] = marker_info.vertex[(3+marker_info.dir)%4][0];
  px[1] = marker_info.vertex[(3+marker_info.dir)%4][1];
  pt[0] = markerSize;
  pt[1] = 0.0;
  correspondance2d2d.push_back(std::pair<kn::Vector3d, kn::Vector3d>(pt,px));
 
  // compute the homography
  return kn::computeHomography(correspondance2d2d,false);
}


///////////////////////////////////////////////////////////
bool cameraCalibration(ProjectiveCamera &Ptmp, 
		       const int markerNumber,
		       const int patternId, // pattern to be used for the camera calibration
		       const ARMarkerInfo *marker_info)
{
  int k = -1;
  double cf = 0.2;

  // find the right marker
  for(int m=0; m<markerNumber; m++)
    {
      if(marker_info[m].id == patternId)
	if(marker_info[m].cf > cf)
	  {
	    k  = m;
	    cf = marker_info[m].cf;
	  }
    }

  // if not detected
  if(k == -1) return false;

  // calibration
  kn::Matrix3x3d H = computeHomography(marker_info[k]);
  computeExternalParametersZhang(H,Ptmp);

  return true;
}


///////////////////////////////////////////////////////////
void idleFunc(void)
{ 
  // image capture
  webcamImage = kn::ImageRGB8u(webcamImage.width(),webcamImage.height(),webcam.getImage()); 

  // ARToolkit square detector
  ARMarkerInfo    *markerInfo;
  int             markerNumber;   // number of detected markers
  int	          threshold = 60;

  // detect the markers in the video frame
  if(arDetectMarkerLite((ARUint8*)webcamImage.begin(), threshold, &markerInfo, &markerNumber) < 0 ) {
    std::cerr << "arDetectMarker : error" << std::endl; 
    exit(0);
  }

  // draw corners
  drawCorners(webcamImage,markerNumber,markerInfo);

  // check
  for(int i=0; i<(int)patternList.size(); ++i)
    {
      // camera calibration
      if(cameraCalibration(projectiveCamera, markerNumber, patternList[i].patternId, markerInfo)){
      // calibration test (draw the origine of the referential)
      drawCube(webcamImage,projectiveCamera);
    }
  }

  // GL 3D camera parameters
  cameraCalibration(projectiveCamera, markerNumber, patternList[2].patternId, markerInfo);
  
  // texture update
  updateTextures(textureId,
		 webcamImage.begin(),
		 webcamImage.width(), 
		 webcamImage.height());
  
  // draw
  glutPostRedisplay(); 
}


/////////////////////////////////////////////////////////////////////////
void kbdFunc(unsigned char key, int x, int y)
{ 
  switch (key)
    {
    case 27 : exit(0);
    }
}


/////////////////////////////////////////////////////////////////////////
/// calcule et affiche le frame rate dans la barre de titre
void frameRate()
{
  static GLfloat time;
  GLfloat time2 = (glutGet(GLUT_ELAPSED_TIME)-time)*1.0e-3;
  time = glutGet(GLUT_ELAPSED_TIME);
  char title[100];
  static int nb=0;
  nb++;
  static double val = 0.0;
  val += 1.0/time2;
  sprintf(title,"fps %3.2lf      |    moy  %3.2lf",1.0/time2, val/(double)nb);
  glutSetWindowTitle(title);
}


///////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // init Camera
  std::cout << "camera initialisation ..." << std::endl;
  initCamera();

  // init AR
  std::cout << "AR toolkit initialisation ..." << std::endl;
  initAR();

  // init Glut
  std::cout << "Glut initialisation ..." << std::endl;
  glutInit(&argc, argv);

  // init GL
  std::cout << "GL initialisation ..." << std::endl;
  initGL();

  // init textures
  std::cout << "textures initialisation ..." << std::endl;
  initTextures(webcamImage,textureId);

  // main loop
  glutMainLoop();

  return 0;
}
