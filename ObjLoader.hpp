#ifndef __OBJ_LOADER_HPP__
#define __OBJ_LOADER_HPP__

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif

#include <iostream>
#include <string>
#include <vector>

class Bbox;

/* vector */
typedef float vec3_t[3];
typedef float vec4_t[4];


/* vertex */
typedef struct
{
  vec4_t xyzw;

} obj_vertex_t;


/* texture coordinates */
typedef struct
{
  vec3_t uvw;

} obj_texCoord_t;


/* normal vector */
typedef struct
{
  vec3_t ijk;

} obj_normal_t;


/* polygon */
typedef struct
{
  GLenum type;        /* primitive type */
  int num_elems;      /* number of vertices */

  int *vert_indices;  /* vertex indices */
  int *uvw_indices;   /* texture coordinate indices */
  int *norm_indices;  /* normal vector indices */

} obj_face_t;


typedef enum _typeObj
{
	HOUSE,
	SETTING
} TypeObj;

class ObjLoader {

	protected:
		int num_verts;              /* number of vertices */
		int num_texCoords;          /* number of texture coords. */
		int num_normals;            /* number of normal vectors */
		int num_faces;              /* number of polygons */

		int has_texCoords;          /* has texture coordinates? */
		int has_normals;            /* has normal vectors? */

		std::vector<obj_vertex_t *> 	vertices;	  /* vertex list        */
		std::vector<obj_texCoord_t *>	texCoords;	/* tex. coord. list   */
		std::vector<obj_normal_t *> 	normals;    /* normal vector list */
		std::vector<obj_face_t *> 		faces;      /* model's polygons   */

		// infos XML
		std::string                   modelFileName;
		GLint                         displayListId;

		Bbox *                        bbox;

	public:
		ObjLoader();
		ObjLoader(std::string filename);
		~ObjLoader();
		
		inline const Bbox & getBbox() const { return *bbox; }
    inline Bbox & setBbox() { return *bbox; }
		
		inline const GLint & getDisplayListId() const { return displayListId;}
		inline GLint & setDisplayListId() { return displayListId;}
		
		inline const std::string & getModelFileName() const { return modelFileName;}
		inline std::string & setModelFileName() { return modelFileName;}

		int FirstPass (FILE *fp);
		int SecondPass (FILE *fp);
		int ReadOBJModel (const char *filename);
		void RenderOBJModel ();

		void initGL();
		void draw();

};

#endif
