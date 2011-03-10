#include "ObjLoader.hpp"

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

ObjLoader::ObjLoader()
{
	num_verts = 0;            
	num_texCoords = 0;          
	num_normals = 0;           
	num_faces = 0;              

	has_texCoords = 0;          
	has_normals = 0;               

	modelFileName = std::string("");
//	bbox = new Bbox();
	
}

ObjLoader::ObjLoader(std::string filename)
{
	num_verts = 0;            
	num_texCoords = 0;          
	num_normals = 0;           
	num_faces = 0;              

	has_texCoords = 0;          
	has_normals = 0;               

	modelFileName = filename;
//	bbox = new Bbox();
}


ObjLoader::~ObjLoader()
{
//	delete bbox;
	
	for(unsigned int i=0; i<vertices.size(); ++i)
	{
	  delete vertices[i];
	}
	
	for(unsigned int i=0; i<texCoords.size(); ++i)
	{
	  delete texCoords[i];
	}
	
	for(unsigned int i=0; i<normals.size(); ++i)
	{
	  delete normals[i];
	}
	
	
	for(unsigned int i=0; i<faces.size(); ++i)
	{
		if (faces[i]->vert_indices)
			free (faces[i]->vert_indices);
		
      if (faces[i]->uvw_indices)
		free (faces[i]->uvw_indices);
			
		if (faces[i]->norm_indices)
		free (faces[i]->norm_indices);	
		
	  delete faces[i];
	}
}


int ObjLoader::FirstPass (FILE *fp)
{
  int v, t, n;
  char buf[256];

  while (!feof (fp))
    {
      /* read whole line */
      fgets (buf, sizeof (buf), fp);

      switch (buf[0])
	{
	case 'v':
	  {
	    if (buf[1] == ' ')
	      {
		/* vertex */
		this->num_verts++;
	      }
	    else if (buf[1] == 't')
	      {
		/* texture coords. */
		this->num_texCoords++;
	      }
	    else if (buf[1] == 'n')
	      {
		/* normal vector */
		this->num_normals++;
	      }
	    else
	      {
		printf ("warning: unknown token \"%s\"! (ignoring)\n", buf);
	      }

	    break;
	  }

	case 'f':
	  {
	    /* face */
	    if (sscanf (buf + 2, "%d/%d/%d", &v, &n, &t) == 3)
	      {
		this->num_faces++;
		this->has_texCoords = 1;
		this->has_normals = 1;
	      }
	    else if (sscanf (buf + 2, "%d//%d", &v, &n) == 2)
	      {
		this->num_faces++;
		this->has_texCoords = 0;
		this->has_normals = 1;
	      }
	    else if (sscanf (buf + 2, "%d/%d", &v, &t) == 2)
	      {
		this->num_faces++;
		this->has_texCoords = 1;
		this->has_normals = 0;
	      }
	    else if (sscanf (buf + 2, "%d", &v) == 1)
	      {
		this->num_faces++;
		this->has_texCoords = 0;
		this->has_normals = 0;
	      }
	    else
	      {
		/* should never be there or the model is very crappy */
		fprintf (stderr, "error: found face with no vertex!\n");
	      }

	    break;
	  }

	case 'g':
	  {
	    /* group */
	    /*	fscanf (fp, "%s", buf); */
	    break;
	  }

	default:
	  break;
	}
    }

  /* check if informations are valid */
  if ((this->has_texCoords && !this->num_texCoords) ||
      (this->has_normals && !this->num_normals))
    {
      fprintf (stderr, "error: contradiction between collected info!\n");
      return 0;
    }

  if (!this->num_verts)
    {
      fprintf (stderr, "error: no vertex found!\n");
      return 0;
    }

  return 1;
}


int ObjLoader::SecondPass (FILE *fp)
{
	
/*	
  obj_vertex_t *pvert = this->vertices;
  obj_texCoord_t *puvw = this->texCoords;
  obj_normal_t *pnorm = this->normals;
  
  */
  
  std::vector<obj_face_t *> pface = this->faces;
  
  char buf[128], *pbuf;
  int i;


  while (!feof (fp))
    {
      /* read whole line */
      fgets (buf, sizeof (buf), fp);

      switch (buf[0])
	{
	case 'v':
	  {
	    if (buf[1] == ' ')
	    {
			obj_vertex_t * tmpVertex = new obj_vertex_t;
			
			/* vertex */
			if (sscanf (buf + 2, "%f %f %f %f",
					&tmpVertex->xyzw[0], &tmpVertex->xyzw[1],
					&tmpVertex->xyzw[2], &tmpVertex->xyzw[3]) != 4)
			  {
				if (sscanf (buf + 2, "%f %f %f", &tmpVertex->xyzw[0],
					&tmpVertex->xyzw[1], &tmpVertex->xyzw[2] ) != 3)
				  {
				fprintf (stderr, "error reading vertex data!\n");
				return 0;
				  }
				else
				{
					tmpVertex->xyzw[3] = 1.0;
				}
			  }

//			bbox->addPoint(tmpVertex->xyzw[0], tmpVertex->xyzw[1], tmpVertex->xyzw[2]);
			vertices.push_back(tmpVertex);
	      }
	    else if (buf[1] == 't')
	    {
			obj_texCoord_t * tmpTexCoord = new obj_texCoord_t;
			
			/* texture coords. */
			if (sscanf (buf + 2, "%f %f %f", &tmpTexCoord->uvw[0],
					&tmpTexCoord->uvw[1], &tmpTexCoord->uvw[2]) != 3)
			  {
				if (sscanf (buf + 2, "%f %f", &tmpTexCoord->uvw[0],
					&tmpTexCoord->uvw[1]) != 2)
				  {
				if (sscanf (buf + 2, "%f", &tmpTexCoord->uvw[0]) != 1)
				  {
					fprintf (stderr, "error reading texture coordinates!\n");
					return 0;
				  }
				else
				  {
					tmpTexCoord->uvw[1] = 0.0;
					tmpTexCoord->uvw[2] = 0.0;
				  }
				  }
				else
				{
					tmpTexCoord->uvw[2] = 0.0;
				 }
			  }

			texCoords.push_back(tmpTexCoord);
	      }
	    else if (buf[1] == 'n')
	    {
			obj_normal_t* tmpNormal = new obj_normal_t;
			
			/* normal vector */
			if (sscanf (buf + 2, "%f %f %f", &tmpNormal->ijk[0],
					&tmpNormal->ijk[1], &tmpNormal->ijk[2]) != 3)
			  {
				fprintf (stderr, "error reading normal vectors!\n");
				return 0;
			  }

			normals.push_back(tmpNormal);
	    }

	    break;
	  }

	case 'f':
	{
		obj_face_t* tmpFace = new obj_face_t;
		
	    pbuf = buf;
	    //int idFace = 0;
	    
	   tmpFace->num_elems = 0;

	    /* count number of vertices for this face */
	    while (*pbuf)
	    {
			if (*pbuf == ' ')
				tmpFace->num_elems++;

			pbuf++;
	    }

	    /* select primitive type */
	    if (tmpFace->num_elems < 3)
	    {
			fprintf (stderr, "error: a face must have at least 3 vertices!\n");
			return 0;
	    }
	    else if (tmpFace->num_elems == 3)
		{
			tmpFace->type = GL_TRIANGLES;
	    }
	    else if (tmpFace->num_elems == 4)
	    {
			tmpFace->type = GL_QUADS;
	    }
	    else
	    {
			tmpFace->type = GL_POLYGON;
	    }

	    /* memory allocation for vertices */
	    tmpFace->vert_indices = (int *)malloc (sizeof (int) * tmpFace->num_elems);

	    if (this->has_texCoords)
	      tmpFace->uvw_indices = (int *)malloc (sizeof (int) * tmpFace->num_elems);

	    if (this->has_normals)
	      tmpFace->norm_indices = (int *)malloc (sizeof (int) * tmpFace->num_elems);

	    /* read face data */
	    pbuf = buf;
	    i = 0;

	    for (i = 0; i < tmpFace->num_elems; ++i)
	    {
			pbuf = strchr (pbuf, ' ');
			pbuf++; /* skip space */

			/* try reading vertices */
			if (sscanf (pbuf, "%d/%d/%d",
					&tmpFace->vert_indices[i],
					&tmpFace->uvw_indices[i],
					&tmpFace->norm_indices[i]) != 3)
			{
				if (sscanf (pbuf, "%d//%d", &tmpFace->vert_indices[i],
					&tmpFace->norm_indices[i]) != 2)
				{
					if (sscanf (pbuf, "%d/%d", &tmpFace->vert_indices[i],
							&tmpFace->uvw_indices[i]) != 2)
					{
						sscanf (pbuf, "%d", &tmpFace->vert_indices[i]);
					}
				}
			}

			/* indices must start at 0 */
			tmpFace->vert_indices[i]--;

			if (this->has_texCoords)
			  tmpFace->uvw_indices[i]--;

			if (this->has_normals)
			  tmpFace->norm_indices[i]--;
	      }

		//pface++;
		faces.push_back(tmpFace);
	    break;
	  }
	}
    }
  return 1;
}


int ObjLoader::ReadOBJModel (const char *filename)
{
  FILE *fp;

  fp = fopen (filename, "r");
  if (!fp)
    {
      fprintf (stderr, "error: couldn't open \"%s\"!", filename);
      return 0;
    }

  /* first pass: read model info */
  
  if (!this->FirstPass (fp))
  {
		fclose (fp);
		return 0;
  }

  rewind (fp);

  /* second pass: read model data */
  if (!this->SecondPass(fp))
  {
      fclose (fp);
      return 0;
  }

  fclose (fp);
  return 1;
}


void ObjLoader::RenderOBJModel ()
{
	
  int i, j;

  for (i = 0; i < this->num_faces; ++i)
  {
		
    glBegin (this->faces[i]->type);
	for (j = 0; j < this->faces[i]->num_elems; ++j)
	  {
		  
	    if (this->has_texCoords)
	      glTexCoord3fv (this->texCoords[this->faces[i]->uvw_indices[j]]->uvw);


	    if (this->has_normals)
	      glNormal3fv (this->normals[this->faces[i]->norm_indices[j]]->ijk);
	      

	    glVertex4fv (this->vertices [this->faces[i]->vert_indices[j]]->xyzw);
	  }
	glEnd();
  }
}


void ObjLoader::initGL()
{ 
	// load model
	ReadOBJModel(modelFileName.c_str());
	
//  bbox->initialize(this);


  displayListId=glGenLists(1);
  if(displayListId==0)
    std::cerr << "erreur d'initialisation de la liste " << std::endl;
  glNewList(displayListId, GL_COMPILE);
    draw();
  glEndList();
}





void ObjLoader::draw()
{   
  RenderOBJModel();
}

