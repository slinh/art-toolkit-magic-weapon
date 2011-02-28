/*************************************************************************** \
 * Copyright (C) by University Paris-Est - MISS team
 * ProjectiveCamera.hpp created in 01 2009.
 * Mail : biri@univ-mlv.fr
 *
 * This file is part of the OpenKraken-vision.
 *
 * The OpenKraken-vision is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * The OpenKraken-vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.	 If not, see <http://www.gnu.org/licenses/>.
 *
\***************************************************************************/

/*
 * Anti-doublon
 */
#ifndef __OPENKN_VISION__PROJECTIVE_CAMERA_HPP__
#define __OPENKN_VISION__PROJECTIVE_CAMERA_HPP__

/*
 * External Includes
 */
#include <vector>


/*
 * Internal Includes
 */
#include <OpenKN/math/Matrix.hpp>
#include <OpenKN/math/Matrix3x3.hpp>
#include <OpenKN/math/Matrix4x4.hpp>
#include <OpenKN/math/Vector.hpp>



	/*
	 * Class definition
	 */

	/**
	 * \brief projective camera class, aka pinhole camera model (for computer vision).
	 The notation used in this class is inspired from "Multiple View Geometry" - Hartley & Zisserman.
	*/
	class ProjectiveCamera {


		/*
		 * class variable
		 */
	protected :
		/**
		 * \brief 3x4 projection matrix P=K[R|-RC]
		 */
		kn::Matrix<double> matP;

		/**
		 * \brief 3x3 internal parameter matrix
		 */
		kn::Matrix3x3<double> matK;

		/**
		 * \brief 3x3 internal parameter inverse matrix (for Zhang method)
		 */
		kn::Matrix3x3<double> matKinverse;

		/**
		 * \brief 3x3 rotation matrix
		 */
		kn::Matrix3x3<double> matR;

		/**
		 * \brief homogeneous camera centre coordinates (4 components)
		 */
		kn::Vector4d vecC;


	public:

		/**
		 * \brief Copy Constructor
		 * \param myProjectiveCamera : Source projective camera
		 */
		ProjectiveCamera(const ProjectiveCamera &myProjectiveCamera);

		/**
		 * \brief Default Constructor (Set ProjectiveCamera as Identity)
    		 */
		ProjectiveCamera();

		/**
		 * \brief constructor from a projection matrix P.
		 * This function performs the following decomposition : P = K[R|-RC].
		 * \param myP : a 3x4 projection matrix
		 * \throw VisionException argument P matrix : invalid size
		 */
		ProjectiveCamera(const kn::Matrix<double>& myP);

		/**
		 * \brief constructor from all the required matrices : P = K[R|-RC].
		 * \param myK : a 3x3 internal parameters matrix.
		 * \param myR : a 3x3 rotation matrix.
		 * \param myC : a 4 homogeneous vector (camera centre coordinates).
		 * \throw VisionException argument : invalid size
		 */
		ProjectiveCamera(const kn::Matrix3x3<double> &myK,
				 const kn::Matrix3x3<double> &myR,
				 const kn::Vector4d &myC);

		/**
		 * \brief constructor from an internal parameter matrix. The projection matrix will be P=K[Id|0]
		 * \param myK : a 3x3 internal parameters matrix.
		 * \throw VisionException argument : invalid size
		 */
		ProjectiveCamera(const kn::Matrix3x3<double> &myK);

		/**
		 * \brief destructor
		 */
		~ProjectiveCamera();


	protected :
		/**
		 * \brief extract K, R and C from P
		 * P = [M|-MC] et M = KR from Multiple View Geometry, Hartley Zisserman, camera model
		 */
		void decompose();

		/**
		 * \brief compute P from K, R and C : P = K[R|-RC].
		 */
		kn::Matrix<double> compose(const kn::Matrix3x3<double> &K,
					   const kn::Matrix3x3<double> &R,
					   const kn::Vector4d &C) const;

		/**
		 * \brief extract C from P
		 */
		void extractCentre();


	public :

		/**
		* \brief Set a default camera intrinsic parameter matrix K, (i.e. centered optical ray, standard focal lenght, squared pixels)
		* \param imageWidth  : camera resolution : width
		* \param imageHeight : camera resolution : height
		*/
		void defaultK(const unsigned int imageWidth, const unsigned imageHeight);

		/**
		 * \brief return P matrix (read-only)
		 */
		inline kn::Matrix<double> P() const{
			return matP;
		};

		/**
		* \brief return K matrix (read-only)
		*/
		inline kn::Matrix3x3<double> K() const{
		  return matK;
		};

		/**
		* \brief return Kinverse \f$ = K^{-1} \f$ matrix (read-only)
		*/
		inline kn::Matrix3x3<double> Kinverse() const{
		  return matKinverse;
		};

		/**
		 * \brief return R matrix (read-only)
		 */
		inline kn::Matrix3x3<double> R() const{
			return matR;
		};

		/**
		 * \brief return C vector in homogeneous coordinates (read-only)
		 */
		inline kn::Vector4d C() const{
			return vecC;
		};

		/**
		 * \brief return M Matrix (M=KR such P=[M|-MC]) (read-only)
		 */
		kn::Matrix<double> M() const;

		/**
		 * \brief return extrinsic Matrix ([R|-RC]) (read-only)
		 */
		kn::Matrix<double> extrinsicMatrix() const;


		/**
		 * \brief project an homogeneous 4-vector on the camera.
		 * \param X : homogeneous 4-vector to be projected
		 * \return the projection x=PX of X (3-homogeneous-vector)
		 */
		inline kn::Vector<double> project(const kn::Vector<double> &X) const{
		       kn::Vector<double> x(matP*X);
		       x.setHomogeneousNormalForm();
		       return x;
		};

		/**
		 * \brief update the camera external parameters
		 * \param R : 3x3 rotation matrix
		 * \param C : the position of the camera : 4-homogeneous-vector
		 */
		inline void updateRC(const kn::Matrix3x3d &R, const kn::Vector4d &C){
			matR = R;
			vecC = C;
			vecC.setHomogeneousNormalForm();
			matP = compose(matK, matR, vecC);
		};

		/**
		* \brief update the camera external parameters
		* \param Pcanonic : 4x3 matrix : Pcanonic=[R|-RC] (C=camera position)
		* \throw VisionException argument : invalid size
		*/
		void updateRC(const kn::Matrixd &Pcanonic);

		/**
		* \brief update the camera external parameters
		* \param R : 3x3 rotation matrix
		*/
		inline void updateR(const kn::Matrix3x3d &R){
		  matR = R;
		  matP = compose(matK, matR, vecC);
		};

		/**
		* \brief update the camera external parameters
		* \param C : the position of the camera : 4-homogeneous-vector
		*/
		inline void updateC(const kn::Vector4d &C){
		  vecC = C;
		  vecC.setHomogeneousNormalForm();
		  matP = compose(matK, matR, vecC);
		};

		/**
		 * \brief update the camera projection matrix
		 * \param P : 4x3 matrix : P=K[R|-RC] (C=camera position)
		 * \throw VisionException argument : invalid size
		 */
	        void updateP(const kn::Matrixd& P);

		/**
		 * \brief update the camera internal parameters matrix
		 * \param K : 3x3 matrix : P=K[R|-RC] (C=camera position)
		 * \throw VisionException argument : invalid size
		 */
	        void updateK(const kn::Matrixd& K);

		/**
		 * \brief rescale the camera internal parameters
		 * \param factor : the factor of rescaling
		 */
		void rescaleK(const double &factor);

		/**
		 * \brief compute the GL projection matrix
		 * The way to use this matrix in an OpenGL program is :
		 * glMatrixMode(GL_PROJECTION);
		 * glLoadMatrixf(GLPMat);
		 * \param cameraWidth width of the original camera (not the OpenGL window size)
		 * \param cameraHeight height of the original camera (not the OpenGL window size)
		 * \param glnear OpenGL near parameter
		 * \param glfar OpenGL far parameter
		 * \param GLPMat the computed GL projection matrix
		 * \author Frankoi
		 */
		void getGLProjectionMatrix(const size_t& cameraWidth,
					   const size_t& cameraHeight,
					   const float& glnear,
					   const float& glfar,
					   float GLPMat[16]) const;

		/**
		 * \brief compute the GL modelview matrix
		 * The way to use this matrix in an OpenGL program is :
		 * glMatrixMode(GL_MODELVIEW);
		 * glLoadMatrixf(GLMMat);
		 * \param GLMMat the computed GL projection matrix
        	 * \author Frankoi
		 */
		void getGLModelviewMatrix(float GLMMat[16]) const;

		/**
		 * \brief compute the principal ray (axis) of the camera corresponding to the ray passing through the camera center C with direction vector m3T (3rd row of M=KR), this principal ray is represented by the axis vector v = det(M)m3T that is directed towards the front of the camera.
                 * \author Vincent
		 */
		kn::Vector3d principalRay() const;


	};


/*
 * End of Anti-doublon
 */
#endif
