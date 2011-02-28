/*************************************************************************** \
 * Copyright (C) by University Paris-Est - MISS team
 * ProjectiveCamera.cpp created in 01 2009.
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
 * External Includes
 */
#include <cstring>



/*
 * Internal Includes
 */

#include "ProjectiveCamera.hpp"
#include <OpenKN/vision/VisionException.hpp>
//#include <OpenKN/vision/EpipolarGeometry.hpp>
#include <OpenKN/math/SVD.hpp>
#include <OpenKN/math/InverseMatrix.hpp>
#include <OpenKN/math/Determinant.hpp>
#include <OpenKN/math/RQDecomposition.hpp>




	/*
	 * \brief Default Constructor (Set ProjectiveCamera as Identity)
	 */
	ProjectiveCamera::ProjectiveCamera():matP(3,4){
	  matP.setIdentity();
	  matR.setIdentity();
	  matK.setIdentity();
	  matKinverse.setIdentity();
	  vecC.setZero();
	  vecC[3] = 1.0;
	}


	/* \brief Copy Constructor
	* \param myProjectiveCamera : Source projective camera
	*/
	ProjectiveCamera::ProjectiveCamera(const ProjectiveCamera &myProjectiveCamera)
		:matP(myProjectiveCamera.matP),
		 matK(myProjectiveCamera.matK),
		 matKinverse(myProjectiveCamera.matKinverse),
		 matR(myProjectiveCamera.matR),
		 vecC(myProjectiveCamera.vecC){
	}

	/*
	* \brief constructor from a projection matrix P.
	* This function performs the following decomposition : P = K[R|-RC].
	* \param myP : a 3x4 projection matrix
	* \throw VisionException argument P matrix : invalid size
	*/
	ProjectiveCamera::ProjectiveCamera(const kn::Matrix<double>& myP)
		:matP(myP){
		// check P size
		if(matP.rows() != 3 || matP.columns()!= 4)
			throw kn::VisionException("invalid size : the input P matrix is not a 3x4 matrix\nTo use the ProjectiveCamera constructor with a K matrix, use kn::Matrix3x3d");

		// extract K, R, C from P
		decompose();

		// inverse K
		matKinverse = kn::inverseMatrixSVD(matK);
	}


	/*
	* \brief constructor from all the required matrices : P = K[R|-RC].
	* \param myK : a 3x3 internal parameters matrix.
	* \param myR : a 3x3 rotation matrix.
	* \param myC : a 4 homogeneous vector (camera centre coordinates).
	* \throw VisionException argument : invalid size
	*/
	ProjectiveCamera::ProjectiveCamera(const kn::Matrix3x3<double> &myK,
					   const kn::Matrix3x3<double> &myR,
					   const kn::Vector4d   &myC)
		:matP(3,4), matK(myK), matR(myR), vecC(myC){

		// homogeneous normal form
		vecC.setHomogeneousNormalForm();

		// P
		matP = compose(matK, matR, vecC);

		// inverse K
		matKinverse = kn::inverseMatrixSVD(matK);
	}


	/*
	* \brief constructor from an internal parameter matrix. The projection matrix will be P=K[Id|0]
	* \param myK : a 3x3 internal parameters matrix.
	* \throw VisionException argument : invalid size
	*/
	ProjectiveCamera::ProjectiveCamera(const kn::Matrix3x3<double> &myK)
		:matP(3,4),matK(myK){

		// K inverse
		matKinverse = kn::inverseMatrixSVD(matK);

		// rotation
		matR.setIdentity();

		// translation
		vecC.setZero();
		vecC[3] = 1.0;

		// P
		matP = compose(matK, matR, vecC);
	}


	/*
	* \brief destructor
	*/
	ProjectiveCamera::~ProjectiveCamera(){
	}


	/*
	* \brief compute P from K, R and C : P = K[R|-RC].
	*/
	kn::Matrix<double> ProjectiveCamera::compose(const kn::Matrix3x3<double> &K,
						     const kn::Matrix3x3<double> &R,
						     const kn::Vector4d   &C) const{
		// P = K[R|-RC]
		kn::Matrix<double> P(3,4);

		// R contribution
		for(unsigned int i=0; i<3; i++)
			for(unsigned int j=0; j<3; j++)
				P[i][j] = R[i][j];

		// -RC contribution
		P.setColumn(3,-R * C.getUnhomogeneous());

		// K contribution
		P = K * P;

		return P;
	}


	/*
	* \brief extract K, R and C from P :
	* P = [M|-MC] et M = KR from Multiple View Geometry, Hartley Zisserman, camera model
	*/
	void ProjectiveCamera::decompose(){
		// M
		kn::Matrix<double> M(3,3);
		for(unsigned int i=0; i<3; i++)
			for(unsigned int j=0; j<3; j++)
				M[i][j] = matP[i][j];

		// K and R
		kn::rqDecomposition3x3(M,matK,matR); /// \todo à tester
		kn::rq3x3MakePositiveDiagonal(matK,matR); /// \todo à tester

		// we set K[3][3] to 1 (cf Multiple view geometry page 143 "finite camera")
		matK = matK / matK[2][2];
		matK.roundZero(1.0e-10);

		// camera centre
		extractCentre();
	}

	/*
	* \brief extract C from P
	*/
	void ProjectiveCamera::extractCentre(){
		// matrix used for every component extraction
		kn::Matrix<double> det(3,3);

		// x
		det[0][0]=matP[0][1]; det[0][1]=matP[0][2]; det[0][2]=matP[0][3];
		det[1][0]=matP[1][1]; det[1][1]=matP[1][2]; det[1][2]=matP[1][3];
		det[2][0]=matP[2][1]; det[2][1]=matP[2][2]; det[2][2]=matP[2][3];
		vecC[0] = kn::determinant3x3(det);

		// y
		det[0][0]=matP[0][0]; det[0][1]=matP[0][2]; det[0][2]=matP[0][3];
		det[1][0]=matP[1][0]; det[1][1]=matP[1][2]; det[1][2]=matP[1][3];
		det[2][0]=matP[2][0]; det[2][1]=matP[2][2]; det[2][2]=matP[2][3];
		vecC[1] = -kn::determinant3x3(det);

		// z
		det[0][0]=matP[0][0]; det[0][1]=matP[0][1]; det[0][2]=matP[0][3];
		det[1][0]=matP[1][0]; det[1][1]=matP[1][1]; det[1][2]=matP[1][3];
		det[2][0]=matP[2][0]; det[2][1]=matP[2][1]; det[2][2]=matP[2][3];
		vecC[2] = kn::determinant3x3(det);

		// w
		det[0][0]=matP[0][0]; det[0][1]=matP[0][1]; det[0][2]=matP[0][2];
		det[1][0]=matP[1][0]; det[1][1]=matP[1][1]; det[1][2]=matP[1][2];
		det[2][0]=matP[2][0]; det[2][1]=matP[2][1]; det[2][2]=matP[2][2];
		vecC[3] = -kn::determinant3x3(det);

		// homogeneous normal form
		vecC.setHomogeneousNormalForm();
	}


	/*
	* \brief Set a default camera intrinsic parameter matrix K, (i.e. centered optical ray, standard focal lenght, squared pixels)
	*/
	void ProjectiveCamera::defaultK(const unsigned int imageWidth, const unsigned imageHeight){

		// compute the default K
		matK.setIdentity();
		double focal = sqrt((double)imageWidth*(double)imageWidth + (double)imageHeight*(double)imageHeight);
		matK[0][0] = matK[1][1] = focal;
		matK[0][2] = imageWidth  * 0.5;
		matK[1][2] = imageHeight * 0.5;

		// update the camera parameters
		matP = compose(matK, matR, vecC);

		// inverse K
		matKinverse = kn::inverseMatrixSVD(matK);
	}


	/*
	 * \brief return M Matrix (M=KR such P=[M|-MC]) (read-only)
	 */
	 kn::Matrix<double> ProjectiveCamera::M() const{
		kn::Matrix<double> M(3,3);

		// build M
		for(unsigned int i=0; i<3; i++)
			for(unsigned int j=0; j<3; j++)
				M[i][j] = matP[i][j];

		return M;
	}



	/**
	 * \brief return extrinsic Matrix ([R|-RC]) (read-only)
	 */
	kn::Matrix<double> ProjectiveCamera::extrinsicMatrix() const{
		kn::Matrix4x4<double> M(0.0);

		for(unsigned int i=0; i<3; i++)
			for(unsigned int j=0; j<3; j++)
				M[i][j] = matR[i][j];
		M(3,3) = 1.0;
		kn::Vector4d C = -M*(vecC/vecC[3]);
		M.setColumn(3,C);
		kn::Matrix<double> extrinsic(3,4);
		for(unsigned int i=0; i<4; i++)
			for(unsigned int j=0; j<3; j++)
				extrinsic[j][i] = M[j][i];
		return extrinsic;
	}

	/*
	 * update the camera external parameters
	 * Pcanonic : 4x3 matrix : Pcanonic=[R|-RC] (C=camera position)
	 */
	void ProjectiveCamera::updateRC(const kn::Matrixd &Pcanonic){
		if(Pcanonic.rows()!=3 && Pcanonic.columns()!=4)
		  throw kn::VisionException("updateRC : Pcanonic should be a 3x4 matrix");

		matP = matK * Pcanonic;
		decompose();
	}


	/*
	 * rescale the camera internal parameters
	 * factor : factor of rescaling
	 */
	void ProjectiveCamera::rescaleK(const double &factor){

		// scale K
		matK*=factor;
		matK(2,2)=1.0;

		// update the camera parameters
		matP = compose(matK, matR, vecC);

		// inverse K
		matKinverse = kn::inverseMatrixSVD(matK);
	}


	/*
	 * update the camera projection matrix parameters
	 * P : 4x3 matrix : P=K[R|-RC] (C=camera position)
	 */
	void ProjectiveCamera::updateP(const kn::Matrixd &P){
		// check P size
		if(P.rows() != 3 && P.columns()!= 4)
			throw kn::VisionException("invalid size : the input P matrix is not a 3x4 matrix");

		matP = P;

		// extract K, R, C from P
		decompose();

		// inverse K
		matKinverse = kn::inverseMatrixSVD(matK);
	 }


	/*
	 * \brief update the camera internal parameters matrix
	 * \param K : 3x3 matrix : P=K[R|-RC] (C=camera position)
	 * \throw VisionException argument : invalid size
	 */
	void ProjectiveCamera::updateK(const kn::Matrixd& K){
		// check K size
		if(K.rows() != 3 && K.columns()!= 3)
			throw kn::VisionException("invalid size : the input K matrix is not a 3x3 matrix");

		matK = K;

		// compose P from R, C and K
		matP = compose(matK, matR, vecC);

		// inverse K
		matKinverse = kn::inverseMatrixSVD(matK);
	}

	/*
	 * \brief compute the GL projection matrix
	 * The way to use this matrix in an OpenGL program is :
	 * glMatrixMode(GL_PROJECTION);
	 * glLoadMatrixf(GLPMat);
	 */
	void ProjectiveCamera::getGLProjectionMatrix(const size_t& camerawidth,
						     const size_t& cameraheight,
						     const float& glnear,
						     const float& glfar,
						     float GLPMat[16]) const{
		// Reset the GL matrix
		memset(GLPMat,0,16*sizeof(float));

		// Original internal parameters matrix is
		// fx skew Cx
		// 0  fy   Cy
		// 0  0	   1
		//
		// OpenGL projection Matrix is
		// 2n/(r-l) 0		 (r+l)/(r-l)   0
		// 0		2n/(t-b) (t+b)/(t-b)   0
		// 0		0		  -(f+n)/(f-n) -2fn/(f-n)
		// 0		0		  -1		   0
		// Keep in mind that GL matrix are transposed and r,l,b and t are in (-1,1) range
		// Y = height-y we also have to negate y parameters (elements 5 and 9)

		GLPMat[0] = float(matK[0][0])*2.0f / float(camerawidth);    // fx -> (-1,1)
		GLPMat[4] = float(matK[0][1])*2.0f / float(camerawidth);    // Skew
		GLPMat[5] = -float(matK[1][1])*2.0f / float(cameraheight);  // fy -> (-1,1)
		GLPMat[8] = 1.0f - 2.0f*float(matK[0][2]) / float(camerawidth);
		GLPMat[9] = 2.0f*float(matK[1][2]) / float(cameraheight)-1.0f;
		GLPMat[10] = -(glnear+glfar)/(glfar-glnear);
		GLPMat[11] = -1.0;
		GLPMat[14] = -2.0*(glnear*glfar)/(glfar-glnear);

	}


	/*
	 * \brief compute the GL modelview matrix
	 * The way to use this matrix in an OpenGL program is :
	 * glMatrixMode(GL_MODELVIEW);
	 * glLoadMatrixf(GLMMat);
	 */
	void ProjectiveCamera::getGLModelviewMatrix(float GLMMat[16]) const{

		kn::Vector3d tmp = -matR*kn::Vector3d(vecC.getUnhomogeneous());

		// Original internal parameters matrix is
		// R1x R1y R1z Tx
		// R2x R2y R2z Ty
		// R3x R3y R3z Tz
		//
		// OpenGL modelview Matrix (transposed version) is
		//
		// R1x R2x R3x 0
		// R1y R2y R3y 0
		// R1z R2z R3z 0
		// Tx  Ty  Tz  1
		//
		// Since original referential is not direct we have to invert the third column of the GL matrix
		//
		// R2x  R1x -R3x 0
		// R2y  R1y -R3y 0
		// R2z  R1z -R3z 0
		// Ty   Tx  -Tz  1

		GLMMat[0] = matR[0][0];
		GLMMat[1] = matR[1][0];
		GLMMat[2] = -matR[2][0];
		GLMMat[3] = 0.0;
		GLMMat[4] = matR[0][1];
		GLMMat[5] = matR[1][1];
		GLMMat[6] = -matR[2][1];
		GLMMat[7] = 0.0;
		GLMMat[8] = matR[0][2];
		GLMMat[9] = matR[1][2];
		GLMMat[10] = -matR[2][2];
		GLMMat[11] = 0.0;
		GLMMat[12] = tmp[0];
		GLMMat[13] = tmp[1];
		GLMMat[14] = -tmp[2];
		GLMMat[15] = 1.0;
	}



      /*
	* \brief compute the principal ray (axis) of the camera corresponding to the ray passing through the camera center C with direction vector m3T (3rd row of M=KR), this principal ray is represented by the axis vector v = det(M)m3T that is directed towards the front of the camera.
        * \author Vincent
	*/
	kn::Vector3d ProjectiveCamera::principalRay() const{
	  kn::Matrix3x3d M(this->M());
	  return kn::Vector3d(kn::determinant3x3(M) * M.getRow(2));
	}


