/*************************************************************************** \
* Copyright (C) by University Paris-Est - MISS team
* CameraCalibrationZhang.cpp created in 04 2010.
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




/*
* Internal Includes
*/
#include <OpenKN/math/SVD.hpp>
#include <OpenKN/math/Solver.hpp>

#include "CameraCalibrationZhang.hpp"
#include <OpenKN/vision/VisionException.hpp>





	/*
	* \brief transform the matrix R into the "nearest" orthogonal matrix
	* \param R : A nearly rotation matrix.
	*/
	void rotationEnforce(kn::Matrix<double> &R){
	  kn::Matrix<double> V(R.rows(), R.columns());
	  kn::Vector<double> D(R.columns());

	  kn::decompositionSVD(R,D,V); // R = R * D * V
	  V.transpose();
	  R = R * V;	// we change D to identity
	}


	/*
	* \brief compute the intrinsic parameters of the camera from a list of homographies (Zhang method). In this function, we use Zhang notation.
	* \param Hlist : a list of 3x3 homography matrices (at least 3).
	* \throw VisionException not enought homographies
	*/
	kn::Matrix3x3d cameraCalibrationZhang(const std::vector<kn::Matrix3x3d> &Hlist){
	  // check the number of homographies
	  if(Hlist.size() < 3)
	    throw kn::VisionException("not enough homographies to calibrate the camera");

	  // calibration step (cf Zhang articles)
	  // one additional line to handle zero skew
	  kn::Matrix<double> V(2*Hlist.size()+1, 6);
	  V.setZero();
	  makeSystemZhang(V,Hlist,true);

	  kn::Vector<double> b(6);
	  kn::solveNullSystemSVD(V, b);
	  b.setHomogeneousNormalForm();

	  // extract parameters (K matrix)
	  return composeInternalParameterZhang(b);
	}


	/*
	* \brief Build the system to be solved to compute the intrinsic parameter matrix K with Zhang method.
	* \param V : the matrix to be built
	* \param Hlist : a list of homographies (at least 3)
	* \param zeroSkew : true if we want to force zero skew
	*/
	void makeSystemZhang(kn::Matrix<double> &V,
		             const std::vector<kn::Matrix3x3d> &Hlist,
			     const bool zeroSkew){

	  std::vector<kn::Matrix3x3d>::const_iterator iter = Hlist.begin();
	  int i = 0;

	  while(iter != Hlist.end())
	  {
	    V.setRow(i,vZhang(*iter,0,1));
	    ++i;
	    V.setRow(i,vZhang(*iter,0,0)-vZhang(*iter,1,1));
	    ++i;
	    ++iter;
	  }

	  // zeroSkew
	  if(zeroSkew) V[V.rows()-1][1] = 1.0;
	}


	/*
	* \brief returns a Vector3d : cf Zhang article and Zhang notation
	* \param h : a matrix
	* \param i : matrix index
	* \param j : matrix index
	*/
	kn::Vector<double> vZhang(const kn::Matrix<double> &h,
				  const int i,
				  const int j){
	  kn::Vector<double> v(6);
	  v[0] = h[0][i] * h[0][j];
	  v[1] = h[0][i] * h[1][j] + h[1][i] * h[0][j];
	  v[2] = h[1][i] * h[1][j];
	  v[3] = h[2][i] * h[0][j] + h[0][i] * h[2][j];
	  v[4] = h[2][i] * h[1][j] + h[1][i] * h[2][j];
	  v[5] = h[2][i] * h[2][j];

	  return v;
	}


	/*
	* \brief build the intrinsic parameter matrix K from a vector b (Zhang notation).
	* \param b : a vector to be converted into an internal parameter 3x3 matrix
	*/
	kn::Matrix3x3<double> composeInternalParameterZhang(const kn::Vector<double> &b){
	  // B = A-T.A-1
	  kn::Matrix<double> B(3,3);
	  B[0][0] = b[0];
	  B[0][1] = B[1][0] = b[1];
	  B[1][1] = b[2];
	  B[0][2] = B[2][0] = b[3];
	  B[1][2] = B[2][1] = b[4];
	  B[2][2] = b[5];

	  // parameters extraction (cf Zhang article appendix)
	  kn::Matrix3x3d K;
	  K.setIdentity();

	  K[1][2]	 = (B[1][0]*B[2][0] - B[0][0]*B[2][1]) / (B[0][0]*B[1][1] - B[1][0]*B[1][0]);
	  double l = B[2][2] - (B[2][0]*B[2][0] + K[1][2]*(B[1][0]*B[2][0] - B[0][0]*B[2][1]))/B[0][0];
	  K[0][0]	 = l/B[0][0] > 0 ? sqrt(l/B[0][0]) : sqrt(-l/B[0][0]);
	  K[1][1]	 = l*B[0][0] / (B[0][0]*B[1][1] - B[1][0]*B[1][0]);
	  K[1][1]	 = K[1][1] > 0 ? sqrt(K[1][1]) : sqrt(-K[1][1]);
	  K[0][1]	 = -B[1][0] * K[0][0]*K[0][0] * K[1][1] / l;
	  K[0][2]	 = (K[0][1]*K[1][2] / K[1][1]) - (B[2][0] *K[0][0]*K[0][0] / l);

	  return K;
	}


	/*
	* \brief compute the new position C and orientation R of the camera (Zhang method).
	* \param H : A 3x3 homography matrix.
	* \throw VisionException argument : invalid size
	*/
	void computeExternalParametersZhang(const kn::Matrix3x3<double> &H,
					    const kn::Matrix3x3<double> &matKinverse,
					    kn::Matrix3x3d &matR,
					    kn::Vector4d &vecC){

	  // scale factor and variables
	  kn::Vector3d h1(H.getColumn(0));
	  kn::Vector3d h2(H.getColumn(1));
	  kn::Vector3d h3(H.getColumn(2));
	  double l = 0.5 * (1.0 / (matKinverse*h1).getNorm()) + 0.5 * (1.0 / (matKinverse*h2).getNorm());

	  // rotation matrix
	  matR.setColumn(0, matKinverse * h1 * l);
		matR.setColumn(1, matKinverse * h2 * l);
		matR.setColumn(2, (matR.getColumn(0))^(matR.getColumn(1)));
	  rotationEnforce(matR);

	  // translation vector
	  kn::Vector3d vecC3 = (matKinverse * h3 * l);

	  // Compute C thanks to t = -RC
	  vecC3 = (-matR.getTranspose()*vecC3);
		vecC[0] = vecC3[0];
		vecC[1] = vecC3[1];
		vecC[2] = vecC3[2];
		vecC[3] = 1.0;
	  //vecC = (-matR.getTranspose()*vecC3).getHomogeneous(1.0);
	}


	/**
	* \brief compute the new position C and orientation R of the camera (Zhang method).
	* \param H : A 3x3 homography matrix.
	* \param camera : the projective camera to update.
	* \throw VisionException argument : invalid size
	*/
	void computeExternalParametersZhang(const kn::Matrix3x3<double> &H,
					    ProjectiveCamera &camera){

	  kn::Matrix3x3d R;
	  kn::Vector4d C;

	  computeExternalParametersZhang(H, camera.Kinverse(), R, C);

	  camera.updateRC(R,C);
	}





