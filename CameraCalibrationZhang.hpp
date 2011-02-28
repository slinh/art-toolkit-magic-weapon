/*************************************************************************** \
* Copyright (C) by University Paris-Est - MISS team
* CameraCalibrationZhang.hpp created in 04 2010.
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
#ifndef __OPENKN_VISION__CAMERA_CALIBRATION_ZHANG_HPP__
#define __OPENKN_VISION__CAMERA_CALIBRATION_ZHANG_HPP__

/*
* External Includes
*/
#include <vector>


/*
* Internal Includes
*/
#include <OpenKN/math/Matrix.hpp>
#include <OpenKN/math/Matrix3x3.hpp>
#include <OpenKN/math/Vector.hpp>

#include "ProjectiveCamera.hpp"


       
 
	/**
	* \brief transform the matrix R into the "nearest" orthogonal matrix
	* \param R : A nearly rotation matrix.
	*/
	void rotationEnforce(kn::Matrix<double> &R);

	/**
	* \brief compute the intrinsic parameters of the camera from a list of homographies (Zhang method). In this function, we use Zhang notation.
	* \param Hlist : a list of 3x3 homography matrices (at least 3).
	* \throw VisionException not enought homographies
	*/
	kn::Matrix3x3d cameraCalibrationZhang(const std::vector<kn::Matrix3x3d> &Hlist);
	
	/**
	* \cond
	* \brief build the intrinsic parameter matrix K from a vector b (Zhang notation).
	* \param b : a vector to be converted into an internal parameter 3x3 matrix
	*/
	kn::Matrix3x3<double> composeInternalParameterZhang(const kn::Vector<double> &b);
	/// \endcond

	/**
	* \cond
	* \brief returns a Vector3d : cf Zhang article and Zhang notation
	* \param V : a matrix
	* \param i : matrix index
	* \param j : matrix index
	*/
	kn::Vector<double> vZhang(const kn::Matrix<double> &V,
				  const int i,
				  const int j);
	/// \endcond
  
	/**
	* \cond
	* \brief Build the system to be solved to compute the intrinsic parameter matrix K with Zhang method.
	* \param V : the matrix to be built
	* \param Hlist : a list of homographies (at least 3)
	* \param zeroSkew : true if we want to force zero skew
	*/
	void makeSystemZhang(kn::Matrix<double> &V,
			     const std::vector<kn::Matrix3x3d> &Hlist,
			     const bool zeroSkew = true);
	/// \endcond

	/**
	* \brief compute the new position C and orientation R of the camera (Zhang method).
	* \param H : A 3x3 homography matrix.
	* \param Kinverse : 3x3 matrix : inverse of the camera internal parameter matrix K.
	* \param matR : the rotation matrix to be computed
	* \param vecC : the position of the camera to be computed
	* \throw VisionException argument : invalid size
	*/
	void computeExternalParametersZhang(const kn::Matrix3x3<double> &H,
					    const kn::Matrix3x3<double> &Kinverse,
					    kn::Matrix3x3d &matR,
					    kn::Vector3d &vecC);

	/**
	* \brief compute the new position C and orientation R of the camera (Zhang method).
	* \param H : A 3x3 homography matrix.
	* \param camera : the projective camera to update.
	* \throw VisionException argument : invalid size
	*/
	void computeExternalParametersZhang(const kn::Matrix3x3<double> &H,
					    ProjectiveCamera &camera);




/*
* End of Anti-doublon
*/
#endif
