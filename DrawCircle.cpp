/***************************************************************************\
 * Copyright (C) by University Paris-Est - MISS team
 * DrawLine.cpp created in 02 2010.
 * Mail : biri@univ-mlv.fr
 *
 * This file is part of the OpenKraken-image.
 *
 * The OpenKraken-image is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * The OpenKraken-image is distributed in the hope that it will be useful,
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
#include <cmath>
#include <cassert>

/*
 * Internal Includes
 */
#include "DrawCircle.hpp"

	void drawCircle(kn::ImageRGB8u &image,
			const int xCenter,
			const int yCenter,
			const double radius,
			const unsigned char r,
			const unsigned char g,
			const unsigned char b){

		int j = (int)radius;
		double d = 0.25 - radius;
		
		for(int i=0; i<(radius/std::sqrt(2.0)); ++i){
			circlesPlotPoints(image, xCenter, yCenter, i, j, r,g,b);
			d += 2*i+1;
			
			if(d>0){
				d += 2-2*j;
				--j;
			}
		}
	}



	void drawCircle(kn::ImageGS8u &image,
			const int xCenter,
			const int yCenter,
			const double radius,
			const unsigned char greyLevel){

		int j = (int)radius;
		double d = 0.25 - radius;
		
		for(int i=0; i<(radius/std::sqrt(2.0)); ++i){
			circlesPlotPoints(image, xCenter, yCenter, i, j, greyLevel);
			d += 2*i+1;
			
			if(d>0){
				d += 2-2*j;
				--j;
			}
		}
	}


	void circlesPlotPoints(kn::ImageRGB8u &image,
			       const int &xCenter,
			       const int &yCenter,
			       const int x, const int y,
			       const unsigned char r,
			       const unsigned char g,
			       const unsigned char b){

		if(yCenter+y>=0 && yCenter+y<(int)image.height() && xCenter+x>=0 && xCenter+x<(int)image.width()){
			image(xCenter+x, yCenter+y)[0] = r;
			image(xCenter+x, yCenter+y)[1] = g;
			image(xCenter+x, yCenter+y)[2] = b;
		}

		if(yCenter-y>=0 && yCenter-y<(int)image.height() && xCenter+x>=0 && xCenter+x<(int)image.width()){
			image(xCenter+x, yCenter-y)[0] = r;
			image(xCenter+x, yCenter-y)[1] = g;
			image(xCenter+x, yCenter-y)[2] = b;
		}

		if(yCenter+y>=0 && yCenter+y<(int)image.height() && xCenter-x>=0 && xCenter-x<(int)image.width()){
			image(xCenter-x, yCenter+y)[0] = r;
			image(xCenter-x, yCenter+y)[1] = g;
			image(xCenter-x, yCenter+y)[2] = b;
		}

		if(yCenter-y>=0 && yCenter-y<(int)image.height() && xCenter-x>=0 && xCenter-x<(int)image.width()){
			image(xCenter-x, yCenter-y)[0] = r;
			image(xCenter-x, yCenter-y)[1] = g;
			image(xCenter-x, yCenter-y)[2] = b;
		}

		if(yCenter+x>=0 && yCenter+x<(int)image.height() && xCenter+y>=0 && xCenter+y<(int)image.width()){
			image(xCenter+y, yCenter+x)[0] = r;
			image(xCenter+y, yCenter+x)[1] = g;
			image(xCenter+y, yCenter+x)[2] = b;
		}

		if(yCenter-x>=0 && yCenter-x<(int)image.height() && xCenter+y>=0 && xCenter+y<(int)image.width()){
			image(xCenter+y, yCenter-x)[0] = r;
			image(xCenter+y, yCenter-x)[1] = g;
			image(xCenter+y, yCenter-x)[2] = b;
		}

		if(yCenter+x>=0 && yCenter+x<(int)image.height() && xCenter-y>=0 && xCenter-y<(int)image.width()){
			image(xCenter-y, yCenter+x)[0] = r;
			image(xCenter-y, yCenter+x)[1] = g;
			image(xCenter-y, yCenter+x)[2] = b;
		}

		if(yCenter-x>=0 && yCenter-x<(int)image.height() && xCenter-y>=0 && xCenter-y<(int)image.width()){
			image(xCenter-y, yCenter-x)[0] = r;
			image(xCenter-y, yCenter-x)[1] = g;
			image(xCenter-y, yCenter-x)[2] = b;
		}
	}


	void circlesPlotPoints(kn::ImageGS8u &image,
			       const int &xCenter,
			       const int &yCenter,
			       const int x, const int y,
			       const unsigned char greyLevel){

		if(yCenter+y>=0 && yCenter+y<(int)image.height() && xCenter+x>=0 && xCenter+x<(int)image.width())
			image(xCenter+x, yCenter+y) = greyLevel;

		if(yCenter-y>=0 && yCenter-y<(int)image.height() && xCenter+x>=0 && xCenter+x<(int)image.width())
			image(xCenter+x, yCenter-y) = greyLevel;

		if(yCenter+y>=0 && yCenter+y<(int)image.height() && xCenter-x>=0 && xCenter-x<(int)image.width())
			image(xCenter-x, yCenter+y) = greyLevel;

		if(yCenter-y>=0 && yCenter-y<(int)image.height() && xCenter-x>=0 && xCenter-x<(int)image.width())
			image(xCenter-x, yCenter-y) = greyLevel;

		if(yCenter+x>=0 && yCenter+x<(int)image.height() && xCenter+y>=0 && xCenter+y<(int)image.width())
			image(xCenter+y, yCenter+x) = greyLevel;

		if(yCenter-x>=0 && yCenter-x<(int)image.height() && xCenter+y>=0 && xCenter+y<(int)image.width())
			image(xCenter+y, yCenter-x) = greyLevel;

		if(yCenter+x>=0 && yCenter+x<(int)image.height() && xCenter-y>=0 && xCenter-y<(int)image.width())
			image(xCenter-y, yCenter+x) = greyLevel;

		if(yCenter-x>=0 && yCenter-x<(int)image.height() && xCenter-y>=0 && xCenter-y<(int)image.width())
			image(xCenter-y, yCenter-x) = greyLevel;
	}


	void drawDisc(kn::ImageRGB8u &image,
			const int xCenter,
			const int yCenter,
			const double radius,
			const unsigned char r,
			const unsigned char g,
			const unsigned char b){

		int xMin = (xCenter-radius) < 0 ? 0 : (xCenter-radius);
		int yMin = (yCenter-radius) < 0 ? 0 : (yCenter-radius);
		int xMax = (xCenter+radius) < image.width() ? (xCenter+radius) : image.width();
		int yMax = (yCenter+radius) < image.height()  ? (yCenter+radius) : image.height();
		double r2 = radius*radius;
	
		for(int x=xMin; x<xMax; x++){
			int rx = std::abs(xCenter - x);
			rx = rx * rx;
			for(int y=yMin; y<yMax; y++){
				int ry = std::abs(yCenter - y);
					
				if(rx + ry*ry <= r2){
					image(x,y)[0] = r;
					image(x,y)[1] = g;
					image(x,y)[2] = b;
				}
			}
		}

	}


	void drawDisc(kn::ImageGS8u &image,
		      const int xCenter,
		      const int yCenter,
		      const double radius,
		      const unsigned char greyLevel){

		int xMin = (xCenter-radius) < 0 ? 0 : (xCenter-radius);
		int yMin = (yCenter-radius) < 0 ? 0 : (yCenter-radius);
		int xMax = (xCenter+radius) < image.width() ? (xCenter+radius) : image.width();
		int yMax = (yCenter+radius) < image.height()  ? (yCenter+radius) : image.height();
		double r2 = radius*radius;

		for(int x=xMin; x<xMax; ++x){
			int rx = std::abs(xCenter - x);
			rx = rx * rx;
			for(int y=yMin; y<yMax; ++y){
				int ry = std::abs(yCenter - y);
				if(rx + ry*ry <= r2)
					image(x,y) = greyLevel;
			}
		}
	}


