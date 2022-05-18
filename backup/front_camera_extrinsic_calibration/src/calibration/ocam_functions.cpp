/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ocam_functions.h"
#include <float.h>
#include <highgui.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//------------------------------------------------------------------------------
void cam2world(double point3D[3], double point2D[2],
               nullmax_perception::OcamModel &ocam_model) {
  std::vector<double> &pol = ocam_model.pol;
  double xc = (ocam_model.xc);
  double yc = (ocam_model.yc);
  double c = (ocam_model.c);
  double d = (ocam_model.d);
  double e = (ocam_model.e);
  int length_pol = (ocam_model.length_pol);
  double invdet =
      1 / (c - d * e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

  double xp = invdet * ((point2D[0] - xc) - d * (point2D[1] - yc));
  double yp = invdet * (-e * (point2D[0] - xc) + c * (point2D[1] - yc));

  double r =
      sqrt(xp * xp +
           yp * yp); // distance [pixels] of  the point from the image center
  double zp = pol[0];
  double r_i = 1;
  int i;

  for (i = 1; i < length_pol; i++) {
    r_i *= r;
    zp += r_i * pol[i];
  }
  zp = -zp; // no test before

  // normalize to unit norm
  double invnorm = 1 / sqrt(xp * xp + yp * yp + zp * zp);

  point3D[0] = invnorm * xp;
  point3D[1] = invnorm * yp;
  point3D[2] = invnorm * zp;
}

//------------------------------------------------------------------------------
void world2cam(double point2D[2], double point3D[3],
               nullmax_perception::OcamModel &ocam_model) {
  std::vector<double> &invpol = ocam_model.invpol;
  double xc = (ocam_model.xc);
  double yc = (ocam_model.yc);
  double c = (ocam_model.c);
  double d = (ocam_model.d);
  double e = (ocam_model.e);
  int length_invpol = (ocam_model.length_invpol);
  double norm = sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1]);
  double theta = atan(point3D[2] / norm);
  double t, t_i;
  double rho, x, y;
  double invnorm;
  int i;

  if (norm != 0) {
    invnorm = 1 / norm;
    t = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++) {
      t_i *= t;
      rho += t_i * invpol[i];
    }

    x = point3D[0] * invnorm * rho;
    y = point3D[1] * invnorm * rho;

    point2D[0] = x * c + y * d + xc;
    point2D[1] = x * e + y + yc;
  } else {
    point2D[0] = xc;
    point2D[1] = yc;
  }
}
//------------------------------------------------------------------------------
void create_perspecive_undistortion_LUT(
    cv::Mat &mapx, cv::Mat &mapy, nullmax_perception::OcamModel &ocam_model,
    const float &focal, const float &cx, const float &cy) {
  int width = mapx.cols;  // New width
  int height = mapx.rows; // New height
  float *data_mapx = mapx.ptr<float>(0);
  float *data_mapy = mapy.ptr<float>(0);
  float Nxc = cx;
  float Nyc = cy;
  float Nz = focal;
  double M[3];
  double m[2];

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      M[0] = (col - Nxc);
      M[1] = (row - Nyc);
      M[2] = -Nz;
      world2cam(m, M, ocam_model);
      *(data_mapx + row * width + col) = (float)m[0];
      *(data_mapy + row * width + col) = (float)m[1];
    }
  }
}

//------------------------------------------------------------------------------
// void create_panoramic_undistortion_LUT ( CvMat *mapx, CvMat *mapy, float
// Rmin, float Rmax, float xc, float yc )
// {
//      int i, j;
//      float theta;
//      int width = mapx->width;
//      int height = mapx->height;
//      float *data_mapx = mapx->data.fl;
//      float *data_mapy = mapy->data.fl;
//      float rho;

//      for (i=0; i<height; i++)
//          for (j=0; j<width; j++)
//          {
//              theta = -((float)j)/width*2*M_PI; // Note, if you would like to
//              flip the image, just inverte the sign of theta rho   = Rmax -
//              (Rmax-Rmin)/height*i;
//              *( data_mapx + i*width+j ) = yc + rho*sin(theta); //in OpenCV
//              "x" is the
//              *( data_mapy + i*width+j ) = xc + rho*cos(theta);
//          }
// }
// TODO: depart fisheye from pinhole