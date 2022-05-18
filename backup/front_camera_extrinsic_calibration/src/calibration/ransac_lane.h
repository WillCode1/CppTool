#ifndef INCLUDE_RANSAC_LANE_H_
#define INCLUDE_RANSAC_LANE_H_

#include "parameter_struct.h"
#include <cmath>
void Fitline2d(const Point2D32f *points, const int count, float *line);
void Fitline2d(const Point2D32f *points, const int count, const float *weights,
               float *line);

void WeightL1(const float *dist, const int count, float *weight);
double CalculateDistance2d(const Point2D32f *points, const int count,
                           const float *line, float *dist);
void RansacLane2d(const Point2D32f *points, const int Cnt,
                  const int num_for_estimate, const float success_probability,
                  const float max_outliers_percentage, float *line_information,
                  const bool is_left_lane);

#endif // INCLUDE_RANSAC_LANE_H_
