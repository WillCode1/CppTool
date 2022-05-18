#include "ransac_lane.h"

const double kEps = 1e-6;

using namespace std;

void FitLine2d(const Point2D32f *points, const int count, const float *weights,
               float *line) {
  double x = 0, y = 0, x2 = 0, y2 = 0, xy = 0, w = 0;
  double dx2, dy2, dxy;
  float t;
  /* Calculating the average of x and y... */
  if (weights == 0) {
    for (int i = 0; i < count; i += 1) {
      x += points[i].x;
      y += points[i].y;
      x2 += points[i].x * points[i].x;
      y2 += points[i].y * points[i].y;
      xy += points[i].x * points[i].y;
    }
    w = float(count);
  } else {
    for (int i = 0; i < count; i += 1) {
      x += weights[i] * points[i].x;
      y += weights[i] * points[i].y;
      x2 += weights[i] * points[i].x * points[i].x;
      y2 += weights[i] * points[i].y * points[i].y;
      xy += weights[i] * points[i].x * points[i].y;
      w += weights[i];
    }
  }

  x /= w;
  y /= w;
  x2 /= w;
  y2 /= w;
  xy /= w;
  dx2 = x2 - x * x;
  dy2 = y2 - y * y;
  dxy = xy - x * y;
  t = float(atan2(2 * dxy, dx2 - dy2) / 2);
  line[0] = float(cos(t));
  line[1] = float(sin(t));
  line[2] = float(x);
  line[3] = float(y);
}

void FitLine2d(const Point2D32f *points, const int count, float *line) {
  // first fitting lane to get parameters of lane
  FitLine2d(points, count, NULL, line);
  // calculate weights and do fitting again
  float *dist = new float[count];
  float *w = new float[count];

  // weighted fitting iteratively
  // the number of iterations is not less than three
  for (int i = 0; i < 5; i++) {
    CalculateDistance2d(points, count, line, dist);
    WeightL1(dist, count, w);
    FitLine2d(points, count, w, line);
  }
  delete[] dist;
  delete[] w;
}

void WeightL1(const float *dist, const int count, float *weight) {
  for (int i = 0; i < count; i++) {
    double t = std::fabs((double)dist[i]);
    weight[i] = (1.0 / double(std::max(t, kEps)));
  }
}

double CalculateDistance2d(const Point2D32f *points, const int count,
                           const float *line, float *dist) {
  float px = line[2], py = line[3];
  float nx = line[1], ny = -line[0];
  double sum_dist = 0.;

  for (int j = 0; j < count; j++) {
    float x, y;
    x = points[j].x - px;
    y = points[j].y - py;

    dist[j] = (float)std::fabs(nx * x + ny * y);
    sum_dist += dist[j];
  }
  return sum_dist;
}

void RansacLane2d(const Point2D32f *points, const int cnt,
                  const int num_for_estimate, const float success_probability,
                  const float max_outliers_percentage, float *line_information,
                  const bool is_left_lane) {
  // 1 − p = (1 − w^n)^k
  // calculate the iterator number
  float numerator = std::log(1.0f - success_probability);
  float denominator = std::log(
      1.0f - std::pow(1.0 - max_outliers_percentage, num_for_estimate));
  // calculate the iterator number
  int ransac_times = int(numerator / denominator + 0.5) / 4;

  int num_data_objects = cnt;
  int max_vote_cnt = 0;
  float temp_line[4];
  float inliers_percentage = 0.0;

  int *chosen = new int[num_data_objects];

  Point2D32f *sub_points = new Point2D32f[num_for_estimate];
  int point_cnt = 0;
  int vote_cnt = 0;
  for (int i = 0; i < ransac_times; i++) {
    // randomly select data for exact model fit ('num_for_estimate' objects).
    memset(chosen, 0, num_data_objects * sizeof(int));
    for (int j = 0; j < num_for_estimate; j++) {
      if (!num_data_objects) {
        break;
      }
      int selected_index = rand() % num_data_objects;
      chosen[selected_index] = 1;
    }

    // fitting line
    point_cnt = 0;
    for (int k = 0; k < num_data_objects; k++) {
      if (chosen[k]) {
        sub_points[point_cnt].x = points[k].x;
        sub_points[point_cnt].y = points[k].y;
        point_cnt++;
      }
    }
    FitLine2d(sub_points, point_cnt, temp_line);

    float a = temp_line[1] / temp_line[0];
    float b = temp_line[3] - a * temp_line[2];

    if (is_left_lane) {
      if (a > -0.4 || a < -2)
        continue;
    } else {
      if (a < 0.4 || a > 2)
        continue;
    }
    vote_cnt = 0;
    for (int k = 0; k < cnt; k++) {
      if (abs(points[k].y - a * points[k].x - b) < 1.0)
        vote_cnt++;
    }

    if (vote_cnt > max_vote_cnt) {
      max_vote_cnt = vote_cnt;
      inliers_percentage = (float)max_vote_cnt / cnt;
      for (int m = 0; m < 4; m++) {
        line_information[m] = temp_line[m];
      }
    }
  }
  delete[] sub_points;
  delete[] chosen;
}
