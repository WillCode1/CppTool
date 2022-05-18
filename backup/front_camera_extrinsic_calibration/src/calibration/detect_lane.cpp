#include "detect_lane.h"

cv::Mat DetectLane(const cv::Mat img_my,
                   const nullmax_perception::RoiData &roi_data,
                   std::vector<cv::Point> &point_l,
                   std::vector<cv::Point> &point_r,
                   const int &canny_threadhold) {

  if (img_my.empty())
    printf("Image read error!\n");

  cv::Mat grad_x, img_canny;
  cv::Point point_tmp;
  cv::Mat img_median;
  cv::medianBlur(img_my, img_median, 7); // median filter
  cv::GaussianBlur(img_median, img_canny, cv::Size(3, 3), 0, 0,
                   cv::BORDER_DEFAULT);

  cv::Canny(img_canny, img_canny, 5, canny_threadhold,
            3); // canny operator to detect edge

  int width_top = roi_data.width_top < roi_data.width_bottom
                      ? roi_data.width_top
                      : roi_data.width_bottom;
  int width_bottom = roi_data.width_top > roi_data.width_bottom
                         ? roi_data.width_top
                         : roi_data.width_bottom;
  int height_min = roi_data.height_min < roi_data.height_max
                       ? roi_data.height_min
                       : roi_data.height_max;
  int height_max = roi_data.height_min > roi_data.height_max
                       ? roi_data.height_min
                       : roi_data.height_max;
  width_top = width_top < 0 ? 0 : width_top;
  width_top = width_top > img_canny.cols ? img_canny.cols : width_top;
  width_bottom = width_bottom < 0 ? 0 : width_bottom;
  width_bottom = width_bottom > img_canny.cols ? img_canny.cols : width_bottom;
  height_min = height_min < 0 ? 0 : height_min;
  height_min = height_min > img_canny.rows ? img_canny.rows : height_min;
  height_max = height_max < 0 ? 0 : height_max;
  height_max = height_max > img_canny.rows ? img_canny.rows : height_max;

  cv::Mat roi = cv::Mat::zeros(img_canny.size(), CV_8U);
  cv::Mat img_canny_roi;
  std::vector<std::vector<cv::Point>> roi_contour;
  std::vector<cv::Point> roi_pts;
  roi_pts.push_back(cv::Point((img_canny.cols - width_top) / 2, height_min));
  roi_pts.push_back(cv::Point((img_canny.cols + width_top) / 2, height_min));
  roi_pts.push_back(cv::Point((img_canny.cols + width_bottom) / 2, height_max));
  roi_pts.push_back(cv::Point((img_canny.cols - width_bottom) / 2, height_max));
  roi_contour.push_back(roi_pts);
  cv::drawContours(roi, roi_contour, -1, cv::Scalar::all(255), -1);
  img_canny.copyTo(img_canny_roi, roi);

  cv::Mat img_show_edge;
  cv::cvtColor(img_median, img_show_edge, CV_GRAY2RGB);

  cv::Scharr(img_median, grad_x, CV_8UC1, 1, 0, 1, 0, cv::BORDER_DEFAULT);
  cv::Mat lane_edge =
      cv::Mat::zeros(img_canny_roi.rows, img_canny_roi.cols, CV_8UC1);
  const int horizontal_edge_thred = 30;
  //--------select marking lane edge--------//
  //--------using sobel grident in x-axis with mask calculated above--------//
  int left_start_col = (img_canny.cols - width_bottom) / 2,
      left_end_col = img_canny_roi.cols / 2;
  int right_start_col = img_canny_roi.cols / 2 + 1,
      right_end_col = (img_canny.cols + width_bottom) / 2;
  // point_l : left img point after canny operator to fit lane.
  // point_r : same to above.
  for (int i = height_min; i < height_max; i++) {
    // left image
    for (int j_l = left_start_col; j_l < left_end_col; j_l++) {
      if (img_canny_roi.at<uchar>(i, j_l) < 200)
        continue;
      uchar grad_x_tmp = grad_x.at<uchar>(i, j_l);
      if (int(grad_x_tmp) < 100) {
        // discard horizontal line
        if (img_canny_roi.at<uchar>(i - 1, j_l - 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i - 1, j_l) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i - 1, j_l + 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_l - 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_l) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_l + 1) < horizontal_edge_thred) {
          img_show_edge.at<cv::Vec3b>(i, j_l) = cv::Vec3b(0, 0, 255);
          continue;
        }

        lane_edge.at<uchar>(i, j_l) = 255;
        img_show_edge.at<cv::Vec3b>(i, j_l) = cv::Vec3b(0, 255, 0);
        point_tmp.x = i;
        point_tmp.y = j_l;
        point_l.push_back(point_tmp);
        // point_l.push_back(cv::Point(i, j_l));
      }
    }
    // right image
    for (int j_r = right_start_col; j_r < right_end_col; j_r++) {
      if (img_canny_roi.at<uchar>(i, j_r) < 200)
        continue;
      uchar grad_x_tmp = grad_x.at<uchar>(i, j_r);
      if (int(grad_x_tmp) > 100) {
        // discard horizontal line
        if (img_canny_roi.at<uchar>(i - 1, j_r - 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i - 1, j_r) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i - 1, j_r + 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_r - 1) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_r) < horizontal_edge_thred &&
            img_canny_roi.at<uchar>(i + 1, j_r + 1) < horizontal_edge_thred) {
          img_show_edge.at<cv::Vec3b>(i, j_r) = cv::Vec3b(0, 0, 255);
          continue;
        }

        lane_edge.at<uchar>(i, j_r) = 255;
        img_show_edge.at<cv::Vec3b>(i, j_r) = cv::Vec3b(0, 255, 0);
        point_tmp.x = i;
        point_tmp.y = j_r;
        point_r.push_back(point_tmp);
      }
    }
  }
  cv::drawContours(img_show_edge, roi_contour, -1, cv::Scalar(255, 0, 0), 1);
  cv::resize(img_show_edge, img_show_edge,
             cv::Size(img_show_edge.cols / 2, img_show_edge.rows / 2));
  cv::imshow("lane_edge", img_show_edge);
  cv::waitKey(1);

  return lane_edge;
}
