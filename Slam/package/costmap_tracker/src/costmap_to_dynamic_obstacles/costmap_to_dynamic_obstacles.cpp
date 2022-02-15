#include <costmap_tracker/costmap_to_dynamic_obstacles/costmap_to_dynamic_obstacles.h>

#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

#include <iostream>
#include <fstream>

// filesysttem
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_tracker::CostmapToDynamicObstacles, costmap_tracker::BaseCostmapToPolygons)

namespace
{

bool matread(const std::string& filename, cv::Mat& mat,
                geometry_msgs::Point& origin, double& resolution)
{
  if(!boost::filesystem::exists(boost::filesystem::path(filename)) ||
     boost::filesystem::file_size(boost::filesystem::path(filename)) < 100)
    return false;

  try{
  using namespace boost::iostreams;
  std::ifstream fs(filename.c_str(), std::ios_base::in | std::fstream::binary);

  filtering_istream in;
  in.push(zlib_decompressor());
  in.push(fs);

  // Header
  int rows, cols, type, channels;
  in.read((char*)&rows, sizeof(int));         // rows
  in.read((char*)&cols, sizeof(int));         // cols
  in.read((char*)&type, sizeof(int));         // type
  in.read((char*)&channels, sizeof(int));     // channels
  in.read((char*)&origin.x, sizeof(double));
  in.read((char*)&origin.y, sizeof(double));
  in.read((char*)&resolution, sizeof(double));
  if(rows*cols < 100 || double(rows)*cols > 6e8 || resolution < 0.01 || resolution > 0.5)
    return false;

  // Data
  mat = cv::Mat(rows, cols, type);
  in.read((char*)mat.data, CV_ELEM_SIZE(type) * rows * cols);
  if(in.gcount() != CV_ELEM_SIZE(type) * rows * cols){
    ROS_WARN("%s incomplete", filename.c_str());
    return false;
  }
  }catch(...){
    return false;
  }

  return true;
}
}
namespace costmap_tracker
{

CostmapToDynamicObstacles::CostmapToDynamicObstacles() : BaseCostmapToDynamicObstacles()
{
  ego_vel_.x = ego_vel_.y = ego_vel_.z = 0;
  costmap_ = nullptr;
  dynamic_recfg_ = nullptr;
}

CostmapToDynamicObstacles::~CostmapToDynamicObstacles()
{
  if(dynamic_recfg_ != nullptr)
    delete dynamic_recfg_;
}

void CostmapToDynamicObstacles::initialize(ros::NodeHandle nh)
{
  costmap_ = nullptr;

  // We need the odometry from the robot to compensate the ego motion
  ros::NodeHandle gn; // create odom topic w.r.t. global node handle
  //odom_sub_ = gn.subscribe(odom_topic_, 1, &CostmapToDynamicObstacles::odomCallback, this);

  nh.param("publish_static_obstacles", publish_static_obstacles_, publish_static_obstacles_);

  map_sub_ = nh.subscribe("/map_disable", 1, &CostmapToDynamicObstacles::incomingMap, this);
  maps_path_sub_ = nh.subscribe("/map_deep_process/maps_path", 1, &CostmapToDynamicObstacles::mapsPathCallback, this);
  got_dist_map_ = false;

  //////////////////////////////////
  // Foreground detection parameters
  BackgroundSubtractor::Params bg_sub_params;

  bg_sub_params.alpha_slow = 0.3;
  nh.param("alpha_slow", bg_sub_params.alpha_slow, bg_sub_params.alpha_slow);

  bg_sub_params.alpha_fast = 0.85;
  nh.param("alpha_fast", bg_sub_params.alpha_fast, bg_sub_params.alpha_fast);

  bg_sub_params.beta = 0.85;
  nh.param("beta", bg_sub_params.beta, bg_sub_params.beta);

  bg_sub_params.min_occupancy_probability = 180;
  nh.param("min_occupancy_probability", bg_sub_params.min_occupancy_probability, bg_sub_params.min_occupancy_probability);

  bg_sub_params.min_sep_between_fast_and_slow_filter = 80;
  nh.param("min_sep_between_slow_and_fast_filter", bg_sub_params.min_sep_between_fast_and_slow_filter, bg_sub_params.min_sep_between_fast_and_slow_filter);

  bg_sub_params.max_occupancy_neighbors = 100;
  nh.param("max_occupancy_neighbors", bg_sub_params.max_occupancy_neighbors, bg_sub_params.max_occupancy_neighbors);

  bg_sub_params.morph_size = 1;
  nh.param("morph_size", bg_sub_params.morph_size, bg_sub_params.morph_size);

  bg_sub_ = std::unique_ptr<BackgroundSubtractor>(new BackgroundSubtractor(bg_sub_params));

  ////////////////////////////
  // Blob detection parameters
  BlobDetector::Params blob_det_params;

  blob_det_params.filterByColor = false; // actually filterByIntensity, always true
  blob_det_params.blobColor = 255;      // Extract light blobs
  blob_det_params.thresholdStep = 256;  // Input for blob detection is already a binary image
  blob_det_params.minThreshold = 127;
  blob_det_params.maxThreshold = 255;
  blob_det_params.minRepeatability = 1;

  blob_det_params.minDistBetweenBlobs = 10;
  nh.param("min_distance_between_blobs", blob_det_params.minDistBetweenBlobs, blob_det_params.minDistBetweenBlobs);

  blob_det_params.filterByArea = true;
  nh.param("filter_by_area", blob_det_params.filterByArea, blob_det_params.filterByArea);

  blob_det_params.minArea = 3; // Filter out blobs with less pixels
  nh.param("min_area", blob_det_params.minArea, blob_det_params.minArea);

  blob_det_params.maxArea = 300;
  nh.param("max_area", blob_det_params.maxArea, blob_det_params.maxArea);

  blob_det_params.filterByCircularity = false; // circularity = 4*pi*area/perimeter^2
  nh.param("filter_by_circularity", blob_det_params.filterByCircularity, blob_det_params.filterByCircularity);

  blob_det_params.minCircularity = 0.2;
  nh.param("min_circularity", blob_det_params.minCircularity, blob_det_params.minCircularity);

  blob_det_params.maxCircularity = 1; // maximal 1 (in case of a circle)
  nh.param("max_circularity", blob_det_params.maxCircularity, blob_det_params.maxCircularity);

  blob_det_params.filterByInertia = false; // Filter blobs based on their elongation
  nh.param("filter_by_intertia", blob_det_params.filterByInertia, blob_det_params.filterByInertia);

  blob_det_params.minInertiaRatio = 0.2;  // minimal 0 (in case of a line)
  nh.param("min_inertia_ratio", blob_det_params.minInertiaRatio, blob_det_params.minInertiaRatio);

  blob_det_params.maxInertiaRatio = 1;    // maximal 1 (in case of a circle)
  nh.param("max_intertia_ratio", blob_det_params.maxInertiaRatio, blob_det_params.maxInertiaRatio);

  blob_det_params.filterByConvexity = false; // Area of the Blob / Area of its convex hull
  nh.param("filter_by_convexity", blob_det_params.filterByConvexity, blob_det_params.filterByConvexity);

  blob_det_params.minConvexity = 0;          // minimal 0
  nh.param("min_convexity", blob_det_params.minConvexity, blob_det_params.minConvexity);

  blob_det_params.maxConvexity = 1;          // maximal 1
  nh.param("max_convexity", blob_det_params.maxConvexity, blob_det_params.maxConvexity);

  blob_det_ = BlobDetector::create(blob_det_params);

  ////////////////////////////////////
  // Tracking parameters
  CTracker::Params tracker_params;
  tracker_params.dt = 0.2;
  nh.param("dt", tracker_params.dt, tracker_params.dt);

  tracker_params.dist_thresh = 60.0;
  nh.param("dist_thresh", tracker_params.dist_thresh, tracker_params.dist_thresh);

  tracker_params.max_allowed_skipped_frames = 3;
  nh.param("max_allowed_skipped_frames", tracker_params.max_allowed_skipped_frames, tracker_params.max_allowed_skipped_frames);

  tracker_params.max_trace_length = 30;
  nh.param("max_trace_length", tracker_params.max_trace_length, tracker_params.max_trace_length);

  tracker_ = std::unique_ptr<CTracker>(new CTracker(tracker_params));


  ////////////////////////////////////
  // Static costmap conversion parameters
  std::string static_converter_plugin = "costmap_tracker::CostmapToPolygonsDBSMCCH";
  nh.param("static_converter_plugin", static_converter_plugin, static_converter_plugin);
  loadStaticCostmapConverterPlugin(static_converter_plugin, nh);


  // setup dynamic reconfigure
  dynamic_recfg_ = new dynamic_reconfigure::Server<CostmapToDynamicObstaclesConfig>(nh);
  dynamic_reconfigure::Server<CostmapToDynamicObstaclesConfig>::CallbackType cb = boost::bind(&CostmapToDynamicObstacles::reconfigureCB, this, _1, _2);
  dynamic_recfg_->setCallback(cb);
}

void CostmapToDynamicObstacles::incomingMap(const nav_msgs::OccupancyGridConstPtr& map)
{
    got_dist_map_ = false;
    const std::vector<signed char>& map_data = map->data;

    cv::Mat full_obs = cv::Mat::zeros(map->info.height, map->info.width, CV_8UC1);
    for(size_t i = 0; i < map_data.size(); i++){
      if(map_data[i] == 0)
        full_obs.data[i] = 255;
    }

    costmap_2d::Costmap2D::mutex_t::scoped_lock lock(*costmap_->getMutex());
    cv::distanceTransform(full_obs, static_distmap_, CV_DIST_L2, 5);
    ROS_INFO("recv map in CostmapToDynamicObstacles layer");
    static_resolution_ = map->info.resolution;
    map_origin_.x = map->info.origin.position.x;
    map_origin_.y = map->info.origin.position.y;
    got_dist_map_ = true;
    //cv::imwrite("/home/chenbo/overlay_ws/dddddd.png", static_distmap_);
}

void CostmapToDynamicObstacles::mapsPathCallback(const std_msgs::String& paths)
{
  ROS_INFO("[CostmapToDynamicObstacles] recv map paths: %s", paths.data.c_str());

  got_dist_map_ = false;
  std::vector<std::string> vPaths;
  boost::split(vPaths, paths.data, boost::is_any_of(" "), boost::token_compress_on);
  if(vPaths.size() < 3) {
    ROS_WARN("maps path might not be right");
    return;
  }

  std::string dist_path = vPaths[1], width_path = vPaths[2];
  std::vector<std::string> vTemp;
  boost::split(vTemp, dist_path, boost::is_any_of(":"), boost::token_compress_on);
  if(vTemp.size() != 2 ||
     vTemp[1].find("fdist") == std::string::npos ||
     !boost::filesystem::exists(boost::filesystem::path(vTemp[1]))){
    ROS_WARN("fbdn map path seem not right");
  }else{
    if(matread(vTemp[1], static_distmap_, map_origin_, static_resolution_)){
      static_distmap_.convertTo(static_distmap_, CV_32FC1);
      ROS_INFO("[CostmapToDynamicObstacles]: load distance map");

      got_dist_map_ = true;
    }else{
      ROS_WARN("[CostmapToDynamicObstacles]: load distance map %s failed", vTemp[1].c_str());
    }
  }
}

void CostmapToDynamicObstacles::filterStaticPts()
{
  /////////////////////////// Foreground detection ////////////////////////////////////
  // Dynamic obstacles are separated from static obstacles
  int origin_x = round((org_x_ - map_origin_.x)/ static_resolution_);
  int origin_y = round((org_y_ - map_origin_.y)/ static_resolution_);
  double scale = costmap_->getResolution()/static_resolution_;

  fg_mask_ = costmap_mat_.clone();
  for(int r = 0; r < fg_mask_.rows; r++)
    for(int c = 0; c < fg_mask_.cols; c++){
      int map_x = (0.5+c)*scale+origin_x, map_y = (0.5+r)*scale+origin_y;
      if(map_x < 0 || map_x >= static_distmap_.cols ||
         map_y < 0 || map_y >= static_distmap_.rows ||
         ((float*)static_distmap_.data)[map_y*static_distmap_.cols + map_x]*static_resolution_ < 0.3
         ){
        fg_mask_.data[r*fg_mask_.cols + c] = 0;
      }
    }

  // Closing Operation
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                              cv::Size(4 + 1, 4 + 1),
                                              cv::Point(2, 2));
  cv::dilate(fg_mask_, fg_mask_, element);
  cv::dilate(fg_mask_, fg_mask_, element);
  cv::erode(fg_mask_, fg_mask_, element);

  //cv::imwrite("/home/chenbo/overlay_ws/fgMask.png", fg_mask_);
}

void CostmapToDynamicObstacles::compute()
{
  if (costmap_mat_.empty() || !got_dist_map_)
    return;

  /////////////////////////// Foreground detection ////////////////////////////////////
  // Dynamic obstacles are separated from static obstacles
  int origin_x = round(org_x_ / costmap_->getResolution());
  int origin_y = round(org_y_ / costmap_->getResolution());
#if 1
  filterStaticPts();
#else

  // ROS_INFO("Origin x  [m]: %f    Origin_y  [m]: %f", org_x_, org_y_);
  // ROS_INFO("Origin x [px]: %d \t Origin_y [px]: %d", originX, originY);

  bg_sub_->apply(costmap_mat_, fg_mask_, origin_x, origin_y);
//cv::imwrite("/home/chenbo/overlay_ws/fgMask.png", fg_mask_);
  // if no foreground object is detected, no ObstacleMsgs need to be published
  if (fg_mask_.empty())
    return;

#endif
  cv::Mat bg_mat;
  if (publish_static_obstacles_)
  {
    // Get static obstacles
    bg_mat = costmap_mat_ - fg_mask_;
    // visualize("bg_mat", bg_mat);
  }

  /////////////////////////////// Blob detection /////////////////////////////////////
  // Centers and contours of Blobs are detected
  blob_det_->detect(fg_mask_, keypoints_);
  std::vector<std::vector<cv::Point>> contours = blob_det_->getContours();
  cv::Point org(origin_x, origin_y);
  for(int i = 0; i < contours.size(); i++)
    for(int j = 0; j < contours[i].size(); j++)
      contours[i][j] += org;

  ////////////////////////////// Tracking ////////////////////////////////////////////
  // Objects are assigned to objects from previous frame based on Hungarian Algorithm
  // Object velocities are estimated using a Kalman Filter
  std::vector<Point_t> detected_centers(keypoints_.size());
  for (int i = 0; i < keypoints_.size(); i++)
  {
    detected_centers.at(i).x = keypoints_.at(i).pt.x+origin_x;
    detected_centers.at(i).y = keypoints_.at(i).pt.y+origin_y;
    detected_centers.at(i).z = 0; // Currently unused!
  }

  tracker_->Update(detected_centers, contours);

  ///////////////////////////////////// Output ///////////////////////////////////////
  /*
  cv::Mat fg_mask_with_keypoints = cv::Mat::zeros(fg_mask_.size(), CV_8UC3);
  cv::cvtColor(fg_mask_, fg_mask_with_keypoints, cv::COLOR_GRAY2BGR);

  for (int i = 0; i < (int)tracker_->tracks.size(); ++i)
    cv::rectangle(fg_mask_with_keypoints, cv::boundingRect(tracker_->tracks[i]->getLastContour()),
                  cv::Scalar(0, 0, 255), 1);

  visualize("fgMaskWithKeyPoints", fg_mask_with_keypoints);
  //cv::imwrite("/home/albers/Desktop/fgMask.png", fgMask);
  //cv::imwrite("/home/albers/Desktop/fgMaskWithKeyPoints.png", fgMaskWithKeypoints);
  */

  //////////////////////////// Fill ObstacleContainerPtr /////////////////////////////
  ObstacleArrayPtr obstacles(new ObstacleArrayMsg);
  // header.seq is automatically filled
  obstacles->header.stamp = ros::Time::now();
  obstacles->header.frame_id = global_frame_; //Global frame /map

  // For all tracked objects
  for (unsigned int i = 0; i < (unsigned int)tracker_->tracks.size(); ++i)
  {
    geometry_msgs::Polygon polygon;

    // TODO directly create polygon inside getContour and avoid copy
    std::vector<Point_t> contour;
    getContour(i, contour); // this method also transforms map to world coordinates

    // convert contour to polygon
    for (const Point_t& pt : contour)
    {
      polygon.points.emplace_back();
      polygon.points.back().x = pt.x;
      polygon.points.back().y = pt.y;
      polygon.points.back().z = 0;
    }

    obstacles->obstacles.emplace_back();
    obstacles->obstacles.back().polygon = polygon;

    // Set obstacle ID
    obstacles->obstacles.back().id = tracker_->tracks.at(i)->track_id;

    // Set orientation
    geometry_msgs::QuaternionStamped orientation;

    Point_t vel = getEstimatedVelocityOfObject(i);
    double yaw = std::atan2(vel.y, vel.x);
    //ROS_INFO("yaw: %f", yaw);
    obstacles->obstacles.back().orientation = tf::createQuaternionMsgFromYaw(yaw);

    // Set velocity
    geometry_msgs::TwistWithCovariance velocities;
    //velocities.twist.linear.x = std::sqrt(vel.x*vel.x + vel.y*vel.y);
    //velocities.twist.linear.y = 0;
    velocities.twist.linear.x = vel.x;
    velocities.twist.linear.y = vel.y; // TODO(roesmann): don't we need to consider the transformation between opencv's and costmap's coordinate frames?
    velocities.twist.linear.z = 0;
    velocities.twist.angular.x = 0;
    velocities.twist.angular.y = 0;
    velocities.twist.angular.z = 0;

    // TODO: use correct covariance matrix
    velocities.covariance = {1, 0, 0, 0, 0, 0,
                             0, 1, 0, 0, 0, 0,
                             0, 0, 1, 0, 0, 0,
                             0, 0, 0, 1, 0, 0,
                             0, 0, 0, 0, 1, 0,
                             0, 0, 0, 0, 0, 1};

    obstacles->obstacles.back().active_time = getTrackActiveTime(i);

    obstacles->obstacles.back().velocities = velocities;
  }

  ////////////////////////// Static obstacles ////////////////////////////
  if (publish_static_obstacles_)
  {
    uchar* img_data = bg_mat.data;
    int width = bg_mat.cols;
    int height = bg_mat.rows;
    int stride = bg_mat.step;

    if (stackedCostmapConversion())
    {
      // Create new costmap with static obstacles (background)
      boost::shared_ptr<costmap_2d::Costmap2D> static_costmap(new costmap_2d::Costmap2D(costmap_->getSizeInCellsX(),
                                                                                        costmap_->getSizeInCellsY(),
                                                                                        costmap_->getResolution(),
                                                                                        org_x_,
                                                                                        org_y_));
      for(int i = 0; i < height; i++)
      {
        for(int j = 0; j < width; j++)
        {
          static_costmap->setCost(j, i, img_data[i * stride + j]);
        }
      }

      // Apply static obstacle conversion plugin
      setStaticCostmap(static_costmap);
      convertStaticObstacles();

      // Store converted static obstacles for publishing
      auto static_polygons = getStaticPolygons();
      for (auto it = static_polygons->begin(); it != static_polygons->end(); ++it)
      {
        obstacles->obstacles.emplace_back();
        obstacles->obstacles.back().polygon = *it;
        obstacles->obstacles.back().velocities.twist.linear.x = 0;
        obstacles->obstacles.back().velocities.twist.linear.y = 0;
        obstacles->obstacles.back().id = -1;
      }
    }
    // Otherwise leave static obstacles point-shaped
    else
    {
      for(int i = 0; i < height; i++)
      {
        for(int j = 0; j < width; j++)
        {
            uchar value = img_data[i * stride + j];
            if (value > 0)
            {
              // obstacle found
              obstacles->obstacles.emplace_back();
              geometry_msgs::Point32 pt;
              pt.x = (double)j*costmap_->getResolution() + org_x_;
              pt.y = (double)i*costmap_->getResolution() + org_y_;
              obstacles->obstacles.back().polygon.points.push_back(pt);
              obstacles->obstacles.back().velocities.twist.linear.x = 0;
              obstacles->obstacles.back().velocities.twist.linear.y = 0;
              obstacles->obstacles.back().id = -1;
            }
        }
      }
    }
  }

  updateObstacleContainer(obstacles);
}

void CostmapToDynamicObstacles::setCostmap2D(costmap_2d::Costmap2D* costmap)
{
  if (!costmap)
    return;

  costmap_ = costmap;

  updateCostmap2D();
}

void CostmapToDynamicObstacles::updateCostmap2D()
{
  if (!costmap_->getMutex())
  {
    ROS_ERROR("Cannot update costmap since the mutex pointer is null");
    return;
  }
  costmap_2d::Costmap2D::mutex_t::scoped_lock lock(*costmap_->getMutex());

  // Initialize costmapMat_ directly with the unsigned char array of costmap_
  //costmap_mat_ = cv::Mat(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), CV_8UC1,
  //                      costmap_->getCharMap()).clone(); // Deep copy: TODO
  costmap_mat_ = cv::Mat(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), CV_8UC1,
                        costmap_->getCharMap());
  org_x_ = costmap_->getOriginX();
  org_y_ = costmap_->getOriginY();
}

ObstacleArrayConstPtr CostmapToDynamicObstacles::getObstacles()
{
  boost::mutex::scoped_lock lock(mutex_);
  return obstacles_;
}

void CostmapToDynamicObstacles::updateObstacleContainer(ObstacleArrayPtr obstacles)
{
  boost::mutex::scoped_lock lock(mutex_);
  obstacles_ = obstacles;
}

Point_t CostmapToDynamicObstacles::getEstimatedVelocityOfObject(unsigned int idx)
{
  // vel [px/s] * costmapResolution [m/px] = vel [m/s]
  Point_t vel = tracker_->tracks.at(idx)->getEstimatedVelocity() * costmap_->getResolution();

  //ROS_INFO("vel x: %f, vel y: %f, vel z: %f", vel.x, vel.y, vel.z);
  // velocity in /map frame
  return vel;
}

double CostmapToDynamicObstacles::getTrackActiveTime(unsigned int idx)
{
  return tracker_->tracks.at(idx)->getActiveTime();
}

void CostmapToDynamicObstacles::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO_ONCE("CostmapToDynamicObstacles: odom received.");

  tf::Quaternion pose;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, pose);

  tf::Vector3 twistLinear;
  tf::vector3MsgToTF(msg->twist.twist.linear, twistLinear);

  // velocity of the robot in x, y and z coordinates
  tf::Vector3 vel = tf::quatRotate(pose, twistLinear);
  ego_vel_.x = vel.x();
  ego_vel_.y = vel.y();
  ego_vel_.z = vel.z();
}

void CostmapToDynamicObstacles::reconfigureCB(CostmapToDynamicObstaclesConfig& config, uint32_t level)
{
  publish_static_obstacles_ = config.publish_static_obstacles;

  // BackgroundSubtraction Parameters
  BackgroundSubtractor::Params bg_sub_params;
  bg_sub_params.alpha_slow = config.alpha_slow;
  bg_sub_params.alpha_fast = config.alpha_fast;
  bg_sub_params.beta = config.beta;
  bg_sub_params.min_sep_between_fast_and_slow_filter = config.min_sep_between_slow_and_fast_filter;
  bg_sub_params.min_occupancy_probability = config.min_occupancy_probability;
  bg_sub_params.max_occupancy_neighbors = config.max_occupancy_neighbors;
  bg_sub_params.morph_size = config.morph_size;
  bg_sub_->updateParameters(bg_sub_params);

  // BlobDetector Parameters
  BlobDetector::Params blob_det_params;
  // necessary, because blobDetParams are otherwise initialized with default values for dark blobs
  blob_det_params.filterByColor = false; // actually filterByIntensity, always true
  blob_det_params.blobColor = 255;      // Extract light blobs
  blob_det_params.thresholdStep = 256;  // Input for blobDetection is already a binary image
  blob_det_params.minThreshold = 127;
  blob_det_params.maxThreshold = 255;
  blob_det_params.minRepeatability = 1;
  blob_det_params.minDistBetweenBlobs = config.min_distance_between_blobs; // TODO: Currently not working as expected
  blob_det_params.filterByArea = config.filter_by_area;
  blob_det_params.minArea = config.min_area;
  blob_det_params.maxArea = config.max_area;
  blob_det_params.filterByCircularity = config.filter_by_circularity;
  blob_det_params.minCircularity = config.min_circularity;
  blob_det_params.maxCircularity = config.max_circularity;
  blob_det_params.filterByInertia = config.filter_by_inertia;
  blob_det_params.minInertiaRatio = config.min_inertia_ratio;
  blob_det_params.maxInertiaRatio = config.max_inertia_ratio;
  blob_det_params.filterByConvexity = config.filter_by_convexity;
  blob_det_params.minConvexity = config.min_convexity;
  blob_det_params.maxConvexity = config.max_convexity;
  blob_det_->updateParameters(blob_det_params);

  // Tracking Parameters
  CTracker::Params tracking_params;
  tracking_params.dt = config.dt;
  tracking_params.dist_thresh = config.dist_thresh;
  tracking_params.max_allowed_skipped_frames = config.max_allowed_skipped_frames;
  tracking_params.max_trace_length = config.max_trace_length;
  tracker_->updateParameters(tracking_params);
}

void CostmapToDynamicObstacles::getContour(unsigned int idx, std::vector<Point_t>& contour)
{
  assert(!tracker_->tracks.empty() && idx < tracker_->tracks.size());

  contour.clear();

  // contour [px] * costmapResolution [m/px] = contour [m]
  std::vector<cv::Point> contour2i = tracker_->tracks.at(idx)->getLastContour();

  contour.reserve(contour2i.size());

  Point_t costmap_origin(org_x_, org_y_, 0);

  for (std::size_t i = 0; i < contour2i.size(); ++i)
  {
    contour.push_back((Point_t(contour2i.at(i).x, contour2i.at(i).y, 0.0)*costmap_->getResolution())
                      ); // Shift to /map
  }

}

void CostmapToDynamicObstacles::visualize(const std::string& name, const cv::Mat& image)
{
  if (!image.empty())
  {
    cv::Mat im = image.clone();
    cv::flip(im, im, 0);
    cv::imshow(name, im);
    cv::waitKey(1);
  }
}
}
