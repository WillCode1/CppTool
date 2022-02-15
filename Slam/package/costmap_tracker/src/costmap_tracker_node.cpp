/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann, Otniel Rinaldo
 *********************************************************************/
#include <thread>
#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <costmap_tracker/costmap_tracker_interface.h>
#include <pluginlib/class_loader.h>

#include <tf2_ros/transform_listener.h>


class CostmapStandaloneConversion
{
public:
  CostmapStandaloneConversion() :
    converter_loader_("costmap_tracker", "costmap_tracker::BaseCostmapToPolygons"),
    n_("~"),
    buffer_(ros::Duration(10)),
    tf_(buffer_)
  {
      // load converter plugin from parameter server, otherwise set default
      std::string tracker_plugin = "costmap_tracker::CostmapToDynamicObstacles";
      n_.param("tracker_plugin", tracker_plugin, tracker_plugin);

      try
      {
        converter_ = converter_loader_.createInstance(tracker_plugin);
      }
      catch(const pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        ros::shutdown();
      }

      ROS_INFO_STREAM("Standalone costmap converter:" << tracker_plugin << " loaded.");

      costmap_ros_ = new costmap_2d::Costmap2DROS("lethal_costmap", buffer_);
      costmap_ros_->pause();

      std::string obstacles_topic = "costmap_obstacles";
      n_.param("obstacles_topic", obstacles_topic, obstacles_topic);

      std::string polygon_marker_topic = "costmap_polygon_markers";
      n_.param("polygon_marker_topic", polygon_marker_topic, polygon_marker_topic);

      std::string odom_topic = "/odom";
      n_.param("odom_topic", odom_topic, odom_topic);

      double dt = 0.2;
      n_.param("dt", dt, dt);

      obstacle_pub_ = n_.advertise<costmap_tracker::ObstacleArrayMsg>(obstacles_topic, 1000);
      marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>(polygon_marker_topic, 10);

      if (converter_)
      {
        costmap_ros_->start();

        converter_->setOdomTopicAndFrame(odom_topic, costmap_ros_->getGlobalFrameID());
        converter_->initialize(n_);
        converter_->setCostmap2D(costmap_ros_->getCostmap());
        //converter_->startWorker(ros::Rate(5), costmap_ros_->getCostmap(), true);

        std::thread t = std::thread(&CostmapStandaloneConversion::track, this, 1.0/dt);
        t.detach();
      }
  }

  void track(double freq)
  {
    ros::Rate r(freq);
    while(!ros::isShuttingDown()){
      r.sleep();

      // convert
      converter_->updateCostmap2D();
      converter_->compute();
      clearCostmap();
      costmap_tracker::ObstacleArrayConstPtr obstacles = converter_->getObstacles();

      if (!obstacles)
        continue;

      obstacle_pub_.publish(obstacles);

      publishAsMarker(costmap_ros_->getGlobalFrameID(), *obstacles, marker_pub_);

    }
  }

  void clearCostmap()
  {
    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap_ros_->getLayeredCostmap()->getPlugins();

    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
      boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;

        boost::shared_ptr<costmap_2d::CostmapLayer> costmap
              = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
        costmap->resetMap(0, 0,
                          costmap_ros_->getCostmap()->getSizeInCellsX(),
                          costmap_ros_->getCostmap()->getSizeInCellsY());
        costmap->addExtraBounds(0, 0,
                                costmap_ros_->getCostmap()->getSizeInCellsX(),
                                costmap_ros_->getCostmap()->getSizeInCellsY());
    }
  }

  void publishAsMarker(const std::string& frame_id, const std::vector<geometry_msgs::PolygonStamped>& polygonStamped, ros::Publisher& marker_pub)
  {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "Polygons";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.1;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    for (std::size_t i=0; i<polygonStamped.size(); ++i)
    {
      for (int j=0; j< (int)polygonStamped[i].polygon.points.size()-1; ++j)
      {
        geometry_msgs::Point line_start;
        line_start.x = polygonStamped[i].polygon.points[j].x;
        line_start.y = polygonStamped[i].polygon.points[j].y;
        line_list.points.push_back(line_start);
        geometry_msgs::Point line_end;
        line_end.x = polygonStamped[i].polygon.points[j+1].x;
        line_end.y = polygonStamped[i].polygon.points[j+1].y;
        line_list.points.push_back(line_end);
      }
      // close loop for current polygon
      if (!polygonStamped[i].polygon.points.empty() && polygonStamped[i].polygon.points.size() != 2 )
      {
        geometry_msgs::Point line_start;
        line_start.x = polygonStamped[i].polygon.points.back().x;
        line_start.y = polygonStamped[i].polygon.points.back().y;
        line_list.points.push_back(line_start);
        if (line_list.points.size() % 2 != 0)
        {
          geometry_msgs::Point line_end;
          line_end.x = polygonStamped[i].polygon.points.front().x;
          line_end.y = polygonStamped[i].polygon.points.front().y;
          line_list.points.push_back(line_end);
        }
      }


    }
    marker_pub.publish(line_list);
  }

  void publishAsMarker(const std::string& frame_id, const costmap_tracker::ObstacleArrayMsg& obstacles, ros::Publisher& marker_pub)
  {
    visualization_msgs::MarkerArray mka;

    visualization_msgs::Marker info;
    info.header.stamp = ros::Time::now();
    info.header.frame_id = frame_id;
    info.ns = "info";
    info.scale.x = 0.2;
    info.scale.y = 0.2;
    info.scale.z = 0.2;
    info.color.a = 1.0;
    info.color.r = 1.0;
    info.color.g = 0.2;
    info.color.b = 0.0;
    info.lifetime = ros::Duration(0.5);
    info.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "Polygons";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.1;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    for (const costmap_tracker::ObstacleMsg& obstacle : obstacles.obstacles)
    {
      info.id = obstacle.id;
      info.text = std::to_string(obstacle.id) + " ("
          + std::to_string(obstacle.velocities.twist.linear.x) + ", "
          + std::to_string(obstacle.velocities.twist.linear.y) + ", "
          + std::to_string(obstacle.active_time) + ")";
      info.pose.position.x = obstacle.polygon.points[0].x + 0.2;
      info.pose.position.y = obstacle.polygon.points[0].y + 0.2;
      info.pose.position.z = 0.2;
      mka.markers.push_back(info);
      for (int j=0; j< (int)obstacle.polygon.points.size()-1; ++j)
      {
        geometry_msgs::Point line_start;
        line_start.x = obstacle.polygon.points[j].x;
        line_start.y = obstacle.polygon.points[j].y;
        line_list.points.push_back(line_start);
        geometry_msgs::Point line_end;
        line_end.x = obstacle.polygon.points[j+1].x;
        line_end.y = obstacle.polygon.points[j+1].y;
        line_list.points.push_back(line_end);
      }
      // close loop for current polygon
      if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2 )
      {
        geometry_msgs::Point line_start;
        line_start.x = obstacle.polygon.points.back().x;
        line_start.y = obstacle.polygon.points.back().y;
        line_list.points.push_back(line_start);
        if (line_list.points.size() % 2 != 0)
        {
          geometry_msgs::Point line_end;
          line_end.x = obstacle.polygon.points.front().x;
          line_end.y = obstacle.polygon.points.front().y;
          line_list.points.push_back(line_end);
        }
      }

    }
    mka.markers.push_back(line_list);
    marker_pub.publish(mka);
  }

private:
  pluginlib::ClassLoader<costmap_tracker::BaseCostmapToPolygons> converter_loader_;
  boost::shared_ptr<costmap_tracker::BaseCostmapToPolygons> converter_;

  ros::NodeHandle n_;
  ros::Timer worker_timer_;
  ros::Publisher obstacle_pub_;
  ros::Publisher marker_pub_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_tracker");

  CostmapStandaloneConversion convert_process;

  ros::spin();

  return 0;
}


