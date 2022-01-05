/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
// MapBuilder是cartographer算法的最顶层设计，MapBuilder包括了两个部分
// 使用TrajectoryBuilders(用于局部子地图)和PoseGraph(用于闭环检测)将完整的SLAM堆栈连接起来。
class MapBuilder : public MapBuilderInterface {
 public:
  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  void FinishTrajectory(int trajectory_id) override;

  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer) override;

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename) override;

  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               bool load_frozen_state) override;

  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const bool load_frozen_state) override;

  mapping::PoseGraphInterface *pose_graph() override {
    return pose_graph_.get();
  }

  int num_trajectory_builders() const override {
    return trajectory_builders_.size();
  }

  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(int trajectory_id) const override {
    return trajectory_builders_.at(trajectory_id).get();
  }

  const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;
  }

 private:
  /*
    trajectory是机器人跑一圈时的轨迹，在这其中需要记录和维护传感器的数据。
    根据这个trajectory上传感器收集的数据，我们可以逐步构建出栅格化的地图Submap，
    但这个submap会随着时间或trajectory的增长而产生误差累积，但trajectory增长到超过一个阈值，则会新增一个submap。
    而PoseGraph是用来进行全局优化，将所有的Submap紧紧tie在一起，构成一个全局的Map，消除误差累积。
   */
  const proto::MapBuilderOptions options_;  //MapBuilder的配置项
  common::ThreadPool thread_pool_;

  std::unique_ptr<PoseGraph> pose_graph_;

  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;  //收集传感器数
  // 每一个TrajectoryBuilder对应了机器人运行了一圈。这个向量列表就管理了整个图中的所有submap。
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>> trajectory_builders_;
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds> all_trajectory_builder_options_;
};

std::unique_ptr<MapBuilderInterface> CreateMapBuilder(const proto::MapBuilderOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
