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

#include "cartographer/mapping/map_builder.h"

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
namespace {

using mapping::proto::SerializedData;

std::vector<std::string> SelectRangeSensorIds(
    const std::set<MapBuilder::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const MapBuilder::SensorId& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == MapBuilder::SensorId::SensorType::RANGE) {
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}

void MaybeAddPureLocalizationTrimmer(const int trajectory_id,
    const proto::TrajectoryBuilderOptions& trajectory_options, PoseGraph* pose_graph) {
  // 如果开启纯定位功能，看到这里我觉得可能cartographer还是保留了纯定位的功能的
  if (trajectory_options.pure_localization()) {
    LOG(WARNING)
        << "'TrajectoryBuilderOptions::pure_localization' field is deprecated. "
           "Use 'TrajectoryBuilderOptions::pure_localization_trimmer' instead.";
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id, 3 /* max_submaps_to_keep */));
    return;
  }
  if (trajectory_options.has_pure_localization_trimmer()) {
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id,
        trajectory_options.pure_localization_trimmer().max_submaps_to_keep()));
  }
}

}  // namespace

MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  CHECK(options.use_trajectory_builder_2d() ^ options.use_trajectory_builder_3d());
  if (options.use_trajectory_builder_2d()) {
    pose_graph_ = absl::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  if (options.use_trajectory_builder_3d()) {
    pose_graph_ = absl::make_unique<PoseGraph3D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem3D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  if (options.collate_by_trajectory()) {//根据collate_by_trajectory的不同，CollatorInterface有两种不同的实现
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>();
  } else {
    sensor_collator_ = absl::make_unique<sensor::Collator>();
  }
}

/*
  MapBuilder::AddTrajectoryBuilder这个函数可以说是cartographer中最重要的一个函数

  一个MapBuilder的类对应了一次建图过程，
  在整个建图过程中，用于全局优化的PoseGraph的对象只有一个，即pose_graph_，而这个变量是在构造函数中就生成了。
  在AddTrajectorybuilder函数中只需要检查一下pose_graph_是否符合PoseGraph2D或PoseGraph3D的情况。
  而一个trajectory对应了机器人运行一圈。在图建好后机器人可能多次运行。
  每一次运行都是新增一条trajectory，因此，需要动态地维护一个trajectory的列表。
  每生成一个trajectory时都是调用AddTrajectoryBuilder来创建的。
 */
//创建一个新的TrajectoryBuilder并返回它的trajectory_id.
int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  const int trajectory_id = trajectory_builders_.size();

  absl::optional<MotionFilter> pose_graph_odometry_motion_filter;
  if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";
    pose_graph_odometry_motion_filter.emplace(
        MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  }
  /*
  * 可以看到，根据是2d建图还是3d建图分为了两种情况。
  * 针对两种不同的情况，首先建立一个LocalTrajectoryBuilder2D或LocalTrajectoryBuilder3D的变量local_trajectory_builder；
  * 这个类是不带Loop Closure的Local Slam, 包含了Pose Extrapolator, Scan Matching等；
  * 但注意，这两个类并没有继承TrajectoryBuilder,并不是一个TrajectoryBuilder的实现，而只是一个工具类
  * 真正地创建一个TrajectoryBuilder是在后面，trajectory_builders_的push_back函数里面。
  * 其中CollatedTrajectoryBuilder继承了接口TrajectoryBuilder;
  * 而前面生成的local_trajectory_builder则用于CreateGlobalTrajectoryBuilder2D函数的第一个参数，用于生成一个CollatedTrajectoryBuilder的智能指针
  */
  if (options_.use_trajectory_builder_3d()) {
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
    //是否有跟该trajectory相关的参数，如果有就设置一下
    if (trajectory_options.has_trajectory_builder_3d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder3D>(
          trajectory_options.trajectory_builder_3d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }
    DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get()));
    //真正地创建一个TrajectoryBuilder
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        CreateGlobalTrajectoryBuilder3D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph3D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  } else {
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_2d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder2D>(
          trajectory_options.trajectory_builder_2d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }
    DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph_.get()));
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        CreateGlobalTrajectoryBuilder2D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph2D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  }
  MaybeAddPureLocalizationTrimmer(trajectory_id, trajectory_options, pose_graph_.get());

  // 如果该轨迹有初始pose；开始一条轨迹前我们是否已知初始位姿。
  // 这对应的情况就是比如说，我们检测到了一个Landmark。那么这时，我们可以新增加一条trajectory，
  // 增加新的trajectory时设置has.initial_trajectory_pose为真，
  // 然后根据机器人与Landmark之间的相对位姿推算机器人相对于世界坐标系的相对位姿。
  // 以该位姿作为新增加的trajectory的初始位姿。这样情况下，在检测到Landmark时就能有效降低累积误差。
  if (trajectory_options.has_initial_trajectory_pose()) {
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    pose_graph_->SetInitialTrajectoryPose(
        trajectory_id, initial_trajectory_pose.to_trajectory_id(),
        transform::ToRigid3(initial_trajectory_pose.relative_pose()),
        common::FromUniversal(initial_trajectory_pose.timestamp()));
  }
  //将一些跟传感器相关的配置项转成proto流，然后统一放到all_trajectory_builder_options_这个向量列表中
  proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
  for (const auto& sensor_id : expected_sensor_ids) {
    *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);
  }
  *options_with_sensor_ids_proto.mutable_trajectory_builder_options() =
      trajectory_options;
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

//从序列化的数据中构造一条trajectory
int MapBuilder::AddTrajectoryForDeserialization(
    const proto::TrajectoryBuilderOptionsWithSensorIds&
        options_with_sensor_ids_proto) {
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.emplace_back();  //这里emplace_back里怎么没有参数呢？
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

//结束一条轨迹
void MapBuilder::FinishTrajectory(const int trajectory_id) {
  //清除对传感器的占用等操作
  sensor_collator_->FinishTrajectory(trajectory_id);
  //完成对trajectory的全局优化
  pose_graph_->FinishTrajectory(trajectory_id);
}

// 根据指定的submap_id来查询submap，把结果放到SubmapQuery::Response中
std::string MapBuilder::SubmapToProto(
    const SubmapId& submap_id, proto::SubmapQuery::Response* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  const auto submap_data = pose_graph_->GetSubmapData(submap_id);
  //一些子图可能在优化过程中被裁掉
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

//调用io::WritePbStream工具，保存所有数据
void MapBuilder::SerializeState(bool include_unfinished_submaps,
                                io::ProtoStreamWriterInterface* const writer) {
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, writer,
                    include_unfinished_submaps);
}

bool MapBuilder::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  io::ProtoStreamWriter writer(filename);
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, &writer,
                    include_unfinished_submaps);
  return (writer.Close());
}

// 从一个proto流中加载SLAM状态
std::map<int, int> MapBuilder::LoadState(
    io::ProtoStreamReaderInterface* const reader, bool load_frozen_state) {
  io::ProtoStreamDeserializer deserializer(reader);

  // Create a copy of the pose_graph_proto, such that we can re-write the trajectory ids.
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const auto& all_builder_options_proto = deserializer.all_trajectory_builder_options();

  //逐条trajectory恢复
  std::map<int, int> trajectory_remapping;
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i) {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
    const auto& options_with_sensor_ids_proto = all_builder_options_proto.options_with_sensor_ids(i);
    const int new_trajectory_id = AddTrajectoryForDeserialization(options_with_sensor_ids_proto);
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    if (load_frozen_state) {
      pose_graph_->FreezeTrajectory(new_trajectory_id);
    }
  }

  // Apply the calculated remapping to constraints in the pose graph proto.
  //恢复trajectory上的节点间的约束关系
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }

  //恢复Submap
  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  //恢复节点的pose
  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto : pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }

  // Set global poses of landmarks.
  // 设置Landmark
  for (const auto& landmark : pose_graph_proto.landmark_poses()) {
    pose_graph_->SetLandmarkPose(landmark.landmark_id(),
                                 transform::ToRigid3(landmark.global_pose()),
                                 true);
  }

  if (options_.use_trajectory_builder_3d()) {
    CHECK_NE(deserializer.header().format_version(),
             io::kFormatVersionWithoutSubmapHistograms)
        << "The pbstream file contains submaps without rotational histograms. "
           "This can be converted with the 'pbstream migrate' tool, see the "
           "Cartographer documentation for details. ";
  }

  //不停读取，直到读完
  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      case SerializedData::kSubmap: {
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
            trajectory_remapping.at(
                proto.submap().submap_id().trajectory_id()));
        const SubmapId submap_id(proto.submap().submap_id().trajectory_id(),
                                 proto.submap().submap_id().submap_index());
        pose_graph_->AddSubmapFromProto(submap_poses.at(submap_id),
                                        proto.submap());
        break;
      }
      case SerializedData::kNode: {
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        pose_graph_->AddNodeFromProto(node_pose, proto.node());
        break;
      }
      case SerializedData::kTrajectoryData: {
        proto.mutable_trajectory_data()->set_trajectory_id(
            trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
        pose_graph_->SetTrajectoryDataFromProto(proto.trajectory_data());
        break;
      }
      case SerializedData::kImuData: {
        if (load_frozen_state) break;
        pose_graph_->AddImuData(
            trajectory_remapping.at(proto.imu_data().trajectory_id()),
            sensor::FromProto(proto.imu_data().imu_data()));
        break;
      }
      case SerializedData::kOdometryData: {
        if (load_frozen_state) break;
        pose_graph_->AddOdometryData(
            trajectory_remapping.at(proto.odometry_data().trajectory_id()),
            sensor::FromProto(proto.odometry_data().odometry_data()));
        break;
      }
      case SerializedData::kFixedFramePoseData: {
        if (load_frozen_state) break;
        pose_graph_->AddFixedFramePoseData(
            trajectory_remapping.at(
                proto.fixed_frame_pose_data().trajectory_id()),
            sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
        break;
      }
      case SerializedData::kLandmarkData: {
        if (load_frozen_state) break;
        pose_graph_->AddLandmarkData(
            trajectory_remapping.at(proto.landmark_data().trajectory_id()),
            sensor::FromProto(proto.landmark_data().landmark_data()));
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  if (load_frozen_state) {
    // Add information about which nodes belong to which submap.
    // This is required, even without constraints.
    for (const proto::PoseGraph::Constraint& constraint_proto : pose_graph_proto.constraint()) {
      if (constraint_proto.tag() != proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      pose_graph_->AddNodeToSubmap(
          NodeId{constraint_proto.node_id().trajectory_id(),
                 constraint_proto.node_id().node_index()},
          SubmapId{constraint_proto.submap_id().trajectory_id(),
                   constraint_proto.submap_id().submap_index()});
    }
  } else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    pose_graph_->AddSerializedConstraints(FromProto(pose_graph_proto.constraint()));
  }
  CHECK(reader->eof());
  return trajectory_remapping;
}

std::map<int, int> MapBuilder::LoadStateFromFile(
    const std::string& state_filename, const bool load_frozen_state) {
  const std::string suffix = ".pbstream";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a "
                    ".pbstream file.";
  }
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  io::ProtoStreamReader stream(state_filename);
  return LoadState(&stream, load_frozen_state);
}

std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const proto::MapBuilderOptions& options) {
  return absl::make_unique<MapBuilder>(options);
}

}  // namespace mapping
}  // namespace cartographer
