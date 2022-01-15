/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_TRAJECTORY_CONNECTIVITY_STATE_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_TRAJECTORY_CONNECTIVITY_STATE_H_

#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/connected_components.h"

namespace cartographer {
namespace mapping {

// A class that tracks the connectivity state between trajectories. Compared to
// ConnectedComponents it tracks additionally the last time that a global
// constraint connected to trajectories.
//
// This class is thread-compatible.
class TrajectoryConnectivityState {
 public:
  TrajectoryConnectivityState() {}

  TrajectoryConnectivityState(const TrajectoryConnectivityState&) = delete;
  TrajectoryConnectivityState& operator=(const TrajectoryConnectivityState&) =
      delete;

  // Add a trajectory which is initially connected to only itself.
  void Add(int trajectory_id);

  // Connect two trajectories. If either trajectory is untracked, it will be
  // tracked. This function is invariant to the order of its arguments. Repeated
  // calls to Connect increment the connectivity count and update the last
  // connected time.
  // 把两个trajectories产生链接。
  // 如果其中任意一个trajectory之前是untracked，它将变成tracked该函数跟参数顺序无关。
  // 重复调用Connect会增加两个trajectory之间的connectivity count并更新last connected time
  void Connect(int trajectory_id_a, int trajectory_id_b, common::Time time);

  // Determines if two trajectories have been (transitively) connected. If
  // either trajectory is not being tracked, returns false, except when it is
  // the same trajectory, where it returns true. This function is invariant to
  // the order of its arguments.
  // 判断两个trajectories是否已经connected. 
  // 如果其中任意一个trajectory没有被tracked，返回false——除非两个trajectory是同一个trajectory是返回true
  // 同样，对参数顺序无关
  bool TransitivelyConnected(int trajectory_id_a, int trajectory_id_b) const;

  // The trajectory IDs, grouped by connectivity.
  // 返回一个int型的向量，这个向量里存的是具有connectivity关系的trajectory的id.
  std::vector<std::vector<int>> Components() const;

  // Return the last connection count between the two trajectories. If either of
  // the trajectories is untracked or they have never been connected returns the
  // beginning of time.
  // 返回两条trajectory的上次链接时间。
  // 如果其中任意个trajectory是untracked或他们之间没有connected, 则返回开始时的时间
  common::Time LastConnectionTime(int trajectory_id_a, int trajectory_id_b);

 private:
  // ConnectedComponents are thread safe.
  // 存储所有的连接关系
  mutable ConnectedComponents connected_components_;

  // Tracks the last time a direct connection between two trajectories has
  // been added. The exception is when a connection between two trajectories
  // connects two formerly unconnected connected components. In this case all
  // bipartite trajectories entries for these components are updated with the
  // new connection time.
  // 返回任意两个指定id的trajectory的上次连接时间
  std::map<std::pair<int, int>, common::Time> last_connection_time_map_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_TRAJECTORY_CONNECTIVITY_STATE_H_
