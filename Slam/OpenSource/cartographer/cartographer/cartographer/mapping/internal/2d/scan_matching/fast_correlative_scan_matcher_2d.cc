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

#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).
class SlidingWindowMaximum {
 public:
  void AddValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  void RemoveValue(const float value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.
  std::deque<float> non_ascending_maxima_;
};

}  // namespace

proto::FastCorrelativeScanMatcherOptions2D
CreateFastCorrelativeScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions2D options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  return options;
}

PrecomputationGrid2D::PrecomputationGrid2D(
    const Grid2D& grid, const CellLimits& limits, const int width,
    std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      min_score_(1.f - grid.GetMaxCorrespondenceCost()),
      max_score_(1.f - grid.GetMinCorrespondenceCost()),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) {
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);
  const int stride = wide_limits_.num_x_cells;
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  std::vector<float>& intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(
        1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(0, y))));
    for (int x = -width + 1; x != 0; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      if (x + width < limits.num_x_cells) {
        current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                          Eigen::Array2i(x + width, y))));
      }
    }
    for (int x = 0; x < limits.num_x_cells - width; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
      current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(
                                        Eigen::Array2i(x + width, y))));
    }
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
    }
    current_values.CheckIsEmpty();
  }
  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
  for (int x = 0; x != wide_limits_.num_x_cells; ++x) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);
    for (int y = -width + 1; y != 0; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
}

uint8 PrecomputationGrid2D::ComputeCellValue(const float probability) const {
  const int cell_value = common::RoundToInt(
      (probability - min_score_) * (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

PrecomputationGridStack2D::PrecomputationGridStack2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options) {
  CHECK_GE(options.branch_and_bound_depth(), 1);
  const int max_width = 1 << (options.branch_and_bound_depth() - 1);
  precomputation_grids_.reserve(options.branch_and_bound_depth());
  std::vector<float> reusable_intermediate_grid;
  const CellLimits limits = grid.limits().cell_limits();
  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);
  for (int i = 0; i != options.branch_and_bound_depth(); ++i) {
    const int width = 1 << i;
    precomputation_grids_.emplace_back(grid, limits, width,
                                       &reusable_intermediate_grid);
  }
}

FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options),
      limits_(grid.limits()),
      precomputation_grid_stack_(
          absl::make_unique<PrecomputationGridStack2D>(grid, options)) {}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}

/** 
 * @brief 给定初始位姿时的匹配
 * @param initial_pose_estimate 初始位姿估计
 * @param point_cloud 将要考察激光点云数据
 * @param min_score 回环检测的最小得分，当做阈值使用
 * @param score 成功匹配后返回匹配得分
 * @param score 成功匹配后返回位姿估计
 * @return 如果闭环检测成功则返回ture，否则返回false
 */
bool FastCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // 计算搜索窗口
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());
  // 实现闭环检测
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

bool FastCorrelativeScanMatcher2D::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // Compute a search window around the center of the submap that includes it
  // fully.
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}

/** 
 * @brief 完成扫描匹配，进而实现闭环检测
 * @param search_parameters 搜索窗口的参数
 * @param initial_pose_estimate 机器人初始位姿
 * @param point_cloud 激光点云数据
 * @param min_score 回环检测匹配最小得分
 * @param score 记录回环检测匹配成功的得分
 * @param pose_estimate 记录回环检测匹配成功的位姿
 * @return 如果闭环检测成功则返回ture，否则返回false
 */
bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);

  // 获取初始位姿估计的方向角，将激光点云中的点都绕Z轴转动相应的角度
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  // 获取搜索窗口下机器人朝向各个方向角时的点云数据
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  // 完成对旋转后的点云数据离散化操作，即将浮点类型的点云数据转换成整型的栅格单元索引
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  // 尽可能的缩小搜索窗口大小，以减少搜索空间，提高搜索效率
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  // 完成对搜索空间的第一次分割，得到初始子空间节点集合
  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  // 完成分支定界搜索，搜索结果放在best_candidate中
  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);
  // 检测最优解的值，如果大于指定阈值min_score就认为匹配成功，修改输入参数指针score和pose_estimate所指对象
  if (best_candidate.score > min_score) {
    *score = best_candidate.score;
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

/** 
 * @brief 完成第一次分支
 * @param discrete_scans 离散化之后的各个搜索方向上的点云数据
 * @param search_parameters 搜索窗口的参数
 */
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const {
  // 完成对搜索空间的初始化分割（生成最低分辨率的所有候选解）
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  // 计算各个候选点的评分并排序
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

/** 
 * @brief 生成最低分辨率的候选解
 * @param search_parameters 搜索窗口的参数
 */
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  // 根据预算图的层数计算初始分割的粒度z^(depth)
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  // 遍历所有搜索方向，累计各个方向下空间的分割数量，得到搜索空间初始分割的子空间数量
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  // 构建各个候选点
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans; ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset = search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset, search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

/** 
 * @brief 计算当前层的候选解与对应分辨率地图中的匹配得分，并按照得分从大到小的进行排序
 * @param precomputation_grid 当前层对应的预计算图
 * @param discrete_scans 待匹配的点云数据
 * @param search_parameters 搜索窗口的参数
 * @param candidates 当前层对应的候选解
 */
void FastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  // 遍历候选解
  for (Candidate2D& candidate : *candidates) {
    int sum = 0;
    // 遍历所有激光点
    for (const Eigen::Array2i& xy_index : discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      //计算累计占用概率log_odd
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }
    // 求平均并转换为概率
    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }
  // 对评分进行降序排序
  std::sort(candidates->begin(), candidates->end(), std::greater<Candidate2D>());
}

/** 
 * @brief 分支定界方法实现
 * @param discrete_scans 离散化的不同搜索角度下的激光点云数据
 * @param search_parameters 搜索窗口的配置参数
 * @param candidates 候选点集合
 * @param candidate_depth 搜索树高度
 * @param min_score 候选点最小评分
 * @return 返回当前分支中的最优解（得分最高且大于min_score）
 */
Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float min_score) const {
  // 如果搜索树高度为0，意味着搜索到叶子节点
  if (candidate_depth == 0) {
    // Return the best candidate.
    // 由于每次迭代过程中对新扩展的候选点进行了降序排序，所以认为队首的这个节点就是最优解，直接返回即可
    return *candidates.begin();
  }

  // 创建一个临时的候选点对象，并为之赋予最小的评分
  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;
  // 遍历所有候选点
  for (const Candidate2D& candidate : candidates) {
    // 如果遇到一个候选点的评分很低，意味着该分支之后的候选点中也没有合适的解。直接跳出循环，说明没有构成回环
    if (candidate.score <= min_score) {
      break;
    }
    // 进行分支， 分解为4个子候选解
    std::vector<Candidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width}) {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    // 对新扩展的候选解评分（也称为定界）同时对评分进行排序
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    // 递归调用BranchAndBound对新扩展候选解进一步执行分支、定界等搜索过程
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
