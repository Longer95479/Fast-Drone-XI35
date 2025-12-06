#pragma once

#include <map>
#include <optional>
#include <vector>

#include "kf_nav/state_utils.h"

class StateManager {
 public:
  StateManager() = default;
  StateManager(const StateManager& other) = delete;
  StateManager& operator=(const StateManager& other) = delete;

  void updateState(const Eigen::VectorXd& delta);
  std::optional<IdxSizePair> insertState(UniID state_id,
                                         const StateType state_type,
                                         const double* data,
                                         bool is_const = false);
  std::vector<IdxSizePair> getConstStateIdxSize();

  Sophus::SO3d& getImuOriState();
  const Sophus::SO3d& getImuOriState() const;

  Eigen::Vector3d& getImuTvecState();
  const Eigen::Vector3d& getImuTvecState() const;

  Eigen::Vector3d& getImuVelState();
  const Eigen::Vector3d& getImuVelState() const;

  Eigen::Vector3d& getBiasGyroState();
  const Eigen::Vector3d& getBiasGyroState() const;

  Eigen::Vector3d& getBiasAccelState();
  const Eigen::Vector3d& getBiasAccelState() const;

  Sophus::SO3d& getExtOdomOriState();
  const Sophus::SO3d& getExtOdomOriState() const;

  Eigen::Vector3d& getExtOdomTvecState();
  const Eigen::Vector3d& getExtOdomTvecState() const;

 public:
  std::map<UniID, SystemState> state_pool;
};