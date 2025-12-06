#include "kf_nav/state_manager.h"

void StateManager::updateState(const Eigen::VectorXd& delta) {
  auto delta_addr = delta.data();
  for (auto& [uid, state] : state_pool) {
    state.update(delta_addr);
    delta_addr += state.localSize();
  }
}

std::optional<IdxSizePair> StateManager::insertState(UniID state_id,
                                                     const StateType state_type,
                                                     const double* data,
                                                     bool is_const) {
  if (state_pool.find(state_id) != state_pool.end())
    return std::nullopt;  // std::optional
  auto state_new = SystemState::create(data, state_type);
  state_new.setConstant(is_const);
  state_pool.insert({state_id, state_new});
  return IdxSizePair(state_new.startIndex(), state_new.localSize());
}

std::vector<IdxSizePair> StateManager::getConstStateIdxSize() {
  std::vector<IdxSizePair> id_size_pairs;
  for (auto& [uid, state] : state_pool) {
    if (state.isConstant()) {
      id_size_pairs.emplace_back(state.startIndex(), state.localSize());
    }
  }
  return id_size_pairs;
}

Sophus::SO3d& StateManager::getImuOriState() {
  return state_pool.at(getImuOriUid()).viewAs<ImuOriState>().so3();
}
const Sophus::SO3d& StateManager::getImuOriState() const {
  return state_pool.at(getImuOriUid()).viewConstAs<ImuOriState>().so3();
}

Eigen::Vector3d& StateManager::getImuTvecState() {
  return state_pool.at(getImuTvecUid()).viewAs<ImuTvecState>().vec();
}
const Eigen::Vector3d& StateManager::getImuTvecState() const {
  return state_pool.at(getImuTvecUid()).viewConstAs<ImuTvecState>().vec();
}

Eigen::Vector3d& StateManager::getImuVelState() {
  return state_pool.at(getImuVelUid()).viewAs<ImuVelState>().vec();
}
const Eigen::Vector3d& StateManager::getImuVelState() const {
  return state_pool.at(getImuTvecUid()).viewConstAs<ImuVelState>().vec();
}

Eigen::Vector3d& StateManager::getBiasGyroState() {
  return state_pool.at(getImuBiasGyroUid()).viewAs<BiasGyroState>().vec();
}
const Eigen::Vector3d& StateManager::getBiasGyroState() const {
  return state_pool.at(getImuBiasGyroUid()).viewConstAs<BiasGyroState>().vec();
}

Eigen::Vector3d& StateManager::getBiasAccelState() {
  return state_pool.at(getImuBiasAccelUid()).viewAs<BiasAccelState>().vec();
}
const Eigen::Vector3d& StateManager::getBiasAccelState() const {
  return state_pool.at(getImuBiasAccelUid())
      .viewConstAs<BiasAccelState>()
      .vec();
}

Sophus::SO3d& StateManager::getExtOdomOriState() {
  return state_pool.at(getExtOdomOriUid()).viewAs<ExtOdomOriState>().so3();
}
const Sophus::SO3d& StateManager::getExtOdomOriState() const {
  return state_pool.at(getExtOdomOriUid()).viewConstAs<ExtOdomOriState>().so3();
}

Eigen::Vector3d& StateManager::getExtOdomTvecState() {
  return state_pool.at(getExtOdomTvecUid()).viewAs<ExtOdomTvecState>().vec();
}
const Eigen::Vector3d& StateManager::getExtOdomTvecState() const {
  return state_pool.at(getExtOdomTvecUid())
      .viewConstAs<ExtOdomTvecState>()
      .vec();
}