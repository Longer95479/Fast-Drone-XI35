#pragma once

#include <stdint.h>

#include <memory>

#define POINTER_ALIAS_DEFINE(TypeName)                              \
  using TypeName##Ptr = std::shared_ptr<TypeName>;                  \
  using TypeName##ConstPtr = std::shared_ptr<const TypeName>;       \
  using TypeName##UniquePtr = std::unique_ptr<TypeName>;            \
  using TypeName##ConstUniquePtr = std::unique_ptr<const TypeName>; \
  using TypeName##WeakPtr = std::weak_ptr<TypeName>;                \
  using TypeName##WeakConstPtr = std::weak_ptr<const TypeName>;

using UniID = uint64_t;

// constexpr
constexpr double kGravityMag = 9.81;

enum class StateType : unsigned short {
  kImuOri = 0,
  kImuTvec,
  kImuVel,
  kImuBiasGyro,
  kImuBiasAccel,
  kExtOdomOri,
  kExtOdomTvec
};

namespace LDim {
constexpr uint16_t Ori = 3;
constexpr uint16_t Vec = 3;
}  // namespace LDim

namespace StateOrder {
constexpr uint16_t ImuOri = 0;
constexpr uint16_t ImuTvec = 1;
constexpr uint16_t ImuVel = 2;
constexpr uint16_t ImuBiasGyro = 3;
constexpr uint16_t ImuBiasAccel = 4;
constexpr uint16_t ExtOdomOri = 5;
constexpr uint16_t ExtOdomTvec = 6;
}  // namespace StateOrder

// use order as uid
inline UniID getImuOriUid() { return StateOrder::ImuOri; }
inline UniID getImuTvecUid() { return StateOrder::ImuTvec; }
inline UniID getImuVelUid() { return StateOrder::ImuVel; }
inline UniID getImuBiasGyroUid() { return StateOrder::ImuBiasGyro; }
inline UniID getImuBiasAccelUid() { return StateOrder::ImuBiasAccel; }
inline UniID getExtOdomOriUid() { return StateOrder::ExtOdomOri; }
inline UniID getExtOdomTvecUid() { return StateOrder::ExtOdomTvec; }
