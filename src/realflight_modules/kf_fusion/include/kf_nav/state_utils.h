#pragma once
#include <array>
#include <type_traits>
#include <variant>

#include "kf_nav/state_base.h"

struct IdxSizePair {
  IdxSizePair() = default;
  IdxSizePair(int start_index_, int size_)
      : start_index(start_index_), size(size_) {}
  int start_index{0};
  int size{0};
};

template <class C, class... Args>
class DefineVariantState {
 public:
  using VariantT = std::variant<C, Args...>;

 public:
  inline int orderIndex() const {
    return std::visit([&](const auto& state) -> int { return state.order(); },
                      state_);
  }
  inline int localSize() const {
    return std::visit(
        [&](const auto& state) -> int { return state.localSize(); }, state_);
  }
  inline int startIndex() const {
    return std::visit(
        [&](const auto& state) -> int { return state.startIndex(); }, state_);
  }
  inline void setConstant(bool flag) {
    std::visit([&](auto& state) -> void { return state.setConstant(flag); },
               state_);
  }
  inline bool isConstant() const {
    return std::visit(
        [&](const auto& state) -> bool { return state.isConstant(); }, state_);
  }
  inline void update(const double* delta) {
    std::visit([&](auto& state) -> void { return state.update(delta); },
               state_);
  }

  template <typename StateType>
  inline auto& viewAs() {
    return std::get<StateType>(state_);
  }

  template <typename StateType>
  inline const auto& viewConstAs() const {
    return std::get<StateType>(state_);
  }

  static DefineVariantState create(const double* data, const StateType& type) {
    DefineVariantState out;
    constexpr int VARIANT_SIZE = std::variant_size<VariantT>::value;
    constexpr auto order_index_map = calculateOrderIndexMap();
    visitAllTypes<VARIANT_SIZE - 1>(data, out, type, order_index_map);
    return out;
  }

  static constexpr int sumOfLocalSize() {
    constexpr int VARIANT_SIZE = std::variant_size_v<VariantT>;
    return sumOfLocalSizeRecursive<VARIANT_SIZE - 1>();
  }

 protected:
  template <int I, typename ArrayType>
  static void visitAllTypes(const double* data, DefineVariantState& res,
                            const StateType& type,
                            const ArrayType& order_index_map) {
    if constexpr (I >= 0) {
      using state_t = typename std::variant_alternative<I, VariantT>::type;
      if (state_t::Type == type) {
        int start_index = order_index_map[state_t::Order];
        state_t val(data, start_index);
        res.state_ = val;
        return;
      } else {
        visitAllTypes<I - 1>(data, res, type, order_index_map);
      }
    }
  }

  static constexpr auto calculateOrderIndexMap() {
    constexpr int VARIANT_SIZE = std::variant_size_v<VariantT>;
    std::array<int, VARIANT_SIZE> order_index_map{};

    int sum = 0;
    for (int i = 0; i < VARIANT_SIZE; ++i) {
      order_index_map[i] = sum;
      sum += getLocalSizeAtOrder<VARIANT_SIZE - 1>(i);
    }
    return order_index_map;
  }

  template <int I>
  static constexpr int getLocalSizeAtOrder(int order) {
    if constexpr (I >= 0) {
      using state_t = typename std::variant_alternative<I, VariantT>::type;
      if (state_t::Order == order) {
        return state_t::LocalDim;
      } else {
        return getLocalSizeAtOrder<I - 1>(order);
      }
    }
    return 0;
  }

  template <int I>
  static constexpr int sumOfLocalSizeRecursive() {
    if constexpr (I < 0) {
      return 0;
    } else {
      return getLocalSizeAt<I>() + sumOfLocalSizeRecursive<I - 1>();
    }
  }

  template <int I>
  static constexpr int getLocalSizeAt() {
    using state_t = typename std::variant_alternative<I, VariantT>::type;
    return state_t::LocalDim;
  }

 protected:
  DefineVariantState() = default;
  VariantT state_;
};

using SystemState =
    DefineVariantState<ImuOriState, ImuTvecState, ImuVelState, BiasGyroState,
                       BiasAccelState, ExtOdomOriState, ExtOdomTvecState>;
constexpr int total_local_size = SystemState::sumOfLocalSize();
