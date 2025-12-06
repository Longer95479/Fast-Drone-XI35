#include <deque>
#include <mutex>
#include <optional>

template <typename MsgType>
class ThreadsafeQueue {
 protected:
  // use deque because it's reverse iterable
  // mutable std::shared_mutex smtx_;
  mutable std::mutex smtx_;
  std::deque<MsgType> msg_buffer_;

 public:
  ThreadsafeQueue() = default;

  bool empty() const {
    std::lock_guard rlk(smtx_);
    return msg_buffer_.empty();
  }

  size_t size() const {
    std::lock_guard rlk(smtx_);
    return msg_buffer_.size();
  }

  // push might block for a long time is event process takes too much time
  virtual void push(MsgType const &msg) {
    // lock and add data
    std::lock_guard wlk(smtx_);
    msg_buffer_.push_back(msg);
  }

  std::vector<MsgType> pop_all() {
    std::lock_guard wlk(smtx_);
    std::vector<MsgType> out;
    out.reserve(msg_buffer_.size());
    while (!msg_buffer_.empty()) {
      out.emplace_back(msg_buffer_.front());
      msg_buffer_.pop_front();
    }
    return out;
  }

  std::optional<MsgType> pop() {
    std::lock_guard wlk(smtx_);
    if (msg_buffer_.empty()) {
      return std::nullopt;
    }
    auto msg = msg_buffer_.front();
    msg_buffer_.pop_front();
    return msg;
  }

  std::optional<MsgType> front() const {
    std::lock_guard rlk(smtx_);
    if (msg_buffer_.empty()) {
      return std::nullopt;
    }
    auto msg = msg_buffer_.front();
    return msg;
  }

  std::optional<MsgType> back() const {
    std::lock_guard rlk(smtx_);
    if (msg_buffer_.empty()) {
      return std::nullopt;
    }
    auto msg = msg_buffer_.back();
    return msg;
  }

  void clear() {
    std::lock_guard wlk(smtx_);
    msg_buffer_.clear();
  }

  template <typename MatchPolicy>
  void iterateByMatcher(MatchPolicy &matcher) const {
    std::lock_guard rlk(smtx_);
    for (auto &msg : msg_buffer_) {
      if (matcher(msg)) {
        return;
      }
    }
  }

  template <typename MatchPolicy>
  void iterateByMatcherReverse(MatchPolicy &matcher) const {
    std::lock_guard rlk(smtx_);
    auto it = msg_buffer_.rbegin();
    auto rend = msg_buffer_.rend();
    while (it != rend) {
      if (matcher(*it)) {
        return;
      }
      ++it;
    }
  }
};
