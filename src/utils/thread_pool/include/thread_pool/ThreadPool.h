#pragma once

#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

#include "SafeQueue.h"

class ThreadPool {
private:
  //消费者仿函数，负责从任务队列里抽取任务并执行
  class ThreadWorker {
  private:
    int m_id;
    ThreadPool * m_pool;
  public:
    ThreadWorker(ThreadPool * pool, const int id)
      : m_pool(pool), m_id(id) {
    }

    void operator()() {
      std::function<void()> func;
      bool dequeued;
      while (!m_pool->m_shutdown) {
        {
          std::unique_lock<std::mutex> lock(m_pool->m_conditional_mutex);
          if (m_pool->m_queue.empty()) {
            m_pool->m_conditional_lock.wait(lock);
          }
          dequeued = m_pool->m_queue.dequeue(func);
        }
        if (dequeued) {
          func();
        }
      }
    }
  };

  bool m_shutdown;
  SafeQueue<std::function<void()>> m_queue;//任务队列
  std::vector<std::thread> m_threads;//工作线程
  std::mutex m_conditional_mutex;//线程池锁，保证同一时刻只有一个ThreadWorker访问线程池（注意与任务队列锁区分开）
  std::condition_variable m_conditional_lock;//取任务条件变量
public:
  ThreadPool(const int n_threads)
    : m_threads(std::vector<std::thread>(n_threads)), m_shutdown(false) {
  }

  ThreadPool(const ThreadPool &) = delete;
  ThreadPool(ThreadPool &&) = delete;

  ThreadPool & operator=(const ThreadPool &) = delete;
  ThreadPool & operator=(ThreadPool &&) = delete;

  // Inits thread pool
  void init();
  // Waits until threads finish their current task and shutdowns the pool
  void shutdown();
  // Submit a function to be executed asynchronously by the pool
  template<typename F, typename...Args>
  auto submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))> //->提前申明函数返回值类型,decltype自动推导类型
  {
    // Create a function with bounded parameters ready to execute
    //使用bind将任务函数包装成std::function，涉及到使用万能引用+std::froward实现完美转发
    std::function<decltype(f(args...))()> func = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
    // Encapsulate it into a shared ptr in order to be able to copy construct / assign 
    //使用std::packaged_task对任务function进行二次包装，使其能返回std::future，即允许异步获取该可调用对象产生的结果
    auto task_ptr = std::make_shared<std::packaged_task<decltype(f(args...))()>>(func);

    // Wrap packaged task into void function
    //使用function进行最后一次打包，主要目的是将函数通过lamda表达式封装成返回值为void的function
    //使用lambda表达式捕获智能指针，其计数+1，生命周期延长
    std::function<void()> wrapper_func = [task_ptr]() {
      (*task_ptr)(); 
    };

    // Enqueue generic wrapper function
    m_queue.enqueue(wrapper_func);

    // Wake up one thread if its waiting
    m_conditional_lock.notify_one();

    // Return future from promise
    return task_ptr->get_future();
  }
};
