#include "../include/thread_pool/ThreadPool.h"

void ThreadPool::init()
{
  for (int i = 0; i < m_threads.size(); ++i) {
    m_threads[i] = std::thread(ThreadWorker(this, i));
  }
}

void ThreadPool::shutdown()
{
  m_shutdown = true;
  m_conditional_lock.notify_all();
  
  for (int i = 0; i < m_threads.size(); ++i) {
    if(m_threads[i].joinable()) {
      m_threads[i].join();
    }
  }
}