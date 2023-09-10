#include <chrono>
#include <thread>
#include <atomic>

#include "../fmtlog.h"

#include "monolithic_examples.h"

using namespace std;

static const int thr_cnt = 4;
static atomic<int> start_cnt;

static size_t msg_cnt = 0;
static size_t cb_size = 0;

static void logcb(int64_t ns, fmtlog::LogLevel level, fmt::string_view location, size_t basePos, fmt::string_view threadName,
           fmt::string_view msg, size_t bodyPos, size_t logFilePos) {
  msg_cnt++;
  cb_size += msg.size();
}

static void threadRun(int id) {
  fmtlog::setThreadName(fmt::format("thread {}", id).c_str());
  start_cnt++;
  while (start_cnt < thr_cnt)
    ;
  for (int i = 0; i < 100000; i++) {
    logi("msg : {}, i: {}", id, i);
    if (i % 1000 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}


#if defined(BUILD_MONOLITHIC)
#define main     fmtlog_multithread_test_main
#endif

int main(void) {
  fmtlog::setLogCB(logcb, fmtlog::INF);
  // fmtlog::closeLogFile();
  fmtlog::setLogFile("multithread.txt");
  // fmtlog::setHeaderPattern("");
  vector<thread> thrs;
  fmtlog::startPollingThread(1);
  for (int i = 0; i < thr_cnt; i++) {
    thrs.emplace_back([i]() { threadRun(i); });
  }

  while (start_cnt < thr_cnt)
    ;
  std::chrono::high_resolution_clock::time_point t0, t1;
  t0 = std::chrono::high_resolution_clock::now();

  for (auto& t : thrs) {
    t.join();
  }

  fmtlog::stopPollingThread();

  t1 = std::chrono::high_resolution_clock::now();
  double span = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();

  fmt::print("total msg_cnt: {}, cb_size: {}, throughput: {:.1f} MB/second, {} msg/sec\n", msg_cnt, cb_size,
             (cb_size / 1000.0 / 1000 / span), (msg_cnt) / span);

  return 0;
}
