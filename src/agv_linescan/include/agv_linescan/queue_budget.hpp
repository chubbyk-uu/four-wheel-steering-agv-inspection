#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <utility>
namespace agv_linescan {
// External synchronization required. Times are monotonic wall-clock seconds.
// Accounts for waiting jobs; in-flight batches are bounded separately.
class QueueBudget {
 public:
  QueueBudget(size_t lines=2048,size_t jobs=512,double age=.15):lineLimit(lines),jobLimit(jobs),ageLimit(age) {
    if(!lines||!jobs||!std::isfinite(age)||age<=0)throw std::invalid_argument("invalid sampling queue budget");
  }
  void Push(size_t count,double now) {
    CheckAge(now);
    if(!count || count>lineLimit-lines || entries.size()>=jobLimit)
      throw std::runtime_error("bounded sampling queue capacity exceeded");
    entries.emplace_back(count,now);lines+=count;
    peakLines=std::max(peakLines,lines);peakJobs=std::max(peakJobs,entries.size());
  }
  void Pop(double now) {
    CheckAge(now);if(entries.empty())throw std::logic_error("empty sampling queue budget");
    lines-=entries.front().first;entries.pop_front();
  }
  void CheckAge(double now) {
    if(!std::isfinite(now))throw std::invalid_argument("nonfinite queue time");
    if(!entries.empty()) {
      double age=now-entries.front().second;
      if(age<0)throw std::logic_error("queue clock moved backwards");
      peakAge=std::max(peakAge,age);
      if(age>ageLimit)throw std::runtime_error("sampling queue wait deadline exceeded");
    }
  }
  void Clear(){entries.clear();lines=0;}
  size_t Jobs()const{return entries.size();}
  const size_t lineLimit,jobLimit;
  const double ageLimit;
  size_t lines=0,peakLines=0,peakJobs=0;
  double peakAge=0;
 private:
  std::deque<std::pair<size_t,double>> entries;
};
}
