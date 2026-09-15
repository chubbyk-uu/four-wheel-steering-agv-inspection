#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace agv_linescan {

// One physical step of everything the capture gate judges. Plain data: recording a
// step is an assignment into a preallocated slot, never an allocation, so the
// physics thread pays the same cost whether or not a fault ever happens.
struct FlightSample {
  double simTime=0;
  double driveRate[4]={};        // rad/s at the shaft, as reported by the joint
  double wheelSpeed[4]={};       // m/s at the contact, rate * radius
  double steerAngle[4]={};
  double suspensionPos[4]={};
  double suspensionVel[4]={};
  double wheelResidual[4]={};    // per wheel, not just the worst
  double vx=0,vy=0,wz=0,residual=0;
  double bodyX=0,bodyY=0,bodyZ=0;
  double bodyQx=0,bodyQy=0,bodyQz=0,bodyQw=1;
  // Differentiated from the previous step's pose: four wheels agreeing with each
  // other but disagreeing with this is whole-vehicle slip, which per-wheel
  // residuals alone cannot show.
  double bodyVx=0,bodyVy=0,bodyVz=0,bodyWz=0;  // base_link frame, matching the fit
  int32_t encoderPolarity=0;
  uint8_t motionState=0;         // index into the caller's motion name table
  // Every guard as it was actually evaluated, so a reader never re-derives them.
  uint8_t passSpeed=1,passLateral=1,passYaw=1,passResidual=1,passData=1,passPolarity=1;
  uint8_t capturing=0;
};

// Fixed-capacity ring trimmed by simulated time. The count limit is what bounds
// memory; the time span is what makes "two seconds" stay two seconds if the
// physics step size ever changes.
class FlightRecorder {
 public:
  FlightRecorder(std::size_t capacity,double span):span_(span) {
    if(capacity==0 || !(span>0))throw std::invalid_argument("invalid flight recorder bounds");
    buffer_.resize(capacity);
  }
  std::size_t capacity() const {return buffer_.size();}
  double span() const {return span_;}
  std::size_t size() const {return count_;}

  void Push(const FlightSample &sample) {
    if(count_==buffer_.size())Drop();
    buffer_[(head_+count_)%buffer_.size()]=sample;
    ++count_;
    while(count_>1 && sample.simTime-buffer_[head_].simTime>span_)Drop();
  }
  void Clear() {head_=0;count_=0;}

  // Guards that run after the step was recorded need to correct their own result
  // in place; a dump that shows a guard passing on the step it rejected is worse
  // than no dump at all.
  FlightSample* Newest() {return count_?&buffer_[(head_+count_-1)%buffer_.size()]:nullptr;}

  // Oldest first. Copying out is the caller's way to hand the physics thread's
  // data to a writer without holding the step.
  std::vector<FlightSample> Snapshot() const {
    std::vector<FlightSample> out;out.reserve(count_);
    for(std::size_t i=0;i<count_;++i)out.push_back(buffer_[(head_+i)%buffer_.size()]);
    return out;
  }

 private:
  void Drop() {head_=(head_+1)%buffer_.size();--count_;}
  std::vector<FlightSample> buffer_;
  std::size_t head_=0,count_=0;
  double span_;
};

// Running residual distribution held in memory, so that "how close does it
// normally run to the gate" can be answered without writing a sample per step.
class ResidualStats {
 public:
  explicit ResidualStats(double limit):limit_(limit>0?limit:1) {}
  void Add(double residual) {
    ++samples_;
    if(!std::isfinite(residual))return;
    max_=std::max(max_,residual);
    if(residual>limit_)++overLimit_;
    if(residual>limit_/2)++overHalf_;
    // Bins span [0, 2*limit); anything beyond lands in the last bin.
    std::size_t bin=static_cast<std::size_t>(residual/(2*limit_)*(kBins-1));
    histogram_[std::min(bin,kBins-1)]++;
  }
  double Quantile(double q) const {
    if(samples_==0)return 0;
    const std::uint64_t want=static_cast<std::uint64_t>(q*static_cast<double>(samples_));
    std::uint64_t seen=0;
    for(std::size_t i=0;i<kBins;++i){
      seen+=histogram_[i];
      if(seen>=want)return (i+1)*(2*limit_)/(kBins-1);
    }
    return max_;
  }
  std::uint64_t samples() const {return samples_;}
  std::uint64_t overLimit() const {return overLimit_;}
  std::uint64_t overHalf() const {return overHalf_;}
  double max() const {return max_;}
  double limit() const {return limit_;}
  static constexpr std::size_t kBins=32;
  const std::array<std::uint64_t,kBins>& histogram() const {return histogram_;}

 private:
  double limit_,max_=0;
  std::uint64_t samples_=0,overLimit_=0,overHalf_=0;
  std::array<std::uint64_t,kBins> histogram_{};
};

}  // namespace agv_linescan
