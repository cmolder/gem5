#ifndef __MEM_MESS_HH__
#define __MEM_MESS_HH__

#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <list>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "mem/abstract_mem.hh"
#include "mem/port.hh"
#include "mem/simple_mem.hh"
#include "params/MessMemory.hh"

namespace gem5
{

namespace memory
{

class MessMemory : public SimpleMemory
{
  private:
    class Curve
    {
      public:
        using Point = std::pair<double, double>;

      private:
        std::vector<Point> curve;

      public:
        void push_back(Point point);
        const Point& min() const;
        const Point& max() const;
        Cycles getClosestLatency(double bandwidth, double& oflow_factor) const;
        void sort();
    };

    struct MessMemoryStats : public statistics::Group
    {
        MessMemoryStats(MessMemory &ctrl);

        void regStats() override;

        MessMemory &ctrl;
        statistics::Scalar totalDispatchedLatency;
        statistics::Scalar latencyDispatchedCount;

        statistics::Scalar totalRequestedBandwidth;
        statistics::Scalar bandwidthRequestedCount;

        statistics::Scalar totalDispatchedBandwidth;
        statistics::Scalar bandwidthDispatchedCount;
    };

    MessMemoryStats stats;

    Tick getClosestMatchingLatency();
    double exponentialIncrement(double, double);\

    std::array<Curve, 51> curveMap;
    uint64_t readReqCount{0};
    uint64_t writeReqCount{0};

    const uint32_t samplingWindow;
    Tick currentLatency{20000};
    Tick targetLatency{20000};
    double lastBandwidth{0};
    Cycles systemLatency{250};
    bool didReceiveFirstRequest{false};
    double latencyOverflowFactor = 0.0;

    static constexpr uint16_t TICKS_PER_NS = 1000;
    uint64_t bytesReadCtrl{0};
    uint64_t bytesWrittenCtrl{0};
    Tick lastUpdate{curTick()};
    Tick lastBandwidthReported{curTick()};

    uint32_t dispatchedRequests{0};

  protected:
    virtual Tick getLatency() const;
    virtual bool recvTimingReq(PacketPtr pkt);
    virtual void dequeue();

  public:
    MessMemory(const MessMemoryParams &p);

};

} // namespace memory
} // namespace gem5

#endif //__MEM_MESS_HH__
