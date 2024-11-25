#include "mem/mess_mem.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/MessMem.hh"
#include "mem/packet.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace memory
{

MessMemory::MessMemory(const MessMemoryParams &p) :
    SimpleMemory(p),
    stats(*this),
    samplingWindow(p.sampling_window)
{

    // Very bad hack to get the ratio between reads and writes
    // from the filename
    auto get_rw_ratio = [](std::string path) {
        std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
        std::string noext =
            base_filename.substr(0, base_filename.find_last_of("."));
        std::string rw_ratio_str = noext.substr(
            noext.find_last_of("bwlat_") + 1
        );
        unsigned n_rw_ratio = std::atoll(rw_ratio_str.c_str());
        return n_rw_ratio;
    };

    // Read from the directory containing the path of the curves
    DPRINTF(MessMem, "Getting curves from %s\n", p.curves_path);

    for (const auto &curve_path :
        std::filesystem::directory_iterator(p.curves_path)) {
        DPRINTF(MessMem, "Reading file: %s\n", curve_path);
        std::ifstream curve_file(curve_path.path());

        Curve curve;
        double tmp_lat;
        double tmp_bw;

        while (curve_file >> tmp_bw >> tmp_lat) {
        curve.push_back(std::make_pair(tmp_lat, tmp_bw));
        }

        auto rw_ratio = get_rw_ratio(curve_path.path());
        panic_if(rw_ratio % 2 != 0, "rw_ratio must be even");
        curve.sort();
        curveMap[rw_ratio >> 1] = curve;
    }
    DPRINTF(MessMem, "Read %d curves from %s\n", curveMap.size(),
            p.curves_path);
}

Tick
MessMemory::getLatency() const
{
    return currentLatency;
}

bool
MessMemory::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(MessMem, "recvTimingReq: request %s addr %#x size %d\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    pkt->headerDelay = 0;
    pkt->payloadDelay = 0;
    if (!didReceiveFirstRequest) {
        lastUpdate = curTick();
        didReceiveFirstRequest = true;
    }

    readReqCount += pkt->isRead();
    writeReqCount += pkt->isWrite();
    bytesReadCtrl += (pkt->isRead() * pkt->getSize());
    bytesWrittenCtrl += (pkt->isWrite() * pkt->getSize());

    // Every samplingWindow requests, update latency
    if (((readReqCount + writeReqCount) % samplingWindow) == 0) {
        targetLatency =
            Tick(double(getClosestLatency()) * (1 + latencyOverflowFactor));

        // Increment in 5% amts
        auto increment = exponentialIncrement(currentLatency, targetLatency);
        auto next_latency = Tick(double(currentLatency) + increment);
        DPRINTF(MessMem,
                "Modifying latency: cur = %lld, next = %lld, target = %lld\n",
                currentLatency, next_latency, targetLatency);
        currentLatency = next_latency;

        bytesReadCtrl = 0;
        bytesWrittenCtrl = 0;
        lastUpdate = curTick();
    }

    return SimpleMemory::recvTimingReq(pkt);
}

void
MessMemory::dequeue()
{
    stats.totalRequestedBandwidth += lastBandwidth;
    stats.bandwidthRequestedCount += 1;
    stats.totalDispatchedLatency += ticksToCycles(currentLatency);
    stats.latencyDispatchedCount += 1;

    SimpleMemory::dequeue();
}

Tick
MessMemory::getClosestMatchingLatency()
{
    unsigned char rw_ratio = std::ceil(
        static_cast<double>(readReqCount)
        / static_cast<double>(readReqCount + writeReqCount)
        * 100.0
    );

    // round (up) to nearest multiple of 2
    rw_ratio = (rw_ratio >> 1) << 1;

    double bytes_rw = bytesReadCtrl + bytesWrittenCtrl;
    double bandwidth_mb =
        bytes_rw / (double(curTick() - lastUpdate) / TICKS_PER_NS) * 1000;
    DPRINTF(MessMem, "RW Ratio is %d: got %d reads and %d writes\n", rw_ratio,
            readReqCount, writeReqCount);
    DPRINTF(MessMem, "Estimated BW is %.4f MB/s\n", bandwidth_mb);

    lastBandwidth = bandwidth_mb;

    // got 51 curves -> store them as their rw_ratio / 2
    const auto &curve = curveMap[rw_ratio >> 1];
    // getClosestMatchingLatency returns lat in cycles
    // I need it in ticks
    auto scaled_latency =
        curve.getClosestMatchingLatency(bandwidth_mb, latencyOverflowFactor);

    if (systemLatency >= scaled_latency) {
        DPRINTF(MessMem, "Got minimum latency, returning 0\n");
        return 0;
    }

    return cyclesToTicks(scaled_latency - systemLatency);
}

double
MessMemory::exponentialIncrement(double current_latency, double target_latency)
{
    const double EXPONENTIAL_INCREMENT_FACTOR = 20.0;
    return (target_latency - current_latency) / EXPONENTIAL_INCREMENT_FACTOR;
}

// Curve Object

void
MessMemory::Curve::push_back(MessMemory::Curve::Point point)
{
    curve.push_back(point);
}

const MessMemory::Curve::Point&
MessMemory::Curve::min() const
{
    return curve.back();
}

const MessMemory::Curve::Point&
MessMemory::Curve::max() const
{
    return curve.front();
}

void
MessMemory::Curve::sort()
{
    push_back(std::make_pair(0, 0));
    std::sort(
        curve.begin(), curve.end(),
        [] (const auto &cur_pair, const auto &other_pair) {
            return cur_pair.second > other_pair.second;
        }
    );
}

Cycles
MessMemory::Curve::getClosestLatency(double bandwidth,
        double &oflow_factor) const
{
    const auto &name = []() { return "::Curve"; };

    // Check if I'm over the range or under
    auto highest_bw = curve.front().second;
    auto lowest_bw = curve.back().second;
    auto highest_lat = curve.front().first;
    auto lowest_lat = curve.back().first;

    // Boundary conditions:
    // I'm outside the curve
    if (bandwidth >= highest_bw) {
        latency_overflow_factor += 0.02;
        DPRINTF(MessMem,
                "Bandwidth over limit: responding with (max) lat = %lld\n",
                highest_lat);
        return Cycles(highest_lat);
    }

    if (bandwidth <= lowest_bw) {
        DPRINTF(MessMem,
                "Bandwidth below limit: responding with (min) lat = %lld\n",
                lowest_lat);
        return Cycles(lowest_lat);
    }

    // Main:
    // Look for a point within the curve,
    // interpolate using the current and next bandwidth value
    const auto &lower_bound_it = std::lower_bound(
        curve.begin(), curve.end(), bandwidth,
        [](const auto& [cur_pair, bandwidth]) {
            return cur_pair.second > bandwidth;
        }
    );

    const auto lower_bound_lat = lower_bound_it->first;
    const auto lower_bound_bw = lower_bound_it->second;

    const auto &upper_bound_it = std::prev(lower_bound_it);
    const auto upper_bound_lat = upper_bound_it->first;
    const auto upper_bound_bw = upper_bound_it->second;

    const auto slope =
        (bandwidth - lower_bound_bw) / (upper_bound_bw - lower_bound_bw);
    const auto latency_increase_from_base =
        (upper_bound_lat - lower_bound_lat) * slope;
    auto latency_value = lower_bound_lat + latency_increase_from_base;

    // Clamp latency to minimum value in the curve
    if (lower_bound_lat == 0)
        latency_value = upper_bound_lat;

    DPRINTF(MessMem, "Got Latency = %f. BW = [%f, %f], LAT = [%f, %f]\n",
            latency_value, lower_bound_bw, upper_bound_bw, lower_bound_lat,
            upper_bound_lat);

    latency_overflow_factor = std::max(0.0, latency_overflow_factor - 0.01);

    return Cycles(latency_value);
}

// Stats
MessMemory::MessMemoryStats::MessMemoryStats(MessMemory &_ctrl) :
    statistics::Group(&_ctrl),
    ctrl(_ctrl),

    ADD_STAT(totalDispatchedLatency, statistics::units::Cycle::get(),
            "Total Latency dispatched by the MessMemory"),
    ADD_STAT(latencyDispatchedCount, statistics::units::Count::get(),
            "Number of times a response got dispatched by the MessMemory"),
    ADD_STAT(totalRequestedBandwidth, statistics::units::Byte::get(),
            "Total bandwidth requested from the controller"),
    ADD_STAT(bandwidthRequestedCount, statistics::units::Count::get(),
            "Number of times bandiwdth was exhamined"),
    ADD_STAT(totalDispatchedBandwidth, statistics::units::Byte::get(),
            "Total bandwidth dispatched from the controller"),
    ADD_STAT(bandwidthDispatchedCount, statistics::units::Count::get(),
            "Number of times bandiwdth-dispatched was exhamined")
{}

void
MessMemory::MessMemoryStats::regStats()
{
    using namespace statistics;

    totalDispatchedLatency = 0;
    latencyDispatchedCount = 0;

    totalRequestedBandwidth = 0;
    bandwidthRequestedCount = 0;

    totalDispatchedBandwidth = 0;
    bandwidthDispatchedCount = 0;
}

} // namespace memory
} // namespace gem5
