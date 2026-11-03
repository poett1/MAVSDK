#pragma once

#include <cstdint>
#include <deque>

#include "mavsdk_export.h"

namespace mavsdk {

/**
 * Estimates the offset and drift between the local clock and a remote clock from
 * TIMESYNC exchanges, without assuming that both legs of the round trip take equally long.
 *
 * Every exchange bounds the true offset (remote minus local):
 *
 *     remote - local_recv  <=  offset  <=  remote - local_send
 *
 * whatever the link asymmetry. A remote that stamps its receive time late (ArduPilot
 * stamps when its main loop gets to the message, up to a whole loop after arrival) moves
 * the round-trip midpoint by half that wait, but never breaks the bounds. The estimator
 * fits the line offset(t) = offset + drift * t that passes between all bounds in the
 * window with the widest margin and reports the centre of the remaining interval. The
 * uncertainty covers every line that fits, so the true offset is never further away.
 *
 * If no line fits (a clock stepped, e.g. the autopilot rebooted), the oldest exchanges are
 * dropped until one does.
 */
class MAVSDK_TEST_EXPORT TimesyncEstimator {
public:
    struct Exchange {
        int64_t local_send_ns; // local time just before the request was sent (ts1)
        int64_t remote_ns; // remote time at which it handled the request (tc1)
        int64_t local_recv_ns; // local time the reply was received
    };

    struct Estimate {
        bool valid{false};
        int64_t local_ns{0}; // local time the offset below refers to
        int64_t offset_ns{0}; // remote minus local, at local_ns
        double drift_ppm{0.0}; // how much faster the remote clock runs, in ppm
        int64_t uncertainty_ns{0}; // farthest the true offset at local_ns can be from offset_ns
        uint32_t exchanges{0}; // exchanges in the window
    };

    explicit TimesyncEstimator(int64_t window_ns = DEFAULT_WINDOW_NS);

    void add(const Exchange& exchange);
    void reset();
    const Estimate& estimate() const { return _estimate; }

    static int64_t to_remote(const Estimate& estimate, int64_t local_ns);
    static int64_t to_local(const Estimate& estimate, int64_t remote_ns);

    static constexpr int64_t DEFAULT_WINDOW_NS = 60'000'000'000;
    static constexpr uint32_t MIN_EXCHANGES = 3;
    // Search range for the drift. Crystals are within 100 ppm; a ceramic resonator was
    // measured at 860 ppm on an ArduPilot H7 board.
    static constexpr double MAX_DRIFT_PPM = 10000.0;

private:
    // Feasible interval [lo, hi] of the offset at the newest exchange for a given drift,
    // relative to _base_ns.
    void interval(double drift_ppm, double& lo, double& hi) const;
    double width(double drift_ppm) const;
    double best_drift() const;
    // Drift between inside (fits) and outside where the band closes.
    double feasible_drift_edge(double inside, double outside) const;
    void update();

    int64_t _window_ns;
    std::deque<Exchange> _exchanges{};
    int64_t _base_ns{0};
    Estimate _estimate{};
};

} // namespace mavsdk
