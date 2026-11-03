#pragma once

#include <cstdint>
#include <random>

#include "mavsdk_export.h"
#include "timesync_estimator.hpp"

namespace mavsdk {

/**
 * When to send the next TIMESYNC request.
 *
 * Every interval gets a random extra delay of up to one 50 Hz autopilot loop. ArduPilot stamps
 * tc1 when its main loop gets to the request, so the request waits up to a loop at the
 * autopilot. Intervals that are whole multiples of the loop (100 ms, 1 s) keep arriving at the
 * same loop phase and see the same wait; only the clock drift moves the phase, so sweeping the
 * loop took ~21 s at any such rate on an ArduPlane board, and until then the estimate was off by
 * up to 6 ms. The random delay samples every phase from the start.
 *
 * After (re)connecting, requests go out at about 10 Hz until the estimate is good, then at
 * about 1 Hz: ArduPilot forwards requests to its other links, so a high rate would also go out
 * over a telemetry radio.
 */
class MAVSDK_TEST_EXPORT TimesyncSchedule {
public:
    explicit TimesyncSchedule(uint64_t seed = std::random_device{}());

    // Start the fast phase, e.g. when the autopilot has (re)connected.
    void restart(int64_t now_ns);

    // Seconds until the next request, given the estimate after the one just sent.
    double next_interval_s(int64_t now_ns, const TimesyncEstimator::Estimate& estimate);

    bool fast() const { return _fast; }

    static constexpr double FAST_INTERVAL_S = 0.1;
    static constexpr double SLOW_INTERVAL_S = 1.0;
    static constexpr double JITTER_S = 0.02;
    // The fast phase lasts at least FAST_MIN_S, then ends once the uncertainty is down to
    // FAST_UNTIL_UNCERTAINTY_NS, and after FAST_MAX_S at the latest.
    static constexpr double FAST_MIN_S = 10.0;
    static constexpr double FAST_MAX_S = 30.0;
    // The reported uncertainty is conservative: it settles at 1.3-1.7 ms while the actual error
    // is 0.3-0.5 ms (ArduPlane over UART and USB), so 1 ms would always run to FAST_MAX_S.
    static constexpr int64_t FAST_UNTIL_UNCERTAINTY_NS = 2'000'000;

private:
    std::mt19937_64 _rng;
    std::uniform_real_distribution<double> _jitter{0.0, JITTER_S};
    int64_t _fast_since_ns{0};
    bool _fast{true};
};

} // namespace mavsdk
