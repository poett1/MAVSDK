#include "timesync_schedule.hpp"

namespace mavsdk {

TimesyncSchedule::TimesyncSchedule(uint64_t seed) : _rng(seed) {}

void TimesyncSchedule::restart(int64_t now_ns)
{
    _fast_since_ns = now_ns;
    _fast = true;
}

double
TimesyncSchedule::next_interval_s(int64_t now_ns, const TimesyncEstimator::Estimate& estimate)
{
    if (_fast) {
        const double elapsed_s = static_cast<double>(now_ns - _fast_since_ns) * 1e-9;
        const bool good = estimate.valid && estimate.uncertainty_ns <= FAST_UNTIL_UNCERTAINTY_NS;
        if (elapsed_s >= FAST_MAX_S || (elapsed_s >= FAST_MIN_S && good)) {
            _fast = false;
        }
    }

    return (_fast ? FAST_INTERVAL_S : SLOW_INTERVAL_S) + _jitter(_rng);
}

} // namespace mavsdk
