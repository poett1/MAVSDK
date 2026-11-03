#include "timesync_schedule.hpp"
#include "timesync_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <functional>

using namespace mavsdk;

namespace {

constexpr int64_t ms = 1'000'000;
constexpr int64_t s = 1'000'000'000;

TimesyncEstimator::Estimate good_estimate()
{
    TimesyncEstimator::Estimate estimate;
    estimate.valid = true;
    estimate.uncertainty_ns = TimesyncSchedule::FAST_UNTIL_UNCERTAINTY_NS;
    return estimate;
}

// An autopilot whose clock runs drift_ppm fast and whose main loop handles requests on ticks
// every 20 ms of its own time, phase_ns into the loop: unlike a randomly drawn wait, this
// keeps evenly spaced requests at the same loop phase, as measured on ArduPlane.
struct LoopLink {
    int64_t offset_ns;
    double drift_ppm;
    int64_t phase_ns;
    static constexpr int64_t loop_ns = 20 * ms;
    static constexpr int64_t min_leg = 200'000;

    int64_t remote_at(int64_t local_ns) const
    {
        return local_ns + offset_ns +
               static_cast<int64_t>(std::llround(static_cast<double>(local_ns) * drift_ppm * 1e-6));
    }

    int64_t local_at(int64_t remote_ns) const
    {
        return static_cast<int64_t>(
            std::llround(static_cast<double>(remote_ns - offset_ns) / (1.0 + drift_ppm * 1e-6)));
    }

    TimesyncEstimator::Exchange exchange(int64_t local_send_ns) const
    {
        // The first tick at or after the arrival. std::ceil, not integer division, which
        // rounds towards zero and so the wrong way for negative times.
        const int64_t arrived = remote_at(local_send_ns + min_leg);
        const auto ticks = static_cast<int64_t>(
            std::ceil(static_cast<double>(arrived - phase_ns) / static_cast<double>(loop_ns)));
        const int64_t handled = ticks * loop_ns + phase_ns;
        return {local_send_ns, handled, local_at(handled) + min_leg};
    }
};

// Worst |error| of the estimate over boots at every 1 ms of loop phase, after up to
// duration_ns of requests spaced by next_interval_ns().
int64_t worst_error_after(int64_t duration_ns, const std::function<int64_t()>& next_interval_ns)
{
    int64_t worst = 0;
    for (int64_t phase = 0; phase < LoopLink::loop_ns; phase += 1 * ms) {
        const LoopLink link{-290 * s, 935.0, phase};
        TimesyncEstimator estimator;
        const int64_t boot = 100 * s;
        for (int64_t t = boot; t < boot + duration_ns; t += next_interval_ns()) {
            estimator.add(link.exchange(t));
        }
        const auto& est = estimator.estimate();
        EXPECT_TRUE(est.valid);
        const int64_t error = est.offset_ns - (link.remote_at(est.local_ns) - est.local_ns);
        EXPECT_LE(std::abs(error), est.uncertainty_ns) << "phase " << phase / ms << " ms";
        worst = std::max(worst, std::abs(error));
    }
    return worst;
}

} // namespace

TEST(TimesyncSchedule, FastWithJitterAfterStart)
{
    TimesyncSchedule schedule(1);
    schedule.restart(0);
    double lowest = 1.0;
    double highest = 0.0;
    for (int i = 0; i < 1000; ++i) {
        // 1000 requests within the fast phase's 10 s minimum.
        const double interval = schedule.next_interval_s(int64_t(i) * 10 * ms, {});
        lowest = std::min(lowest, interval);
        highest = std::max(highest, interval);
    }
    EXPECT_TRUE(schedule.fast());
    EXPECT_GE(lowest, TimesyncSchedule::FAST_INTERVAL_S);
    EXPECT_LE(highest, TimesyncSchedule::FAST_INTERVAL_S + TimesyncSchedule::JITTER_S);
    // The delay has to spread over the whole loop, not just a part of it.
    EXPECT_LT(lowest, TimesyncSchedule::FAST_INTERVAL_S + 0.002);
    EXPECT_GT(highest, TimesyncSchedule::FAST_INTERVAL_S + TimesyncSchedule::JITTER_S - 0.002);
}

TEST(TimesyncSchedule, StaysFastForTheMinimumEvenWhenGood)
{
    TimesyncSchedule schedule(1);
    schedule.restart(5 * s);
    EXPECT_LT(schedule.next_interval_s(14 * s, good_estimate()), 0.5);
    EXPECT_TRUE(schedule.fast());
}

TEST(TimesyncSchedule, SlowsDownOnceGood)
{
    TimesyncSchedule schedule(1);
    schedule.restart(5 * s);
    auto poor = good_estimate();
    poor.uncertainty_ns += 1;
    EXPECT_LT(schedule.next_interval_s(20 * s, poor), 0.5);
    const double interval = schedule.next_interval_s(20 * s, good_estimate());
    EXPECT_FALSE(schedule.fast());
    EXPECT_GE(interval, TimesyncSchedule::SLOW_INTERVAL_S);
    EXPECT_LE(interval, TimesyncSchedule::SLOW_INTERVAL_S + TimesyncSchedule::JITTER_S);
    // And stays slow even if the estimate gets worse again.
    EXPECT_GE(schedule.next_interval_s(21 * s, poor), TimesyncSchedule::SLOW_INTERVAL_S);
}

TEST(TimesyncSchedule, SlowsDownAfterTheMaximumAnyway)
{
    TimesyncSchedule schedule(1);
    schedule.restart(0);
    EXPECT_LT(schedule.next_interval_s(29 * s, {}), 0.5);
    EXPECT_GE(schedule.next_interval_s(30 * s, {}), TimesyncSchedule::SLOW_INTERVAL_S);
}

TEST(TimesyncSchedule, RestartGoesFastAgain)
{
    TimesyncSchedule schedule(1);
    schedule.restart(0);
    schedule.next_interval_s(60 * s, good_estimate());
    ASSERT_FALSE(schedule.fast());
    schedule.restart(61 * s);
    EXPECT_TRUE(schedule.fast());
    EXPECT_LT(schedule.next_interval_s(62 * s, good_estimate()), 0.5);
}

// The reason for the jitter: evenly spaced requests stay at one loop phase, so for some
// phases the estimate is still 5-7 ms off after 5 and 10 s. With the schedule's delays the
// worst phase is within 2 ms after 5 s and within 1 ms after 10 s (0.35-1.7 ms and
// 0.24-0.69 ms over ten seeds, matching replays of ArduPlane recordings).
TEST(TimesyncSchedule, RandomDelayFindsTheLoopPhaseWithinSeconds)
{
    const auto even = [] { return 100 * ms; };
    EXPECT_GT(worst_error_after(5 * s, even), 5 * ms);
    EXPECT_GT(worst_error_after(10 * s, even), 4 * ms);

    for (const int64_t duration : {5 * s, 10 * s}) {
        TimesyncSchedule schedule(7);
        schedule.restart(0);
        const int64_t jittered_worst = worst_error_after(duration, [&] {
            return static_cast<int64_t>(std::llround(schedule.next_interval_s(0, {}) * 1e9));
        });
        EXPECT_LT(jittered_worst, duration == 5 * s ? 2 * ms : 1 * ms)
            << "after " << duration / s << " s";
    }
}
