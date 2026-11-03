#include "timesync_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

using namespace mavsdk;

namespace {

constexpr int64_t ms = 1'000'000;
constexpr int64_t s = 1'000'000'000;

// A remote clock running drift_ppm fast with offset offset_ns at local time 0, and a link
// modelled on ArduPilot over USB: the request waits up to one 20 ms loop before tc1 is
// stamped, and the reply goes out right away. Each leg takes at least 0.2 ms; no estimate
// can be tighter than the sum of the two minimum delays.
struct Link {
    int64_t offset_ns;
    double drift_ppm;
    std::mt19937 rng{42};
    static constexpr int64_t min_leg = 200'000;

    int64_t remote_at(int64_t local_ns) const
    {
        return local_ns + offset_ns +
               static_cast<int64_t>(std::llround(static_cast<double>(local_ns) * drift_ppm * 1e-6));
    }

    TimesyncEstimator::Exchange exchange(int64_t local_send_ns)
    {
        std::uniform_int_distribution<int64_t> loop_wait(0, 20 * ms);
        std::exponential_distribution<double> jitter(1.0 / (0.5 * ms));
        const int64_t handled = local_send_ns + min_leg + loop_wait(rng);
        const int64_t recv = handled + min_leg + static_cast<int64_t>(jitter(rng));
        return {local_send_ns, remote_at(handled), recv};
    }
};

int64_t true_offset_at(const Link& link, int64_t local_ns)
{
    return link.remote_at(local_ns) - local_ns;
}

} // namespace

TEST(TimesyncEstimator, NotValidUntilEnoughExchanges)
{
    TimesyncEstimator estimator;
    Link link{-290 * s, 0.0};
    estimator.add(link.exchange(1 * s));
    EXPECT_FALSE(estimator.estimate().valid);
    EXPECT_EQ(estimator.estimate().drift_ppm, 0.0);
    estimator.add(link.exchange(2 * s));
    estimator.add(link.exchange(3 * s));
    EXPECT_TRUE(estimator.estimate().valid);
    EXPECT_EQ(estimator.estimate().exchanges, 3u);
}

TEST(TimesyncEstimator, TrueOffsetWithinUncertaintyDespiteAsymmetricLink)
{
    TimesyncEstimator estimator;
    Link link{-290 * s, 860.0};
    int64_t worst_error = 0;
    int64_t worst_midpoint_error = 0;
    for (int64_t t = 1 * s; t < 120 * s; t += 1 * s) {
        const auto e = link.exchange(t);
        estimator.add(e);
        const auto& est = estimator.estimate();
        if (!est.valid) {
            continue;
        }
        const int64_t error = est.offset_ns - true_offset_at(link, est.local_ns);
        EXPECT_LE(std::abs(error), est.uncertainty_ns) << "at t=" << t / s << " s";
        if (est.exchanges >= 10) {
            worst_error = std::max(worst_error, std::abs(error));
        }

        const int64_t midpoint = e.remote_ns - (e.local_send_ns + e.local_recv_ns) / 2;
        worst_midpoint_error = std::max(
            worst_midpoint_error,
            std::abs(midpoint - true_offset_at(link, (e.local_send_ns + e.local_recv_ns) / 2)));
    }
    // The round-trip midpoint carries half the loop wait; the bound fit does not.
    EXPECT_GT(worst_midpoint_error, 5 * ms);
    EXPECT_LT(worst_error, 1 * ms);
    EXPECT_NEAR(estimator.estimate().drift_ppm, 860.0, 20.0);
}

TEST(TimesyncEstimator, ConvergesWithMoreExchanges)
{
    TimesyncEstimator estimator;
    Link link{12345 * ms, -40.0};
    for (int64_t t = 0; t < 60 * s; t += 100 * ms) {
        estimator.add(link.exchange(t));
    }
    const auto& est = estimator.estimate();
    EXPECT_LT(est.uncertainty_ns, 500'000);
    EXPECT_LE(std::abs(est.offset_ns - true_offset_at(link, est.local_ns)), est.uncertainty_ns);
    EXPECT_NEAR(est.drift_ppm, -40.0, 5.0);
}

TEST(TimesyncEstimator, RecoversFromRemoteClockStep)
{
    TimesyncEstimator estimator;
    Link before{-290 * s, 860.0};
    for (int64_t t = 1 * s; t <= 30 * s; t += 1 * s) {
        estimator.add(before.exchange(t));
    }
    // Autopilot reboot: its clock starts again from zero.
    Link after{-31 * s, 860.0};
    for (int64_t t = 31 * s; t <= 40 * s; t += 1 * s) {
        estimator.add(after.exchange(t));
    }
    const auto& est = estimator.estimate();
    EXPECT_TRUE(est.valid);
    EXPECT_LE(est.exchanges, 10u);
    EXPECT_LE(std::abs(est.offset_ns - true_offset_at(after, est.local_ns)), est.uncertainty_ns);
}

// A clock step in a full 10 Hz window: everything before the step goes, in one bisection
// rather than one drift search per dropped exchange (which stalled the MAVLink thread for
// 35 ms on a Raspberry Pi 5), and the exchanges after it are all kept.
TEST(TimesyncEstimator, ClockStepInFullWindowDropsOnlyTheOldClock)
{
    TimesyncEstimator estimator;
    Link before{-290 * s, 935.0};
    int64_t t = 100 * s;
    for (; t < 160 * s; t += 100 * ms) {
        estimator.add(before.exchange(t));
    }
    ASSERT_EQ(estimator.estimate().exchanges, 600u);

    Link after{-300 * s, 935.0};
    for (int i = 0; i < 5; ++i, t += 100 * ms) {
        estimator.add(after.exchange(t));
    }
    const auto& est = estimator.estimate();
    EXPECT_EQ(est.exchanges, 5u);
    EXPECT_TRUE(est.valid);
    EXPECT_LE(std::abs(est.offset_ns - true_offset_at(after, est.local_ns)), est.uncertainty_ns);
}

TEST(TimesyncEstimator, WindowDropsOldExchanges)
{
    TimesyncEstimator estimator{10 * s};
    Link link{0, 0.0};
    for (int64_t t = 0; t <= 30 * s; t += 1 * s) {
        estimator.add(link.exchange(t));
    }
    EXPECT_EQ(estimator.estimate().exchanges, 11u);
}

TEST(TimesyncEstimator, ConversionsRoundTrip)
{
    TimesyncEstimator::Estimate est;
    est.valid = true;
    est.local_ns = 1000 * s;
    est.offset_ns = -290 * s;
    est.drift_ppm = 860.0;
    for (int64_t local : {int64_t{0}, 999 * s, 1000 * s, 1001 * s, 5000 * s}) {
        const int64_t remote = TimesyncEstimator::to_remote(est, local);
        EXPECT_NEAR(
            static_cast<double>(TimesyncEstimator::to_local(est, remote)),
            static_cast<double>(local),
            2.0);
    }
    // At the reference time there is no drift term.
    EXPECT_EQ(TimesyncEstimator::to_remote(est, est.local_ns), est.local_ns + est.offset_ns);
    // One second later the remote clock has gained 860 us.
    EXPECT_EQ(
        TimesyncEstimator::to_remote(est, est.local_ns + 1 * s),
        est.local_ns + 1 * s + est.offset_ns + 860'000);
}

TEST(TimesyncEstimator, IgnoresImpossibleExchange)
{
    TimesyncEstimator estimator;
    estimator.add({10 * s, 5 * s, 9 * s});
    EXPECT_EQ(estimator.estimate().exchanges, 0u);
}

// A few exchanges 100 ms apart cannot bound the drift, and the widest band then sat at the
// end of the search range (-10000 ppm, seen right after start on ArduPlane). The estimate
// should keep a plausible drift until the exchanges tell it, and stay within its uncertainty.
TEST(TimesyncEstimator, UnboundedDriftStaysAtZero)
{
    TimesyncEstimator estimator;
    Link link{-290 * s, 860.0};
    for (int64_t t = 100 * s; t < 100 * s + 500 * ms; t += 100 * ms) {
        estimator.add(link.exchange(t));
    }
    const auto& est = estimator.estimate();
    ASSERT_TRUE(est.valid);
    EXPECT_EQ(est.drift_ppm, 0.0);
    const int64_t error = est.offset_ns - true_offset_at(link, est.local_ns);
    EXPECT_LE(std::abs(error), est.uncertainty_ns);

    // Once the exchanges span long enough, the drift comes from them again.
    for (int64_t t = 101 * s; t < 160 * s; t += 1 * s) {
        estimator.add(link.exchange(t));
    }
    EXPECT_NEAR(estimator.estimate().drift_ppm, 860.0, 20.0);
}
