#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include "call_every_handler.hpp"
#include "mavsdk_time.hpp"
#include "mavlink_include.hpp"
#include "timesync_estimator.hpp"

namespace mavsdk {

class SystemImpl;

class Timesync {
public:
    explicit Timesync(SystemImpl& parent);
    ~Timesync();

    void enable();

    // Thread-safe snapshot of the current estimate. The local clock is steady_clock.
    TimesyncEstimator::Estimate estimate() const;

    Timesync(const Timesync&) = delete;
    Timesync& operator=(const Timesync&) = delete;

private:
    SystemImpl& _system_impl;

    void process_timesync(const mavlink_message_t& message);
    void send_timesync(int64_t tc1, int64_t ts1);
    void send_timesync_tick();
    int64_t steady_now_ns();
    bool take_outstanding(int64_t ts1);
    void update_autopilot_time(const TimesyncEstimator::Estimate& estimate);

    static constexpr double TIMESYNC_SEND_INTERVAL_S = 1.0;
    CallEveryHandler::Cookie _send_cookie{};

    // ts1 of our requests in flight. A reply is only used if it echoes one of these: on a
    // shared link the autopilot also answers other agents' requests, and ArduPilot forwards
    // requests between its links.
    static constexpr size_t MAX_OUTSTANDING = 16;
    std::array<int64_t, MAX_OUTSTANDING> _outstanding{};
    size_t _outstanding_next{0};

    mutable std::mutex _estimator_mutex{};
    TimesyncEstimator _estimator{};

    bool _is_enabled{false};
};
} // namespace mavsdk
