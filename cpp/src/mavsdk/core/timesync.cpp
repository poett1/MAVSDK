#include "timesync.hpp"
#include "log.hpp"
#include "mavlink_address.hpp"
#include "system_impl.hpp"

#include <algorithm>

// The message exchange follows mavros' sys_time plugin; the estimate does not, see
// timesync_estimator.hpp.

namespace mavsdk {

Timesync::Timesync(SystemImpl& parent) : _system_impl(parent) {}

Timesync::~Timesync()
{
    // Blocking variants, so that neither the tick nor a message can be dispatched into us
    // while we are being destroyed.
    _system_impl.remove_call_every_blocking(_send_cookie);
    _system_impl.unregister_all_mavlink_message_handlers_blocking(this);
}

void Timesync::enable()
{
    if (_is_enabled) {
        // System::enable_timesync() is public API and used to be idempotent, so keep it that
        // way: registering the tick twice would double the timesync rate and leak a cookie.
        return;
    }

    _is_enabled = true;
    _system_impl.register_mavlink_message_handler(
        MAVLINK_MSG_ID_TIMESYNC,
        [this](const mavlink_message_t& message) { process_timesync(message); },
        this);

    // This used to be a do_work() that compared elapsed time on every pass of the system
    // work poll. It is just a fixed interval, so let CallEveryHandler keep the time.
    _send_cookie = _system_impl.add_call_every(
        [this]() { send_timesync_tick(); }, static_cast<float>(TIMESYNC_SEND_INTERVAL_S));
}

TimesyncEstimator::Estimate Timesync::estimate() const
{
    std::lock_guard<std::mutex> lock(_estimator_mutex);
    return _estimator.estimate();
}

int64_t Timesync::steady_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               _system_impl.get_time().steady_time().time_since_epoch())
        .count();
}

void Timesync::send_timesync_tick()
{
    if (!_is_enabled) {
        return;
    }

    if (!_system_impl.is_connected()) {
        // The autopilot may come back rebooted; start from scratch rather than wait for the
        // old exchanges to be found inconsistent.
        std::lock_guard<std::mutex> lock(_estimator_mutex);
        _estimator.reset();
        return;
    }

    const int64_t ts1 = steady_now_ns();
    _outstanding[_outstanding_next] = ts1;
    _outstanding_next = (_outstanding_next + 1) % MAX_OUTSTANDING;
    send_timesync(0, ts1);
}

bool Timesync::take_outstanding(int64_t ts1)
{
    if (ts1 == 0) {
        return false;
    }
    auto it = std::find(_outstanding.begin(), _outstanding.end(), ts1);
    if (it == _outstanding.end()) {
        return false;
    }
    *it = 0;
    return true;
}

void Timesync::process_timesync(const mavlink_message_t& message)
{
    const int64_t now_ns = steady_now_ns();

    mavlink_timesync_t timesync{};
    mavlink_msg_timesync_decode(&message, &timesync);

    if (timesync.tc1 == 0) {
        // A request from the autopilot: answer with our estimate of its own clock, so that
        // it sees us as synchronised to it.
        const auto est = estimate();
        if (est.valid) {
            send_timesync(TimesyncEstimator::to_remote(est, now_ns), timesync.ts1);
        }
        return;
    }

    if (!take_outstanding(timesync.ts1)) {
        return;
    }

    TimesyncEstimator::Estimate est;
    {
        std::lock_guard<std::mutex> lock(_estimator_mutex);
        _estimator.add({timesync.ts1, timesync.tc1, now_ns});
        est = _estimator.estimate();
    }
    if (est.valid) {
        update_autopilot_time(est);
    }
}

void Timesync::update_autopilot_time(const TimesyncEstimator::Estimate& estimate)
{
    // AutopilotTime, which the mocap plugin uses, is kept relative to the system clock and
    // can only be shifted, so shift it by what it is off from the estimate.
    auto& autopilot_time = _system_impl.get_autopilot_time();
    const int64_t wanted_ns = TimesyncEstimator::to_remote(estimate, steady_now_ns());
    const int64_t current_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   autopilot_time.now().time_since_epoch())
                                   .count();
    autopilot_time.shift_time_by(std::chrono::nanoseconds(wanted_ns - current_ns));
}

void Timesync::send_timesync(int64_t tc1, int64_t ts1)
{
    _system_impl.queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_message_t message;
        mavlink_msg_timesync_pack_chan(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &message,
            tc1,
            ts1,
            0,
            0);
        return message;
    });
}

} // namespace mavsdk
