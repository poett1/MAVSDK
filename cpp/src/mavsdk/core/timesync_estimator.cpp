#include "timesync_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mavsdk {

TimesyncEstimator::TimesyncEstimator(int64_t window_ns) : _window_ns(window_ns) {}

void TimesyncEstimator::reset()
{
    _exchanges.clear();
    _estimate = {};
}

void TimesyncEstimator::add(const Exchange& exchange)
{
    if (exchange.local_recv_ns < exchange.local_send_ns) {
        return;
    }
    _exchanges.push_back(exchange);
    while (!_exchanges.empty() &&
           _exchanges.back().local_send_ns - _exchanges.front().local_send_ns > _window_ns) {
        _exchanges.pop_front();
    }
    update();
}

void TimesyncEstimator::interval(double drift_ppm, double& lo, double& hi) const
{
    // Times relative to the newest exchange, offsets relative to _base_ns, both in doubles:
    // the window is at most minutes long, so the values stay far inside double precision.
    const int64_t t_ref = _exchanges.back().local_send_ns;
    const double rate = drift_ppm * 1e-6;
    lo = -std::numeric_limits<double>::infinity();
    hi = std::numeric_limits<double>::infinity();
    for (const auto& e : _exchanges) {
        const double t = static_cast<double>(e.local_send_ns - t_ref);
        const double drift = rate * t;
        lo = std::max(lo, static_cast<double>(e.remote_ns - e.local_recv_ns - _base_ns) - drift);
        hi = std::min(hi, static_cast<double>(e.remote_ns - e.local_send_ns - _base_ns) - drift);
    }
}

double TimesyncEstimator::width(double drift_ppm) const
{
    double lo, hi;
    interval(drift_ppm, lo, hi);
    return hi - lo;
}

double TimesyncEstimator::best_drift() const
{
    // The width is a minimum of linear functions minus a maximum of linear functions of the
    // drift, so it is concave and a golden-section search finds its maximum.
    const double inv_phi = (std::sqrt(5.0) - 1.0) / 2.0;
    double a = -MAX_DRIFT_PPM;
    double b = MAX_DRIFT_PPM;
    double c = b - inv_phi * (b - a);
    double d = a + inv_phi * (b - a);
    double wc = width(c);
    double wd = width(d);
    while (b - a > 1e-4) {
        if (wc >= wd) {
            b = d;
            d = c;
            wd = wc;
            c = b - inv_phi * (b - a);
            wc = width(c);
        } else {
            a = c;
            c = d;
            wc = wd;
            d = a + inv_phi * (b - a);
            wd = width(d);
        }
    }
    return (a + b) / 2.0;
}

double TimesyncEstimator::feasible_drift_edge(double inside, double outside) const
{
    // The width is concave, so between a feasible drift and the end of the search range
    // it crosses zero at most once.
    if (width(outside) >= 0.0) {
        return outside;
    }
    for (int i = 0; i < 60; ++i) {
        const double mid = (inside + outside) / 2.0;
        if (width(mid) >= 0.0) {
            inside = mid;
        } else {
            outside = mid;
        }
    }
    return inside;
}

void TimesyncEstimator::update()
{
    if (_exchanges.empty()) {
        _estimate = {};
        return;
    }

    const auto& newest = _exchanges.back();
    _base_ns = newest.remote_ns - newest.local_recv_ns;

    double drift_ppm = 0.0;
    double lo = 0.0;
    double hi = 0.0;
    while (true) {
        // A single exchange cannot tell drift; keep it at zero rather than at whatever
        // end of the flat search range the search lands on.
        drift_ppm = _exchanges.size() > 1 ? best_drift() : 0.0;
        interval(drift_ppm, lo, hi);
        if (hi >= lo || _exchanges.size() == 1) {
            break;
        }
        // No line fits: a clock stepped. The newest exchange is the one to trust.
        _exchanges.pop_front();
    }

    // The true clock line may have any drift that still fits, and each gives its own
    // interval. All exchanges lie at or before the reference time, so both interval edges
    // rise with the drift: the offsets any fitting line allows run from the lower edge at
    // the smallest fitting drift to the upper edge at the largest.
    double drift_min = drift_ppm;
    double drift_max = drift_ppm;
    if (_exchanges.size() > 1) {
        drift_min = feasible_drift_edge(drift_ppm, -MAX_DRIFT_PPM);
        drift_max = feasible_drift_edge(drift_ppm, MAX_DRIFT_PPM);
        // Until the exchanges bound the drift (a few close together right after start),
        // the widest band sits at whichever end of the search range, -10000 ppm or so,
        // which then also extrapolates the offset badly. Take the fitting drift closest to
        // zero instead; the uncertainty below covers all of them either way.
        const double edge = MAX_DRIFT_PPM * 0.999;
        if (drift_min <= -edge || drift_max >= edge) {
            drift_ppm = std::clamp(0.0, drift_min, drift_max);
            interval(drift_ppm, lo, hi);
        }
    }

    // The estimate is the centre of the interval at the chosen drift.
    const double centre = (lo + hi) / 2.0;
    double reach = (hi - lo) / 2.0;
    if (_exchanges.size() > 1) {
        double lo_min, hi_ignored, lo_ignored, hi_max;
        interval(drift_min, lo_min, hi_ignored);
        interval(drift_max, lo_ignored, hi_max);
        reach = std::max({reach, centre - lo_min, hi_max - centre});
    }

    _estimate.exchanges = static_cast<uint32_t>(_exchanges.size());
    _estimate.valid = _estimate.exchanges >= MIN_EXCHANGES;
    _estimate.local_ns = newest.local_send_ns;
    _estimate.offset_ns = _base_ns + static_cast<int64_t>(std::llround(centre));
    _estimate.drift_ppm = drift_ppm;
    _estimate.uncertainty_ns = static_cast<int64_t>(std::llround(reach));
}

int64_t TimesyncEstimator::to_remote(const Estimate& estimate, int64_t local_ns)
{
    const double dt = static_cast<double>(local_ns - estimate.local_ns);
    return local_ns + estimate.offset_ns +
           static_cast<int64_t>(std::llround(dt * estimate.drift_ppm * 1e-6));
}

int64_t TimesyncEstimator::to_local(const Estimate& estimate, int64_t remote_ns)
{
    // remote = local + offset + drift * (local - local_ref), solved for local.
    const int64_t remote_ref = estimate.local_ns + estimate.offset_ns;
    const double dt_remote = static_cast<double>(remote_ns - remote_ref);
    return estimate.local_ns +
           static_cast<int64_t>(std::llround(dt_remote / (1.0 + estimate.drift_ppm * 1e-6)));
}

} // namespace mavsdk
