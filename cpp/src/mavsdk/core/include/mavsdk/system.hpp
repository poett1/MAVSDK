#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <array>
#include <functional>
#include <vector>

#include "autopilot.hpp"
#include "component_type.hpp"
#include "deprecated.hpp"
#include "handle.hpp"
#include "vehicle.hpp"
#include "mavsdk_export.h"

namespace mavsdk {

class SystemImpl;
class MavsdkImpl;
class PluginImplBase;

/**
 * @brief This class represents a system, made up of one or more components
 * (e.g. autopilot, cameras, servos, gimbals, etc).
 *
 * System objects are used to interact with UAVs (including their components) and standalone
 * cameras. They are not created directly by application code, but are returned by the Mavsdk
 * class.
 */
class MAVSDK_PUBLIC System {
public:
    /** @private Constructor, used internally
     *
     * This constructor is not (and should not be) directly called by application code.
     *
     * @param parent `MavsdkImpl` dependency.
     */
    explicit System(MavsdkImpl& parent);
    /**
     * @brief Destructor.
     */
    ~System();

    /**
     * @brief Initialize the system.
     *
     * This is not (and should not be) directly called by application code.
     *
     * @param system_id System id.
     * @param component_id Component id.
     */
    void init(uint32_t system_id, uint8_t component_id) const;

    /**
     * @brief Checks whether the system has an autopilot.
     * @return `true` if it has an autopilot, `false` otherwise.
     */
    bool has_autopilot() const;

    /**
     * @brief Checks whether the system is a standalone (non-autopilot).
     * @return `true` if stand alone, `false` otherwise.
     */
    bool is_standalone() const;

    /**
     * @brief Checks whether the system has a camera with the given camera ID.
     *
     * A System may have several cameras, with IDs starting from 0.
     * When called without passing a camera ID, it checks whether the system has
     * any camera.
     * @param camera_id ID of the camera starting from 0 onwards.
     * @return `true` if camera with the given camera ID is found, `false` otherwise.
     */
    bool has_camera(int camera_id = -1) const;

    /**
     * @brief Checks whether the system has a gimbal.
     * @return `true` if the system has a gimbal, false otherwise.
     */
    bool has_gimbal() const;

    /**
     * @brief Checks if the system is connected.
     *
     * A system is connected when heartbeats are arriving (discovered and not timed out).
     * @return `true` if the system is connected.
     */
    bool is_connected() const;

    /**
     * @brief MAVLink System ID of connected system.
     *
     * @note: this is 0 if nothing is connected yet.
     *
     * @return the system ID.
     */
    uint32_t get_system_id() const;

    /**
     * @brief MAVLink component IDs of connected system.
     *
     * @note: all components that have been seen at least once will be listed.
     *
     * @return a list of all component ids
     */
    std::vector<uint8_t> component_ids() const;

    /**
     * @brief type for is connected callback.
     */
    using IsConnectedCallback = std::function<void(bool)>;

    /**
     * @brief handle type to unsubscribe from subscribe_is_connected.
     */
    using IsConnectedHandle = Handle<bool>;

    /**
     * @brief Subscribe to callback to be called when system connection state changes.
     *
     * @param callback Callback which will be called.
     */
    IsConnectedHandle subscribe_is_connected(const IsConnectedCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_is_connected.
     */
    void unsubscribe_is_connected(IsConnectedHandle handle);

    /**
     * @brief type for component discovery callback
     */
    using ComponentDiscoveredCallback = std::function<void(ComponentType)>;

    /**
     * @brief type for component discovery callback handle
     */
    using ComponentDiscoveredHandle = Handle<ComponentType>;

    /**
     * @brief type for component discovery callback with component ID
     */
    using ComponentDiscoveredIdCallback = std::function<void(ComponentType, uint8_t)>;

    /**
     * @brief type for component discovery callback handle with component ID
     */
    using ComponentDiscoveredIdHandle = Handle<ComponentType, uint8_t>;

    /**
     * @brief Subscribe to be called when a component is discovered.
     *
     * @param callback a function of type void(ComponentType) which will be
     * called with the component type of the new component.
     */
    ComponentDiscoveredHandle
    subscribe_component_discovered(const ComponentDiscoveredCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_component_discovered
     */
    void unsubscribe_component_discovered(ComponentDiscoveredHandle handle);

    /**
     * @brief Subscribe to be called when a component is discovered.
     *
     * @param callback a function of type void(ComponentType) which will be
     * called with the component type and the component id of the new component.
     */
    ComponentDiscoveredIdHandle
    subscribe_component_discovered_id(const ComponentDiscoveredIdCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_component_discovered_id
     */
    void unsubscribe_component_discovered_id(ComponentDiscoveredIdHandle handle);

    /**
     * @brief Enable time synchronization using the TIMESYNC messages.
     *
     * Sends TIMESYNC requests and estimates the offset and drift of the autopilot clock
     * against the local `std::chrono::steady_clock` (`CLOCK_MONOTONIC` on Linux). See
     * timesync_status().
     *
     * After the autopilot (re)connects, requests go out at about 10 Hz for 10 to 30 s, until
     * the uncertainty is down to 2 ms, then at about 1 Hz. Each interval gets a random extra
     * delay of up to 20 ms, so that requests do not all meet the same phase of the
     * autopilot's main loop. Call this as early as possible: the estimate needs a few seconds
     * of requests, and is good to a millisecond or better after about 5 s.
     */
    void enable_timesync();

    /**
     * @brief State of the TIMESYNC estimate.
     *
     * Each TIMESYNC round trip bounds the true offset between the clocks, whatever the
     * delays on the two legs. The estimate is the clock line that fits the round trips of
     * the last minute with the widest margin, not the round-trip midpoint, which is off by
     * half of any time the autopilot holds the request before stamping it (up to a whole
     * main loop on ArduPilot). `uncertainty_ns` covers every line that still fits.
     */
    struct TimesyncStatus {
        bool valid{false}; /**< @brief Enough round trips for an estimate. */
        int64_t local_time_ns{0}; /**< @brief steady_clock time the offset refers to, in ns. */
        int64_t offset_ns{0}; /**< @brief Autopilot minus local time at local_time_ns, in ns. */
        double drift_ppm{0.0}; /**< @brief How much faster the autopilot clock runs, in ppm. */
        int64_t uncertainty_ns{0}; /**< @brief Farthest the true offset can be, at local_time_ns. */
        uint32_t exchanges{0}; /**< @brief Round trips the estimate is based on. */
    };

    /**
     * @brief Get the state of the TIMESYNC estimate.
     *
     * Requires enable_timesync().
     *
     * @return the current estimate; `valid` is false until enough round trips arrived.
     */
    TimesyncStatus timesync_status() const;

    /**
     * @brief Convert a local steady_clock time to autopilot time.
     *
     * @param local_time_ns `std::chrono::steady_clock` time in ns since its epoch
     * (`CLOCK_MONOTONIC` on Linux).
     * @return the autopilot time in ns (the clock it uses in TIMESYNC, its boot time for
     * ArduPilot and PX4), or nothing while timesync_status() is not valid.
     */
    std::optional<int64_t> local_to_autopilot_time_ns(int64_t local_time_ns) const;

    /**
     * @brief Convert an autopilot time to local steady_clock time.
     *
     * @param autopilot_time_ns autopilot time in ns, e.g. a MAVLink `time_usec` times 1000.
     * @return the `std::chrono::steady_clock` time in ns since its epoch, or nothing while
     * timesync_status() is not valid.
     */
    std::optional<int64_t> autopilot_to_local_time_ns(int64_t autopilot_time_ns) const;

    /**
     * @brief Get autopilot type.
     *
     * @return autopilot type discovered.
     */
    Autopilot autopilot_type() const;

    /**
     * @brief Get vehicle type.
     *
     * @return vehicle type discovered.
     */
    Vehicle vehicle_type() const;

    /**
     * @brief Copy constructor (object is not copyable).
     */
    System(const System&) = delete;

    /**
     * @brief Equality operator (object is not copyable).
     */
    const System& operator=(const System&) = delete;

private:
    std::shared_ptr<SystemImpl> system_impl() { return _system_impl; };

    /*
     * MavsdkImpl and PluginImplBase need access to SystemImpl class.
     * This is not pretty, but it's not easy to hide the methods from library
     * users if not like that (or with an ugly reinterpret_cast).
     */
    friend MavsdkImpl;
    friend PluginImplBase;

    std::shared_ptr<SystemImpl> _system_impl;
};

} // namespace mavsdk
