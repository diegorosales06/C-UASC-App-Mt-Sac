package com.dji.sdk.sample.demo.virtualstickwaypoint;

/**
 * MissionCallback
 *
 * Interface that WaypointMissionController uses to push UI updates back to
 * VirtualStickWaypointView without holding a direct reference to any widget.
 * Every method is called on the main thread by the controller.
 */
public interface MissionCallback {

    /**
     * Fired whenever the top-level mission status string should change.
     * Examples: "Mission: RUNNING", "Mission: STOPPED", "Mission: RTH",
     *           "Mission: DWELLING - WP2 (8s remaining)"
     */
    void onStatusChanged(String status);

    /**
     * Fired whenever the "Target WP" label should be refreshed.
     * e.g. "Target WP2: (34.048414, -117.837468) @ 3.0m" or "Target: —"
     */
    void onTargetLabelChanged(String label);

    /**
     * Fired when a line should be appended to the scrollable mission log.
     */
    void onLogMessage(String message);

    /**
     * Fired when the enabled/disabled state of mission control buttons changes.
     *
     * @param missionActive true  → Start disabled, Stop enabled, waypoint buttons disabled.
     *                      false → Start enabled,  Stop disabled, waypoint buttons enabled.
     */
    void onMissionActiveChanged(boolean missionActive);

    /**
     * Fired on every control loop tick so the View can update position and
     * speed labels. Also used by offline debug to push simulated position.
     *
     * @param positionLabel e.g. "Drone: (34.048510, -117.837831)  alt: 3.0m"
     * @param speedLabel    e.g. "Speed: 2.34 m/s"
     */
    void onTelemetryUpdated(String positionLabel, String speedLabel);
}