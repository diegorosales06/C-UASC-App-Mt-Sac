package com.dji.sdk.sample.demo.virtualstickwaypoint;

/**
 * MissionCallback
 *
 * Interface that WaypointMissionController uses to push UI updates back to
 * VirtualStickWaypointView without holding a direct reference to any Android
 * widget. Every method is called on the main thread by the controller.
 *
 * Implementing this in a separate file keeps WaypointMissionController free of
 * any View or widget imports — all it knows is that someone is listening.
 */
public interface MissionCallback {

    /**
     * Fired whenever the top-level mission status string should change.
     * Examples: "Mission: RUNNING", "Mission: STOPPED", "Mission: RTH",
     *           "Mission: DWELLING - WP2 (8s remaining)"
     *
     * @param status Human-readable status string for tvStatus.
     */
    void onStatusChanged(String status);

    /**
     * Fired whenever the "Target WP" label should be refreshed.
     * The controller passes the fully formatted string so the View just
     * calls tvCurrentWaypoint.setText(label).
     *
     * @param label Formatted target string, e.g. "Target WP2: (34.048414, -117.837468) @ 3.0m"
     *              or "Target: —" when no waypoint is active.
     */
    void onTargetLabelChanged(String label);

    /**
     * Fired when a line should be appended to the scrollable mission log.
     *
     * @param message One line of log text (no trailing newline needed).
     */
    void onLogMessage(String message);

    /**
     * Fired when the enabled/disabled state of the mission control buttons
     * needs to change. Called at mission start, stop, RTH, and dwell entry/exit.
     *
     * @param missionActive true  → Start disabled, Stop enabled, waypoint buttons disabled.
     *                      false → Start enabled,  Stop disabled, waypoint buttons enabled.
     */
    void onMissionActiveChanged(boolean missionActive);

    /**
     * Fired on every control loop tick so the View can clear tvDroneSpeed
     * to "Speed: 0.00 m/s" during dwell and restore it when moving.
     * Also used in offline debug to push simulated position strings.
     *
     * @param positionLabel Full position string, e.g.
     *                      "Drone: (34.048510, -117.837831)  alt: 3.0m"
     *                      or "Drone: offline sim (34.048510, -117.837831)  alt: 3.0m"
     * @param speedLabel    Speed string, e.g. "Speed: 2.34 m/s"
     */
    void onTelemetryUpdated(String positionLabel, String speedLabel);
}