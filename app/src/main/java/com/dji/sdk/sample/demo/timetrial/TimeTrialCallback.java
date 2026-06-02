package com.dji.sdk.sample.demo.timetrial;

/**
 * TimeTrialCallback
 *
 * Interface that TimeTrialMissionController uses to push UI updates back to
 * TimeTrialView without holding a direct reference to any Android widget.
 * Every method is called on the main thread by the controller.
 *
 * Extends the base mission callback pattern with time-trial-specific timing
 * methods for elapsed time display and per-waypoint split recording.
 */
public interface TimeTrialCallback {

    /**
     * Fired whenever the top-level mission status string should change.
     * Examples: "Time Trial: RUNNING", "Time Trial: STOPPED", "Time Trial: RTH"
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
     * Fired when button enabled/disabled states need to change.
     *
     * @param missionActive true  → Start disabled, Stop enabled, waypoint buttons disabled.
     *                      false → Start enabled,  Stop disabled, waypoint buttons enabled.
     */
    void onMissionActiveChanged(boolean missionActive);

    /**
     * Fired on every control loop tick with current drone position and speed.
     *
     * @param positionLabel e.g. "Drone: (34.048510, -117.837831)  alt: 3.0m"
     * @param speedLabel    e.g. "Speed: 8.43 m/s"
     */
    void onTelemetryUpdated(String positionLabel, String speedLabel);

    /**
     * Fired every 100ms while the mission is running to update the elapsed
     * time display and last recorded split.
     *
     * @param elapsed   Total elapsed time string, e.g. "00:00:14.3"
     * @param lastSplit Last waypoint split string, e.g. "WP2: 00:00:06.1"
     *                  or "—" if no waypoint has been passed yet.
     */
    void onTimingUpdated(String elapsed, String lastSplit);

    /**
     * Fired immediately when the drone passes through a waypoint.
     * Used to append a split entry to the log with the waypoint number and time.
     *
     * @param wpNumber  1-based waypoint number that was just passed.
     * @param splitTime Formatted split time string, e.g. "00:00:06.1"
     * @param totalTime Formatted total elapsed time at the moment of the split.
     */
    void onSplitRecorded(int wpNumber, String splitTime, String totalTime);
}