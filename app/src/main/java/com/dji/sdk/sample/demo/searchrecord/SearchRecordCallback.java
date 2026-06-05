package com.dji.sdk.sample.demo.searchrecord;

/**
 * Connects {@link SearchRecordController} (flight logic, no Android widgets) back
 * to {@link SearchRecordView} (UI only). Every method is invoked on the main
 * thread by the controller, so implementations may touch views directly.
 */
public interface SearchRecordCallback {

    /** High-level mission status line, e.g. "Search: RUNNING". */
    void onStatusChanged(String status);

    /** Appends a line to the on-screen log. */
    void onLogMessage(String message);

    /** Current target / phase label, e.g. "Lane 2/8 → (lat, lng)". */
    void onTargetLabelChanged(String label);

    /** Live position + speed telemetry. */
    void onTelemetryUpdated(String positionLabel, String speedLabel);

    /** Toggles button enablement when a mission starts/stops. */
    void onMissionActiveChanged(boolean missionActive);
}
