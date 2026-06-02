package com.dji.sdk.sample.demo.virtualstickwaypoint;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import com.dji.sdk.sample.demo.geofencing.FlightLogger;
import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.utils.OfflineDebugConfig;

import java.util.Timer;
import java.util.TimerTask;

import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;

/**
 * WaypointMissionController
 *
 * Owns the entire mission state machine, the 20Hz control loop, waypoint dwell
 * logic, Virtual Stick commands, RTH, FlightLogger, and offline debug simulation.
 *
 * This class has no Android widget imports — all UI updates are pushed back to
 * VirtualStickWaypointView through the MissionCallback interface.
 *
 * ── Waypoint dwell ────────────────────────────────────────────────────────────
 * When the drone reaches a waypoint (within ACCEPTANCE_RADIUS_M horizontally and
 * ALTITUDE_ACCEPTANCE_M vertically) it enters a dwell:
 *   1. A zero-velocity hover command is sent.
 *   2. isDwelling = true — the control loop sends only hover commands until dwell ends.
 *   3. A countdown Handler fires once per second to update the status label.
 *   4. After WAYPOINT_DWELL_MS the drone advances to the next waypoint (or RTH).
 *
 * ── Changing the dwell duration ───────────────────────────────────────────────
 * Find WAYPOINT_DWELL_MS below. Change the value in milliseconds.
 * 10_000L = 10 seconds. 5_000L = 5 seconds.
 *
 * ── Offline debug mode ────────────────────────────────────────────────────────
 * When OfflineDebugConfig.OFFLINE_DEBUG_MODE = true:
 *   - No DJI SDK calls are made.
 *   - Simulated drone position is updated mathematically each control tick.
 *   - Dwell still waits the full WAYPOINT_DWELL_MS so you can verify the countdown.
 *   - RTH is replaced by finishOfflineDebugMission().
 *
 * ── Control law ───────────────────────────────────────────────────────────────
 * PHASE 1 — CRUISE  (distance > brakingDistance): fly at MAX_HORIZONTAL_SPEED.
 * PHASE 2 — BRAKING (distance ≤ brakingDistance): PD controller.
 *   horizontalSpeed = Kp * distance − Kd * distanceRate  (clamped [MIN, MAX])
 *   verticalSpeed   = Kp_VERTICAL * altError              (clamped ±MAX_VERTICAL)
 * brakingDistance   = maxSpeed² / (2 * deceleration)
 */
public class WaypointMissionController {

    private static final String TAG = "VSWMissionController";

    // ── Dwell duration ────────────────────────────────────────────────────────
    // Time (ms) the drone hovers at each waypoint before proceeding.
    // Change this value to adjust the stop duration.
    private static final long WAYPOINT_DWELL_MS = 10_000L;

    // ── Control loop timing ───────────────────────────────────────────────────
    private static final long  CONTROL_LOOP_INTERVAL_MS = 50;
    private static final float DT = CONTROL_LOOP_INTERVAL_MS / 1000.0f;

    // ── Speed limits ──────────────────────────────────────────────────────────
    private static final float MIN_HORIZONTAL_SPEED = 0.3f;
    private static final float MAX_VERTICAL_SPEED   = 2.0f;

    // ── Dependencies ─────────────────────────────────────────────────────────
    private final WaypointStore   store;
    private final TuningPanel     tuning;
    private final MissionCallback callback;
    private final Context         context;

    // ── DJI ──────────────────────────────────────────────────────────────────
    private FlightController flightController;

    // ── Mission state ─────────────────────────────────────────────────────────
    private boolean missionRunning       = false;
    private int     currentWaypointIndex = 0;
    private double  previousDistance     = 0.0;

    // ── Dwell state ───────────────────────────────────────────────────────────
    private boolean isDwelling    = false;
    private long    dwellEndTimeMs = 0L;

    // ── GPS state ─────────────────────────────────────────────────────────────
    private double  currentLat  = 0.0;
    private double  currentLng  = 0.0;
    private float   currentAlt  = 0.0f;
    private boolean hasValidGPS = false;

    // ── Timer driving the control loop ────────────────────────────────────────
    private Timer controlTimer;

    // ── Main-thread handler ───────────────────────────────────────────────────
    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    // ── FlightLogger ──────────────────────────────────────────────────────────
    private FlightLogger flightLogger;

    // ── Dwell countdown runnable ──────────────────────────────────────────────
    private Runnable countdownRunnable;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @param context  Application or Activity context (used for FlightLogger).
     * @param store    Shared WaypointStore — read-only during mission.
     * @param tuning   TuningPanel — parameter values read each control tick.
     * @param callback MissionCallback implemented by VirtualStickWaypointView.
     */
    public WaypointMissionController(Context context,
                                     WaypointStore store,
                                     TuningPanel tuning,
                                     MissionCallback callback) {
        this.context  = context.getApplicationContext();
        this.store    = store;
        this.tuning   = tuning;
        this.callback = callback;
    }

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * Initialises the DJI FlightController and attaches the GPS state callback.
     * In offline debug mode, sets up a simulated start position instead.
     */
    public void initFlightController() {
        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
            prepareOfflineDebugPosition();
            callback.onStatusChanged("Mission: OFFLINE DEBUG");
            callback.onLogMessage("Offline debug mode — drone connection skipped.");
            return;
        }

        if (DJISampleApplication.getProductInstance() == null) {
            callback.onLogMessage("No aircraft connected.");
            return;
        }

        if (DJISampleApplication.getProductInstance() instanceof Aircraft) {
            Aircraft aircraft = (Aircraft) DJISampleApplication.getProductInstance();
            flightController = aircraft.getFlightController();
            if (flightController != null) {
                flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
                flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
                flightController.setStateCallback(this::onFlightControllerState);
                callback.onLogMessage("FlightController ready.");
            } else {
                callback.onLogMessage("FlightController unavailable.");
            }
        } else {
            callback.onLogMessage("No aircraft connected.");
        }
    }

    /**
     * Starts the waypoint mission. Validates preconditions, enables Virtual
     * Stick mode, resets state, starts the 20Hz control loop.
     */
    public void startMission() {
        if (store.size() < 1) {
            callback.onLogMessage("Add at least 1 waypoint.");
            return;
        }

        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
            startOfflineDebugMission();
            return;
        }

        if (flightController == null) {
            initFlightController();
            if (flightController == null) {
                callback.onLogMessage("Flight controller not available.");
                return;
            }
        }

        if (!hasValidGPS) {
            callback.onLogMessage("Waiting for GPS fix.");
            return;
        }

        flightController.setVirtualStickModeEnabled(true, error -> {
            if (error != null) {
                callback.onLogMessage("Failed to enable Virtual Stick: "
                        + error.getDescription());
                return;
            }
            callback.onLogMessage("Virtual Stick mode enabled.");
            resetMissionState();
            missionRunning = true;

            flightLogger = new FlightLogger(context);
            flightLogger.start();
            callback.onLogMessage("Logging to: " + flightLogger.getLogFilePath());

            mainHandler.post(() -> {
                callback.onStatusChanged("Mission: RUNNING");
                callback.onMissionActiveChanged(true);
                callback.onTargetLabelChanged(buildTargetLabel());
            });

            callback.onLogMessage("Mission started. Flying to waypoint 1.");
            startControlLoop();
        });
    }

    /**
     * Stops the mission immediately. Cancels any active dwell, sends a
     * zero-velocity hover, disables Virtual Stick.
     */
    public void stopMission() {
        missionRunning = false;
        isDwelling     = false;
        cancelCountdown();
        stopControlLoop();

        if (flightLogger != null) {
            flightLogger.stop();
            callback.onLogMessage("Log saved to: " + flightLogger.getLogFilePath());
        }

        sendVelocityCommand(0, 0, 0);

        if (flightController != null) {
            flightController.setVirtualStickModeEnabled(false, error -> {
                if (error == null) {
                    callback.onLogMessage("Virtual Stick disabled — manual control restored.");
                } else {
                    callback.onLogMessage("Error disabling Virtual Stick: "
                            + error.getDescription());
                }
            });
        }

        mainHandler.post(() -> {
            callback.onStatusChanged("Mission: STOPPED");
            callback.onMissionActiveChanged(false);
            callback.onTargetLabelChanged("Target: —");
        });

        callback.onLogMessage("Mission stopped by user.");
    }

    /**
     * Detaches callbacks and cleans up resources.
     * Call from VirtualStickWaypointView.onDetachedFromWindow().
     */
    public void onDetached() {
        if (missionRunning) {
            missionRunning = false;
            isDwelling     = false;
            cancelCountdown();
            stopControlLoop();
            sendVelocityCommand(0, 0, 0);
            if (flightLogger != null) flightLogger.stop();
        }
        if (flightController != null) {
            flightController.setStateCallback(null);
            flightController.setVirtualStickModeEnabled(false, null);
        }
    }

    // =========================================================================
    // Flight Controller State Callback  (~10 Hz)
    // =========================================================================

    private void onFlightControllerState(FlightControllerState state) {
        LocationCoordinate3D loc = state.getAircraftLocation();
        if (loc == null) return;

        double lat = loc.getLatitude();
        double lng = loc.getLongitude();
        float  alt = loc.getAltitude();

        if (lat == 0.0 && lng == 0.0) {
            hasValidGPS = false;
            return;
        }

        currentLat  = lat;
        currentLng  = lng;
        currentAlt  = alt;
        hasValidGPS = true;

        float vx = state.getVelocityX();
        float vy = state.getVelocityY();
        float groundSpeed = (float) Math.sqrt(vx * vx + vy * vy);

        if (missionRunning) {
            Log.d(TAG, String.format("speed=%.2f m/s  vx=%.2f  vy=%.2f",
                    groundSpeed, vx, vy));
        }

        String posLabel   = String.format("Drone: (%.6f, %.6f)  alt: %.1fm", lat, lng, alt);
        String speedLabel = String.format("Speed: %.2f m/s", groundSpeed);
        mainHandler.post(() -> callback.onTelemetryUpdated(posLabel, speedLabel));
    }

    // =========================================================================
    // Control Loop
    // =========================================================================

    private void startControlLoop() {
        controlTimer = new Timer();
        controlTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() { runControlStep(); }
        }, 0, CONTROL_LOOP_INTERVAL_MS);
    }

    private void stopControlLoop() {
        if (controlTimer != null) {
            controlTimer.cancel();
            controlTimer = null;
        }
    }

    /**
     * One iteration of the 20Hz control loop.
     *
     * If isDwelling, only hover commands are sent.
     * Otherwise: compute distance + bearing, run PD controller, send command.
     */
    private void runControlStep() {
        if (!missionRunning || !hasValidGPS) return;
        if (currentWaypointIndex >= store.size()) return;

        // ── Dwell phase: hover until the timer fires ──────────────────────────
        if (isDwelling) {
            sendVelocityCommand(0, 0, 0);
            if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
                mainHandler.post(() -> callback.onTelemetryUpdated(
                        String.format("Drone: offline sim (%.6f, %.6f)  alt: %.1fm",
                                currentLat, currentLng, currentAlt),
                        "Speed: 0.00 m/s"));
            }
            return;
        }

        // ── Normal navigation ─────────────────────────────────────────────────
        double[] target    = store.getWaypoint(currentWaypointIndex);
        double   targetLat = target[0];
        double   targetLng = target[1];
        float    targetAlt = (float) target[2];

        double distanceM = WaypointNavigationMath.haversineDistance(
                currentLat, currentLng, targetLat, targetLng);
        float altError = targetAlt - currentAlt;

        Log.d(TAG, String.format("WP%d dist=%.2fm altErr=%.2fm lat=%.6f lng=%.6f",
                currentWaypointIndex + 1, distanceM, altError, currentLat, currentLng));

        if (flightLogger != null) flightLogger.log(currentLat, currentLng, true);

        // ── Waypoint acceptance ───────────────────────────────────────────────
        boolean horizontalReached =
                distanceM <= WaypointNavigationMath.ACCEPTANCE_RADIUS_M;
        boolean verticalReached   =
                Math.abs(altError) <= WaypointNavigationMath.ALTITUDE_ACCEPTANCE_M;

        if (horizontalReached && verticalReached) {
            enterDwell();
            return;
        }

        // ── Feedforward + PD horizontal speed controller ──────────────────────
        float maxSpeed     = tuning.getMaxHorizontalSpeed();
        float deceleration = tuning.getDeceleration();
        float brakingDist  = tuning.getBrakingDistanceM();

        float horizontalSpeed;
        if (distanceM > brakingDist) {
            // PHASE 1 — CRUISE: full speed
            horizontalSpeed = maxSpeed;
        } else {
            // PHASE 2 — BRAKING: PD controller
            double distanceRate = (distanceM - previousDistance) / DT;
            horizontalSpeed = (float)(tuning.getKpHorizontal() * distanceM
                    - tuning.getKdHorizontal() * distanceRate);
            horizontalSpeed = Math.max(MIN_HORIZONTAL_SPEED,
                    Math.min(maxSpeed, horizontalSpeed));
        }

        previousDistance = distanceM;

        // ── Velocity decomposition via bearing ────────────────────────────────
        double bearingRad    = WaypointNavigationMath.bearing(
                currentLat, currentLng, targetLat, targetLng);
        float  northVelocity = (float)(horizontalSpeed * Math.cos(bearingRad));
        float  eastVelocity  = (float)(horizontalSpeed * Math.sin(bearingRad));

        // DJI GROUND + VELOCITY: pitch = east, roll = north
        float pitchVelocity    = eastVelocity;
        float rollVelocity     = northVelocity;
        float verticalVelocity = tuning.getKpVertical() * altError;
        verticalVelocity = Math.max(-MAX_VERTICAL_SPEED,
                Math.min(MAX_VERTICAL_SPEED, verticalVelocity));

        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
            applyOfflineDebugVelocity(pitchVelocity, rollVelocity, verticalVelocity);
        } else {
            sendVelocityCommand(pitchVelocity, rollVelocity, verticalVelocity);
        }
    }

    // =========================================================================
    // Dwell Logic
    // =========================================================================

    /**
     * Called when the drone first enters the acceptance radius of a waypoint.
     * Stops the drone, marks dwell active, starts the per-second countdown.
     */
    private void enterDwell() {
        isDwelling     = true;
        dwellEndTimeMs = System.currentTimeMillis() + WAYPOINT_DWELL_MS;
        sendVelocityCommand(0, 0, 0);

        final int wpNumber = currentWaypointIndex + 1;
        callback.onLogMessage(String.format(
                "Waypoint %d reached! Dwelling for %.0f seconds.",
                wpNumber, WAYPOINT_DWELL_MS / 1000.0));

        previousDistance = 0.0; // reset D term for the next leg

        startCountdown(wpNumber);

        // Schedule end of dwell
        mainHandler.postDelayed(this::exitDwell, WAYPOINT_DWELL_MS);
    }

    /**
     * Updates the status label with a countdown once per second during dwell.
     */
    private void startCountdown(int wpNumber) {
        cancelCountdown();
        countdownRunnable = new Runnable() {
            @Override
            public void run() {
                if (!isDwelling) return;
                long remainingMs  = dwellEndTimeMs - System.currentTimeMillis();
                long remainingSec = Math.max(0, (remainingMs + 999) / 1000);
                callback.onStatusChanged(String.format(
                        "Mission: DWELLING - WP%d (%ds remaining)",
                        wpNumber, remainingSec));
                if (remainingSec > 0) {
                    mainHandler.postDelayed(this, 1000);
                }
            }
        };
        mainHandler.post(countdownRunnable);
    }

    private void cancelCountdown() {
        if (countdownRunnable != null) {
            mainHandler.removeCallbacks(countdownRunnable);
            countdownRunnable = null;
        }
    }

    /**
     * Called after WAYPOINT_DWELL_MS has elapsed.
     * Advances the waypoint index or triggers RTH if all waypoints are done.
     */
    private void exitDwell() {
        if (!missionRunning) return;
        isDwelling = false;
        cancelCountdown();

        currentWaypointIndex++;

        if (currentWaypointIndex >= store.size()) {
            callback.onLogMessage("All waypoints complete. Initiating RTH.");
            missionRunning = false;
            stopControlLoop();
            sendVelocityCommand(0, 0, 0);
            if (flightLogger != null) {
                flightLogger.stop();
                callback.onLogMessage("Log saved to: " + flightLogger.getLogFilePath());
            }
            if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
                finishOfflineDebugMission();
                return;
            }
            triggerRTH();
            return;
        }

        callback.onStatusChanged("Mission: RUNNING");
        callback.onTargetLabelChanged(buildTargetLabel());
        callback.onLogMessage("Flying to waypoint " + (currentWaypointIndex + 1) + ".");
    }

    // =========================================================================
    // Virtual Stick Command
    // =========================================================================

    /**
     * Sends a FlightControlData packet to the drone.
     *
     * @param pitch    East velocity  (m/s) — positive = fly East
     * @param roll     North velocity (m/s) — positive = fly North
     * @param vertical Climb rate     (m/s) — positive = climb
     */
    private void sendVelocityCommand(float pitch, float roll, float vertical) {
        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) return;
        if (flightController == null) return;

        FlightControlData data = new FlightControlData(pitch, roll, 0f, vertical);
        flightController.sendVirtualStickFlightControlData(data, error -> {
            if (error != null) {
                Log.e(TAG, "Virtual stick send error: " + error.getDescription());
            }
        });
    }

    // =========================================================================
    // Return to Home
    // =========================================================================

    private void triggerRTH() {
        if (flightController == null) return;
        flightController.setVirtualStickModeEnabled(false, error ->
                flightController.startGoHome(rthError -> {
                    if (rthError == null) {
                        mainHandler.post(() -> {
                            callback.onLogMessage("RTH initiated successfully.");
                            callback.onStatusChanged("Mission: RTH");
                            callback.onMissionActiveChanged(false);
                        });
                    } else {
                        callback.onLogMessage("RTH failed: " + rthError.getDescription());
                    }
                }));
    }

    // =========================================================================
    // Offline Debug Simulation
    // =========================================================================

    /**
     * Sets the simulated drone position to 20m south-west of the first waypoint.
     */
    private void prepareOfflineDebugPosition() {
        if (store.isEmpty()) {
            currentLat = 34.048510;
            currentLng = -117.837831;
            currentAlt = 0.0f;
        } else {
            double[] first = store.getWaypoint(0);
            currentLat = first[0] - WaypointNavigationMath.metersToLatitudeDegrees(
                    WaypointNavigationMath.OFFLINE_START_OFFSET_M);
            currentLng = first[1] - WaypointNavigationMath.metersToLongitudeDegrees(
                    WaypointNavigationMath.OFFLINE_START_OFFSET_M, first[0]);
            currentAlt = 0.0f;
        }
        hasValidGPS = true;
        mainHandler.post(() -> callback.onTelemetryUpdated(
                String.format("Drone: offline sim (%.6f, %.6f)  alt: %.1fm",
                        currentLat, currentLng, currentAlt),
                "Speed: 0.00 m/s"));
    }

    private void startOfflineDebugMission() {
        prepareOfflineDebugPosition();
        resetMissionState();
        missionRunning = true;

        mainHandler.post(() -> {
            callback.onStatusChanged("Mission: OFFLINE SIM");
            callback.onMissionActiveChanged(true);
            callback.onTargetLabelChanged(buildTargetLabel());
        });

        callback.onLogMessage("Offline debug mission started.");
        startControlLoop();
    }

    private void finishOfflineDebugMission() {
        mainHandler.post(() -> {
            callback.onLogMessage("Offline debug mission complete.");
            callback.onStatusChanged("Mission: OFFLINE COMPLETE");
            callback.onMissionActiveChanged(false);
            callback.onTargetLabelChanged("Target: —");
            callback.onTelemetryUpdated(
                    String.format("Drone: offline sim (%.6f, %.6f)  alt: %.1fm",
                            currentLat, currentLng, currentAlt),
                    "Speed: 0.00 m/s");
        });
    }

    /**
     * Moves the simulated drone position by integrating velocity over DT.
     * Called instead of sendVelocityCommand() in offline debug mode.
     */
    private void applyOfflineDebugVelocity(float pitchVelocity,
                                           float rollVelocity,
                                           float verticalVelocity) {
        // pitch = east, roll = north in DJI GROUND mode
        double northMeters = rollVelocity  * DT;
        double eastMeters  = pitchVelocity * DT;

        currentLat += WaypointNavigationMath.metersToLatitudeDegrees(northMeters);
        currentLng += WaypointNavigationMath.metersToLongitudeDegrees(eastMeters, currentLat);
        currentAlt += verticalVelocity * DT;
        if (currentAlt < 0.0f) currentAlt = 0.0f;

        float groundSpeed = (float) Math.sqrt(
                pitchVelocity * pitchVelocity + rollVelocity * rollVelocity);

        mainHandler.post(() -> callback.onTelemetryUpdated(
                String.format("Drone: offline sim (%.6f, %.6f)  alt: %.1fm",
                        currentLat, currentLng, currentAlt),
                String.format("Speed: %.2f m/s", groundSpeed)));
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    private void resetMissionState() {
        currentWaypointIndex = 0;
        previousDistance     = 0.0;
        isDwelling           = false;
        cancelCountdown();
    }

    private String buildTargetLabel() {
        if (currentWaypointIndex < store.size()) {
            double[] wp = store.getWaypoint(currentWaypointIndex);
            return String.format("Target WP%d: (%.6f, %.6f) @ %.1fm",
                    currentWaypointIndex + 1, wp[0], wp[1], wp[2]);
        }
        return "Target: —";
    }
}