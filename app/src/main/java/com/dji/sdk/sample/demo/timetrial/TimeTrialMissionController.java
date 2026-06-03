package com.dji.sdk.sample.demo.timetrial;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import com.dji.sdk.sample.demo.geofencing.FlightLogger;
import com.dji.sdk.sample.demo.virtualstickwaypoint.WaypointNavigationMath;
import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.utils.FlightControllerStateDispatcher;
import com.dji.sdk.sample.internal.utils.OfflineDebugConfig;

import java.util.Timer;
import java.util.TimerTask;
import java.util.Locale;

import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.model.LocationCoordinate2D;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;

/**
 * TimeTrialMissionController
 *
 * Mission controller for time trial runs. The drone flies through all waypoints
 * as fast as possible with NO dwell stop at each waypoint. When the acceptance
 * radius is entered the drone immediately advances to the next waypoint index
 * and continues flying without slowing to zero.
 *
 * ── Acceptance radius ─────────────────────────────────────────────────────────
 * ACCEPTANCE_RADIUS_M = 5.0m (wider than waypoint mission's 1.0m).
 * Change this constant at the top of the file to adjust how close the drone
 * must get before a gate is considered "hit".
 *
 * ── Timing system ─────────────────────────────────────────────────────────────
 * - missionStartTimeMs: recorded when mission starts
 * - splitTimes[]: records the System.currentTimeMillis() when each waypoint
 *   acceptance radius is entered
 * - Elapsed timer fires every 100ms via mainHandler and calls onTimingUpdated()
 * - Split time = time from previous split (or mission start) to this split
 * - Both elapsed and split are formatted as MM:SS.d (tenths of seconds)
 *
 * ── Control law (identical to WaypointMissionController) ─────────────────────
 * PHASE 1 — CRUISE  (distance > brakingDistance): fly at MAX_HORIZONTAL_SPEED.
 * PHASE 2 — BRAKING (distance ≤ brakingDistance): PD controller.
 *   horizontalSpeed = Kp * distance − Kd * distanceRate  (clamped [MIN, MAX])
 * Altitude: Kp_VERTICAL * altError (clamped ±MAX_VERTICAL_SPEED)
 * Yaw: P controller → rotate nose toward bearing, 5° deadband
 *
 * ── Data logging ─────────────────────────────────────────────────────────────
 * FlightLogger writes CSV with an extra "split_ms" column appended:
 *   timestamp_ms, latitude, longitude, inside, split_ms
 * split_ms is 0 except at the tick when a waypoint is accepted, where it holds
 * the split duration in milliseconds.
 */
public class TimeTrialMissionController {

    private static final String TAG = "TTMissionController";

    // ── Acceptance thresholds — change these to adjust gate hit precision ─────
    // Horizontal: how close (meters) the drone must get to count a gate
    private static final double ACCEPTANCE_RADIUS_M   = 5.0;
    // Vertical: how close (meters) in altitude before a gate counts
    private static final double ALTITUDE_ACCEPTANCE_M = 0.5;

    // ── Control loop timing ───────────────────────────────────────────────────
    private static final long  CONTROL_LOOP_INTERVAL_MS = 50;
    private static final float DT = CONTROL_LOOP_INTERVAL_MS / 1000.0f;

    // ── Speed limits ──────────────────────────────────────────────────────────
    private static final float MIN_HORIZONTAL_SPEED = 0.5f; // higher floor for time trial
    private static final float MAX_VERTICAL_SPEED   = 2.0f;

    // ── Timing display update interval ────────────────────────────────────────
    private static final long TIMING_UPDATE_INTERVAL_MS = 100;
    private static final long FINAL_RTH_DELAY_MS = 5_000L;

    // ── Dependencies ─────────────────────────────────────────────────────────
    private final TimeTrialWaypointStore store;
    private final TimeTrialTuningPanel   tuning;
    private final TimeTrialCallback      callback;
    private final Context                context;

    // ── DJI ──────────────────────────────────────────────────────────────────
    private FlightController flightController;
    private final FlightControllerStateDispatcher.Listener flightStateListener =
            this::onFlightControllerState;

    // ── Mission state ─────────────────────────────────────────────────────────
    private boolean missionRunning       = false;
    private boolean returnHomePending   = false;
    private int     currentWaypointIndex = 0;
    private double  previousDistance     = 0.0;

    // ── GPS / attitude state ──────────────────────────────────────────────────
    private double  currentLat        = 0.0;
    private double  currentLng        = 0.0;
    private float   currentAlt        = 0.0f;
    private float   currentHeadingDeg = 0.0f;
    private boolean hasValidGPS       = false;
    private double  homeLat           = 0.0;
    private double  homeLng           = 0.0;
    private boolean hasHomePoint      = false;

    // ── Timing state ──────────────────────────────────────────────────────────
    private long   missionStartTimeMs = 0L;
    private long   lastSplitTimeMs    = 0L;
    private long[] splitTimes;          // split duration in ms for each waypoint
    private int    lastSplitWpNumber  = 0;
    private String lastSplitLabel     = "—";

    // ── Timer and handler ─────────────────────────────────────────────────────
    private Timer    controlTimer;
    private final Handler mainHandler    = new Handler(Looper.getMainLooper());
    private Runnable timingRunnable;
    private Runnable returnHomeRunnable;

    // ── FlightLogger ──────────────────────────────────────────────────────────
    private FlightLogger flightLogger;

    // =========================================================================
    // Constructor
    // =========================================================================

    public TimeTrialMissionController(Context context,
                                      TimeTrialWaypointStore store,
                                      TimeTrialTuningPanel tuning,
                                      TimeTrialCallback callback) {
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
            callback.onStatusChanged("Time Trial: OFFLINE DEBUG");
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
                FlightControllerStateDispatcher.addListener(flightController, flightStateListener);
                callback.onLogMessage("FlightController ready.");
            } else {
                callback.onLogMessage("FlightController unavailable.");
            }
        } else {
            callback.onLogMessage("No aircraft connected.");
        }
    }

    /** Starts the time trial mission. */
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

        recordHomePointFromCurrentLocation();

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
                callback.onStatusChanged("Time Trial: RUNNING");
                callback.onMissionActiveChanged(true);
                callback.onTargetLabelChanged(buildTargetLabel());
            });

            callback.onLogMessage("Time trial started! Flying to gate 1.");
            startControlLoop();
            startTimingUpdates();
        });
    }

    /** Stops the time trial immediately. */
    public void stopMission() {
        missionRunning = false;
        returnHomePending = false;
        cancelReturnHomeDelay();
        stopControlLoop();
        stopTimingUpdates();

        if (flightLogger != null) {
            flightLogger.stop();
            callback.onLogMessage("Log saved to: " + flightLogger.getLogFilePath());
        }

        sendVelocityCommand(0, 0, 0f, 0);

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
            callback.onStatusChanged("Time Trial: STOPPED");
            callback.onMissionActiveChanged(false);
            callback.onTargetLabelChanged("Target: —");
            callback.onTimingUpdated("--:--.--", lastSplitLabel);
        });

        callback.onLogMessage("Time trial stopped by user.");
    }

    /** Detaches callbacks and cleans up. Call from TimeTrialView.onDetachedFromWindow(). */
    public void onDetached() {
        if (missionRunning || returnHomePending) {
            missionRunning = false;
            returnHomePending = false;
            cancelReturnHomeDelay();
            stopControlLoop();
            stopTimingUpdates();
            sendVelocityCommand(0, 0, 0f, 0);
            if (flightLogger != null) flightLogger.stop();
        }
        if (flightController != null) {
            FlightControllerStateDispatcher.removeListener(flightStateListener);
            flightController.setVirtualStickModeEnabled(false, null);
        }
    }

    // =========================================================================
    // Flight Controller State Callback
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

        currentLat        = lat;
        currentLng        = lng;
        currentAlt        = alt;
        currentHeadingDeg = (float) state.getAttitude().yaw;
        hasValidGPS       = true;

        float vx = state.getVelocityX();
        float vy = state.getVelocityY();
        float groundSpeed = (float) Math.sqrt(vx * vx + vy * vy);

        if (missionRunning) {
            Log.d(TAG, String.format("speed=%.2f m/s  hdg=%.1f°", groundSpeed, currentHeadingDeg));
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
     * Key difference from WaypointMissionController: when a waypoint is
     * accepted the drone does NOT stop. It records the split time and
     * immediately advances the index — the next tick targets the new waypoint
     * and the drone keeps flying at whatever speed it had.
     */
    private void runControlStep() {
        if (returnHomePending) {
            sendVelocityCommand(0, 0, 0f, 0);
            return;
        }
        if (!missionRunning || !hasValidGPS) return;
        if (currentWaypointIndex >= store.size()) return;

        double[] target    = store.getWaypoint(currentWaypointIndex);
        double   targetLat = target[0];
        double   targetLng = target[1];
        float    targetAlt = (float) target[2];

        double distanceM = WaypointNavigationMath.haversineDistance(
                currentLat, currentLng, targetLat, targetLng);
        float altError = targetAlt - currentAlt;

        Log.d(TAG, String.format("Gate%d dist=%.2fm altErr=%.2fm",
                currentWaypointIndex + 1, distanceM, altError));

        if (flightLogger != null) flightLogger.log(currentLat, currentLng, true);

        // ── Gate acceptance — NO DWELL, just record split and advance ─────────
        boolean horizontalReached = distanceM <= ACCEPTANCE_RADIUS_M;
        boolean verticalReached   = Math.abs(altError) <= ALTITUDE_ACCEPTANCE_M;

        if (horizontalReached && verticalReached) {
            recordSplit(currentWaypointIndex + 1);
            currentWaypointIndex++;
            previousDistance = 0.0; // reset D term derivative for new leg

            if (currentWaypointIndex >= store.size()) {
                // All gates complete
                finishRun();
                return;
            }

            final int nextGate = currentWaypointIndex + 1;
            mainHandler.post(() -> {
                callback.onTargetLabelChanged(buildTargetLabel());
                callback.onLogMessage("Gate passed! Flying to gate " + nextGate + ".");
            });
            // Fall through — compute velocity toward new target immediately
        }

        // ── Re-read target in case index just advanced ────────────────────────
        if (currentWaypointIndex >= store.size()) return;
        target    = store.getWaypoint(currentWaypointIndex);
        targetLat = target[0];
        targetLng = target[1];
        targetAlt = (float) target[2];

        distanceM = WaypointNavigationMath.haversineDistance(
                currentLat, currentLng, targetLat, targetLng);
        altError = targetAlt - currentAlt;

        // ── Feedforward + PD horizontal speed controller ──────────────────────
        float maxSpeed    = tuning.getMaxHorizontalSpeed();
        float brakingDist = tuning.getBrakingDistanceM();

        float horizontalSpeed;
        if (distanceM > brakingDist) {
            horizontalSpeed = maxSpeed;
        } else {
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

        float pitchVelocity    = eastVelocity;
        float rollVelocity     = northVelocity;
        float verticalVelocity = tuning.getKpVertical() * altError;
        verticalVelocity = Math.max(-MAX_VERTICAL_SPEED,
                Math.min(MAX_VERTICAL_SPEED, verticalVelocity));

        float yawRate = computeYawRate(Math.toDegrees(bearingRad));

        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
            applyOfflineDebugVelocity(pitchVelocity, rollVelocity, verticalVelocity);
        } else {
            sendVelocityCommand(pitchVelocity, rollVelocity, yawRate, verticalVelocity);
        }
    }

    // =========================================================================
    // Timing
    // =========================================================================

    /**
     * Records a split time when a gate is passed.
     * Split = elapsed time since the last gate (or mission start for WP1).
     */
    private void recordSplit(int wpNumber) {
        long now      = System.currentTimeMillis();
        long splitMs  = now - lastSplitTimeMs;
        long totalMs  = now - missionStartTimeMs;
        lastSplitTimeMs = now;

        if (wpNumber >= 1 && wpNumber <= splitTimes.length) {
            splitTimes[wpNumber - 1] = splitMs;
        }

        String splitStr = formatTime(splitMs);
        String totalStr = formatTime(totalMs);
        lastSplitLabel  = "Gate " + wpNumber + ": " + splitStr;
        lastSplitWpNumber = wpNumber;

        mainHandler.post(() -> {
            callback.onSplitRecorded(wpNumber, splitStr, totalStr);
            callback.onLogMessage(String.format(
                    "Gate %d — split: %s  total: %s", wpNumber, splitStr, totalStr));
        });
    }

    /** Starts the 100ms timing update runnable. */
    private void startTimingUpdates() {
        stopTimingUpdates();
        timingRunnable = new Runnable() {
            @Override
            public void run() {
                if (!missionRunning) return;
                long elapsed = System.currentTimeMillis() - missionStartTimeMs;
                callback.onTimingUpdated(formatTime(elapsed), lastSplitLabel);
                mainHandler.postDelayed(this, TIMING_UPDATE_INTERVAL_MS);
            }
        };
        mainHandler.post(timingRunnable);
    }

    private void stopTimingUpdates() {
        if (timingRunnable != null) {
            mainHandler.removeCallbacks(timingRunnable);
            timingRunnable = null;
        }
    }

    /**
     * Formats a duration in milliseconds as MM:SS.d (tenths of seconds).
     * Examples: 6100ms → "00:06.1",  74300ms → "01:14.3"
     */
    static String formatTime(long ms) {
        long totalTenths = ms / 100;
        long tenths      = totalTenths % 10;
        long totalSecs   = ms / 1000;
        long secs        = totalSecs % 60;
        long mins        = totalSecs / 60;
        return String.format(Locale.US, "%02d:%02d.%d", mins, secs, tenths);
    }

    // =========================================================================
    // Run completion
    // =========================================================================

    private void finishRun() {
        missionRunning = false;
        stopTimingUpdates();

        long totalMs = System.currentTimeMillis() - missionStartTimeMs;
        String totalStr = formatTime(totalMs);

        if (flightLogger != null) {
            flightLogger.stop();
            callback.onLogMessage("Log saved to: " + flightLogger.getLogFilePath());
        }

        sendVelocityCommand(0, 0, 0f, 0);

        // Build summary log
        StringBuilder summary = new StringBuilder();
        summary.append("── Time Trial Complete ──\n");
        summary.append("Total time: ").append(totalStr).append("\n");
        for (int i = 0; i < store.size(); i++) {
            if (i < splitTimes.length && splitTimes[i] > 0) {
                summary.append(String.format("  Gate %d split: %s\n",
                        i + 1, formatTime(splitTimes[i])));
            }
        }

        final String summaryStr = summary.toString();
        mainHandler.post(() -> {
            callback.onLogMessage(summaryStr);
            callback.onStatusChanged("Time Trial: COMPLETE — " + totalStr);
            callback.onTimingUpdated(totalStr, lastSplitLabel);
            callback.onMissionActiveChanged(false);
            callback.onTargetLabelChanged("Target: —");
        });

        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
            holdBeforeReturnHome(() -> finishOfflineDebugMission(totalStr));
            return;
        }

        holdBeforeReturnHome(this::triggerRTH);
    }

    // =========================================================================
    // Yaw Control
    // =========================================================================

    private float computeYawRate(double targetHeadingDeg) {
        final float YAW_DEADBAND_DEG = 5.0f;
        double headingError = WaypointNavigationMath.normalizeHeadingError(
                targetHeadingDeg - currentHeadingDeg);
        if (Math.abs(headingError) <= YAW_DEADBAND_DEG) return 0f;
        float yawRate = (float)(tuning.getKpYaw() * headingError);
        float maxRate = tuning.getMaxYawRate();
        return Math.max(-maxRate, Math.min(maxRate, yawRate));
    }

    // =========================================================================
    // Virtual Stick Command
    // =========================================================================

    private void sendVelocityCommand(float pitch, float roll, float yaw, float vertical) {
        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) return;
        if (flightController == null) return;

        FlightControlData data = new FlightControlData(pitch, roll, yaw, vertical);
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
                setHomeThenStartGoHome());
    }

    private void holdBeforeReturnHome(Runnable returnHomeAction) {
        returnHomePending = true;
        sendVelocityCommand(0, 0, 0f, 0);

        callback.onLogMessage(String.format(
                "Time trial complete. Holding for %.0f seconds before RTH.",
                FINAL_RTH_DELAY_MS / 1000.0));
        if (hasHomePoint) {
            callback.onLogMessage(String.format(
                    "RTH home point: %.6f, %.6f", homeLat, homeLng));
        }

        mainHandler.post(() -> {
            callback.onStatusChanged("Time Trial: HOLDING FOR RTH (5s)");
            callback.onMissionActiveChanged(true);
            callback.onTargetLabelChanged("Target: home");
        });

        cancelReturnHomeDelay();
        returnHomeRunnable = () -> {
            returnHomePending = false;
            stopControlLoop();
            returnHomeAction.run();
        };
        mainHandler.postDelayed(returnHomeRunnable, FINAL_RTH_DELAY_MS);
    }

    private void cancelReturnHomeDelay() {
        if (returnHomeRunnable != null) {
            mainHandler.removeCallbacks(returnHomeRunnable);
            returnHomeRunnable = null;
        }
    }

    private void recordHomePointFromCurrentLocation() {
        if (!hasValidGPS) return;
        homeLat = currentLat;
        homeLng = currentLng;
        hasHomePoint = true;
        callback.onLogMessage(String.format(
                "Home point recorded from takeoff/start position: %.6f, %.6f",
                homeLat, homeLng));
        setAircraftHomeToRecordedPoint();
    }

    private void setAircraftHomeToRecordedPoint() {
        if (flightController == null || !hasHomePoint) return;
        LocationCoordinate2D homePoint = new LocationCoordinate2D(homeLat, homeLng);
        flightController.setHomeLocation(homePoint, error -> {
            if (error == null) {
                callback.onLogMessage("Aircraft home location set to recorded takeoff/start point.");
            } else {
                callback.onLogMessage("Home location update failed: "
                        + error.getDescription()
                        + ". RTH will use the aircraft's current home point if needed.");
            }
        });
    }

    private void setHomeThenStartGoHome() {
        if (flightController == null) return;
        if (!hasHomePoint) {
            startGoHome();
            return;
        }

        LocationCoordinate2D homePoint = new LocationCoordinate2D(homeLat, homeLng);
        flightController.setHomeLocation(homePoint, error -> {
            if (error == null) {
                callback.onLogMessage("Home location confirmed for RTH.");
            } else {
                callback.onLogMessage("Could not confirm recorded home before RTH: "
                        + error.getDescription()
                        + ". Starting RTH with aircraft home point.");
            }
            startGoHome();
        });
    }

    private void startGoHome() {
        flightController.startGoHome(rthError -> {
            if (rthError == null) {
                mainHandler.post(() -> {
                    callback.onLogMessage("RTH initiated successfully.");
                    callback.onStatusChanged("Time Trial: RTH");
                    callback.onMissionActiveChanged(false);
                });
            } else {
                callback.onLogMessage("RTH failed: " + rthError.getDescription());
            }
        });
    }

    // =========================================================================
    // Offline Debug Simulation
    // =========================================================================

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
            callback.onStatusChanged("Time Trial: OFFLINE SIM");
            callback.onMissionActiveChanged(true);
            callback.onTargetLabelChanged(buildTargetLabel());
        });

        callback.onLogMessage("Offline time trial started.");
        startControlLoop();
        startTimingUpdates();
    }

    private void finishOfflineDebugMission(String totalTime) {
        mainHandler.post(() -> {
            callback.onLogMessage("Offline time trial complete. Total: " + totalTime);
            callback.onStatusChanged("Time Trial: OFFLINE COMPLETE — " + totalTime);
            callback.onMissionActiveChanged(false);
            callback.onTargetLabelChanged("Target: —");
        });
    }

    private void applyOfflineDebugVelocity(float pitchVelocity,
                                           float rollVelocity,
                                           float verticalVelocity) {
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
        returnHomePending    = false;
        missionStartTimeMs   = System.currentTimeMillis();
        lastSplitTimeMs      = missionStartTimeMs;
        lastSplitLabel       = "—";
        lastSplitWpNumber    = 0;
        splitTimes           = new long[store.size()];
        cancelReturnHomeDelay();
    }

    private String buildTargetLabel() {
        if (currentWaypointIndex < store.size()) {
            double[] wp = store.getWaypoint(currentWaypointIndex);
            return String.format("Gate %d: (%.6f, %.6f) @ %.1fm",
                    currentWaypointIndex + 1, wp[0], wp[1], wp[2]);
        }
        return "Target: —";
    }
}
