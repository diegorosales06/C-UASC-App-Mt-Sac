package com.dji.sdk.sample.demo.searchrecord;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import com.dji.sdk.sample.demo.virtualstickwaypoint.WaypointNavigationMath;
import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.utils.FlightControllerStateDispatcher;
import com.dji.sdk.sample.internal.utils.ModuleVerificationUtil;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;

import dji.common.camera.SettingsDefinitions;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.model.LocationCoordinate2D;
import dji.sdk.camera.Camera;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;

/**
 * SearchRecordController
 *
 * Autonomous "search and record" mission:
 *   1. Take off (or start from a hover) and climb to the search altitude.
 *   2. Point the gimbal straight down and START recording video to the drone's
 *      SD card.
 *   3. Fly a lawnmower sweep over the 4-corner box (see {@link LawnmowerPath}),
 *      holding the search altitude, while logging position to a CSV at ~4 Hz.
 *   4. When the sweep finishes (or the user stops), STOP recording and return
 *      to home.
 *
 * Competition rule: the UAS must stay above 25 ft (7.62 m) while searching.
 * The search altitude is clamped to a safe floor and the vertical controller
 * never targets anything below it, so the drone will not descend through it.
 *
 * The flight control law mirrors the (working) time-trial controller: a cruise
 * phase at max speed, a PD braking phase near each waypoint, a vertical P
 * controller holding altitude, and a yaw P controller pointing the nose along
 * the track. Velocity commands are clamped to the DJI virtual-stick limits so
 * the SDK never rejects a packet with "Param illegal".
 */
public class SearchRecordController {

    private static final String TAG = "SearchRecordCtrl";

    // ── Competition / safety ──────────────────────────────────────────────────
    public  static final float MIN_SEARCH_ALTITUDE_M = 7.62f;  // 25 ft hard floor
    private static final float ALTITUDE_SAFETY_MARGIN_M = 1.0f; // fly a bit above it

    // ── Acceptance + control loop ──────────────────────────────────────────────
    private static final double ACCEPTANCE_RADIUS_M     = 2.5;
    private static final long   CONTROL_LOOP_INTERVAL_MS = 50;
    private static final float  DT = CONTROL_LOOP_INTERVAL_MS / 1000.0f;
    private static final long   CSV_LOG_INTERVAL_MS = 250;       // ~4 Hz
    private static final float  TAKEOFF_STABLE_ALT_M = 1.0f;
    private static final long   FINAL_RTH_DELAY_MS = 4_000L;

    // ── Control gains (fixed — search favours smooth, steady coverage) ─────────
    private static final float  KP_HORIZONTAL = 0.45f;
    private static final float  KD_HORIZONTAL = 0.30f;
    private static final float  KP_VERTICAL   = 0.6f;
    private static final float  KP_YAW        = 2.0f;
    private static final float  MAX_YAW_RATE  = 45.0f;
    private static final float  DECELERATION  = 2.0f;
    private static final float  MIN_HORIZONTAL_SPEED = 1.0f;
    private static final float  MAX_VERTICAL_SPEED   = 2.0f;
    private static final float  YAW_DEADBAND_DEG     = 5.0f;
    private static final float  NADIR_GIMBAL_PITCH_DEG = -90.0f;
    private static final double GIMBAL_ROTATION_TIME_S = 1.0;

    // ── DJI virtual-stick hard limits (out-of-range → "Param illegal") ─────────
    private static final float DJI_MAX_HORIZONTAL_VELOCITY = 15.0f;
    private static final float DJI_MAX_VERTICAL_VELOCITY   = 4.0f;
    private static final float DJI_MAX_YAW_RATE            = 100.0f;

    // ── Dependencies ───────────────────────────────────────────────────────────
    private final Context              context;
    private final SearchRecordCallback callback;

    // ── DJI ────────────────────────────────────────────────────────────────────
    private FlightController flightController;
    private Camera           camera;
    private Gimbal           gimbal;
    private final FlightControllerStateDispatcher.Listener flightStateListener =
            this::onFlightControllerState;

    // ── Mission config ──────────────────────────────────────────────────────────
    private List<double[]> path;             // {lat, lng} waypoints
    private float          searchAltitudeM;
    private float          maxHorizontalSpeed;

    // ── Mission state ────────────────────────────────────────────────────────────
    private volatile boolean missionRunning    = false;
    private volatile boolean isTakingOff        = false;
    private volatile boolean returnHomePending  = false;
    private volatile boolean recordingVideo     = false;
    private int     currentWaypointIndex = 0;
    private double  previousDistance     = Double.NaN;
    private long    lastCsvLogMs         = 0L;

    // ── GPS / attitude state ──────────────────────────────────────────────────────
    private volatile double  currentLat = 0.0;
    private volatile double  currentLng = 0.0;
    private volatile float   currentAlt = 0.0f;
    private volatile float   currentHeadingDeg = 0.0f;
    private volatile boolean hasValidGPS = false;
    private double  homeLat = 0.0;
    private double  homeLng = 0.0;
    private boolean hasHomePoint = false;

    // ── Timer / handler / logger ───────────────────────────────────────────────────
    private Timer controlTimer;
    private Runnable returnHomeRunnable;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private SearchPositionLogger positionLogger;

    // ── Manual position marks (separate CSV, works any time there is a GPS fix) ──
    private SearchPositionLogger markLogger;
    private int markCount = 0;

    public SearchRecordController(Context context, SearchRecordCallback callback) {
        this.context  = context.getApplicationContext();
        this.callback = callback;
    }

    // =========================================================================
    // Setup
    // =========================================================================

    public void initFlightController() {
        if (DJISampleApplication.getProductInstance() == null) {
            callback.onLogMessage("No aircraft connected.");
            return;
        }
        if (!(DJISampleApplication.getProductInstance() instanceof Aircraft)) {
            callback.onLogMessage("No aircraft connected.");
            return;
        }
        Aircraft aircraft = (Aircraft) DJISampleApplication.getProductInstance();
        flightController = aircraft.getFlightController();
        camera           = aircraft.getCamera();
        gimbal           = aircraft.getGimbal();

        if (flightController == null) {
            callback.onLogMessage("FlightController unavailable.");
            return;
        }
        flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
        flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
        flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
        flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
        FlightControllerStateDispatcher.addListener(flightController, flightStateListener);
        callback.onLogMessage("FlightController ready.");
    }

    // =========================================================================
    // Mission control
    // =========================================================================

    /**
     * Validates inputs, builds the lawnmower path, and starts the mission.
     *
     * @param corners 4 corners in perimeter order, each {lat, lng}
     */
    public void startMission(double[][] corners, float altitudeM,
                             float maxSpeedMps, double trackSpacingM) {
        if (corners == null || corners.length != 4) {
            callback.onLogMessage("Need exactly 4 corners.");
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

        // Clamp config to safe ranges.
        searchAltitudeM = Math.max(MIN_SEARCH_ALTITUDE_M + ALTITUDE_SAFETY_MARGIN_M, altitudeM);
        if (altitudeM < searchAltitudeM) {
            callback.onLogMessage(String.format(Locale.US,
                    "Altitude %.1fm is below the 25ft floor; using %.1fm.",
                    altitudeM, searchAltitudeM));
        }
        maxHorizontalSpeed = clamp(maxSpeedMps, MIN_HORIZONTAL_SPEED, DJI_MAX_HORIZONTAL_VELOCITY);

        path = LawnmowerPath.generate(
                corners[0], corners[1], corners[2], corners[3], trackSpacingM);
        if (path.isEmpty()) {
            callback.onLogMessage("Generated path is empty — check corners/spacing.");
            return;
        }
        callback.onLogMessage(String.format(Locale.US,
                "Lawnmower path: %d waypoints, alt %.1fm, spacing %.1fm, speed %.1fm/s.",
                path.size(), searchAltitudeM, trackSpacingM, maxHorizontalSpeed));

        recordHomePointFromCurrentLocation();

        if (flightController.getState() != null && flightController.getState().isFlying()) {
            callback.onLogMessage("Drone already airborne — skipping takeoff.");
            enableVirtualStickAndBegin();
            return;
        }

        callback.onLogMessage("Drone on ground — initiating takeoff.");
        mainHandler.post(() -> {
            callback.onStatusChanged("Search: TAKING OFF");
            callback.onMissionActiveChanged(true);
        });
        isTakingOff = true;
        flightController.startTakeoff(error -> {
            if (error != null) {
                isTakingOff = false;
                callback.onLogMessage("Takeoff failed: " + error.getDescription());
                mainHandler.post(() -> {
                    callback.onStatusChanged("Search: IDLE");
                    callback.onMissionActiveChanged(false);
                });
                return;
            }
            callback.onLogMessage("Takeoff accepted. Climbing...");
        });
    }

    private void enableVirtualStickAndBegin() {
        if (flightController == null) return;
        isTakingOff = false;
        setAircraftHomeToRecordedPoint();

        flightController.setVirtualStickModeEnabled(true, error -> {
            if (error != null) {
                callback.onLogMessage("Failed to enable Virtual Stick: " + error.getDescription());
                mainHandler.post(() -> {
                    callback.onStatusChanged("Search: IDLE");
                    callback.onMissionActiveChanged(false);
                });
                return;
            }
            flightController.setVirtualStickAdvancedModeEnabled(true);
            flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
            flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
            flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
            callback.onLogMessage("Virtual Stick enabled.");

            currentWaypointIndex = 0;
            previousDistance     = Double.NaN;
            lastCsvLogMs         = 0L;
            returnHomePending    = false;
            missionRunning       = true;

            pointGimbalDown();
            startPositionLog();
            startVideoRecording();

            mainHandler.post(() -> {
                callback.onStatusChanged("Search: RUNNING");
                callback.onMissionActiveChanged(true);
                callback.onTargetLabelChanged(buildTargetLabel());
            });
            callback.onLogMessage("Search started. Flying lane 1.");
            startControlLoop();
        });
    }

    public void stopMission() {
        if (!missionRunning && !isTakingOff && !returnHomePending) {
            callback.onLogMessage("No mission running.");
        }
        missionRunning   = false;
        isTakingOff      = false;
        returnHomePending = false;
        cancelReturnHomeDelay();
        stopControlLoop();
        stopVideoRecording();
        stopPositionLog();

        sendVelocityCommand(0, 0, 0f, 0);
        if (flightController != null) {
            flightController.setVirtualStickModeEnabled(false, error -> {
                if (error == null) {
                    callback.onLogMessage("Virtual Stick disabled — manual control restored.");
                } else {
                    callback.onLogMessage("Error disabling Virtual Stick: " + error.getDescription());
                }
            });
        }
        mainHandler.post(() -> {
            callback.onStatusChanged("Search: STOPPED");
            callback.onMissionActiveChanged(false);
            callback.onTargetLabelChanged("Target: —");
        });
        callback.onLogMessage("Search stopped by user.");
    }

    public void requestReturnToHome() {
        if (flightController == null) {
            initFlightController();
            if (flightController == null) {
                callback.onLogMessage("Flight controller not available.");
                return;
            }
        }
        missionRunning    = false;
        isTakingOff       = false;
        returnHomePending = false;
        cancelReturnHomeDelay();
        stopControlLoop();
        stopVideoRecording();
        stopPositionLog();
        sendVelocityCommand(0, 0, 0f, 0);
        callback.onLogMessage("Manual RTH requested.");
        mainHandler.post(() -> {
            callback.onStatusChanged("Search: RTH REQUESTED");
            callback.onMissionActiveChanged(false);
            callback.onTargetLabelChanged("Target: home");
        });
        triggerRTH();
    }

    public void onDetached() {
        if (missionRunning || isTakingOff || returnHomePending) {
            missionRunning = false;
            isTakingOff = false;
            returnHomePending = false;
            cancelReturnHomeDelay();
            stopControlLoop();
            stopVideoRecording();
            stopPositionLog();
            sendVelocityCommand(0, 0, 0f, 0);
        }
        if (markLogger != null) {
            callback.onLogMessage("Marks CSV saved (" + markCount + " marks): "
                    + markLogger.getAbsolutePath());
            markLogger = null;
        }
        if (flightController != null) {
            FlightControllerStateDispatcher.removeListener(flightStateListener);
            flightController.setVirtualStickModeEnabled(false, null);
        }
    }

    // =========================================================================
    // Flight controller state
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
        currentLat = lat;
        currentLng = lng;
        currentAlt = alt;
        currentHeadingDeg = (float) state.getAttitude().yaw;
        hasValidGPS = true;

        if (isTakingOff && state.isFlying() && currentAlt >= TAKEOFF_STABLE_ALT_M) {
            isTakingOff = false;
            callback.onLogMessage(String.format(Locale.US,
                    "Stable hover (%.1fm). Starting search.", currentAlt));
            enableVirtualStickAndBegin();
        }

        float vx = state.getVelocityX();
        float vy = state.getVelocityY();
        float groundSpeed = (float) Math.sqrt(vx * vx + vy * vy);
        String posLabel   = String.format(Locale.US,
                "Drone: (%.6f, %.6f)  alt: %.1fm", lat, lng, alt);
        String speedLabel = String.format(Locale.US, "Speed: %.2f m/s", groundSpeed);
        mainHandler.post(() -> callback.onTelemetryUpdated(posLabel, speedLabel));
    }

    // =========================================================================
    // Control loop
    // =========================================================================

    private void startControlLoop() {
        controlTimer = new Timer();
        controlTimer.scheduleAtFixedRate(new TimerTask() {
            @Override public void run() { runControlStep(); }
        }, 0, CONTROL_LOOP_INTERVAL_MS);
    }

    private void stopControlLoop() {
        if (controlTimer != null) {
            controlTimer.cancel();
            controlTimer = null;
        }
    }

    private void runControlStep() {
        if (returnHomePending) {
            sendVelocityCommand(0, 0, 0f, 0);
            return;
        }
        if (!missionRunning || !hasValidGPS) return;
        if (currentWaypointIndex >= path.size()) return;

        double[] target = path.get(currentWaypointIndex);
        double targetLat = target[0];
        double targetLng = target[1];

        double distanceM = WaypointNavigationMath.haversineDistance(
                currentLat, currentLng, targetLat, targetLng);
        float altError = searchAltitudeM - currentAlt;

        logPositionThrottled();

        // ── Pass-through acceptance — no dwell ────────────────────────────────
        if (distanceM <= ACCEPTANCE_RADIUS_M) {
            currentWaypointIndex++;
            previousDistance = Double.NaN;
            if (currentWaypointIndex >= path.size()) {
                finishSearch();
                return;
            }
            mainHandler.post(() -> callback.onTargetLabelChanged(buildTargetLabel()));
            target = path.get(currentWaypointIndex);
            targetLat = target[0];
            targetLng = target[1];
            distanceM = WaypointNavigationMath.haversineDistance(
                    currentLat, currentLng, targetLat, targetLng);
        }

        // ── Horizontal speed: cruise or PD braking ────────────────────────────
        float brakingDist = (maxHorizontalSpeed * maxHorizontalSpeed) / (2.0f * DECELERATION);
        float horizontalSpeed;
        if (Double.isNaN(previousDistance) || distanceM > brakingDist) {
            horizontalSpeed = maxHorizontalSpeed;
        } else {
            double distanceRate = (distanceM - previousDistance) / DT;
            horizontalSpeed = (float) (KP_HORIZONTAL * distanceM - KD_HORIZONTAL * distanceRate);
            horizontalSpeed = clamp(horizontalSpeed, MIN_HORIZONTAL_SPEED, maxHorizontalSpeed);
        }
        previousDistance = distanceM;

        // ── Decompose into ground-frame velocity ──────────────────────────────
        double bearingRad = WaypointNavigationMath.bearing(
                currentLat, currentLng, targetLat, targetLng);
        float northVelocity = (float) (horizontalSpeed * Math.cos(bearingRad));
        float eastVelocity  = (float) (horizontalSpeed * Math.sin(bearingRad));

        float pitchVelocity = eastVelocity;   // GROUND frame: pitch = east
        float rollVelocity  = northVelocity;  // GROUND frame: roll  = north
        float verticalVelocity = clamp(KP_VERTICAL * altError, -MAX_VERTICAL_SPEED, MAX_VERTICAL_SPEED);
        float yawRate = computeYawRate(Math.toDegrees(bearingRad));

        sendVelocityCommand(pitchVelocity, rollVelocity, yawRate, verticalVelocity);
    }

    private void finishSearch() {
        missionRunning = false;
        stopVideoRecording();
        stopPositionLog();
        sendVelocityCommand(0, 0, 0f, 0);
        callback.onLogMessage("── Search complete — all lanes covered. ──");
        if (positionLogger != null) {
            callback.onLogMessage("Position log: " + positionLogger.getAbsolutePath());
        }
        mainHandler.post(() -> {
            callback.onStatusChanged("Search: COMPLETE");
            callback.onTargetLabelChanged("Target: home");
            callback.onMissionActiveChanged(true);
        });
        holdBeforeReturnHome();
    }

    // =========================================================================
    // Video recording (to the drone's SD card)
    // =========================================================================

    private void startVideoRecording() {
        if (camera == null) {
            callback.onLogMessage("No camera available — recording skipped.");
            return;
        }
        Runnable record = () -> camera.startRecordVideo(error -> {
            if (error == null) {
                recordingVideo = true;
                callback.onLogMessage("Video recording STARTED (saved to drone SD card).");
            } else {
                callback.onLogMessage("Start recording failed: " + error.getDescription());
            }
        });

        // Mavic Air 2 uses the flat-camera-mode API; older cameras use setMode.
        if (ModuleVerificationUtil.isMavicAir2()) {
            camera.setFlatMode(SettingsDefinitions.FlatCameraMode.VIDEO_NORMAL, e -> record.run());
        } else {
            camera.setMode(SettingsDefinitions.CameraMode.RECORD_VIDEO, e -> record.run());
        }
    }

    private void stopVideoRecording() {
        if (camera == null || !recordingVideo) return;
        recordingVideo = false;
        camera.stopRecordVideo(error -> {
            if (error == null) {
                callback.onLogMessage("Video recording STOPPED.");
            } else {
                callback.onLogMessage("Stop recording failed: " + error.getDescription());
            }
        });
    }

    private void pointGimbalDown() {
        if (gimbal == null) {
            callback.onLogMessage("Gimbal unavailable; camera not pointed down.");
            return;
        }
        Rotation rotation = new Rotation.Builder()
                .mode(RotationMode.ABSOLUTE_ANGLE)
                .pitch(NADIR_GIMBAL_PITCH_DEG)
                .yaw(Rotation.NO_ROTATION)
                .roll(Rotation.NO_ROTATION)
                .time(GIMBAL_ROTATION_TIME_S)
                .build();
        gimbal.rotate(rotation, error -> {
            if (error == null) {
                callback.onLogMessage("Gimbal pointed straight down for search video.");
            } else {
                callback.onLogMessage("Gimbal pitch-down failed: " + error.getDescription());
            }
        });
    }

    // =========================================================================
    // Position log
    // =========================================================================

    private void startPositionLog() {
        try {
            positionLogger = new SearchPositionLogger(context);
            callback.onLogMessage("Logging position to: " + positionLogger.getAbsolutePath());
        } catch (IOException e) {
            positionLogger = null;
            callback.onLogMessage("Could not open position log: " + e.getMessage());
        }
    }

    private void logPositionThrottled() {
        if (positionLogger == null) return;
        long now = System.currentTimeMillis();
        if (now - lastCsvLogMs < CSV_LOG_INTERVAL_MS) return;
        lastCsvLogMs = now;
        String phase = "lane " + (currentWaypointIndex + 1) + "/" + path.size();
        try {
            positionLogger.log(currentLat, currentLng, currentAlt, currentHeadingDeg, phase);
        } catch (IOException e) {
            Log.e(TAG, "CSV write failed", e);
        }
    }

    private void stopPositionLog() {
        if (positionLogger != null) {
            callback.onLogMessage("Position log saved: " + positionLogger.getAbsolutePath());
            positionLogger = null;
        }
    }

    /**
     * Logs the current timestamp + GPS position to a separate "marks" CSV.
     * Works whenever there is a valid GPS fix — independent of whether an
     * autonomous search is running — so it can be used to tag spots while flying
     * manually. The marks file is created lazily on the first press and reused
     * for the rest of the screen session.
     */
    public void logManualMark() {
        if (!hasValidGPS) {
            callback.onLogMessage("No GPS fix yet — cannot mark position.");
            return;
        }
        if (markLogger == null) {
            try {
                markLogger = new SearchPositionLogger(context, "marks_");
                callback.onLogMessage("Marks CSV: " + markLogger.getAbsolutePath());
            } catch (IOException e) {
                callback.onLogMessage("Could not open marks CSV: " + e.getMessage());
                return;
            }
        }
        markCount++;
        double lat = currentLat;
        double lng = currentLng;
        float  alt = currentAlt;
        float  hdg = currentHeadingDeg;
        String tag = "MARK " + markCount;
        try {
            markLogger.log(lat, lng, alt, hdg, tag);
            // Also drop a tagged row into the main flight log (if a run is active)
            // so the mark appears inline with the continuous track.
            if (positionLogger != null) {
                positionLogger.log(lat, lng, alt, hdg, tag);
            }
            callback.onLogMessage(String.format(Locale.US,
                    "📍 Mark #%d logged: (%.7f, %.7f)  alt %.1fm%s", markCount, lat, lng, alt,
                    positionLogger != null ? " (also in flight log)" : ""));
        } catch (IOException e) {
            callback.onLogMessage("Mark write failed: " + e.getMessage());
        }
    }

    // =========================================================================
    // Virtual stick + yaw
    // =========================================================================

    private void sendVelocityCommand(float pitch, float roll, float yaw, float vertical) {
        if (flightController == null) return;
        final float safePitch    = clampFinite(pitch,    DJI_MAX_HORIZONTAL_VELOCITY);
        final float safeRoll     = clampFinite(roll,     DJI_MAX_HORIZONTAL_VELOCITY);
        final float safeYaw      = clampFinite(yaw,      DJI_MAX_YAW_RATE);
        final float safeVertical = clampFinite(vertical, DJI_MAX_VERTICAL_VELOCITY);
        FlightControlData data = new FlightControlData(safePitch, safeRoll, safeYaw, safeVertical);
        flightController.sendVirtualStickFlightControlData(data, error -> {
            if (error != null) {
                Log.e(TAG, "Virtual stick send error: " + error.getDescription());
            }
        });
    }

    private float computeYawRate(double targetHeadingDeg) {
        double headingError = WaypointNavigationMath.normalizeHeadingError(
                targetHeadingDeg - currentHeadingDeg);
        if (Math.abs(headingError) <= YAW_DEADBAND_DEG) return 0f;
        float yawRate = (float) (KP_YAW * headingError);
        return clamp(yawRate, -MAX_YAW_RATE, MAX_YAW_RATE);
    }

    // =========================================================================
    // Return to home
    // =========================================================================

    private void holdBeforeReturnHome() {
        returnHomePending = true;
        sendVelocityCommand(0, 0, 0f, 0);
        callback.onLogMessage(String.format(Locale.US,
                "Holding %.0fs before RTH.", FINAL_RTH_DELAY_MS / 1000.0));
        cancelReturnHomeDelay();
        returnHomeRunnable = () -> {
            returnHomePending = false;
            stopControlLoop();
            triggerRTH();
        };
        mainHandler.postDelayed(returnHomeRunnable, FINAL_RTH_DELAY_MS);
    }

    private void cancelReturnHomeDelay() {
        if (returnHomeRunnable != null) {
            mainHandler.removeCallbacks(returnHomeRunnable);
            returnHomeRunnable = null;
        }
    }

    private void triggerRTH() {
        if (flightController == null) return;
        flightController.setVirtualStickModeEnabled(false, error -> {
            if (error != null) {
                callback.onLogMessage("Error disabling Virtual Stick: " + error.getDescription());
            }
            callback.onLogMessage("Starting RTH.");
            startGoHome();
        });
    }

    private void startGoHome() {
        flightController.startGoHome(rthError -> {
            if (rthError == null) {
                mainHandler.post(() -> {
                    callback.onLogMessage("RTH initiated.");
                    callback.onStatusChanged("Search: RTH");
                    callback.onMissionActiveChanged(false);
                });
            } else {
                callback.onLogMessage("RTH failed: " + rthError.getDescription());
            }
        });
    }

    private void recordHomePointFromCurrentLocation() {
        if (!hasValidGPS) return;
        homeLat = currentLat;
        homeLng = currentLng;
        hasHomePoint = true;
        callback.onLogMessage(String.format(Locale.US,
                "Home recorded: %.6f, %.6f", homeLat, homeLng));
    }

    private void setAircraftHomeToRecordedPoint() {
        if (flightController == null || !hasHomePoint) return;
        LocationCoordinate2D home = new LocationCoordinate2D(homeLat, homeLng);
        flightController.setHomeLocation(home, error -> {
            if (error != null) {
                callback.onLogMessage("Home set failed: " + error.getDescription()
                        + " (RTH will use aircraft home).");
            }
        });
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    private String buildTargetLabel() {
        if (path != null && currentWaypointIndex < path.size()) {
            double[] wp = path.get(currentWaypointIndex);
            return String.format(Locale.US, "Lane wp %d/%d → (%.6f, %.6f) @ %.1fm",
                    currentWaypointIndex + 1, path.size(), wp[0], wp[1], searchAltitudeM);
        }
        return "Target: —";
    }

    public File getPositionLogFile() {
        return positionLogger == null ? null : positionLogger.getCsvFile();
    }

    private static float clamp(float v, float min, float max) {
        return Math.max(min, Math.min(max, v));
    }

    /** Clamps to ±limit and converts any NaN/Infinity to 0 — the SDK rejects both. */
    private static float clampFinite(float value, float limit) {
        if (Float.isNaN(value) || Float.isInfinite(value)) return 0f;
        return Math.max(-limit, Math.min(limit, value));
    }
}
