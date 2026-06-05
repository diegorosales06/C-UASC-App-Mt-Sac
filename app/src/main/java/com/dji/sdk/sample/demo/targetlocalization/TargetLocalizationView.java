package com.dji.sdk.sample.demo.targetlocalization;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.util.AttributeSet;
import android.util.Log;
import android.view.Gravity;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;

import com.dji.sdk.sample.R;
import com.dji.sdk.sample.demo.virtualstickwaypoint.WaypointNavigationMath;
import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.utils.FlightControllerStateDispatcher;
import com.dji.sdk.sample.internal.view.PresentableView;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import dji.common.error.DJIError;
import dji.common.flightcontroller.Attitude;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.FlightMode;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.camera.SettingsDefinitions;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;

public class TargetLocalizationView extends LinearLayout
        implements PresentableView, TextureView.SurfaceTextureListener {

    private static final String TAG = "TargetLocalization";

    private static final int ANALYSIS_WIDTH_PX = 360;
    private static final long ANALYSIS_INTERVAL_MS = 500L;
    private static final long DETECTION_STALE_MS = 1_500L;
    private static final long CENTER_INTERVAL_MS = 100L;
    private static final float MIN_SEARCH_ALTITUDE_M = 7.62f;
    private static final float CENTER_DEADBAND_NORM = 0.08f;
    private static final float CENTER_DONE_NORM = 0.06f;
    private static final long CENTER_DONE_HOLD_MS = 1_200L;
    private static final float CENTER_KP = 1.15f;
    private static final float MAX_CENTER_SPEED_MPS = 1.2f;
    private static final float NADIR_GIMBAL_PITCH_DEG = -90f;
    private static final double GIMBAL_ROTATION_TIME_SECONDS = 1.5;
    private static final long AUTO_PHOTO_COOLDOWN_MS = 8_000L;
    private static final long SEARCH_CONTROL_INTERVAL_MS = 100L;
    private static final long SEARCH_DETECTION_SUPPRESS_MS = 6_000L;
    private static final double SEARCH_ACCEPTANCE_RADIUS_M = 2.5;
    private static final double SEARCH_ALTITUDE_ACCEPTANCE_M = 0.8;
    private static final float SEARCH_KP_HORIZONTAL = 0.45f;
    private static final float SEARCH_MIN_HORIZONTAL_SPEED_MPS = 0.35f;
    private static final float SEARCH_MAX_HORIZONTAL_SPEED_MPS = 2.5f;
    private static final float SEARCH_MAX_VERTICAL_SPEED_MPS = 1.5f;
    private static final float SEARCH_KP_VERTICAL = 0.6f;
    private static final float SEARCH_KP_YAW = 0.35f;
    private static final float SEARCH_MAX_YAW_RATE_DEG_S = 25f;
    private static final float DEFAULT_SEARCH_ALTITUDE_M = 12f;
    private static final double SEARCH_TRACK_OVERLAP = 0.55;
    private static final int MAX_SEARCH_WAYPOINTS = 240;
    private static final double CAMERA_HORIZONTAL_FOV_DEG = 78.8;
    private static final double CAMERA_VERTICAL_FOV_DEG = 63.0;

    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final ExecutorService detectorExecutor = Executors.newSingleThreadExecutor();
    private final TargetLocalizationDetector detector = new TargetLocalizationDetector();
    private final List<TargetRecord> targetRecords = new ArrayList<>();

    private TextureView videoTextureView;
    private TargetLocalizationOverlayView overlayView;
    private DJICodecManager codecManager;
    private VideoFeeder.VideoDataListener videoDataListener;

    private TextView statusText;
    private TextView telemetryText;
    private TextView detectionText;
    private TextView searchText;
    private TextView targetListText;
    private TextView logText;
    private EditText boundaryEditText;
    private EditText searchAltitudeEditText;
    private EditText trackSpacingEditText;
    private EditText idOverrideEditText;
    private EditText positionTagEditText;

    private Button startDetectionButton;
    private Button stopDetectionButton;
    private Button startSearchButton;
    private Button stopSearchButton;
    private Button centerButton;
    private Button stopCenterButton;
    private Button captureButton;
    private Button recordPositionButton;

    private DronePositionCsvRecorder positionCsvRecorder;
    private ScrollView controlsScrollView;

    private FlightController flightController;
    private Camera camera;
    private Gimbal gimbal;
    private final FlightControllerStateDispatcher.Listener flightStateListener =
            this::onFlightControllerState;

    private volatile double currentLat;
    private volatile double currentLng;
    private volatile float currentAlt;
    private volatile double currentYawDeg;
    private volatile boolean hasValidGps;
    private volatile String flightModeLabel = "unknown";

    private boolean detectionActive;
    private boolean detectionInFlight;
    private boolean centeringActive;
    private boolean virtualStickEnabledByThisView;
    private long lastAnalysisMs;
    private long centeredSinceMs;
    private long lastAutoPhotoMs;
    private boolean centeredTargetCaptured;
    private boolean photoCaptureInProgress;
    private boolean searchActive;
    private boolean searchPausedForTarget;
    private boolean searchVirtualStickEnabled;
    private long ignoreDetectionsUntilMs;
    private int searchWaypointIndex;
    private float searchAltitudeM = DEFAULT_SEARCH_ALTITUDE_M;
    private List<SearchWaypoint> searchWaypoints = Collections.emptyList();
    private TargetLocalizationDetector.Detection bestDetection;
    private List<TargetLocalizationDetector.Detection> latestDetections = Collections.emptyList();

    private final Runnable centerControlRunnable = new Runnable() {
        @Override
        public void run() {
            if (!centeringActive) {
                return;
            }
            runCenterControlStep();
            mainHandler.postDelayed(this, CENTER_INTERVAL_MS);
        }
    };

    private final Runnable searchControlRunnable = new Runnable() {
        @Override
        public void run() {
            if (!searchActive) {
                return;
            }
            runSearchControlStep();
            mainHandler.postDelayed(this, SEARCH_CONTROL_INTERVAL_MS);
        }
    };

    public TargetLocalizationView(Context context) {
        super(context);
        init(context);
    }

    public TargetLocalizationView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    @Override
    public int getDescription() {
        return R.string.target_localization_title;
    }

    @NonNull
    @Override
    public String getHint() {
        return getClass().getSimpleName() + ".java";
    }

    @Override
    protected void onAttachedToWindow() {
        super.onAttachedToWindow();
        initPositionCsvRecorder();
        initFlightController();
        registerVideoFeed();
        startDetection();
    }

    @Override
    protected void onDetachedFromWindow() {
        stopSearch("Search stopped because Target Localization closed.");
        stopDetection();
        stopCenterAssist(null);
        unregisterVideoFeed();
        if (flightController != null) {
            FlightControllerStateDispatcher.removeListener(flightStateListener);
        }
        detectorExecutor.shutdownNow();
        super.onDetachedFromWindow();
    }

    private void init(Context context) {
        setOrientation(VERTICAL);
        setPadding(dp(16), dp(16), dp(16), dp(16));

        TextView title = new TextView(context);
        title.setText("Target Localization");
        title.setTextSize(20f);
        title.setGravity(Gravity.START);
        title.setPadding(0, 0, 0, dp(8));
        addView(title, new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));

        FrameLayout videoFrame = new FrameLayout(context);
        videoFrame.setBackgroundColor(Color.BLACK);
        addView(videoFrame, new LayoutParams(LayoutParams.MATCH_PARENT, dp(430)));

        videoTextureView = new TextureView(context);
        videoTextureView.setSurfaceTextureListener(this);
        videoFrame.addView(videoTextureView, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT));

        overlayView = new TargetLocalizationOverlayView(context);
        videoFrame.addView(overlayView, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT));

        controlsScrollView = new ScrollView(context);
        controlsScrollView.setFillViewport(false);
        addView(controlsScrollView, new LayoutParams(LayoutParams.MATCH_PARENT, 0, 1f));

        LinearLayout controls = new LinearLayout(context);
        controls.setOrientation(VERTICAL);
        controls.setPadding(0, dp(12), 0, dp(16));
        controlsScrollView.addView(controls, new ScrollView.LayoutParams(
                ScrollView.LayoutParams.MATCH_PARENT,
                ScrollView.LayoutParams.WRAP_CONTENT));

        statusText = makeText(context, "Status: waiting for video", 15f);
        telemetryText = makeText(context, "Drone: unknown", 13f);
        detectionText = makeText(context, "Detection: idle", 13f);
        searchText = makeText(context, "Search: idle", 13f);
        controls.addView(statusText);
        controls.addView(telemetryText);
        controls.addView(detectionText);
        controls.addView(searchText);

        LinearLayout detectionRow = makeRow(context);
        startDetectionButton = makeButton(context, "Start Detection");
        startDetectionButton.setOnClickListener(v -> startDetection());
        detectionRow.addView(startDetectionButton, weightedParams(true));

        stopDetectionButton = makeButton(context, "Stop Detection");
        stopDetectionButton.setOnClickListener(v -> stopDetection());
        detectionRow.addView(stopDetectionButton, weightedParams(false));
        controls.addView(detectionRow);

        TextView boundaryLabel = makeText(context,
                "Rough Boundary Coordinates (lat,lng per line; 2 corners or polygon):", 13f);
        boundaryLabel.setPadding(0, dp(8), 0, dp(2));
        controls.addView(boundaryLabel);

        boundaryEditText = new EditText(context);
        boundaryEditText.setHint("34.000000,-117.000000\n34.000200,-117.000400");
        boundaryEditText.setMinLines(3);
        boundaryEditText.setGravity(Gravity.TOP | Gravity.START);
        boundaryEditText.setInputType(InputType.TYPE_CLASS_TEXT
                | InputType.TYPE_TEXT_FLAG_MULTI_LINE
                | InputType.TYPE_TEXT_FLAG_NO_SUGGESTIONS);
        controls.addView(boundaryEditText, new LayoutParams(
                LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));

        LinearLayout searchSettingsRow = makeRow(context);
        searchAltitudeEditText = new EditText(context);
        searchAltitudeEditText.setHint("Alt m");
        searchAltitudeEditText.setText(String.format(Locale.US, "%.1f", DEFAULT_SEARCH_ALTITUDE_M));
        searchAltitudeEditText.setSingleLine(true);
        searchAltitudeEditText.setInputType(InputType.TYPE_CLASS_NUMBER
                | InputType.TYPE_NUMBER_FLAG_DECIMAL);
        searchSettingsRow.addView(searchAltitudeEditText, weightedParams(true));

        trackSpacingEditText = new EditText(context);
        trackSpacingEditText.setHint("Spacing m (auto)");
        trackSpacingEditText.setSingleLine(true);
        trackSpacingEditText.setInputType(InputType.TYPE_CLASS_NUMBER
                | InputType.TYPE_NUMBER_FLAG_DECIMAL);
        searchSettingsRow.addView(trackSpacingEditText, weightedParams(false));
        controls.addView(searchSettingsRow);

        LinearLayout searchRow = makeRow(context);
        startSearchButton = makeButton(context, "Start Search");
        startSearchButton.setOnClickListener(v -> startSearchMission());
        searchRow.addView(startSearchButton, weightedParams(true));

        stopSearchButton = makeButton(context, "Stop Search");
        stopSearchButton.setEnabled(false);
        stopSearchButton.setOnClickListener(v -> stopSearch("Search stopped by user."));
        searchRow.addView(stopSearchButton, weightedParams(false));
        controls.addView(searchRow);

        LinearLayout centerRow = makeRow(context);
        centerButton = makeButton(context, "Center Assist");
        centerButton.setOnClickListener(v -> startCenterAssist());
        centerRow.addView(centerButton, weightedParams(true));

        stopCenterButton = makeButton(context, "Stop Center");
        stopCenterButton.setEnabled(false);
        stopCenterButton.setOnClickListener(v -> stopCenterAssist("Center assist stopped."));
        centerRow.addView(stopCenterButton, weightedParams(false));
        controls.addView(centerRow);

        LinearLayout captureRow = makeRow(context);
        idOverrideEditText = new EditText(context);
        idOverrideEditText.setHint("Target ID");
        idOverrideEditText.setSingleLine(true);
        idOverrideEditText.setInputType(InputType.TYPE_CLASS_TEXT);
        captureRow.addView(idOverrideEditText, weightedParams(true));

        captureButton = makeButton(context, "Capture Target");
        captureButton.setOnClickListener(v -> captureCurrentTarget());
        captureRow.addView(captureButton, weightedParams(false));
        controls.addView(captureRow);

        LinearLayout positionRow = makeRow(context);

        positionTagEditText = new EditText(context);
        positionTagEditText.setHint("Position Tag");
        positionTagEditText.setSingleLine(true);
        positionTagEditText.setInputType(InputType.TYPE_CLASS_NUMBER);
        positionRow.addView(positionTagEditText, weightedParams(true));

        recordPositionButton = makeButton(context, "Record Position");
        recordPositionButton.setOnClickListener(v -> recordCurrentDronePosition());
        positionRow.addView(recordPositionButton, weightedParams(false));

        controls.addView(positionRow);

        LinearLayout fileRow = makeRow(context);
        Button exportButton = makeButton(context, "Export CSV");
        exportButton.setOnClickListener(v -> exportCsv());
        fileRow.addView(exportButton, weightedParams(true));

        Button clearButton = makeButton(context, "Clear List");
        clearButton.setOnClickListener(v -> clearTargetList());
        fileRow.addView(clearButton, weightedParams(false));
        controls.addView(fileRow);

        TextView altitudeText = makeText(context,
                String.format(Locale.US,
                        "Search floor: %.2f m / 25 ft", MIN_SEARCH_ALTITUDE_M),
                12f);
        altitudeText.setPadding(0, dp(8), 0, dp(4));
        controls.addView(altitudeText);

        targetListText = makeText(context, "Captured Targets:\n", 12f);
        controls.addView(targetListText);

        logText = makeText(context, "Log:\n", 11f);
        controls.addView(logText);

        videoDataListener = (bytes, size) -> {
            if (codecManager != null) {
                codecManager.sendDataToDecoder(bytes, size);
            }
        };

        updateButtons();
    }

    private void initFlightController() {
        if (flightController != null) {
            return;
        }
        if (!(DJISampleApplication.getProductInstance() instanceof Aircraft)) {
            appendLog("No aircraft connected.");
            return;
        }

        Aircraft aircraft = (Aircraft) DJISampleApplication.getProductInstance();
        flightController = aircraft.getFlightController();
        camera = aircraft.getCamera();
        gimbal = aircraft.getGimbal();
        if (flightController == null) {
            appendLog("FlightController unavailable.");
            return;
        }

        FlightControllerStateDispatcher.addListener(flightController, flightStateListener);
        appendLog("FlightController ready.");
    }

    private void initCameraAndGimbal() {
        if (!(DJISampleApplication.getProductInstance() instanceof Aircraft)) {
            return;
        }
        Aircraft aircraft = (Aircraft) DJISampleApplication.getProductInstance();
        if (camera == null) {
            camera = aircraft.getCamera();
        }
        if (gimbal == null) {
            gimbal = aircraft.getGimbal();
        }
    }

    private void registerVideoFeed() {
        try {
            VideoFeeder.VideoFeed primaryFeed = VideoFeeder.getInstance().getPrimaryVideoFeed();
            if (primaryFeed != null
                    && videoDataListener != null
                    && !primaryFeed.getListeners().contains(videoDataListener)) {
                primaryFeed.addVideoDataListener(videoDataListener);
                appendLog("Primary camera feed connected.");
            }
        } catch (Exception e) {
            appendLog("Unable to connect camera feed: " + e.getMessage());
        }
    }

    private void unregisterVideoFeed() {
        try {
            VideoFeeder.VideoFeed primaryFeed = VideoFeeder.getInstance().getPrimaryVideoFeed();
            if (primaryFeed != null && videoDataListener != null) {
                primaryFeed.removeVideoDataListener(videoDataListener);
            }
        } catch (Exception ignored) {
        }
    }

    private void startDetection() {
        initCameraAndGimbal();
        tiltGimbalDownForSearch();
        detectionActive = true;
        statusText.setText("Status: detecting targets");
        appendLog("Target detection started.");
        updateButtons();
    }

    private void tiltGimbalDownForSearch() {
        if (gimbal == null) {
            appendLog("Gimbal unavailable; cannot point camera down.");
            return;
        }

        Rotation rotation = new Rotation.Builder()
                .mode(RotationMode.ABSOLUTE_ANGLE)
                .pitch(NADIR_GIMBAL_PITCH_DEG)
                .yaw(Rotation.NO_ROTATION)
                .roll(Rotation.NO_ROTATION)
                .time(GIMBAL_ROTATION_TIME_SECONDS)
                .build();
        gimbal.rotate(rotation, error -> {
            if (error == null) {
                appendLog("Gimbal pitched down to -90 deg for target search.");
            } else {
                appendLog("Gimbal pitch-down failed: " + error.getDescription());
            }
        });
    }

    private void stopDetection() {
        if (searchActive) {
            stopSearch("Search stopped because detection was stopped.");
        }
        detectionActive = false;
        detectionInFlight = false;
        latestDetections = Collections.emptyList();
        bestDetection = null;
        if (overlayView != null) {
            overlayView.setDetections(latestDetections, null);
        }
        if (detectionText != null) {
            detectionText.setText("Detection: stopped");
        }
        updateButtons();
    }

    private void startSearchMission() {
        initFlightController();
        initCameraAndGimbal();
        if (flightController == null) {
            showToast("Flight controller not available.");
            return;
        }
        if (!hasValidGps) {
            showToast("Waiting for GPS fix before search.");
            return;
        }

        List<GeoPoint> boundary;
        try {
            boundary = parseBoundaryPoints(boundaryEditText.getText().toString());
        } catch (IllegalArgumentException e) {
            showToast(e.getMessage());
            appendLog("Search boundary rejected: " + e.getMessage());
            return;
        }

        float requestedAltitude;
        try {
            requestedAltitude = parseSearchAltitude();
        } catch (IllegalArgumentException e) {
            showToast(e.getMessage());
            return;
        }

        double spacingM;
        try {
            spacingM = parseTrackSpacing(requestedAltitude);
        } catch (IllegalArgumentException e) {
            showToast(e.getMessage());
            return;
        }

        List<SearchWaypoint> generated = buildLawnmowerWaypoints(boundary, requestedAltitude, spacingM);
        if (generated.isEmpty()) {
            showToast("No search path generated from boundary.");
            appendLog("Search path generation returned no waypoints.");
            return;
        }
        if (generated.size() > MAX_SEARCH_WAYPOINTS) {
            showToast("Search path too large. Increase spacing or shrink boundary.");
            appendLog(String.format(Locale.US,
                    "Search path rejected: %d waypoints exceeds limit %d.",
                    generated.size(), MAX_SEARCH_WAYPOINTS));
            return;
        }

        startDetection();

        flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
        flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
        flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
        flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);

        searchAltitudeM = requestedAltitude;
        searchWaypoints = generated;
        searchWaypointIndex = closestSearchWaypointIndex(generated);
        searchPausedForTarget = false;
        ignoreDetectionsUntilMs = System.currentTimeMillis() + SEARCH_DETECTION_SUPPRESS_MS;

        flightController.setVirtualStickModeEnabled(true, error -> mainHandler.post(() -> {
            if (error != null) {
                appendLog("Failed to enable Virtual Stick for search: " + error.getDescription());
                showToast("Search start failed.");
                return;
            }

            flightController.setVirtualStickAdvancedModeEnabled(true);
            searchVirtualStickEnabled = true;
            searchActive = true;
            appendLog(String.format(Locale.US,
                    "Search started: %d waypoints, altitude %.1fm, spacing %.1fm.",
                    searchWaypoints.size(), searchAltitudeM, spacingM));
            updateSearchText();
            updateButtons();
            mainHandler.removeCallbacks(searchControlRunnable);
            mainHandler.post(searchControlRunnable);
        }));
    }

    private void stopSearch(String reason) {
        boolean wasActive = searchActive || searchVirtualStickEnabled || searchPausedForTarget;
        if (!wasActive) {
            return;
        }

        searchActive = false;
        searchPausedForTarget = false;
        mainHandler.removeCallbacks(searchControlRunnable);

        if (centeringActive) {
            stopCenterAssist(null);
        } else {
            sendGroundVelocityCommand(0f, 0f, 0f, 0f);
        }

        if (flightController != null && searchVirtualStickEnabled) {
            flightController.setVirtualStickModeEnabled(false, error -> {
                if (error != null) {
                    appendLog("Search Virtual Stick disable warning: " + error.getDescription());
                }
            });
        }

        searchVirtualStickEnabled = false;
        if (reason != null && !reason.isEmpty()) {
            appendLog(reason);
        }
        statusText.setText(detectionActive ? "Status: detecting targets" : "Status: idle");
        updateSearchText();
        updateButtons();
    }

    private void finishSearch() {
        searchActive = false;
        searchPausedForTarget = false;
        mainHandler.removeCallbacks(searchControlRunnable);
        sendGroundVelocityCommand(0f, 0f, 0f, 0f);
        if (flightController != null && searchVirtualStickEnabled) {
            flightController.setVirtualStickModeEnabled(false, error -> {
                if (error == null) {
                    appendLog("Search complete. Virtual Stick disabled.");
                } else {
                    appendLog("Search complete, but Virtual Stick disable warning: "
                            + error.getDescription());
                }
            });
        }
        searchVirtualStickEnabled = false;
        statusText.setText("Status: search complete");
        updateSearchText();
        updateButtons();
    }

    private void runSearchControlStep() {
        if (!searchActive) {
            return;
        }
        if (!hasValidGps) {
            searchText.setText("Search: waiting for GPS");
            sendGroundVelocityCommand(0f, 0f, 0f, 0f);
            return;
        }
        if (currentAlt < MIN_SEARCH_ALTITUDE_M - 0.5f) {
            searchText.setText("Search: climbing to safe altitude");
        }
        if (centeringActive || photoCaptureInProgress) {
            searchPausedForTarget = true;
            sendGroundVelocityCommand(0f, 0f, 0f, 0f);
            return;
        }

        long now = System.currentTimeMillis();
        if (bestDetection != null
                && !isBestDetectionStale()
                && now >= ignoreDetectionsUntilMs) {
            searchPausedForTarget = true;
            sendGroundVelocityCommand(0f, 0f, 0f, 0f);
            appendLog("Search target candidate found. Pausing grid and starting center assist.");
            startCenterAssist();
            return;
        }

        searchPausedForTarget = false;
        if (searchWaypointIndex >= searchWaypoints.size()) {
            finishSearch();
            return;
        }

        SearchWaypoint target = searchWaypoints.get(searchWaypointIndex);
        double distanceM = WaypointNavigationMath.haversineDistance(
                currentLat, currentLng, target.latitude, target.longitude);
        float altitudeError = target.altitudeMeters - currentAlt;

        if (distanceM <= SEARCH_ACCEPTANCE_RADIUS_M
                && Math.abs(altitudeError) <= SEARCH_ALTITUDE_ACCEPTANCE_M) {
            searchWaypointIndex++;
            if (searchWaypointIndex >= searchWaypoints.size()) {
                finishSearch();
                return;
            }
            target = searchWaypoints.get(searchWaypointIndex);
            distanceM = WaypointNavigationMath.haversineDistance(
                    currentLat, currentLng, target.latitude, target.longitude);
            altitudeError = target.altitudeMeters - currentAlt;
            appendLog(String.format(Locale.US,
                    "Search waypoint %d/%d.",
                    searchWaypointIndex + 1, searchWaypoints.size()));
        }

        double bearingRad = WaypointNavigationMath.bearing(
                currentLat, currentLng, target.latitude, target.longitude);
        float horizontalSpeed = clamp((float) (SEARCH_KP_HORIZONTAL * distanceM),
                SEARCH_MIN_HORIZONTAL_SPEED_MPS, SEARCH_MAX_HORIZONTAL_SPEED_MPS);
        if (distanceM <= SEARCH_ACCEPTANCE_RADIUS_M) {
            horizontalSpeed = 0f;
        }

        float eastVelocity = (float) (horizontalSpeed * Math.sin(bearingRad));
        float northVelocity = (float) (horizontalSpeed * Math.cos(bearingRad));
        float verticalVelocity = clamp(SEARCH_KP_VERTICAL * altitudeError,
                -SEARCH_MAX_VERTICAL_SPEED_MPS, SEARCH_MAX_VERTICAL_SPEED_MPS);
        if (currentAlt <= MIN_SEARCH_ALTITUDE_M && verticalVelocity < 0f) {
            verticalVelocity = 0f;
        }

        float yawRate = computeYawRate(Math.toDegrees(bearingRad));
        sendGroundVelocityCommand(eastVelocity, northVelocity, yawRate, verticalVelocity);
        updateSearchText();
    }

    private void sendGroundVelocityCommand(float east, float north, float yawRate, float vertical) {
        if (flightController == null) {
            return;
        }
        FlightControlData data = new FlightControlData(east, north, yawRate, vertical);
        flightController.sendVirtualStickFlightControlData(data, error -> {
            if (error != null) {
                Log.e(TAG, "Search virtual stick send error: " + error.getDescription());
            }
        });
    }

    private float computeYawRate(double targetHeadingDeg) {
        double headingError = WaypointNavigationMath.normalizeHeadingError(
                targetHeadingDeg - currentYawDeg);
        if (Math.abs(headingError) < 5.0) {
            return 0f;
        }
        return clamp((float) (SEARCH_KP_YAW * headingError),
                -SEARCH_MAX_YAW_RATE_DEG_S, SEARCH_MAX_YAW_RATE_DEG_S);
    }

    private float parseSearchAltitude() {
        String text = searchAltitudeEditText.getText().toString().trim();
        float altitude = DEFAULT_SEARCH_ALTITUDE_M;
        if (!text.isEmpty()) {
            try {
                altitude = Float.parseFloat(text);
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("Invalid search altitude.");
            }
        }
        if (altitude < MIN_SEARCH_ALTITUDE_M) {
            throw new IllegalArgumentException("Search altitude must be at least 7.62m / 25ft.");
        }
        return altitude;
    }

    private double parseTrackSpacing(float altitudeM) {
        String text = trackSpacingEditText.getText().toString().trim();
        if (!text.isEmpty()) {
            try {
                double spacing = Double.parseDouble(text);
                if (spacing < 2.0) {
                    throw new IllegalArgumentException("Track spacing must be at least 2m.");
                }
                return spacing;
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("Invalid track spacing.");
            }
        }

        double footprintX = 2.0 * altitudeM * Math.tan(Math.toRadians(CAMERA_HORIZONTAL_FOV_DEG * 0.5));
        double footprintY = 2.0 * altitudeM * Math.tan(Math.toRadians(CAMERA_VERTICAL_FOV_DEG * 0.5));
        return Math.max(2.0, Math.min(footprintX, footprintY) * SEARCH_TRACK_OVERLAP);
    }

    private List<GeoPoint> parseBoundaryPoints(String text) {
        List<GeoPoint> points = new ArrayList<>();
        String[] rows = text.split("[\\r\\n;]+");
        for (String row : rows) {
            String cleaned = row.trim()
                    .replace("(", "")
                    .replace(")", "")
                    .replace("[", "")
                    .replace("]", "");
            if (cleaned.isEmpty()) {
                continue;
            }
            String[] parts = cleaned.split("[,\\s]+");
            if (parts.length < 2) {
                throw new IllegalArgumentException("Boundary rows must be lat,lng.");
            }
            try {
                double lat = Double.parseDouble(parts[0]);
                double lng = Double.parseDouble(parts[1]);
                if (lat < -90.0 || lat > 90.0 || lng < -180.0 || lng > 180.0) {
                    throw new IllegalArgumentException("Boundary coordinate out of range.");
                }
                points.add(new GeoPoint(lat, lng));
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("Invalid boundary coordinate.");
            }
        }

        if (points.size() == 2) {
            GeoPoint a = points.get(0);
            GeoPoint b = points.get(1);
            double south = Math.min(a.latitude, b.latitude);
            double north = Math.max(a.latitude, b.latitude);
            double west = Math.min(a.longitude, b.longitude);
            double east = Math.max(a.longitude, b.longitude);
            List<GeoPoint> rectangle = new ArrayList<>();
            rectangle.add(new GeoPoint(south, west));
            rectangle.add(new GeoPoint(south, east));
            rectangle.add(new GeoPoint(north, east));
            rectangle.add(new GeoPoint(north, west));
            return rectangle;
        }

        if (points.size() < 3) {
            throw new IllegalArgumentException("Enter 2 corners or at least 3 polygon points.");
        }
        return points;
    }

    private List<SearchWaypoint> buildLawnmowerWaypoints(List<GeoPoint> boundary,
                                                         float altitudeM,
                                                         double spacingM) {
        if (boundary.isEmpty()) {
            return Collections.emptyList();
        }

        GeoPoint origin = boundary.get(0);
        List<LocalPoint> polygon = new ArrayList<>();
        double minNorth = Double.MAX_VALUE;
        double maxNorth = -Double.MAX_VALUE;
        for (GeoPoint point : boundary) {
            LocalPoint local = toLocalPoint(point, origin);
            polygon.add(local);
            minNorth = Math.min(minNorth, local.northMeters);
            maxNorth = Math.max(maxNorth, local.northMeters);
        }

        List<SearchWaypoint> waypoints = new ArrayList<>();
        boolean reverse = false;
        for (double north = minNorth; north <= maxNorth + 0.1; north += spacingM) {
            List<Double> intersections = horizontalIntersections(polygon, north);
            Collections.sort(intersections);
            for (int i = 0; i + 1 < intersections.size(); i += 2) {
                double west = intersections.get(i);
                double east = intersections.get(i + 1);
                if (Math.abs(east - west) < 1.0) {
                    continue;
                }

                if (reverse) {
                    waypoints.add(toSearchWaypoint(east, north, origin, altitudeM));
                    waypoints.add(toSearchWaypoint(west, north, origin, altitudeM));
                } else {
                    waypoints.add(toSearchWaypoint(west, north, origin, altitudeM));
                    waypoints.add(toSearchWaypoint(east, north, origin, altitudeM));
                }
                reverse = !reverse;
            }
        }

        if (waypoints.size() < 2) {
            waypoints.clear();
            LocalPoint centroid = centroid(polygon);
            waypoints.add(toSearchWaypoint(centroid.eastMeters, centroid.northMeters,
                    origin, altitudeM));
        }
        return waypoints;
    }

    private List<Double> horizontalIntersections(List<LocalPoint> polygon, double north) {
        List<Double> intersections = new ArrayList<>();
        for (int i = 0; i < polygon.size(); i++) {
            LocalPoint a = polygon.get(i);
            LocalPoint b = polygon.get((i + 1) % polygon.size());
            boolean crosses = (a.northMeters <= north && b.northMeters > north)
                    || (b.northMeters <= north && a.northMeters > north);
            if (!crosses) {
                continue;
            }
            double t = (north - a.northMeters) / (b.northMeters - a.northMeters);
            intersections.add(a.eastMeters + t * (b.eastMeters - a.eastMeters));
        }
        return intersections;
    }

    private int closestSearchWaypointIndex(List<SearchWaypoint> waypoints) {
        int closestIndex = 0;
        double closestDistance = Double.MAX_VALUE;
        for (int i = 0; i < waypoints.size(); i++) {
            SearchWaypoint waypoint = waypoints.get(i);
            double distance = WaypointNavigationMath.haversineDistance(
                    currentLat, currentLng, waypoint.latitude, waypoint.longitude);
            if (distance < closestDistance) {
                closestDistance = distance;
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    private void updateSearchText() {
        if (searchText == null) {
            return;
        }
        if (!searchActive && !searchVirtualStickEnabled) {
            searchText.setText("Search: idle");
            return;
        }
        searchText.setText(String.format(Locale.US,
                "Search: %s | wp %d/%d | alt %.1fm",
                searchPausedForTarget ? "target pause" : "grid",
                Math.min(searchWaypointIndex + 1, searchWaypoints.size()),
                searchWaypoints.size(),
                searchAltitudeM));
    }

    private LocalPoint toLocalPoint(GeoPoint point, GeoPoint origin) {
        double north = Math.toRadians(point.latitude - origin.latitude)
                * WaypointNavigationMath.EARTH_RADIUS_M;
        double east = Math.toRadians(point.longitude - origin.longitude)
                * WaypointNavigationMath.EARTH_RADIUS_M
                * Math.cos(Math.toRadians(origin.latitude));
        return new LocalPoint(east, north);
    }

    private SearchWaypoint toSearchWaypoint(double eastMeters,
                                            double northMeters,
                                            GeoPoint origin,
                                            float altitudeM) {
        double latitude = origin.latitude
                + WaypointNavigationMath.metersToLatitudeDegrees(northMeters);
        double longitude = origin.longitude
                + WaypointNavigationMath.metersToLongitudeDegrees(eastMeters, origin.latitude);
        return new SearchWaypoint(latitude, longitude, altitudeM);
    }

    private LocalPoint centroid(List<LocalPoint> points) {
        double east = 0.0;
        double north = 0.0;
        for (LocalPoint point : points) {
            east += point.eastMeters;
            north += point.northMeters;
        }
        int count = Math.max(1, points.size());
        return new LocalPoint(east / count, north / count);
    }

    private void analyzeFrameIfReady() {
        if (!detectionActive || detectionInFlight || videoTextureView == null
                || !videoTextureView.isAvailable()) {
            return;
        }

        long now = System.currentTimeMillis();
        if (now - lastAnalysisMs < ANALYSIS_INTERVAL_MS) {
            return;
        }
        lastAnalysisMs = now;

        int viewWidth = Math.max(1, videoTextureView.getWidth());
        int viewHeight = Math.max(1, videoTextureView.getHeight());
        int analysisHeight = Math.max(160,
                Math.min(280, Math.round(ANALYSIS_WIDTH_PX * viewHeight / (float) viewWidth)));

        Bitmap frame;
        try {
            frame = videoTextureView.getBitmap(ANALYSIS_WIDTH_PX, analysisHeight);
        } catch (RuntimeException e) {
            appendLog("Frame capture failed: " + e.getMessage());
            return;
        }

        if (frame == null) {
            return;
        }

        detectionInFlight = true;
        detectorExecutor.submit(() -> {
            List<TargetLocalizationDetector.Detection> detections;
            try {
                detections = detector.detect(frame);
            } catch (RuntimeException e) {
                Log.e(TAG, "Detection failed", e);
                detections = Collections.emptyList();
            } finally {
                frame.recycle();
            }

            final List<TargetLocalizationDetector.Detection> finalDetections = detections;
            mainHandler.post(() -> {
                detectionInFlight = false;
                if (!detectionActive) {
                    return;
                }
                latestDetections = finalDetections;
                bestDetection = finalDetections.isEmpty() ? null : finalDetections.get(0);
                overlayView.setDetections(latestDetections, bestDetection);
                updateDetectionText();
            });
        });
    }

    private void updateDetectionText() {
        if (bestDetection == null) {
            detectionText.setText("Detection: searching...");
            return;
        }

        GeoEstimate estimate = estimateTargetLocation(bestDetection);
        String id = bestDetection.targetId == null ? "?" : bestDetection.targetId;
        if (estimate != null) {
            detectionText.setText(String.format(Locale.US,
                    "Detection: %d target(s), best ID %s, %.0f%%, offset x %.2f y %.2f, est %.6f %.6f",
                    latestDetections.size(),
                    id,
                    bestDetection.confidence * 100f,
                    bestDetection.centerXNorm,
                    bestDetection.centerYNorm,
                    estimate.latitude,
                    estimate.longitude));
        } else {
            detectionText.setText(String.format(Locale.US,
                    "Detection: %d target(s), best ID %s, %.0f%%, offset x %.2f y %.2f",
                    latestDetections.size(),
                    id,
                    bestDetection.confidence * 100f,
                    bestDetection.centerXNorm,
                    bestDetection.centerYNorm));
        }
    }

    private void initPositionCsvRecorder() {
        try {
            positionCsvRecorder = new DronePositionCsvRecorder(getContext());
            appendLog("Manual drone position CSV started: "
                    + positionCsvRecorder.getCsvFile().getAbsolutePath());
        } catch (IOException e) {
            positionCsvRecorder = null;
            appendLog("Manual drone position CSV failed to start: " + e.getMessage());
            showToast("Position CSV setup failed.");
        }

        updateButtons();
    }


    private void startCenterAssist() {
        initFlightController();
        if (flightController == null) {
            showToast("Flight controller not available.");
            return;
        }
        if (bestDetection == null || isBestDetectionStale()) {
            showToast("No fresh target detection to center on.");
            return;
        }
        if (!hasValidGps) {
            showToast("Waiting for aircraft GPS/altitude.");
            return;
        }
        if (currentAlt < MIN_SEARCH_ALTITUDE_M) {
            showToast("Center assist requires altitude at or above 7.62 m / 25 ft.");
            appendLog(String.format(Locale.US,
                    "Center assist blocked: current altitude %.1fm is below 7.62m.",
                    currentAlt));
            return;
        }

        flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
        flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
        flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
        flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);

        flightController.setVirtualStickModeEnabled(true, error -> mainHandler.post(() -> {
            if (error != null) {
                showToast("Virtual Stick failed: " + error.getDescription());
                appendLog("Unable to start center assist: " + error.getDescription());
                if (searchActive) {
                    searchPausedForTarget = false;
                    ignoreDetectionsUntilMs = System.currentTimeMillis() + SEARCH_DETECTION_SUPPRESS_MS;
                    updateSearchText();
                }
                return;
            }

            flightController.setVirtualStickAdvancedModeEnabled(true);
            virtualStickEnabledByThisView = true;
            centeringActive = true;
            centeredSinceMs = 0L;
            centeredTargetCaptured = false;
            statusText.setText("Status: center assist active");
            overlayView.setCenteringActive(true);
            appendLog("Center assist started.");
            updateButtons();
            mainHandler.removeCallbacks(centerControlRunnable);
            mainHandler.post(centerControlRunnable);
        }));
    }

    private void stopCenterAssist(String reason) {
        boolean wasActive = centeringActive || virtualStickEnabledByThisView;
        centeringActive = false;
        mainHandler.removeCallbacks(centerControlRunnable);
        if (overlayView != null) {
            overlayView.setCenteringActive(false);
        }

        if (flightController != null && wasActive) {
            sendBodyVelocityCommand(0f, 0f, 0f, 0f);
            if (searchActive) {
                flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
                flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
                searchPausedForTarget = false;
                ignoreDetectionsUntilMs = System.currentTimeMillis() + SEARCH_DETECTION_SUPPRESS_MS;
                updateSearchText();
            } else if (virtualStickEnabledByThisView) {
                flightController.setVirtualStickModeEnabled(false, error -> {
                    if (error != null) {
                        appendLog("Virtual Stick disable warning: " + error.getDescription());
                    }
                });
            }
        }
        virtualStickEnabledByThisView = false;

        if (reason != null && !reason.isEmpty()) {
            appendLog(reason);
            if (searchActive) {
                statusText.setText("Status: search resuming");
            } else {
                statusText.setText(detectionActive ? "Status: detecting targets" : "Status: idle");
            }
        }
        updateButtons();
    }

    private void runCenterControlStep() {
        TargetLocalizationDetector.Detection detection = bestDetection;
        if (detection == null || isBestDetectionStale()) {
            sendBodyVelocityCommand(0f, 0f, 0f, 0f);
            statusText.setText("Status: center assist waiting for target");
            centeredSinceMs = 0L;
            return;
        }

        if (currentAlt < MIN_SEARCH_ALTITUDE_M) {
            sendBodyVelocityCommand(0f, 0f, 0f, 0f);
            stopCenterAssist("Center assist stopped: altitude below 7.62 m / 25 ft.");
            return;
        }

        float offset = detection.offsetMagnitude();
        if (offset <= CENTER_DONE_NORM) {
            sendBodyVelocityCommand(0f, 0f, 0f, 0f);
            long now = System.currentTimeMillis();
            if (centeredSinceMs == 0L) {
                centeredSinceMs = now;
            } else if (now - centeredSinceMs >= CENTER_DONE_HOLD_MS) {
                captureCenteredTargetOnce();
                stopCenterAssist("Target centered. Photo requested and hovering in place.");
            }
            return;
        }

        centeredSinceMs = 0L;
        float rightVelocity = Math.abs(detection.centerXNorm) < CENTER_DEADBAND_NORM
                ? 0f
                : clamp(detection.centerXNorm * CENTER_KP, -MAX_CENTER_SPEED_MPS, MAX_CENTER_SPEED_MPS);
        float forwardVelocity = Math.abs(detection.centerYNorm) < CENTER_DEADBAND_NORM
                ? 0f
                : clamp(-detection.centerYNorm * CENTER_KP, -MAX_CENTER_SPEED_MPS, MAX_CENTER_SPEED_MPS);

        sendBodyVelocityCommand(forwardVelocity, rightVelocity, 0f, 0f);
        statusText.setText(String.format(Locale.US,
                "Status: centering x %.2f y %.2f", detection.centerXNorm, detection.centerYNorm));
    }

    private void sendBodyVelocityCommand(float forward, float right, float yawRate, float vertical) {
        if (flightController == null) {
            return;
        }
        FlightControlData data = new FlightControlData(forward, right, yawRate, vertical);
        flightController.sendVirtualStickFlightControlData(data, error -> {
            if (error != null) {
                Log.e(TAG, "Virtual stick send error: " + error.getDescription());
            }
        });
    }


    private void recordCurrentDronePosition() {
        if (positionCsvRecorder == null) {
            showToast("Position CSV is not ready.");
            appendLog("Record position failed: CSV recorder unavailable.");
            return;
        }

        String tag = positionTagEditText.getText().toString().trim();

        if (tag.isEmpty()) {
            showToast("Enter a tag first.");
            return;
        }

        if (!hasValidGps) {
            showToast("Waiting for valid drone GPS.");
            appendLog("Record position blocked: no valid GPS.");
            return;
        }

        try {
            positionCsvRecorder.recordPosition(currentLat, currentLng, tag);

            showToast("Drone position recorded.");
            appendLog(String.format(
                    Locale.US,
                    "Recorded drone position %.8f, %.8f with tag %s.",
                    currentLat,
                    currentLng,
                    tag
            ));
        } catch (IOException e) {
            showToast("Position CSV write failed.");
            appendLog("Record position failed: " + e.getMessage());
        }
    }
    private void captureCurrentTarget() {
        captureCurrentTarget(true, "Manual capture");
    }

    private boolean captureCurrentTarget(boolean requestPhoto, String sourceLabel) {
        TargetLocalizationDetector.Detection detection = bestDetection;
        if (detection == null || isBestDetectionStale()) {
            showToast("No fresh target detection to capture.");
            return false;
        }

        GeoEstimate estimate = estimateTargetLocation(detection);
        if (estimate == null) {
            showToast("Waiting for valid GPS/altitude before capture.");
            if (requestPhoto) {
                shootTargetPhoto(sourceLabel + " photo without GPS estimate");
            }
            return false;
        }

        String manualId = idOverrideEditText.getText().toString().trim();
        String id = manualId.isEmpty() ? detection.targetId : manualId;
        if (id == null || id.trim().isEmpty()) {
            id = "?";
        }

        TargetRecord record = new TargetRecord(
                targetRecords.size() + 1,
                id,
                estimate.latitude,
                estimate.longitude,
                currentAlt,
                currentYawDeg,
                detection.confidence,
                detection.centerXNorm,
                detection.centerYNorm,
                System.currentTimeMillis());
        targetRecords.add(record);
        updateTargetListText();
        appendLog(String.format(Locale.US,
                "Captured target %s at %.6f, %.6f (confidence %.0f%%).",
                record.targetId,
                record.latitude,
                record.longitude,
                record.confidence * 100f));
        if (requestPhoto) {
            shootTargetPhoto(sourceLabel + " photo");
        }
        return true;
    }

    private void captureCenteredTargetOnce() {
        if (centeredTargetCaptured) {
            return;
        }
        centeredTargetCaptured = true;
        long now = System.currentTimeMillis();
        boolean requestPhoto = now - lastAutoPhotoMs >= AUTO_PHOTO_COOLDOWN_MS;
        if (requestPhoto) {
            lastAutoPhotoMs = now;
        }
        captureCurrentTarget(requestPhoto, "Centered target");
    }

    private void shootTargetPhoto(String reason) {
        initCameraAndGimbal();
        if (camera == null) {
            appendLog("Camera unavailable; cannot take target photo.");
            return;
        }
        if (photoCaptureInProgress) {
            appendLog("Photo capture already in progress.");
            return;
        }

        photoCaptureInProgress = true;
        appendLog("Preparing camera for " + reason + ".");
        if (camera.isFlatCameraModeSupported()) {
            camera.setFlatMode(SettingsDefinitions.FlatCameraMode.PHOTO_SINGLE, error -> {
                if (error != null) {
                    appendLog("Flat photo mode warning: " + error.getDescription());
                }
                startShootPhoto(reason);
            });
        } else {
            camera.setMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, error -> {
                if (error != null) {
                    appendLog("Camera photo mode warning: " + error.getDescription());
                }
                camera.setShootPhotoMode(SettingsDefinitions.ShootPhotoMode.SINGLE, modeError -> {
                    if (modeError != null) {
                        appendLog("Single-photo mode warning: " + modeError.getDescription());
                    }
                    startShootPhoto(reason);
                });
            });
        }
    }

    private void startShootPhoto(String reason) {
        if (camera == null) {
            photoCaptureInProgress = false;
            return;
        }

        camera.startShootPhoto(error -> {
            photoCaptureInProgress = false;
            if (error == null) {
                appendLog("Target photo captured: " + reason + ".");
                showToast("Target photo captured.");
            } else {
                appendLog("Target photo failed: " + error.getDescription());
                showToast("Target photo failed.");
            }
        });
    }

    private GeoEstimate estimateTargetLocation(TargetLocalizationDetector.Detection detection) {
        if (!hasValidGps || currentAlt <= 0.5f || detection == null) {
            return null;
        }

        double altitude = Math.max(0.5, currentAlt);
        double rightAngle = Math.toRadians(detection.centerXNorm * CAMERA_HORIZONTAL_FOV_DEG * 0.5);
        double downAngle = Math.toRadians(detection.centerYNorm * CAMERA_VERTICAL_FOV_DEG * 0.5);
        double rightMeters = Math.tan(rightAngle) * altitude;
        double forwardMeters = -Math.tan(downAngle) * altitude;

        double yawRad = Math.toRadians(currentYawDeg);
        double eastMeters = forwardMeters * Math.sin(yawRad) + rightMeters * Math.cos(yawRad);
        double northMeters = forwardMeters * Math.cos(yawRad) - rightMeters * Math.sin(yawRad);

        double lat = currentLat + WaypointNavigationMath.metersToLatitudeDegrees(northMeters);
        double lng = currentLng + WaypointNavigationMath.metersToLongitudeDegrees(eastMeters, currentLat);
        return new GeoEstimate(lat, lng, northMeters, eastMeters);
    }

    private boolean isBestDetectionStale() {
        return bestDetection == null
                || System.currentTimeMillis() - bestDetection.timestampMs > DETECTION_STALE_MS;
    }

    private void clearTargetList() {
        targetRecords.clear();
        updateTargetListText();
        appendLog("Captured target list cleared.");
    }

    private void updateTargetListText() {
        StringBuilder builder = new StringBuilder("Captured Targets:\n");
        if (targetRecords.isEmpty()) {
            builder.append("  none\n");
        } else {
            for (TargetRecord record : targetRecords) {
                builder.append(String.format(Locale.US,
                        "  %02d | ID %s | %.6f, %.6f | alt %.1fm | conf %.0f%%\n",
                        record.sequence,
                        record.targetId,
                        record.latitude,
                        record.longitude,
                        record.altitudeMeters,
                        record.confidence * 100f));
            }
        }
        targetListText.setText(builder.toString());
    }

    private void exportCsv() {
        if (targetRecords.isEmpty()) {
            showToast("No captured targets to export.");
            return;
        }

        File baseDir = getContext().getExternalFilesDir(null);
        if (baseDir == null) {
            baseDir = getContext().getFilesDir();
        }
        File dir = new File(baseDir, "target-localization");
        if (!dir.exists() && !dir.mkdirs()) {
            showToast("Unable to create export folder.");
            return;
        }

        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        File file = new File(dir, "target_localization_" + timestamp + ".csv");
        try (FileWriter writer = new FileWriter(file)) {
            writer.write("sequence,target_id,latitude,longitude,altitude_m,yaw_deg,confidence,offset_x,offset_y,timestamp_ms\n");
            for (TargetRecord record : targetRecords) {
                writer.write(String.format(Locale.US,
                        "%d,%s,%.8f,%.8f,%.2f,%.2f,%.3f,%.4f,%.4f,%d\n",
                        record.sequence,
                        csv(record.targetId),
                        record.latitude,
                        record.longitude,
                        record.altitudeMeters,
                        record.yawDeg,
                        record.confidence,
                        record.offsetX,
                        record.offsetY,
                        record.timestampMs));
            }
        } catch (IOException e) {
            showToast("CSV export failed.");
            appendLog("CSV export failed: " + e.getMessage());
            return;
        }

        showToast("Target CSV exported.");
        appendLog("CSV exported: " + file.getAbsolutePath());
    }

    private void onFlightControllerState(FlightControllerState state) {
        if (state == null) {
            return;
        }

        LocationCoordinate3D location = state.getAircraftLocation();
        if (location != null) {
            double lat = location.getLatitude();
            double lng = location.getLongitude();
            float alt = location.getAltitude();
            if (lat != 0.0 || lng != 0.0) {
                currentLat = lat;
                currentLng = lng;
                currentAlt = alt;
                hasValidGps = true;
            } else {
                hasValidGps = false;
            }
        }

        Attitude attitude = state.getAttitude();
        if (attitude != null) {
            currentYawDeg = attitude.yaw;
        }

        FlightMode mode = state.getFlightMode();
        if (mode != null) {
            flightModeLabel = mode.name();
        } else if (state.getFlightModeString() != null) {
            flightModeLabel = state.getFlightModeString();
        }

        if (centeringActive && (state.isGoingHome() || state.getFlightMode() == FlightMode.GO_HOME)) {
            post(() -> stopCenterAssist("Center assist stopped: aircraft is returning home."));
        }
        if (searchActive && (state.isGoingHome() || state.getFlightMode() == FlightMode.GO_HOME)) {
            post(() -> stopSearch("Search stopped: aircraft is returning home."));
        }

        post(this::updateTelemetryText);
    }

    private void updateTelemetryText() {
        if (!hasValidGps) {
            telemetryText.setText("Drone: waiting for GPS");
            return;
        }

        telemetryText.setText(String.format(Locale.US,
                "Drone: %.6f, %.6f | alt %.1fm | yaw %.0f | mode %s",
                currentLat,
                currentLng,
                currentAlt,
                currentYawDeg,
                flightModeLabel));
    }

    private void updateButtons() {
        if (startDetectionButton == null) {
            return;
        }

        startDetectionButton.setEnabled(!detectionActive);
        stopDetectionButton.setEnabled(detectionActive);
        startSearchButton.setEnabled(!searchActive);
        stopSearchButton.setEnabled(searchActive || searchVirtualStickEnabled);
        centerButton.setEnabled(detectionActive && !centeringActive);
        stopCenterButton.setEnabled(centeringActive);
        captureButton.setEnabled(detectionActive);

        if (recordPositionButton != null) {
            recordPositionButton.setEnabled(positionCsvRecorder != null);
        }
    }

    private TextView makeText(Context context, String text, float sizeSp) {
        TextView view = new TextView(context);
        view.setText(text);
        view.setTextSize(sizeSp);
        view.setPadding(0, 0, 0, dp(6));
        return view;
    }

    private LinearLayout makeRow(Context context) {
        LinearLayout row = new LinearLayout(context);
        row.setOrientation(HORIZONTAL);
        row.setPadding(0, dp(6), 0, dp(6));
        return row;
    }

    private Button makeButton(Context context, String text) {
        Button button = new Button(context);
        button.setText(text);
        button.setAllCaps(false);
        return button;
    }

    private LayoutParams weightedParams(boolean addMarginEnd) {
        LayoutParams params = new LayoutParams(0, LayoutParams.WRAP_CONTENT, 1f);
        if (addMarginEnd) {
            params.setMarginEnd(dp(8));
        }
        return params;
    }

    private void appendLog(String message) {
        if (logText == null) {
            return;
        }
        post(() -> {
            String timestamp = new SimpleDateFormat("HH:mm:ss", Locale.US).format(new Date());
            logText.append(String.format(Locale.US, "%s  %s\n", timestamp, message));
            controlsScrollView.post(() -> controlsScrollView.fullScroll(View.FOCUS_DOWN));
        });
    }

    private void showToast(String message) {
        Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show();
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    private static String csv(String value) {
        if (value == null) {
            return "";
        }
        String escaped = value.replace("\"", "\"\"");
        return "\"" + escaped + "\"";
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        if (codecManager == null) {
            codecManager = new DJICodecManager(getContext(), surface, width, height);
        }
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        if (codecManager != null) {
            codecManager.cleanSurface();
            codecManager.destroyCodec();
            codecManager = null;
        }
        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        analyzeFrameIfReady();
    }

    private static final class GeoPoint {
        final double latitude;
        final double longitude;

        GeoPoint(double latitude, double longitude) {
            this.latitude = latitude;
            this.longitude = longitude;
        }
    }

    private static final class LocalPoint {
        final double eastMeters;
        final double northMeters;

        LocalPoint(double eastMeters, double northMeters) {
            this.eastMeters = eastMeters;
            this.northMeters = northMeters;
        }
    }

    private static final class SearchWaypoint {
        final double latitude;
        final double longitude;
        final float altitudeMeters;

        SearchWaypoint(double latitude, double longitude, float altitudeMeters) {
            this.latitude = latitude;
            this.longitude = longitude;
            this.altitudeMeters = altitudeMeters;
        }
    }

    private static final class GeoEstimate {
        final double latitude;
        final double longitude;
        final double northMeters;
        final double eastMeters;

        GeoEstimate(double latitude, double longitude, double northMeters, double eastMeters) {
            this.latitude = latitude;
            this.longitude = longitude;
            this.northMeters = northMeters;
            this.eastMeters = eastMeters;
        }
    }

    private static final class TargetRecord {
        final int sequence;
        final String targetId;
        final double latitude;
        final double longitude;
        final float altitudeMeters;
        final double yawDeg;
        final float confidence;
        final float offsetX;
        final float offsetY;
        final long timestampMs;

        TargetRecord(int sequence,
                     String targetId,
                     double latitude,
                     double longitude,
                     float altitudeMeters,
                     double yawDeg,
                     float confidence,
                     float offsetX,
                     float offsetY,
                     long timestampMs) {
            this.sequence = sequence;
            this.targetId = targetId;
            this.latitude = latitude;
            this.longitude = longitude;
            this.altitudeMeters = altitudeMeters;
            this.yawDeg = yawDeg;
            this.confidence = confidence;
            this.offsetX = offsetX;
            this.offsetY = offsetY;
            this.timestampMs = timestampMs;
        }
    }
}
