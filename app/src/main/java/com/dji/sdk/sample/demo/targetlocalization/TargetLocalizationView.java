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
    private TextView targetListText;
    private TextView logText;
    private EditText idOverrideEditText;
    private EditText positionTagEditText;

    private Button startDetectionButton;
    private Button stopDetectionButton;
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
        controls.addView(statusText);
        controls.addView(telemetryText);
        controls.addView(detectionText);

        LinearLayout detectionRow = makeRow(context);
        startDetectionButton = makeButton(context, "Start Detection");
        startDetectionButton.setOnClickListener(v -> startDetection());
        detectionRow.addView(startDetectionButton, weightedParams(true));

        stopDetectionButton = makeButton(context, "Stop Detection");
        stopDetectionButton.setOnClickListener(v -> stopDetection());
        detectionRow.addView(stopDetectionButton, weightedParams(false));
        controls.addView(detectionRow);

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
            if (virtualStickEnabledByThisView) {
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
            statusText.setText(detectionActive ? "Status: detecting targets" : "Status: idle");
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
        centerButton.setEnabled(!centeringActive);
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
