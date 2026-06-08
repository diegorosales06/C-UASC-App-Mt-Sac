package com.dji.sdk.sample.demo.drop;

import android.content.Context;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.util.AttributeSet;
import android.util.Log;
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

import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.utils.FlightControllerStateDispatcher;
import com.dji.sdk.sample.internal.utils.ReturnHomeCommand;
import com.dji.sdk.sample.internal.view.PresentableView;
import com.dji.sdk.sample.demo.geofencing.FlightLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
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
import dji.common.util.CommonCallbacks;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;

public class PayloadDropMissionView extends LinearLayout
        implements PresentableView, TextureView.SurfaceTextureListener {

    private static final String TAG = "DropMissionView";

    private FlightLogger flightLogger;
    private PayloadDropController payloadDropController = new PayloadDropController();

    private static final long CONTROL_LOOP_INTERVAL_MS = 50;
    private static final long FINAL_RTH_DELAY_MS = 5_000L;
    private static final double DROP_RADIUS_M = 0.2;
    private static final float MIN_DROP_ALTITUDE_M = 8.0f;
    private static final float Kp_HORIZONTAL = 0.4f;
    private static final float Kp_VERTICAL = 0.6f;
    private static final float MAX_HORIZONTAL_SPEED = 5.0f;
    private static final float MIN_HORIZONTAL_SPEED = 0.3f;
    private static final float MAX_VERTICAL_SPEED = 2.0f;
    private static final float TAKEOFF_STABLE_ALT_M = 1.0f;
    private static final double EARTH_RADIUS_M = 6_371_000.0;
    private static final double ACCEPTANCE_RADIUS_M = 2.0;
    private static final double ALTITUDE_ACCEPTANCE_M = 0.5;

    private boolean missionRunning = false;
    private boolean isTakingOff = false;
    private double dropTargetLat = DropTargetStore.DEFAULT_DROP_LAT;
    private double dropTargetLng = DropTargetStore.DEFAULT_DROP_LNG;
    private float dropTargetAlt = DropTargetStore.DEFAULT_DROP_ALT;
    private boolean dropTargetSet = true;

    private double currentLat = 0.0;
    private double currentLng = 0.0;
    private float currentAlt = 0.0f;
    private boolean hasValidGPS = false;
    private boolean hasDropped = false;
    private boolean returnHomePending = false;
    private double homeLat = 0.0;
    private double homeLng = 0.0;
    private boolean hasHomePoint = false;

    private FlightController flightController;
    private Timer controlTimer;
    private Runnable returnHomeRunnable;
    private final FlightControllerStateDispatcher.Listener flightStateListener =
            this::onFlightControllerState;

    // UI
    private TextView tvStatus;
    private TextView tvDronePos;
    private EditText etDropLat;
    private EditText etDropLng;
    private EditText etDropAlt;
    private Button btnStart;
    private Button btnStop;
    private Button btnReturnHome;
    private ScrollView scrollLog;
    private TextView tvLog;

    // ── Live downward preview + gimbal ────────────────────────────────────────
    private static final int PREVIEW_HEIGHT_DP = 400;
    private static final float NADIR_GIMBAL_PITCH_DEG = -90f;
    private static final double GIMBAL_ROTATION_TIME_S = 1.0;
    private TextureView videoTextureView;
    private DJICodecManager codecManager;
    private VideoFeeder.VideoDataListener videoDataListener;
    private Gimbal gimbal;

    public PayloadDropMissionView(Context context) {
        super(context);
        init(context);
    }

    public PayloadDropMissionView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    @Override
    public int getDescription() { return 0; }

    @NonNull
    @Override
    public String getHint() { return this.getClass().getSimpleName() + ".java"; }

    private void init(Context context) {
        setOrientation(VERTICAL);
        setPadding(24, 24, 24, 24);

        // ── Live downward camera preview (pinned at top) ──────────────────────
        FrameLayout videoFrame = new FrameLayout(context);
        videoFrame.setBackgroundColor(Color.BLACK);
        addView(videoFrame, new LayoutParams(LayoutParams.MATCH_PARENT, dp(PREVIEW_HEIGHT_DP)));

        videoTextureView = new TextureView(context);
        videoTextureView.setSurfaceTextureListener(this);
        videoFrame.addView(videoTextureView, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT));
        videoDataListener = (bytes, size) -> {
            if (codecManager != null) {
                codecManager.sendDataToDecoder(bytes, size);
            }
        };

        // ── Everything else scrolls below the preview ─────────────────────────
        scrollLog = new ScrollView(context);
        addView(scrollLog, new LayoutParams(LayoutParams.MATCH_PARENT, 0, 1f));
        LinearLayout content = new LinearLayout(context);
        content.setOrientation(VERTICAL);
        content.setPadding(0, dp(8), 0, dp(8));
        scrollLog.addView(content, new ScrollView.LayoutParams(
                ScrollView.LayoutParams.MATCH_PARENT, ScrollView.LayoutParams.WRAP_CONTENT));

        tvStatus = new TextView(context);
        tvStatus.setText("Mission: IDLE");
        tvStatus.setTextSize(16f);
        tvStatus.setPadding(0, 0, 0, 6);
        content.addView(tvStatus);

        tvDronePos = new TextView(context);
        tvDronePos.setText("Drone: unknown");
        tvDronePos.setTextSize(13f);
        tvDronePos.setPadding(0, 0, 0, 14);
        content.addView(tvDronePos);

        TextView dropLabel = new TextView(context);
        dropLabel.setText("Drop Target Coordinates:");
        dropLabel.setTextSize(14f);
        dropLabel.setPadding(0, 0, 0, 4);
        content.addView(dropLabel);

        LinearLayout dropInputRow = new LinearLayout(context);
        dropInputRow.setOrientation(HORIZONTAL);
        dropInputRow.setPadding(0, 8, 0, 8);

        etDropLat = new EditText(context);
        etDropLat.setHint("Drop Lat");
        etDropLat.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams p1 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        p1.setMarginEnd(6);
        etDropLat.setLayoutParams(p1);
        dropInputRow.addView(etDropLat);

        etDropLng = new EditText(context);
        etDropLng.setHint("Drop Lng");
        etDropLng.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams p2 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        p2.setMarginEnd(6);
        etDropLng.setLayoutParams(p2);
        dropInputRow.addView(etDropLng);

        etDropAlt = new EditText(context);
        etDropAlt.setHint("Alt (m)");
        etDropAlt.setText("8");
        etDropAlt.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL);
        LinearLayout.LayoutParams p3 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.6f);
        etDropAlt.setLayoutParams(p3);
        dropInputRow.addView(etDropAlt);

        DropTargetStore.DropTarget savedDropTarget = DropTargetStore.load(context);
        dropTargetLat = savedDropTarget.latitude;
        dropTargetLng = savedDropTarget.longitude;
        dropTargetAlt = savedDropTarget.altitudeMeters;
        etDropLat.setText(String.valueOf(dropTargetLat));
        etDropLng.setText(String.valueOf(dropTargetLng));
        etDropAlt.setText(String.valueOf(dropTargetAlt));

        // Persist the drop target the moment it is edited — not only on Start —
        // so a typed coordinate survives switching mission modes / leaving the
        // screen (this view is recreated and reloads from DropTargetStore).
        android.text.TextWatcher dropTargetWatcher = new android.text.TextWatcher() {
            @Override public void beforeTextChanged(CharSequence s, int a, int b, int c) {}
            @Override public void onTextChanged(CharSequence s, int a, int b, int c) {}
            @Override public void afterTextChanged(android.text.Editable s) {
                persistDropTargetFromFields();
            }
        };
        etDropLat.addTextChangedListener(dropTargetWatcher);
        etDropLng.addTextChangedListener(dropTargetWatcher);
        etDropAlt.addTextChangedListener(dropTargetWatcher);

        content.addView(dropInputRow);

        LinearLayout btnRow = new LinearLayout(context);
        btnRow.setOrientation(HORIZONTAL);
        btnRow.setPadding(0, 8, 0, 16);

        btnStart = new Button(context);
        btnStart.setText("Start Mission");
        LinearLayout.LayoutParams bp1 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bp1.setMarginEnd(8);
        btnStart.setLayoutParams(bp1);
        btnStart.setOnClickListener(v -> onStartMission());
        btnRow.addView(btnStart);

        btnStop = new Button(context);
        btnStop.setText("Stop Mission");
        LinearLayout.LayoutParams bp2 = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bp2.setMarginEnd(8);
        btnStop.setLayoutParams(bp2);
        btnStop.setEnabled(false);
        btnStop.setOnClickListener(v -> onStopMission());
        btnRow.addView(btnStop);

        btnReturnHome = new Button(context);
        btnReturnHome.setText("RTH");
        btnReturnHome.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.8f));
        btnReturnHome.setOnClickListener(v -> requestReturnToHome());
        btnRow.addView(btnReturnHome);

        content.addView(btnRow);

        // ── Manual pin actuation (ground testing / reloading) ─────────────────
        LinearLayout pinRow = new LinearLayout(context);
        pinRow.setOrientation(HORIZONTAL);
        pinRow.setPadding(0, 0, 0, 16);

        Button btnTestDrop = new Button(context);
        btnTestDrop.setText("Test Drop (open pin)");
        LinearLayout.LayoutParams tp = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        tp.setMarginEnd(8);
        btnTestDrop.setLayoutParams(tp);
        btnTestDrop.setOnClickListener(v -> onTestDrop());
        pinRow.addView(btnTestDrop);

        Button btnClosePin = new Button(context);
        btnClosePin.setText("Close Pin (load)");
        btnClosePin.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        btnClosePin.setOnClickListener(v -> onClosePin());
        pinRow.addView(btnClosePin);

        content.addView(pinRow);

        tvLog = new TextView(context);
        tvLog.setText("Log:\n");
        tvLog.setTextSize(11f);
        content.addView(tvLog);

        initFlightController();
    }

    private void initFlightController() {
        if (DJISampleApplication.getProductInstance() == null) {
            appendLog("No aircraft connected.");
            return;
        }
        if (DJISampleApplication.getProductInstance() instanceof Aircraft) {
            Aircraft aircraft = (Aircraft) DJISampleApplication.getProductInstance();
            flightController = aircraft.getFlightController();
            gimbal = aircraft.getGimbal();
            if (flightController != null) {
                flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
                flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
                FlightControllerStateDispatcher.addListener(flightController, flightStateListener);
                appendLog("FlightController ready.");
            } else {
                appendLog("FlightController unavailable.");
            }
        } else {
            appendLog("No aircraft connected.");
        }
    }

    private void onFlightControllerState(FlightControllerState state) {
        LocationCoordinate3D loc = state.getAircraftLocation();
        if (loc == null) return;
        double lat = loc.getLatitude();
        double lng = loc.getLongitude();
        float alt = loc.getAltitude();
        if (lat == 0.0 && lng == 0.0) { hasValidGPS = false; return; }
        currentLat = lat;
        currentLng = lng;
        currentAlt = alt;
        hasValidGPS = true;
        if (isTakingOff && state.isFlying() && currentAlt >= TAKEOFF_STABLE_ALT_M) {
            isTakingOff = false;
            appendLog(String.format("Stable hover reached (%.1fm). Starting drop mission.", currentAlt));
            post(() -> tvStatus.setText("Mission: STABILIZED"));
            enableVirtualStickAndBegin();
        }
        post(() -> tvDronePos.setText(String.format("Drone: (%.6f, %.6f)  alt: %.1fm", lat, lng, alt)));
    }

    private void onStartMission() {
        if (flightController == null) {
            initFlightController();
            if (flightController == null) { showToast("Flight controller not available."); return; }
        }
        if (!hasValidGPS) { showToast("Waiting for GPS fix."); return; }

        String dropLatStr = etDropLat.getText().toString().trim();
        String dropLngStr = etDropLng.getText().toString().trim();
        String dropAltStr = etDropAlt.getText().toString().trim();

        if (dropLatStr.isEmpty() || dropLngStr.isEmpty()) {
            showToast("Enter drop target latitude and longitude.");
            return;
        }

        try {
            dropTargetLat = Double.parseDouble(dropLatStr);
            dropTargetLng = Double.parseDouble(dropLngStr);
            dropTargetAlt = dropAltStr.isEmpty() ? 8.0f : Float.parseFloat(dropAltStr);
        } catch (NumberFormatException e) {
            showToast("Invalid coordinates.");
            return;
        }

        if (dropTargetLat < -90 || dropTargetLat > 90 || dropTargetLng < -180 || dropTargetLng > 180) {
            showToast("Coordinates out of range.");
            return;
        }

        if (dropTargetAlt < MIN_DROP_ALTITUDE_M) {
            showToast("Drop altitude must be at least 6.1 m / 20 ft.");
            return;
        }

        recordHomePointFromCurrentLocation();

        dropTargetSet = true;
        DropTargetStore.save(getContext(), dropTargetLat, dropTargetLng, dropTargetAlt);

        if (flightController.getState() != null && flightController.getState().isFlying()) {
            appendLog("Drone already airborne - skipping takeoff.");
            enableVirtualStickAndBegin();
            return;
        }

        appendLog("Drone on ground - initiating takeoff.");
        post(() -> {
            tvStatus.setText("Mission: TAKING OFF");
            btnStart.setEnabled(false);
            btnStop.setEnabled(true);
        });
        isTakingOff = true;
        flightController.startTakeoff(error -> {
            if (error != null) {
                isTakingOff = false;
                appendLog("Takeoff failed: " + error.getDescription());
                post(() -> {
                    tvStatus.setText("Mission: IDLE");
                    btnStart.setEnabled(true);
                    btnStop.setEnabled(false);
                });
                return;
            }
            appendLog("Takeoff command accepted. Climbing...");
        });
    }

    private void enableVirtualStickAndBegin() {
        if (flightController == null) return;
        flightController.setVirtualStickModeEnabled(true, error -> {
            if (error != null) {
                isTakingOff = false;
                appendLog("Failed to enable Virtual Stick: " + error.getDescription());
                post(() -> {
                    tvStatus.setText("Mission: IDLE");
                    btnStart.setEnabled(true);
                    btnStop.setEnabled(false);
                });
                return;
            }
            appendLog("Virtual Stick mode enabled.");
            isTakingOff = false;
            missionRunning = true;
            hasDropped = false;
            payloadDropController.reset();
            appendLog("Payload drop system armed.");
            appendLog(String.format("Flying to drop target: (%.6f, %.6f) @ %.1fm", dropTargetLat, dropTargetLng, dropTargetAlt));

            flightLogger = new FlightLogger(getContext());
            flightLogger.start();

            post(() -> {
                tvStatus.setText("Mission: RUNNING");
                btnStart.setEnabled(false);
                btnStop.setEnabled(true);
            });

            startControlLoop();
        });
    }

    /**
     * Parses the drop-target fields and saves them to DropTargetStore if lat/lng
     * are valid. Called on every edit so the coordinate persists across mission
     * mode switches / view recreation without needing to press Start. Partial or
     * invalid input (mid-typing) is ignored, keeping the last valid saved value.
     */
    private void persistDropTargetFromFields() {
        String latStr = etDropLat.getText().toString().trim();
        String lngStr = etDropLng.getText().toString().trim();
        String altStr = etDropAlt.getText().toString().trim();
        if (latStr.isEmpty() || lngStr.isEmpty()) return;

        double lat, lng;
        float alt;
        try {
            lat = Double.parseDouble(latStr);
            lng = Double.parseDouble(lngStr);
            alt = altStr.isEmpty() ? DropTargetStore.DEFAULT_DROP_ALT : Float.parseFloat(altStr);
        } catch (NumberFormatException e) {
            return; // mid-typing / invalid — keep the last saved value
        }
        if (lat < -90 || lat > 90 || lng < -180 || lng > 180) return;
        if (Float.isNaN(alt) || Float.isInfinite(alt) || alt < 0f) {
            alt = DropTargetStore.DEFAULT_DROP_ALT;
        }

        dropTargetLat = lat;
        dropTargetLng = lng;
        dropTargetAlt = alt;
        dropTargetSet = true;
        DropTargetStore.save(getContext(), lat, lng, alt);
    }

    private void onTestDrop() {
        if (flightController == null) {
            initFlightController();
            if (flightController == null) {
                showToast("Flight controller not available.");
                return;
            }
        }
        appendLog("Manual test drop requested (opens pin).");
        payloadDropController.dropPayload(flightController, this::appendLog);
    }

    private void onClosePin() {
        if (flightController == null) {
            initFlightController();
            if (flightController == null) {
                showToast("Flight controller not available.");
                return;
            }
        }
        appendLog("Closing pin (reload).");
        payloadDropController.closePin(flightController, this::appendLog);
    }

    private void onStopMission() {
        missionRunning = false;
        isTakingOff = false;
        returnHomePending = false;
        cancelReturnHomeDelay();
        stopControlLoop();
        if (flightLogger != null) { flightLogger.stop(); }
        sendVelocityCommand(0, 0, 0);
        if (flightController != null) {
            flightController.setVirtualStickModeEnabled(false, error -> {
                if (error == null) appendLog("Virtual Stick disabled — manual control restored.");
            });
        }
        post(() -> {
            tvStatus.setText("Mission: STOPPED");
            btnStart.setEnabled(true);
            btnStop.setEnabled(false);
        });
        appendLog("Mission stopped.");
    }

    public void requestReturnToHome() {
        if (flightController == null) {
            initFlightController();
            if (flightController == null) {
                showToast("Flight controller not available.");
                return;
            }
        }

        missionRunning = false;
        isTakingOff = false;
        returnHomePending = false;
        cancelReturnHomeDelay();
        stopControlLoop();
        if (flightLogger != null) { flightLogger.stop(); }
        sendVelocityCommand(0, 0, 0);
        appendLog("Manual RTH requested.");
        post(() -> {
            tvStatus.setText("Mission: RTH REQUESTED");
            btnStart.setEnabled(true);
            btnStop.setEnabled(false);
        });
        triggerRTH();
    }

    private void startControlLoop() {
        controlTimer = new Timer();
        controlTimer.scheduleAtFixedRate(new TimerTask() {
            @Override public void run() { runControlStep(); }
        }, 0, CONTROL_LOOP_INTERVAL_MS);
    }

    private void stopControlLoop() {
        if (controlTimer != null) { controlTimer.cancel(); controlTimer = null; }
    }

    private void runControlStep() {
        if (returnHomePending) {
            sendVelocityCommand(0, 0, 0);
            return;
        }
        if (!missionRunning || !hasValidGPS) return;

        double distanceM = haversineDistance(currentLat, currentLng, dropTargetLat, dropTargetLng);
        float altError = dropTargetAlt - currentAlt;

        if (flightLogger != null) flightLogger.log(currentLat, currentLng, true);

        if (dropTargetSet && !payloadDropController.hasDropped()) {
            boolean closeToDropTarget = distanceM <= DROP_RADIUS_M;
            boolean highEnoughToDrop = currentAlt >= MIN_DROP_ALTITUDE_M;

            if (closeToDropTarget && highEnoughToDrop) {
                appendLog(String.format("Drop target reached! Distance: %.2fm", distanceM));
                sendVelocityCommand(0, 0, 0);
                hasDropped = true;
                missionRunning = false;
                if (flightLogger != null) flightLogger.stop();
                releasePayloadThenReturnHome();
                return;
            }
        }

        float horizontalSpeed = (float)(Kp_HORIZONTAL * distanceM);
        horizontalSpeed = Math.max(MIN_HORIZONTAL_SPEED, Math.min(MAX_HORIZONTAL_SPEED, horizontalSpeed));

        double bearingRad = bearing(currentLat, currentLng, dropTargetLat, dropTargetLng);
        float northVelocity = (float)(horizontalSpeed * Math.cos(bearingRad));
        float eastVelocity  = (float)(horizontalSpeed * Math.sin(bearingRad));

        float verticalVelocity = Kp_VERTICAL * altError;
        verticalVelocity = Math.max(-MAX_VERTICAL_SPEED, Math.min(MAX_VERTICAL_SPEED, verticalVelocity));

        sendVelocityCommand(eastVelocity, northVelocity, verticalVelocity);
    }

    private void sendVelocityCommand(float pitch, float roll, float vertical) {
        if (flightController == null) return;
        FlightControlData data = new FlightControlData(pitch, roll, 0f, vertical);
        flightController.sendVirtualStickFlightControlData(data, error -> {
            if (error != null) Log.e(TAG, "Virtual stick error: " + error.getDescription());
        });
    }

    private void triggerRTH() {
        if (flightController == null) return;
        flightController.setVirtualStickModeEnabled(false, error -> {
            if (error != null) appendLog("Error disabling Virtual Stick: " + error.getDescription());
            appendLog("Starting RTH with aircraft's current home point.");
            startGoHome();
        });
    }

    /**
     * Disables Virtual Stick, actuates the payload release, then holds before RTH.
     *
     * The release is a FlightAssistant fill-light command. Issuing it while
     * Virtual Stick still owns the flight controller is the one thing different
     * from the LED Control screen (which works), so we hand control back first.
     * With Virtual Stick off the aircraft holds a GPS hover at the drop point.
     */
    private void releasePayloadThenReturnHome() {
        if (flightController == null) {
            payloadDropController.dropPayload(null, this::appendLog);
            holdBeforeReturnHome();
            return;
        }
        appendLog("At drop point — disabling Virtual Stick to release payload.");
        flightController.setVirtualStickModeEnabled(false, error -> {
            if (error != null) {
                appendLog("Virtual Stick disable before drop failed: " + error.getDescription());
            }
            payloadDropController.dropPayload(flightController, this::appendLog);
            holdBeforeReturnHome();
        });
    }

    private void holdBeforeReturnHome() {
        returnHomePending = true;
        sendVelocityCommand(0, 0, 0);
        appendLog(String.format("Mission complete. Holding for %.0f seconds before RTH.",
                FINAL_RTH_DELAY_MS / 1000.0));
        if (hasHomePoint) {
            appendLog(String.format("RTH home point: %.6f, %.6f", homeLat, homeLng));
        }
        post(() -> {
            tvStatus.setText("Mission: HOLDING FOR RTH (5s)");
            btnStart.setEnabled(false);
            btnStop.setEnabled(true);
        });

        cancelReturnHomeDelay();
        returnHomeRunnable = () -> {
            returnHomePending = false;
            stopControlLoop();
            triggerRTH();
        };
        postDelayed(returnHomeRunnable, FINAL_RTH_DELAY_MS);
    }

    private void cancelReturnHomeDelay() {
        if (returnHomeRunnable != null) {
            removeCallbacks(returnHomeRunnable);
            returnHomeRunnable = null;
        }
    }

    private void recordHomePointFromCurrentLocation() {
        if (!hasValidGPS) return;
        homeLat = currentLat;
        homeLng = currentLng;
        hasHomePoint = true;
        appendLog(String.format("Home point recorded from takeoff/start position: %.6f, %.6f",
                homeLat, homeLng));
        setAircraftHomeToRecordedPoint();
    }

    private void setAircraftHomeToRecordedPoint() {
        if (flightController == null || !hasHomePoint) return;
        LocationCoordinate2D homePoint = new LocationCoordinate2D(homeLat, homeLng);
        flightController.setHomeLocation(homePoint, error -> {
            if (error == null) {
                appendLog("Aircraft home location set to recorded takeoff/start point.");
            } else {
                appendLog("Home location update failed: "
                        + error.getDescription()
                        + ". RTH will use the aircraft's current home point if needed.");
            }
        });
    }

    private void startGoHome() {
        ReturnHomeCommand.setGoHomeHeightToCurrentAltitude(flightController, this::appendLog, () ->
                flightController.startGoHome(rthError -> {
            if (rthError == null) {
                post(() -> {
                    tvStatus.setText("Mission: RTH");
                    btnStart.setEnabled(true);
                    btnStop.setEnabled(false);
                });
                appendLog("RTH initiated.");
            } else {
                appendLog("RTH failed: " + rthError.getDescription());
            }
        }));
    }

    static double haversineDistance(double lat1, double lng1, double lat2, double lng2) {
        double dLat = Math.toRadians(lat2 - lat1);
        double dLng = Math.toRadians(lng2 - lng1);
        double a = Math.sin(dLat/2)*Math.sin(dLat/2)
                + Math.cos(Math.toRadians(lat1))*Math.cos(Math.toRadians(lat2))
                * Math.sin(dLng/2)*Math.sin(dLng/2);
        return EARTH_RADIUS_M * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    }

    static double bearing(double lat1, double lng1, double lat2, double lng2) {
        double lat1R = Math.toRadians(lat1), lat2R = Math.toRadians(lat2);
        double dLng = Math.toRadians(lng2 - lng1);
        double y = Math.sin(dLng)*Math.cos(lat2R);
        double x = Math.cos(lat1R)*Math.sin(lat2R) - Math.sin(lat1R)*Math.cos(lat2R)*Math.cos(dLng);
        return (Math.atan2(y, x) + 2*Math.PI) % (2*Math.PI);
    }

    private void appendLog(String message) {
        post(() -> { tvLog.append(message + "\n"); scrollLog.post(() -> scrollLog.fullScroll(View.FOCUS_DOWN)); });
    }

    private void showToast(String message) {
        post(() -> Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show());
    }

    @Override
    protected void onAttachedToWindow() {
        super.onAttachedToWindow();
        registerVideoFeed();
        pointGimbalDown();
    }

    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
        // Catch-all: make sure the latest typed coordinate is saved before this
        // view is torn down (e.g. switching mission modes).
        persistDropTargetFromFields();
        unregisterVideoFeed();
        if (codecManager != null) {
            codecManager.cleanSurface();
            codecManager.destroyCodec();
            codecManager = null;
        }
        if (missionRunning || isTakingOff || returnHomePending) {
            missionRunning = false;
            isTakingOff = false;
            returnHomePending = false;
            cancelReturnHomeDelay();
            stopControlLoop();
            sendVelocityCommand(0,0,0);
            if (flightLogger != null) flightLogger.stop();
        }
        if (flightController != null) {
            FlightControllerStateDispatcher.removeListener(flightStateListener);
            flightController.setVirtualStickModeEnabled(false, null);
        }
    }

    // ── Live downward preview + gimbal ────────────────────────────────────────

    /** Rotates the gimbal to point straight down so the preview shows the ground. */
    private void pointGimbalDown() {
        if (gimbal == null && DJISampleApplication.getProductInstance() instanceof Aircraft) {
            gimbal = ((Aircraft) DJISampleApplication.getProductInstance()).getGimbal();
        }
        if (gimbal == null) {
            appendLog("Gimbal unavailable; cannot point camera down.");
            return;
        }
        Rotation rotation = new Rotation.Builder()
                .mode(RotationMode.ABSOLUTE_ANGLE)
                .pitch(NADIR_GIMBAL_PITCH_DEG)
                .yaw(Rotation.NO_ROTATION)
                .roll(Rotation.NO_ROTATION)
                .time(GIMBAL_ROTATION_TIME_S)
                .build();
        gimbal.rotate(rotation, error ->
                appendLog(error == null
                        ? "Gimbal pointed straight down (-90 deg)."
                        : "Gimbal pitch-down failed: " + error.getDescription()));
    }

    private void registerVideoFeed() {
        try {
            VideoFeeder.VideoFeed feed = VideoFeeder.getInstance().getPrimaryVideoFeed();
            if (feed != null
                    && videoDataListener != null
                    && !feed.getListeners().contains(videoDataListener)) {
                feed.addVideoDataListener(videoDataListener);
            }
        } catch (Exception ignored) {
        }
    }

    private void unregisterVideoFeed() {
        try {
            VideoFeeder.VideoFeed feed = VideoFeeder.getInstance().getPrimaryVideoFeed();
            if (feed != null && videoDataListener != null) {
                feed.removeVideoDataListener(videoDataListener);
            }
        } catch (Exception ignored) {
        }
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
    }

    private int dp(int value) {
        return (int) (value * getResources().getDisplayMetrics().density);
    }
}
