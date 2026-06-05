package com.dji.sdk.sample.demo.drop;

import android.content.Context;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
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
import dji.common.model.LocationCoordinate2D;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;

public class PayloadDropMissionView extends LinearLayout implements PresentableView {

    private static final String TAG = "DropMissionView";

    private FlightLogger flightLogger;
    private PayloadDropController payloadDropController = new PayloadDropController();

    private static final long CONTROL_LOOP_INTERVAL_MS = 50;
    private static final long FINAL_RTH_DELAY_MS = 5_000L;
    private static final double DROP_RADIUS_M = 2.0;
    private static final float MIN_DROP_ALTITUDE_M = 6.1f;
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

        tvStatus = new TextView(context);
        tvStatus.setText("Mission: IDLE");
        tvStatus.setTextSize(16f);
        tvStatus.setPadding(0, 0, 0, 6);
        addView(tvStatus);

        tvDronePos = new TextView(context);
        tvDronePos.setText("Drone: unknown");
        tvDronePos.setTextSize(13f);
        tvDronePos.setPadding(0, 0, 0, 14);
        addView(tvDronePos);

        TextView dropLabel = new TextView(context);
        dropLabel.setText("Drop Target Coordinates:");
        dropLabel.setTextSize(14f);
        dropLabel.setPadding(0, 0, 0, 4);
        addView(dropLabel);

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

        addView(dropInputRow);

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

        addView(btnRow);

        tvLog = new TextView(context);
        tvLog.setText("Log:\n");
        tvLog.setTextSize(11f);

        scrollLog = new ScrollView(context);
        scrollLog.setLayoutParams(new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 500));
        scrollLog.addView(tvLog);
        addView(scrollLog);

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
                payloadDropController.dropPayload(flightController);
                hasDropped = true;
                appendLog("Payload dropped! Initiating RTH.");
                missionRunning = false;
                if (flightLogger != null) flightLogger.stop();
                holdBeforeReturnHome();
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
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
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
}
