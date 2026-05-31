package com.dji.sdk.sample.demo.virtualstickwaypoint;

import android.content.Context;
import android.content.Intent;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;
import android.app.Activity;
import android.app.AlertDialog;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;

import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.view.PresentableView;
import com.dji.sdk.sample.demo.geofencing.FlightLogger;
import com.dji.sdk.sample.internal.utils.OfflineDebugConfig;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;

/**
 * VirtualStickWaypointView
 *
 * Autonomous GPS waypoint navigation for the DJI Mavic Air 2 (and other drones
 * that do NOT support WaypointMission) using Virtual Stick commands.
 *
 * How it works:
 *  1. User defines a list of GPS waypoints (lat, lng, altitude).
 *  2. On "Start Mission", the drone takes off and climbs to the first waypoint's altitude.
 *  3. A control loop runs at 20Hz via a Timer:
 *       a. Read current drone GPS position and altitude from FlightController state.
 *       b. Compute distance and bearing to the current target waypoint using
 *          the Haversine formula.
 *       c. Compute altitude error (target alt - current alt).
 *       d. Convert bearing into North/South (pitch) and East/West (roll) velocity
 *          commands using trigonometry.
 *       e. Scale speed proportionally to distance (P-controller) so the drone
 *          slows down as it approaches the waypoint.
 *       f. Send FlightControlData to the drone via Virtual Stick.
 *  4. When the drone is within ACCEPTANCE_RADIUS_M of the target waypoint AND
 *     within ALTITUDE_ACCEPTANCE_M of the target altitude, it advances to the
 *     next waypoint.
 *  5. After the last waypoint, the drone initiates RTH.
 *
 * Control law (proportional controller):
 *   horizontalSpeed = Kp_horizontal * distanceToWaypoint
 *   verticalSpeed   = Kp_vertical   * altitudeError
 *   Both are clamped to safe max values.
 *
 * Coordinate system used: GROUND (NED — North/East/Down body frame).
 *   pitch > 0 = fly North
 *   roll  > 0 = fly East
 *   verticalThrottle > 0 = climb
 *   yaw = 0 (heading held constant; drone nose stays fixed)
 *
 * To register in the sample app menu:
 *   Add an entry in DemoListView.java pointing to VirtualStickWaypointView.class
 *
 * Package: com.dji.sdk.sample.demo.virtualstickwaypoint
 */
public class VirtualStickWaypointView extends LinearLayout implements PresentableView {

    private static final String TAG = "VSWaypointView";

    // FlightLogger — writes lat/lng/waypoint data to CSV during mission
    private FlightLogger flightLogger;

    // ---------------------------------------------------------------------------------
    // Control loop timing
    // ---------------------------------------------------------------------------------

    private static final long CONTROL_LOOP_INTERVAL_MS = 50;
    private static final double ACCEPTANCE_RADIUS_M = 5.0; // updated to match your 5m target
    private static final double ALTITUDE_ACCEPTANCE_M = 0.5;
    private static final double EARTH_RADIUS_M = 6_371_000.0;
    private static final float  MIN_HORIZONTAL_SPEED = 0.3f;
    private static final float  MAX_VERTICAL_SPEED   = 2.0f;
    private static final double OFFLINE_START_OFFSET_M = 20.0;

    // ---------------------------------------------------------------------------------
    // Tunable control parameters — adjustable in-app via +/- buttons
    // ---------------------------------------------------------------------------------

    // Kp: proportional gain — how hard the P term reacts to position error
    // during the braking phase. Start at 0.4, increase if drone stops short,
    // decrease if it oscillates around the waypoint.
    private float Kp_HORIZONTAL = 0.4f;

    // Kd: derivative gain — braking force proportional to how fast distance
    // is shrinking. Higher = harder braking at high speed = less overshoot.
    // Start at 0.3, increase if drone overshoots, decrease if it brakes too early.
    private float Kd_HORIZONTAL = 0.3f;

    // Kp vertical — unchanged from before
    private float Kp_VERTICAL = 0.6f;

    // Max cruise speed — drone flies at this speed during the feedforward phase
    // (when far from waypoint). Mavic Air 2 hardware max ~19 m/s; keep below 12
    // until you've verified behavior in the field.
    private float MAX_HORIZONTAL_SPEED = 8.0f;

    // Deceleration rate used to compute braking distance (m/s²).
    // brakingDistance = maxSpeed² / (2 * deceleration)
    // At 8 m/s and 2.0 m/s² → 16m braking distance.
    // Increase to start braking earlier (safer), decrease for more aggressive approach.
    private float DECELERATION = 2.0f;

    // ---------------------------------------------------------------------------------
    // PD controller state — previousDistance needed to compute the D term
    // ---------------------------------------------------------------------------------
    private double previousDistance = 0.0;
    // dt in seconds — matches CONTROL_LOOP_INTERVAL_MS
    private static final float DT = CONTROL_LOOP_INTERVAL_MS / 1000.0f;

    // ---------------------------------------------------------------------------------
    // Tuning UI labels — updated whenever a parameter changes
    // ---------------------------------------------------------------------------------
    private TextView tvKp;
    private TextView tvKd;
    private TextView tvSpeed;
    private TextView tvBraking;

    // ---------------------------------------------------------------------------------
    // Waypoint storage
    // Each entry is a double[3]: { latitude, longitude, altitudeMeters }
    // ---------------------------------------------------------------------------------
    private final List<double[]> waypointList = new ArrayList<>();

    // Index of the waypoint the drone is currently flying toward
    private int currentWaypointIndex = 0;

    // ---------------------------------------------------------------------------------
    // Mission state
    // ---------------------------------------------------------------------------------
    private boolean missionRunning = false;

    // Latest drone position — updated by the FlightController state callback
    private double currentLat = 0.0;
    private double currentLng = 0.0;
    private float  currentAlt = 0.0f;  // altitude above takeoff point (meters)
    private boolean hasValidGPS = false;

    // ---------------------------------------------------------------------------------
    // DJI
    // ---------------------------------------------------------------------------------
    private FlightController flightController;

    // Timer that drives the 20Hz control loop
    private Timer controlTimer;

    // ---------------------------------------------------------------------------------
    // UI components
    // ---------------------------------------------------------------------------------
    private TextView   tvStatus;
    private TextView   tvCurrentWaypoint;
    private TextView   tvDronePos;
    private TextView   tvDroneSpeed;     // live ground speed display
    private TextView   tvWaypointList;
    private EditText   etLat;
    private EditText   etLng;
    private EditText   etAlt;
    private Button     btnAddWaypoint;
    private Button     btnImportQr;
    private Button     btnEditWaypoint;
    private Button     btnClearWaypoints;
    private Button     btnStart;
    private Button     btnStop;
    private ScrollView scrollLog;
    private TextView   tvLog;

    // ---------------------------------------------------------------------------------
    // Constructors
    // ---------------------------------------------------------------------------------

    public VirtualStickWaypointView(Context context) {
        super(context);
        init(context);
    }

    public VirtualStickWaypointView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    // ---------------------------------------------------------------------------------
    // PresentableView
    // ---------------------------------------------------------------------------------

    @Override
    public int getDescription() {
        return 0;
    }

    @NonNull
    @Override
    public String getHint() {
        return this.getClass().getSimpleName() + ".java";
    }

    // ---------------------------------------------------------------------------------
    // UI Construction
    // ---------------------------------------------------------------------------------

    private void init(Context context) {

        setOrientation(VERTICAL);
        setPadding(24, 24, 24, 24);

        // Status label
        tvStatus = new TextView(context);
        tvStatus.setText("Mission: IDLE");
        tvStatus.setTextSize(16f);
        tvStatus.setPadding(0, 0, 0, 6);
        addView(tvStatus);

        // Current waypoint target label
        tvCurrentWaypoint = new TextView(context);
        tvCurrentWaypoint.setText("Target: —");
        tvCurrentWaypoint.setTextSize(13f);
        tvCurrentWaypoint.setPadding(0, 0, 0, 6);
        addView(tvCurrentWaypoint);

        // Drone position label
        tvDronePos = new TextView(context);
        tvDronePos.setText("Drone: unknown");
        tvDronePos.setTextSize(13f);
        tvDronePos.setPadding(0, 0, 0, 4);
        addView(tvDronePos);

        // Live ground speed — computed from velocity components in state callback
        tvDroneSpeed = new TextView(context);
        tvDroneSpeed.setText("Speed: — m/s");
        tvDroneSpeed.setTextSize(13f);
        tvDroneSpeed.setPadding(0, 0, 0, 14);
        addView(tvDroneSpeed);

        // Lat / Lng / Alt input row
        LinearLayout inputRow = new LinearLayout(context);
        inputRow.setOrientation(HORIZONTAL);

        etLat = new EditText(context);
        etLat.setHint("Latitude");
        etLat.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams etP1 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        etP1.setMarginEnd(6);
        etLat.setLayoutParams(etP1);
        inputRow.addView(etLat);

        etLng = new EditText(context);
        etLng.setHint("Longitude");
        etLng.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams etP2 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        etP2.setMarginEnd(6);
        etLng.setLayoutParams(etP2);
        inputRow.addView(etLng);

        etAlt = new EditText(context);
        etAlt.setHint("Alt (m)");
        etAlt.setText("10");
        etAlt.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL);
        LinearLayout.LayoutParams etP3 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.6f);
        etAlt.setLayoutParams(etP3);
        inputRow.addView(etAlt);

        addView(inputRow);

        // Add / Import buttons
        LinearLayout btnRow1 = new LinearLayout(context);
        btnRow1.setOrientation(HORIZONTAL);
        btnRow1.setPadding(0, 8, 0, 8);

        btnAddWaypoint = new Button(context);
        btnAddWaypoint.setText("Add Waypoint");
        LinearLayout.LayoutParams bP1 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP1.setMarginEnd(8);
        btnAddWaypoint.setLayoutParams(bP1);
        btnAddWaypoint.setOnClickListener(v -> onAddWaypoint());
        btnRow1.addView(btnAddWaypoint);

        btnImportQr = new Button(context);
        btnImportQr.setText("Import QR");
        LinearLayout.LayoutParams bP2 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        btnImportQr.setLayoutParams(bP2);
        btnImportQr.setOnClickListener(v -> onImportWaypointsQr());
        btnRow1.addView(btnImportQr);

        addView(btnRow1);

        // Edit / Clear buttons
        LinearLayout btnRowEdit = new LinearLayout(context);
        btnRowEdit.setOrientation(HORIZONTAL);
        btnRowEdit.setPadding(0, 0, 0, 8);

        btnEditWaypoint = new Button(context);
        btnEditWaypoint.setText("Edit Waypoint");
        LinearLayout.LayoutParams bP3 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP3.setMarginEnd(8);
        btnEditWaypoint.setLayoutParams(bP3);
        btnEditWaypoint.setOnClickListener(v -> onEditWaypoint());
        btnRowEdit.addView(btnEditWaypoint);

        btnClearWaypoints = new Button(context);
        btnClearWaypoints.setText("Clear All");
        LinearLayout.LayoutParams bP4 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        btnClearWaypoints.setLayoutParams(bP4);
        btnClearWaypoints.setOnClickListener(v -> onClearWaypoints());
        btnRowEdit.addView(btnClearWaypoints);

        addView(btnRowEdit);

        // Waypoint list display
        tvWaypointList = new TextView(context);
        tvWaypointList.setText("Waypoints: (none)");
        tvWaypointList.setTextSize(12f);
        tvWaypointList.setPadding(0, 0, 0, 12);
        tvWaypointList.setOnClickListener(v -> onEditWaypoint());
        addView(tvWaypointList);

        // add default waypoints only after tvWaypointList exists
        // lat, log, altitude(m)
        waypointList.add(new double[]{34.048510, -117.837831, 3});
        waypointList.add(new double[]{34.048414, -117.837468,  3});
        waypointList.add(new double[]{34.048140, -117.837227, 3});
        waypointList.add(new double[]{34.048121, -117.837787, 3});
        waypointList.add(new double[]{34.047934, -117.837956, 3});
        waypointList.add(new double[]{34.047717, -117.837637, 3});
        waypointList.add(new double[]{34.047729, -117.837246, 3});
        refreshWaypointList();

        // Start / Stop buttons
        LinearLayout btnRow2 = new LinearLayout(context);
        btnRow2.setOrientation(HORIZONTAL);
        btnRow2.setPadding(0, 0, 0, 16);

        btnStart = new Button(context);
        btnStart.setText("Start Mission");
        LinearLayout.LayoutParams bP5 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP5.setMarginEnd(8);
        btnStart.setLayoutParams(bP5);
        btnStart.setOnClickListener(v -> onStartMission());
        btnRow2.addView(btnStart);

        btnStop = new Button(context);
        btnStop.setText("Stop Mission");
        LinearLayout.LayoutParams bP6 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        btnStop.setLayoutParams(bP6);
        btnStop.setEnabled(false);
        btnStop.setOnClickListener(v -> onStopMission());
        btnRow2.addView(btnStop);

        addView(btnRow2);

        // ---- Tuning panel ----
        // All parameters adjustable in-app without redeploying.
        // Changes take effect immediately on the next control loop tick.

        TextView tuningHeader = new TextView(context);
        tuningHeader.setText("── Tuning ──────────────────");
        tuningHeader.setTextSize(12f);
        tuningHeader.setPadding(0, 8, 0, 4);
        addView(tuningHeader);

        // Kp row
        tvKp = new TextView(context);
        tvKp.setTextSize(12f);
        addView(buildTuningRow(context, "Kp (P gain)", tvKp,
                () -> { Kp_HORIZONTAL = Math.max(0.05f, Kp_HORIZONTAL - 0.05f); refreshTuningLabels(); },
                () -> { Kp_HORIZONTAL = Math.min(2.0f,  Kp_HORIZONTAL + 0.05f); refreshTuningLabels(); }));

        // Kd row
        tvKd = new TextView(context);
        tvKd.setTextSize(12f);
        addView(buildTuningRow(context, "Kd (D gain)", tvKd,
                () -> { Kd_HORIZONTAL = Math.max(0.0f,  Kd_HORIZONTAL - 0.05f); refreshTuningLabels(); },
                () -> { Kd_HORIZONTAL = Math.min(2.0f,  Kd_HORIZONTAL + 0.05f); refreshTuningLabels(); }));

        // Max speed row
        tvSpeed = new TextView(context);
        tvSpeed.setTextSize(12f);
        addView(buildTuningRow(context, "Max Speed (m/s)", tvSpeed,
                () -> { MAX_HORIZONTAL_SPEED = Math.max(1.0f,  MAX_HORIZONTAL_SPEED - 0.5f); refreshTuningLabels(); },
                () -> { MAX_HORIZONTAL_SPEED = MAX_HORIZONTAL_SPEED + 0.5f; refreshTuningLabels(); }));

        // Deceleration / braking distance row
        tvBraking = new TextView(context);
        tvBraking.setTextSize(12f);
        addView(buildTuningRow(context, "Decel (m/s²)", tvBraking,
                () -> { DECELERATION = Math.max(0.5f, DECELERATION - 0.25f); refreshTuningLabels(); },
                () -> { DECELERATION = Math.min(5.0f, DECELERATION + 0.25f); refreshTuningLabels(); }));

        refreshTuningLabels();

        // Scrollable log
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

    // ---------------------------------------------------------------------------------
    // Tuning UI helpers
    // ---------------------------------------------------------------------------------

    /**
     * Builds a single tuning row: [label]  [-]  [value display]  [+]
     * Tapping - or + fires the corresponding Runnable and refreshes labels.
     */
    private LinearLayout buildTuningRow(Context context, String label,
                                        TextView valueDisplay,
                                        Runnable onDecrement,
                                        Runnable onIncrement) {
        LinearLayout row = new LinearLayout(context);
        row.setOrientation(HORIZONTAL);
        row.setPadding(0, 2, 0, 2);

        TextView lbl = new TextView(context);
        lbl.setText(label);
        lbl.setTextSize(12f);
        LinearLayout.LayoutParams lblP = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1.4f);
        lbl.setLayoutParams(lblP);
        row.addView(lbl);

        Button btnMinus = new Button(context);
        btnMinus.setText("−");
        btnMinus.setTextSize(14f);
        LinearLayout.LayoutParams btnP = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.5f);
        btnMinus.setLayoutParams(btnP);
        btnMinus.setOnClickListener(v -> onDecrement.run());
        row.addView(btnMinus);

        LinearLayout.LayoutParams valP = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.8f);
        valueDisplay.setLayoutParams(valP);
        valueDisplay.setGravity(android.view.Gravity.CENTER);
        row.addView(valueDisplay);

        Button btnPlus = new Button(context);
        btnPlus.setText("+");
        btnPlus.setTextSize(14f);
        LinearLayout.LayoutParams btnP2 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.5f);
        btnPlus.setLayoutParams(btnP2);
        btnPlus.setOnClickListener(v -> onIncrement.run());
        row.addView(btnPlus);

        return row;
    }

    /**
     * Refreshes all tuning value labels to reflect current parameter values.
     * Also logs current config so you can see what ran during a flight when
     * reviewing the Logcat after landing.
     */
    private void refreshTuningLabels() {
        float brakingDist = (MAX_HORIZONTAL_SPEED * MAX_HORIZONTAL_SPEED) / (2 * DECELERATION);
        if (tvKp     != null) tvKp.setText(String.format("%.2f", Kp_HORIZONTAL));
        if (tvKd     != null) tvKd.setText(String.format("%.2f", Kd_HORIZONTAL));
        if (tvSpeed  != null) tvSpeed.setText(String.format("%.1f", MAX_HORIZONTAL_SPEED));
        if (tvBraking != null) tvBraking.setText(String.format("%.1f (→%.1fm)", DECELERATION, brakingDist));
        Log.d(TAG, String.format("Tuning: Kp=%.2f Kd=%.2f speed=%.1f decel=%.2f brakeDist=%.1fm",
                Kp_HORIZONTAL, Kd_HORIZONTAL, MAX_HORIZONTAL_SPEED, DECELERATION, brakingDist));
    }

    // ---------------------------------------------------------------------------------
    // DJI initialisation
    // ---------------------------------------------------------------------------------

    private void initFlightController() {
        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
            prepareOfflineDebugPosition();
            tvStatus.setText("Mission: OFFLINE DEBUG");
            appendLog("Offline debug mode enabled. Drone/controller connection is skipped.");
            return;
        }

        if (DJISampleApplication.getProductInstance() == null) {
            appendLog("No aircraft connected.");
            return;  // exit safely instead of crashing
        }
        if (DJISampleApplication.getProductInstance() instanceof Aircraft) {
            Aircraft aircraft = (Aircraft) DJISampleApplication.getProductInstance();
            flightController = aircraft.getFlightController();
            if (flightController != null) {
                // Configure Virtual Stick control modes before enabling.
                // VELOCITY mode means our pitch/roll values are in m/s, not degrees.
                // GROUND coordinate system means pitch/roll are relative to true
                // North/East, not the drone's nose heading — much easier to work with.
                flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
                flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);

                // Attach the state callback to continuously read GPS position
                flightController.setStateCallback(this::onFlightControllerState);

                appendLog("FlightController ready.");
            } else {
                appendLog("FlightController unavailable.");
            }
        } else {
            appendLog("No aircraft connected.");
        }
    }

    // ---------------------------------------------------------------------------------
    // Flight Controller State Callback
    // Fires ~10Hz — keeps currentLat/Lng/Alt up to date for the control loop
    // ---------------------------------------------------------------------------------

    private void onFlightControllerState(FlightControllerState state) {
        LocationCoordinate3D loc = state.getAircraftLocation();
        if (loc == null) return;

        double lat = loc.getLatitude();
        double lng = loc.getLongitude();
        float  alt = loc.getAltitude();

        // 0,0 means no GPS fix — ignore
        if (lat == 0.0 && lng == 0.0) {
            hasValidGPS = false;
            return;
        }

        currentLat = lat;
        currentLng = lng;
        currentAlt = alt;
        hasValidGPS = true;

        // Compute ground speed from velocity components.
        // getVelocityX = North velocity (m/s)
        // getVelocityY = East velocity (m/s)
        // getVelocityZ = vertical velocity (m/s, positive = down in NED)
        // Ground speed = magnitude of horizontal velocity vector only
        float vx = state.getVelocityX(); // North
        float vy = state.getVelocityY(); // East
        float groundSpeed = (float) Math.sqrt(vx * vx + vy * vy);

        // Log speed when mission is running so Logcat captures it per tick
        if (missionRunning) {
            Log.d(TAG, String.format("speed=%.2f m/s  vx=%.2f  vy=%.2f", groundSpeed, vx, vy));
        }

        // Update UI on main thread
        post(() -> {
            tvDronePos.setText(String.format(
                    "Drone: (%.6f, %.6f)  alt: %.1fm", lat, lng, alt));
            tvDroneSpeed.setText(String.format("Speed: %.2f m/s", groundSpeed));
        });
    }

    // ---------------------------------------------------------------------------------
    // Button handlers
    // ---------------------------------------------------------------------------------

    private void onAddWaypoint() {
        String latStr = etLat.getText().toString().trim();
        String lngStr = etLng.getText().toString().trim();
        String altStr = etAlt.getText().toString().trim();

        double[] waypoint;
        try {
            waypoint = parseWaypointFields(latStr, lngStr, altStr);
        } catch (IllegalArgumentException e) {
            showToast(e.getMessage());
            return;
        }

        waypointList.add(waypoint);
        etLat.setText("");
        etLng.setText("");
        refreshWaypointList();
        appendLog(String.format("Added waypoint %d: (%.6f, %.6f) @ %.1fm",
                waypointList.size(), waypoint[0], waypoint[1], waypoint[2]));
    }

    private void onImportWaypointsQr() {
        if (missionRunning) {
            showToast("Stop the mission before importing waypoints.");
            return;
        }

        Context context = getContext();
        if (!(context instanceof Activity)) {
            showToast("QR scanner needs an activity context.");
            return;
        }

        QrWaypointScanActivity.setResultListener(payload -> post(() -> handleWaypointQrPayload(payload)));
        context.startActivity(new Intent(context, QrWaypointScanActivity.class));
        appendLog("Waypoint QR scanner opened.");
    }

    private void handleWaypointQrPayload(String payload) {
        if (missionRunning) {
            showToast("Stop the mission before importing waypoints.");
            return;
        }

        final List<double[]> importedWaypoints;
        try {
            importedWaypoints = parseWaypointQrPayload(payload);
        } catch (IllegalArgumentException e) {
            showToast(e.getMessage());
            appendLog("QR import failed: " + e.getMessage());
            return;
        }

        StringBuilder preview = new StringBuilder();
        preview.append("Found ").append(importedWaypoints.size()).append(" waypoint");
        if (importedWaypoints.size() != 1) preview.append("s");
        preview.append(":\n\n");
        for (int i = 0; i < importedWaypoints.size(); i++) {
            double[] wp = importedWaypoints.get(i);
            preview.append(String.format("%d: %.6f, %.6f @ %.1fm\n",
                    i + 1, wp[0], wp[1], wp[2]));
        }

        new AlertDialog.Builder(getContext())
                .setTitle("Import Waypoints")
                .setMessage(preview.toString())
                .setPositiveButton("Append", (dialog, which) -> {
                    waypointList.addAll(copyWaypoints(importedWaypoints));
                    refreshWaypointList();
                    appendLog("Imported " + importedWaypoints.size() + " waypoint(s) from QR.");
                })
                .setNeutralButton("Replace", (dialog, which) -> {
                    waypointList.clear();
                    waypointList.addAll(copyWaypoints(importedWaypoints));
                    currentWaypointIndex = 0;
                    refreshWaypointList();
                    updateTargetLabel();
                    appendLog("Replaced waypoint list with " + importedWaypoints.size() + " QR waypoint(s).");
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void onEditWaypoint() {
        if (missionRunning) {
            showToast("Stop the mission before editing waypoints.");
            return;
        }
        if (waypointList.isEmpty()) {
            showToast("No waypoints to edit.");
            return;
        }

        String[] waypointLabels = new String[waypointList.size()];
        for (int i = 0; i < waypointList.size(); i++) {
            double[] wp = waypointList.get(i);
            waypointLabels[i] = String.format("WP%d: %.6f, %.6f @ %.1fm",
                    i + 1, wp[0], wp[1], wp[2]);
        }

        new AlertDialog.Builder(getContext())
                .setTitle("Edit Waypoint")
                .setItems(waypointLabels, (dialog, which) -> showEditWaypointDialog(which))
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void showEditWaypointDialog(int waypointIndex) {
        if (waypointIndex < 0 || waypointIndex >= waypointList.size()) {
            showToast("Waypoint no longer exists.");
            return;
        }

        double[] waypoint = waypointList.get(waypointIndex);
        LinearLayout editor = new LinearLayout(getContext());
        editor.setOrientation(VERTICAL);
        int padding = 32;
        editor.setPadding(padding, padding / 2, padding, 0);

        EditText latInput = buildCoordinateEditor("Latitude", String.format("%.7f", waypoint[0]));
        EditText lngInput = buildCoordinateEditor("Longitude", String.format("%.7f", waypoint[1]));
        EditText altInput = buildAltitudeEditor("Altitude (m)", String.format("%.1f", waypoint[2]));

        editor.addView(latInput);
        editor.addView(lngInput);
        editor.addView(altInput);

        AlertDialog dialog = new AlertDialog.Builder(getContext())
                .setTitle("Waypoint " + (waypointIndex + 1))
                .setView(editor)
                .setPositiveButton("Save", null)
                .setNegativeButton("Cancel", null)
                .create();

        dialog.setOnShowListener(shownDialog -> dialog.getButton(AlertDialog.BUTTON_POSITIVE)
                .setOnClickListener(v -> {
                    double[] updatedWaypoint;
                    try {
                        updatedWaypoint = parseWaypointFields(
                                latInput.getText().toString().trim(),
                                lngInput.getText().toString().trim(),
                                altInput.getText().toString().trim());
                    } catch (IllegalArgumentException e) {
                        showToast(e.getMessage());
                        return;
                    }

                    waypointList.set(waypointIndex, updatedWaypoint);
                    refreshWaypointList();
                    updateTargetLabel();
                    appendLog(String.format("Updated waypoint %d: (%.6f, %.6f) @ %.1fm",
                            waypointIndex + 1, updatedWaypoint[0], updatedWaypoint[1], updatedWaypoint[2]));
                    dialog.dismiss();
                }));

        dialog.show();
    }

    private void onClearWaypoints() {
        if (missionRunning) {
            showToast("Stop the mission before clearing waypoints.");
            return;
        }
        waypointList.clear();
        refreshWaypointList();
        appendLog("Waypoints cleared.");
    }

    private EditText buildCoordinateEditor(String hint, String value) {
        EditText editText = new EditText(getContext());
        editText.setHint(hint);
        editText.setText(value);
        editText.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        return editText;
    }

    private EditText buildAltitudeEditor(String hint, String value) {
        EditText editText = new EditText(getContext());
        editText.setHint(hint);
        editText.setText(value);
        editText.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL);
        return editText;
    }

    private double[] parseWaypointFields(String latStr, String lngStr, String altStr) {
        if (latStr.isEmpty() || lngStr.isEmpty()) {
            throw new IllegalArgumentException("Enter latitude and longitude.");
        }

        double lat;
        double lng;
        float alt;
        try {
            lat = Double.parseDouble(latStr);
            lng = Double.parseDouble(lngStr);
            alt = altStr.isEmpty() ? 10.0f : Float.parseFloat(altStr);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("Invalid coordinates.");
        }

        return validateWaypoint(lat, lng, alt);
    }

    private List<double[]> parseWaypointQrPayload(String payload) {
        if (payload == null || payload.trim().isEmpty()) {
            throw new IllegalArgumentException("QR code was empty.");
        }

        String trimmedPayload = payload.trim();
        try {
            if (trimmedPayload.startsWith("{")) {
                return parseWaypointJson(new JSONObject(trimmedPayload));
            }
            if (trimmedPayload.startsWith("[")) {
                return parseWaypointJsonArray(new JSONArray(trimmedPayload));
            }
        } catch (JSONException e) {
            throw new IllegalArgumentException("QR JSON was not valid.");
        }

        return parseWaypointText(trimmedPayload);
    }

    private List<double[]> parseWaypointJson(JSONObject root) throws JSONException {
        String type = root.optString("type", root.optString("mission", ""));
        validateWaypointQrType(type);

        JSONArray points = root.optJSONArray("points");
        if (points == null) {
            points = root.optJSONArray("waypoints");
        }
        if (points != null) {
            return parseWaypointJsonArray(points);
        }

        List<double[]> result = new ArrayList<>();
        result.add(parseWaypointJsonObject(root));
        return result;
    }

    private List<double[]> parseWaypointJsonArray(JSONArray points) throws JSONException {
        List<double[]> result = new ArrayList<>();
        for (int i = 0; i < points.length(); i++) {
            Object value = points.get(i);
            if (!(value instanceof JSONObject)) {
                throw new IllegalArgumentException("Waypoint JSON points must be objects.");
            }
            result.add(parseWaypointJsonObject((JSONObject) value));
        }
        if (result.isEmpty()) {
            throw new IllegalArgumentException("QR code did not contain waypoints.");
        }
        return result;
    }

    private double[] parseWaypointJsonObject(JSONObject point) {
        double lat = readRequiredJsonDouble(point, "lat", "latitude");
        double lng = readRequiredJsonDouble(point, "lng", "lon", "longitude");
        float alt = (float) readOptionalJsonDouble(point, getDefaultImportAltitude(), "alt", "altitude");
        return validateWaypoint(lat, lng, alt);
    }

    private List<double[]> parseWaypointText(String payload) {
        List<double[]> result = new ArrayList<>();
        String[] lines = payload.split("\\r?\\n");
        boolean sawDataLine = false;

        for (String rawLine : lines) {
            String line = rawLine.trim();
            if (line.isEmpty() || line.startsWith("#")) {
                continue;
            }

            if (!sawDataLine && !line.contains(",")) {
                validateWaypointQrType(line);
                sawDataLine = true;
                continue;
            }

            sawDataLine = true;
            result.add(parseWaypointTextLine(line));
        }

        if (result.isEmpty()) {
            throw new IllegalArgumentException("QR code did not contain waypoints.");
        }
        return result;
    }

    private double[] parseWaypointTextLine(String line) {
        String[] columns = line.split(",");
        for (int i = 0; i < columns.length; i++) {
            columns[i] = columns[i].trim();
        }

        if (columns.length < 2) {
            throw new IllegalArgumentException("Waypoint QR lines need latitude and longitude.");
        }

        int latIndex;
        int lngIndex;
        int altIndex;

        if (isNumeric(columns[0])) {
            latIndex = 0;
            lngIndex = 1;
            altIndex = 2;
        } else if (columns.length >= 4 && !isNumeric(columns[1])) {
            validateWaypointQrType(columns[1]);
            latIndex = 2;
            lngIndex = 3;
            altIndex = 4;
        } else {
            latIndex = 1;
            lngIndex = 2;
            altIndex = 3;
        }

        if (columns.length <= lngIndex) {
            throw new IllegalArgumentException("Waypoint QR lines need latitude and longitude.");
        }

        try {
            double lat = Double.parseDouble(columns[latIndex]);
            double lng = Double.parseDouble(columns[lngIndex]);
            float alt = columns.length > altIndex && !columns[altIndex].isEmpty()
                    ? Float.parseFloat(columns[altIndex])
                    : getDefaultImportAltitude();
            return validateWaypoint(lat, lng, alt);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("Waypoint QR contains invalid coordinates.");
        }
    }

    private void validateWaypointQrType(String type) {
        String normalizedType = type == null ? "" : type.trim().toUpperCase();
        if (normalizedType.isEmpty()
                || normalizedType.equals("WAYPOINTS")
                || normalizedType.equals("WAYPOINT")
                || normalizedType.equals("VS_WAYPOINTS")) {
            return;
        }
        if (normalizedType.equals("PACKAGE_DROP") || normalizedType.equals("DROP_TARGET")) {
            throw new IllegalArgumentException("This QR contains a package drop target, not waypoints.");
        }
        if (normalizedType.equals("TIME_TRIAL") || normalizedType.equals("CIRCUIT_TIME_TRIAL")) {
            throw new IllegalArgumentException("This QR contains a time trial route, not waypoints.");
        }
        throw new IllegalArgumentException("This QR is not a waypoint QR.");
    }

    private double[] validateWaypoint(double lat, double lng, float alt) {
        if (Double.isNaN(lat) || Double.isNaN(lng) || Float.isNaN(alt)
                || Double.isInfinite(lat) || Double.isInfinite(lng) || Float.isInfinite(alt)) {
            throw new IllegalArgumentException("Coordinates must be finite numbers.");
        }
        if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
            throw new IllegalArgumentException("Coordinates out of valid GPS range.");
        }
        if (alt < 0) {
            throw new IllegalArgumentException("Altitude must be 0m or higher.");
        }
        return new double[]{lat, lng, alt};
    }

    private float getDefaultImportAltitude() {
        String altStr = etAlt.getText().toString().trim();
        if (altStr.isEmpty()) {
            return 10.0f;
        }
        try {
            float altitude = Float.parseFloat(altStr);
            return altitude >= 0 ? altitude : 10.0f;
        } catch (NumberFormatException e) {
            return 10.0f;
        }
    }

    private double readRequiredJsonDouble(JSONObject object, String... keys) {
        for (String key : keys) {
            if (object.has(key)) {
                return object.optDouble(key, Double.NaN);
            }
        }
        throw new IllegalArgumentException("Waypoint JSON is missing " + keys[0] + ".");
    }

    private double readOptionalJsonDouble(JSONObject object, double defaultValue, String... keys) {
        for (String key : keys) {
            if (object.has(key)) {
                return object.optDouble(key, defaultValue);
            }
        }
        return defaultValue;
    }

    private boolean isNumeric(String value) {
        if (value == null || value.trim().isEmpty()) {
            return false;
        }
        try {
            Double.parseDouble(value.trim());
            return true;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    private List<double[]> copyWaypoints(List<double[]> source) {
        List<double[]> copy = new ArrayList<>();
        for (double[] waypoint : source) {
            copy.add(new double[]{waypoint[0], waypoint[1], waypoint[2]});
        }
        return copy;
    }

    private void prepareOfflineDebugPosition() {
        if (waypointList.isEmpty()) {
            currentLat = 34.048510;
            currentLng = -117.837831;
            currentAlt = 0.0f;
        } else {
            double[] firstWaypoint = waypointList.get(0);
            currentLat = firstWaypoint[0] - metersToLatitudeDegrees(OFFLINE_START_OFFSET_M);
            currentLng = firstWaypoint[1] - metersToLongitudeDegrees(OFFLINE_START_OFFSET_M, firstWaypoint[0]);
            currentAlt = 0.0f;
        }

        hasValidGPS = true;
        post(() -> {
            tvDronePos.setText(String.format(
                    "Drone: offline sim (%.6f, %.6f)  alt: %.1fm",
                    currentLat, currentLng, currentAlt));
            tvDroneSpeed.setText("Speed: 0.00 m/s");
        });
    }

    private void startOfflineDebugMission() {
        prepareOfflineDebugPosition();
        currentWaypointIndex = 0;
        previousDistance = 0.0;
        missionRunning = true;

        post(() -> {
            tvStatus.setText("Mission: OFFLINE SIM");
            btnStart.setEnabled(false);
            btnStop.setEnabled(true);
            btnAddWaypoint.setEnabled(false);
            btnImportQr.setEnabled(false);
            btnEditWaypoint.setEnabled(false);
            btnClearWaypoints.setEnabled(false);
            updateTargetLabel();
        });

        appendLog("Offline debug mission started. Simulating waypoint travel.");
        startControlLoop();
    }

    private void finishOfflineDebugMission() {
        post(() -> {
            appendLog("Offline debug mission complete.");
            tvStatus.setText("Mission: OFFLINE COMPLETE");
            btnStart.setEnabled(true);
            btnStop.setEnabled(false);
            btnAddWaypoint.setEnabled(true);
            btnImportQr.setEnabled(true);
            btnEditWaypoint.setEnabled(true);
            btnClearWaypoints.setEnabled(true);
            tvCurrentWaypoint.setText("Target: —");
            tvDroneSpeed.setText("Speed: 0.00 m/s");
        });
    }

    private void applyOfflineDebugVelocity(float pitchVelocity, float rollVelocity, float verticalVelocity) {
        double northMeters = rollVelocity * DT;
        double eastMeters = pitchVelocity * DT;

        currentLat += metersToLatitudeDegrees(northMeters);
        currentLng += metersToLongitudeDegrees(eastMeters, currentLat);
        currentAlt += verticalVelocity * DT;
        if (currentAlt < 0.0f) {
            currentAlt = 0.0f;
        }

        float groundSpeed = (float) Math.sqrt(pitchVelocity * pitchVelocity + rollVelocity * rollVelocity);
        post(() -> {
            tvDronePos.setText(String.format(
                    "Drone: offline sim (%.6f, %.6f)  alt: %.1fm",
                    currentLat, currentLng, currentAlt));
            tvDroneSpeed.setText(String.format("Speed: %.2f m/s", groundSpeed));
        });
    }

    private double metersToLatitudeDegrees(double meters) {
        return Math.toDegrees(meters / EARTH_RADIUS_M);
    }

    private double metersToLongitudeDegrees(double meters, double latitude) {
        double latitudeRadius = EARTH_RADIUS_M * Math.cos(Math.toRadians(latitude));
        if (Math.abs(latitudeRadius) < 1.0) {
            return 0.0;
        }
        return Math.toDegrees(meters / latitudeRadius);
    }

    /**
     * Starts the waypoint mission.
     *
     * Steps:
     *  1. Validate we have waypoints and a flight controller.
     *  2. Enable Virtual Stick mode on the flight controller.
     *  3. Reset waypoint index to 0.
     *  4. Start the 20Hz control loop timer.
     */
    private void onStartMission() {
        if (waypointList.size() < 1) {
            showToast("Add at least 1 waypoint.");
            return;
        }

        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
            startOfflineDebugMission();
            return;
        }

        if (flightController == null) {
            initFlightController();
            if (flightController == null) {
                showToast("Flight controller not available.");
                return;
            }
        }
        if (!hasValidGPS) {
            showToast("Waiting for GPS fix.");
            return;
        }

        // Enable Virtual Stick mode — required before sending any FlightControlData.
        // The drone will reject commands if this isn't enabled first.
        flightController.setVirtualStickModeEnabled(true, error -> {
            if (error != null) {
                appendLog("Failed to enable Virtual Stick: " + error.getDescription());
                return;
            }
            appendLog("Virtual Stick mode enabled.");
            currentWaypointIndex = 0;
            previousDistance = 0.0;
            missionRunning = true;

            // Start flight logger — creates a new timestamped CSV file
            flightLogger = new FlightLogger(getContext());
            flightLogger.start();
            appendLog("Logging to: " + flightLogger.getLogFilePath());

            post(() -> {
                tvStatus.setText("Mission: RUNNING");
                btnStart.setEnabled(false);
                btnStop.setEnabled(true);
                btnAddWaypoint.setEnabled(false);
                btnImportQr.setEnabled(false);
                btnEditWaypoint.setEnabled(false);
                btnClearWaypoints.setEnabled(false);
                updateTargetLabel();
            });

            appendLog("Mission started. Flying to waypoint 1.");
            startControlLoop();
        });
    }

    /**
     * Stops the mission immediately.
     * Sends a zero-velocity command to halt the drone in place, then
     * disables Virtual Stick mode so the pilot can take back manual control.
     */
    private void onStopMission() {
        missionRunning = false;
        stopControlLoop();

        // Stop the logger and flush to disk
        if (flightLogger != null) {
            flightLogger.stop();
            appendLog("Log saved to: " + flightLogger.getLogFilePath());
        }

        // Send one final zero command to stop all movement
        sendVelocityCommand(0, 0, 0);

        // Disable Virtual Stick — returns control to the remote controller
        if (flightController != null) {
            flightController.setVirtualStickModeEnabled(false, error -> {
                if (error == null) {
                    appendLog("Virtual Stick disabled — manual control restored.");
                } else {
                    appendLog("Error disabling Virtual Stick: " + error.getDescription());
                }
            });
        }

        post(() -> {
            tvStatus.setText("Mission: STOPPED");
            btnStart.setEnabled(true);
            btnStop.setEnabled(false);
            btnAddWaypoint.setEnabled(true);
            btnImportQr.setEnabled(true);
            btnEditWaypoint.setEnabled(true);
            btnClearWaypoints.setEnabled(true);
            tvCurrentWaypoint.setText("Target: —");
        });

        appendLog("Mission stopped by user.");
    }

    // ---------------------------------------------------------------------------------
    // Control Loop — runs at 20Hz via a Timer
    // ---------------------------------------------------------------------------------

    private void startControlLoop() {
        controlTimer = new Timer();
        controlTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                // Each tick: read position, compute commands, send to drone
                runControlStep();
            }
        }, 0, CONTROL_LOOP_INTERVAL_MS);
    }

    private void stopControlLoop() {
        if (controlTimer != null) {
            controlTimer.cancel();
            controlTimer = null;
        }
    }

    /**
     *
     * One iteration of the control loop.
     * This is the core of the waypoint navigation logic:
     *  1. Get the current target waypoint coordinates.
     *  2. Compute horizontal distance and bearing to it using Haversine.
     *  3. Compute altitude error.
     *  4. If within acceptance radius AND altitude is correct → advance to next waypoint.
     *  5. Otherwise → compute velocity commands and send them.
     */
    private void runControlStep() {
        if (!missionRunning || !hasValidGPS) return;
        if (currentWaypointIndex >= waypointList.size()) return;

        double[] target  = waypointList.get(currentWaypointIndex);
        double targetLat = target[0];
        double targetLng = target[1];
        float  targetAlt = (float) target[2];

        double distanceM = haversineDistance(currentLat, currentLng, targetLat, targetLng);
        float  altError  = targetAlt - currentAlt;

        Log.d(TAG, String.format("WP%d dist=%.2fm altErr=%.2fm lat=%.6f lng=%.6f",
                currentWaypointIndex + 1, distanceM, altError, currentLat, currentLng));

        if (flightLogger != null) flightLogger.log(currentLat, currentLng, true);

        // --- Waypoint acceptance ---
        boolean horizontalReached = distanceM <= ACCEPTANCE_RADIUS_M;
        boolean verticalReached   = Math.abs(altError) <= ALTITUDE_ACCEPTANCE_M;

        if (horizontalReached && verticalReached) {
            appendLog(String.format("Waypoint %d reached! (dist=%.2fm altErr=%.2fm)",
                    currentWaypointIndex + 1, distanceM, altError));
            previousDistance = 0.0; // reset D term state for next waypoint
            currentWaypointIndex++;

            if (currentWaypointIndex >= waypointList.size()) {
                appendLog("All waypoints complete. Initiating RTH.");
                missionRunning = false;
                stopControlLoop();
                sendVelocityCommand(0, 0, 0);
                if (flightLogger != null) {
                    flightLogger.stop();
                    appendLog("Log saved to: " + flightLogger.getLogFilePath());
                }
                if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
                    finishOfflineDebugMission();
                    return;
                }
                triggerRTH();
                return;
            }

            final int nextIdx = currentWaypointIndex;
            post(() -> { appendLog("Flying to waypoint " + (nextIdx + 1) + "."); updateTargetLabel(); });
            return;
        }

        // -------------------------------------------------------------------------
        // Feedforward + PD horizontal speed controller
        //
        // Two phases based on distance to waypoint:
        //
        // PHASE 1 — CRUISE (distance > brakingDistance):
        //   Fly at MAX_HORIZONTAL_SPEED regardless of how far away the waypoint is.
        //   Pure feedforward — the P and D terms do nothing here.
        //   This is what gives you maximum speed on long transits.
        //
        // PHASE 2 — BRAKING (distance <= brakingDistance):
        //   PD controller takes over.
        //   P term: proportional to remaining distance — slows as drone approaches.
        //   D term: proportional to rate of closure — brakes harder when closing fast.
        //           distanceRate is negative when closing (distance shrinking),
        //           so -Kd * distanceRate adds a positive (braking) contribution.
        //   Together they produce a smooth deceleration that arrives at ~5m accuracy.
        //
        // brakingDistance = v² / (2*a) — standard physics stopping distance formula.
        // -------------------------------------------------------------------------

        float brakingDistance = (MAX_HORIZONTAL_SPEED * MAX_HORIZONTAL_SPEED) / (2 * DECELERATION);

        float horizontalSpeed;
        if (distanceM > brakingDistance) {
            // PHASE 1: full cruise speed
            horizontalSpeed = MAX_HORIZONTAL_SPEED;
        } else {
            // PHASE 2: PD braking
            // D term — rate of change of distance (negative = closing in)
            double distanceRate = (distanceM - previousDistance) / DT;

            // P term reacts to current error, D term brakes against fast closure
            horizontalSpeed = (float)(Kp_HORIZONTAL * distanceM - Kd_HORIZONTAL * distanceRate);
            horizontalSpeed = Math.max(MIN_HORIZONTAL_SPEED,
                    Math.min(MAX_HORIZONTAL_SPEED, horizontalSpeed));
        }

        // Store distance for D term on next tick
        previousDistance = distanceM;

        // --- Bearing and velocity decomposition ---
        double bearingRad   = bearing(currentLat, currentLng, targetLat, targetLng);
        float northVelocity = (float)(horizontalSpeed * Math.cos(bearingRad));
        float eastVelocity  = (float)(horizontalSpeed * Math.sin(bearingRad));

        // DJI GROUND + VELOCITY: pitch = east/west, roll = north/south
        float pitchVelocity = eastVelocity;
        float rollVelocity  = northVelocity;

        // Proportional vertical
        float verticalVelocity = Kp_VERTICAL * altError;
        verticalVelocity = Math.max(-MAX_VERTICAL_SPEED, Math.min(MAX_VERTICAL_SPEED, verticalVelocity));

        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
            applyOfflineDebugVelocity(pitchVelocity, rollVelocity, verticalVelocity);
        } else {
            sendVelocityCommand(pitchVelocity, rollVelocity, verticalVelocity);
        }
    }

    // ---------------------------------------------------------------------------------
    // Virtual Stick Command
    // ---------------------------------------------------------------------------------

    /**
     * Sends a FlightControlData packet to the drone via Virtual Stick.
     *
     * Parameters (all in m/s in VELOCITY + GROUND mode):
     *  @param pitch    North/South velocity — positive = fly North
     *  @param roll     East/West velocity  — positive = fly East
     *  @param vertical Climb/descend rate  — positive = climb
     *
     * Yaw is held at 0 (no rotation). The drone's nose stays fixed.
     * To make the nose follow the flight direction, replace 0f with a
     * computed yaw rate based on bearingRad.
     */
    private void sendVelocityCommand(float pitch, float roll, float vertical) {
        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) return;
        if (flightController == null) return;

        // FlightControlData constructor: (pitch, roll, yaw, vertical)
        FlightControlData data = new FlightControlData(pitch, roll, 0f, vertical);

        flightController.sendVirtualStickFlightControlData(data, error -> {
            if (error != null) {
                Log.e(TAG, "Virtual stick send error: " + error.getDescription());
            }
        });
    }

    // ---------------------------------------------------------------------------------
    // Return to Home
    // ---------------------------------------------------------------------------------

    private void triggerRTH() {
        if (OfflineDebugConfig.OFFLINE_DEBUG_MODE) {
            finishOfflineDebugMission();
            return;
        }
        if (flightController == null) return;

        // Disable Virtual Stick first so RTH can take over
        flightController.setVirtualStickModeEnabled(false, error -> {
            flightController.startGoHome(rthError -> {
                if (rthError == null) {
                    post(() -> {
                        appendLog("RTH initiated successfully.");
                        tvStatus.setText("Mission: RTH");
                        btnStart.setEnabled(true);
                        btnStop.setEnabled(false);
                        btnAddWaypoint.setEnabled(true);
                        btnImportQr.setEnabled(true);
                        btnEditWaypoint.setEnabled(true);
                        btnClearWaypoints.setEnabled(true);
                    });
                } else {
                    post(() -> appendLog("RTH failed: " + rthError.getDescription()));
                }
            });
        });
    }

    // ---------------------------------------------------------------------------------
    // Navigation Math
    // ---------------------------------------------------------------------------------

    /**
     * Haversine formula — computes the great-circle distance in meters between
     * two GPS coordinates.
     *
     * This is the standard formula for GPS distance calculation. It accounts for
     * the curvature of the Earth, though at drone-scale distances (~100s of meters)
     * a flat-Earth approximation would also work. Haversine is used here for correctness.
     *
     * @param lat1 current latitude  (degrees)
     * @param lng1 current longitude (degrees)
     * @param lat2 target latitude   (degrees)
     * @param lng2 target longitude  (degrees)
     * @return distance in meters
     */
    static double haversineDistance(double lat1, double lng1,
                                    double lat2, double lng2) {
        double dLat = Math.toRadians(lat2 - lat1);
        double dLng = Math.toRadians(lng2 - lng1);

        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(dLng / 2) * Math.sin(dLng / 2);

        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return EARTH_RADIUS_M * c;
    }

    /**
     * Computes the initial bearing (heading) from point 1 to point 2.
     *
     * Returns a value in radians, measured clockwise from true North (0 = North,
     * π/2 = East, π = South, 3π/2 = West).
     *
     * This bearing is used to decompose horizontal speed into North (pitch)
     * and East (roll) components via cos() and sin() respectively.
     *
     * @return bearing in radians [0, 2π)
     */
    static double bearing(double lat1, double lng1, double lat2, double lng2) {
        double lat1Rad = Math.toRadians(lat1);
        double lat2Rad = Math.toRadians(lat2);
        double dLng    = Math.toRadians(lng2 - lng1);

        double y = Math.sin(dLng) * Math.cos(lat2Rad);
        double x = Math.cos(lat1Rad) * Math.sin(lat2Rad)
                - Math.sin(lat1Rad) * Math.cos(lat2Rad) * Math.cos(dLng);

        // atan2 returns [-π, π]; normalize to [0, 2π]
        return (Math.atan2(y, x) + 2 * Math.PI) % (2 * Math.PI);
    }

    // ---------------------------------------------------------------------------------
    // UI helpers
    // ---------------------------------------------------------------------------------

    private void updateTargetLabel() {
        if (currentWaypointIndex < waypointList.size()) {
            double[] wp = waypointList.get(currentWaypointIndex);
            tvCurrentWaypoint.setText(String.format(
                    "Target WP%d: (%.6f, %.6f) @ %.1fm",
                    currentWaypointIndex + 1, wp[0], wp[1], wp[2]));
        } else {
            tvCurrentWaypoint.setText("Target: —");
        }
    }

    private void refreshWaypointList() {
        if (waypointList.isEmpty()) {
            tvWaypointList.setText("Waypoints: (none)");
            return;
        }
        StringBuilder sb = new StringBuilder("Waypoints:\n");
        for (int i = 0; i < waypointList.size(); i++) {
            double[] wp = waypointList.get(i);
            sb.append(String.format("  %d: (%.6f, %.6f) @ %.1fm\n",
                    i + 1, wp[0], wp[1], wp[2]));
        }
        tvWaypointList.setText(sb.toString());
    }

    private void appendLog(String message) {
        post(() -> {
            tvLog.append(message + "\n");
            scrollLog.post(() -> scrollLog.fullScroll(View.FOCUS_DOWN));
        });
    }

    private void showToast(String message) {
        post(() -> Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show());
    }

    // ---------------------------------------------------------------------------------
    // Lifecycle
    // ---------------------------------------------------------------------------------

    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
        // If mission is running when user navigates away, stop everything safely
        if (missionRunning) {
            missionRunning = false;
            stopControlLoop();
            sendVelocityCommand(0, 0, 0);
            if (flightLogger != null) flightLogger.stop();
        }
        // Always detach state callback and disable virtual stick on exit
        if (flightController != null) {
            flightController.setStateCallback(null);
            flightController.setVirtualStickModeEnabled(false, null);
        }
        QrWaypointScanActivity.clearResultListener();
    }
}
