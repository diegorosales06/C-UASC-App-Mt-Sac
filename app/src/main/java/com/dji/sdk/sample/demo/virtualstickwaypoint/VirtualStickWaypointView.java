package com.dji.sdk.sample.demo.virtualstickwaypoint;

import android.app.AlertDialog;
import android.content.Context;
import android.content.Intent;
import android.util.AttributeSet;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;
import android.app.Activity;

import androidx.annotation.NonNull;

import com.dji.sdk.sample.internal.view.PresentableView;

import java.util.List;

/**
 * VirtualStickWaypointView
 *
 * Entry point for the DemoList — the class DemoListView.java points to.
 *
 * Responsibilities (this file only):
 *   - Build and own all Android widgets (TextViews, EditTexts, Buttons, ScrollView).
 *   - Wire button click handlers, which delegate immediately to WaypointMissionController.
 *   - Implement MissionCallback to receive UI update events from the controller.
 *   - Launch QrWaypointScanActivity and forward the scanned payload to WaypointStore.
 *
 * No navigation math, no mission logic, no DJI SDK calls live here.
 *
 * File map:
 *   WaypointMissionController  — mission state machine, control loop, dwell, RTH
 *   WaypointStore              — waypoint list, SharedPrefs persistence, QR parsing
 *   WaypointNavigationMath     — haversine, bearing, coordinate conversion
 *   TuningPanel                — PD parameter UI and SharedPrefs persistence
 *   MissionCallback            — interface connecting controller → view
 */
public class VirtualStickWaypointView extends LinearLayout
        implements PresentableView, MissionCallback {

    private static final String TAG = "VSWaypointView";

    // ── Collaborators ─────────────────────────────────────────────────────────
    private WaypointStore             store;
    private TuningPanel               tuning;
    private WaypointMissionController controller;

    // ── UI widgets ────────────────────────────────────────────────────────────
    private TextView   tvStatus;
    private TextView   tvCurrentWaypoint;
    private TextView   tvDronePos;
    private TextView   tvDroneSpeed;
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

    // =========================================================================
    // Constructors
    // =========================================================================

    public VirtualStickWaypointView(Context context) {
        super(context);
        init(context);
    }

    public VirtualStickWaypointView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    // =========================================================================
    // PresentableView
    // =========================================================================

    @Override
    public int getDescription() { return 0; }

    @NonNull
    @Override
    public String getHint() { return this.getClass().getSimpleName() + ".java"; }

    // =========================================================================
    // MissionCallback implementation
    // All methods are already called on the main thread by the controller.
    // =========================================================================

    @Override
    public void onStatusChanged(String status) {
        tvStatus.setText(status);
    }

    @Override
    public void onTargetLabelChanged(String label) {
        tvCurrentWaypoint.setText(label);
    }

    @Override
    public void onLogMessage(String message) {
        post(() -> {
            tvLog.append(message + "\n");
            scrollLog.post(() -> scrollLog.fullScroll(View.FOCUS_DOWN));
        });
    }

    @Override
    public void onMissionActiveChanged(boolean missionActive) {
        btnStart.setEnabled(!missionActive);
        btnStop.setEnabled(missionActive);
        btnAddWaypoint.setEnabled(!missionActive);
        btnImportQr.setEnabled(!missionActive);
        btnEditWaypoint.setEnabled(!missionActive);
        btnClearWaypoints.setEnabled(!missionActive);
        if (!missionActive) {
            tvCurrentWaypoint.setText("Target: —");
        }
    }

    @Override
    public void onTelemetryUpdated(String positionLabel, String speedLabel) {
        tvDronePos.setText(positionLabel);
        tvDroneSpeed.setText(speedLabel);
    }

    // =========================================================================
    // UI Construction
    // =========================================================================

    private void init(Context context) {
        setOrientation(VERTICAL);
        setPadding(24, 24, 24, 24);

        // ── Collaborators ─────────────────────────────────────────────────────
        store      = new WaypointStore(context);
        tuning     = new TuningPanel(context);
        controller = new WaypointMissionController(context, store, tuning, this);

        // ── Status labels ─────────────────────────────────────────────────────
        tvStatus = new TextView(context);
        tvStatus.setText("Mission: IDLE");
        tvStatus.setTextSize(16f);
        tvStatus.setPadding(0, 0, 0, 6);
        addView(tvStatus);

        tvCurrentWaypoint = new TextView(context);
        tvCurrentWaypoint.setText("Target: —");
        tvCurrentWaypoint.setTextSize(13f);
        tvCurrentWaypoint.setPadding(0, 0, 0, 6);
        addView(tvCurrentWaypoint);

        tvDronePos = new TextView(context);
        tvDronePos.setText("Drone: unknown");
        tvDronePos.setTextSize(13f);
        tvDronePos.setPadding(0, 0, 0, 4);
        addView(tvDronePos);

        tvDroneSpeed = new TextView(context);
        tvDroneSpeed.setText("Speed: — m/s");
        tvDroneSpeed.setTextSize(13f);
        tvDroneSpeed.setPadding(0, 0, 0, 14);
        addView(tvDroneSpeed);

        // ── Coordinate input row ──────────────────────────────────────────────
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
        etAlt.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.6f));
        inputRow.addView(etAlt);

        addView(inputRow);

        // ── Add / Import buttons ──────────────────────────────────────────────
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
        btnImportQr.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        btnImportQr.setOnClickListener(v -> onImportWaypointsQr());
        btnRow1.addView(btnImportQr);

        addView(btnRow1);

        // ── Edit / Clear buttons ──────────────────────────────────────────────
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
        btnClearWaypoints.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        btnClearWaypoints.setOnClickListener(v -> onClearWaypoints());
        btnRowEdit.addView(btnClearWaypoints);

        addView(btnRowEdit);

        // ── Waypoint list display ─────────────────────────────────────────────
        tvWaypointList = new TextView(context);
        tvWaypointList.setTextSize(12f);
        tvWaypointList.setPadding(0, 0, 0, 12);
        tvWaypointList.setOnClickListener(v -> onEditWaypoint());
        addView(tvWaypointList);
        refreshWaypointListDisplay();

        // ── Start / Stop buttons ──────────────────────────────────────────────
        LinearLayout btnRow2 = new LinearLayout(context);
        btnRow2.setOrientation(HORIZONTAL);
        btnRow2.setPadding(0, 0, 0, 16);

        btnStart = new Button(context);
        btnStart.setText("Start Mission");
        LinearLayout.LayoutParams bP5 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP5.setMarginEnd(8);
        btnStart.setLayoutParams(bP5);
        btnStart.setOnClickListener(v -> controller.startMission());
        btnRow2.addView(btnStart);

        btnStop = new Button(context);
        btnStop.setText("Stop Mission");
        btnStop.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        btnStop.setEnabled(false);
        btnStop.setOnClickListener(v -> controller.stopMission());
        btnRow2.addView(btnStop);

        addView(btnRow2);

        // ── Tuning panel ──────────────────────────────────────────────────────
        tuning.setChangeListener((kp, kd, kpV, speed, decel, brakeDist) ->
                onLogMessage(String.format(
                        "Tuning: Kp=%.2f Kd=%.2f KpV=%.2f speed=%.1f decel=%.2f brakeD=%.1fm",
                        kp, kd, kpV, speed, decel, brakeDist)));
        addView(tuning.buildPanelView(context));

        // ── Scrollable log ────────────────────────────────────────────────────
        tvLog = new TextView(context);
        tvLog.setText("Log:\n");
        tvLog.setTextSize(11f);

        scrollLog = new ScrollView(context);
        scrollLog.setLayoutParams(new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 500));
        scrollLog.addView(tvLog);
        addView(scrollLog);

        controller.initFlightController();
    }

    // =========================================================================
    // Button handlers
    // =========================================================================

    private void onAddWaypoint() {
        String latStr = etLat.getText().toString().trim();
        String lngStr = etLng.getText().toString().trim();
        String altStr = etAlt.getText().toString().trim();

        if (latStr.isEmpty() || lngStr.isEmpty()) {
            showToast("Enter latitude and longitude.");
            return;
        }

        try {
            double lat = Double.parseDouble(latStr);
            double lng = Double.parseDouble(lngStr);
            float  alt = altStr.isEmpty() ? 10.0f : Float.parseFloat(altStr);
            store.addWaypoint(lat, lng, alt);
            etLat.setText("");
            etLng.setText("");
            refreshWaypointListDisplay();
            onLogMessage(String.format("Added waypoint %d: (%.6f, %.6f) @ %.1fm",
                    store.size(), lat, lng, alt));
        } catch (NumberFormatException e) {
            showToast("Invalid coordinates.");
        } catch (IllegalArgumentException e) {
            showToast(e.getMessage());
        }
    }

    private void onImportWaypointsQr() {
        Context ctx = getContext();
        if (!(ctx instanceof Activity)) {
            showToast("QR scanner needs an activity context.");
            return;
        }
        QrWaypointScanActivity.setResultListener(
                payload -> post(() -> handleWaypointQrPayload(payload)));
        ctx.startActivity(new Intent(ctx, QrWaypointScanActivity.class));
        onLogMessage("Waypoint QR scanner opened.");
    }

    private void handleWaypointQrPayload(String payload) {
        final List<double[]> imported;
        try {
            imported = store.parseWaypointQrPayload(payload);
        } catch (IllegalArgumentException e) {
            showToast(e.getMessage());
            onLogMessage("QR import failed: " + e.getMessage());
            return;
        }

        StringBuilder preview = new StringBuilder();
        preview.append("Found ").append(imported.size()).append(" waypoint");
        if (imported.size() != 1) preview.append("s");
        preview.append(":\n\n");
        for (int i = 0; i < imported.size(); i++) {
            double[] wp = imported.get(i);
            preview.append(String.format("%d: %.6f, %.6f @ %.1fm\n",
                    i + 1, wp[0], wp[1], wp[2]));
        }

        new AlertDialog.Builder(getContext())
                .setTitle("Import Waypoints")
                .setMessage(preview.toString())
                .setPositiveButton("Append", (dialog, which) -> {
                    store.appendWaypoints(imported);
                    refreshWaypointListDisplay();
                    onLogMessage("Imported " + imported.size()
                            + " waypoint(s) from QR (appended).");
                })
                .setNeutralButton("Replace", (dialog, which) -> {
                    store.replaceWaypoints(imported);
                    refreshWaypointListDisplay();
                    onLogMessage("Replaced waypoint list with "
                            + imported.size() + " QR waypoint(s).");
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void onEditWaypoint() {
        if (store.isEmpty()) {
            showToast("No waypoints to edit.");
            return;
        }

        List<double[]> waypoints = store.getWaypoints();
        String[] labels = new String[waypoints.size()];
        for (int i = 0; i < waypoints.size(); i++) {
            double[] wp = waypoints.get(i);
            labels[i] = String.format("WP%d: %.6f, %.6f @ %.1fm",
                    i + 1, wp[0], wp[1], wp[2]);
        }

        new AlertDialog.Builder(getContext())
                .setTitle("Edit Waypoint")
                .setItems(labels, (dialog, which) -> showEditWaypointDialog(which))
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void showEditWaypointDialog(int index) {
        if (index < 0 || index >= store.size()) {
            showToast("Waypoint no longer exists.");
            return;
        }

        double[] wp = store.getWaypoint(index);

        LinearLayout editor = new LinearLayout(getContext());
        editor.setOrientation(VERTICAL);
        editor.setPadding(32, 16, 32, 0);

        EditText latInput = buildEditorField("Latitude",   String.format("%.7f", wp[0]), true);
        EditText lngInput = buildEditorField("Longitude",  String.format("%.7f", wp[1]), true);
        EditText altInput = buildEditorField("Altitude (m)", String.format("%.1f", wp[2]), false);

        editor.addView(latInput);
        editor.addView(lngInput);
        editor.addView(altInput);

        AlertDialog dialog = new AlertDialog.Builder(getContext())
                .setTitle("Waypoint " + (index + 1))
                .setView(editor)
                .setPositiveButton("Save", null)
                .setNegativeButton("Cancel", null)
                .create();

        dialog.setOnShowListener(d ->
                dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(v -> {
                    try {
                        double lat = Double.parseDouble(latInput.getText().toString().trim());
                        double lng = Double.parseDouble(lngInput.getText().toString().trim());
                        float  alt = Float.parseFloat(altInput.getText().toString().trim());
                        store.editWaypoint(index, lat, lng, alt);
                        refreshWaypointListDisplay();
                        onLogMessage(String.format(
                                "Updated waypoint %d: (%.6f, %.6f) @ %.1fm",
                                index + 1, lat, lng, alt));
                        dialog.dismiss();
                    } catch (NumberFormatException e) {
                        showToast("Invalid coordinates.");
                    } catch (IllegalArgumentException e) {
                        showToast(e.getMessage());
                    }
                }));

        dialog.show();
    }

    private void onClearWaypoints() {
        store.clearWaypoints();
        refreshWaypointListDisplay();
        onLogMessage("Waypoints cleared.");
    }

    // =========================================================================
    // UI helpers
    // =========================================================================

    private void refreshWaypointListDisplay() {
        if (store.isEmpty()) {
            tvWaypointList.setText("Waypoints: (none)");
            return;
        }
        StringBuilder sb = new StringBuilder("Waypoints:\n");
        List<double[]> wps = store.getWaypoints();
        for (int i = 0; i < wps.size(); i++) {
            double[] wp = wps.get(i);
            sb.append(String.format("  %d: (%.6f, %.6f) @ %.1fm\n",
                    i + 1, wp[0], wp[1], wp[2]));
        }
        tvWaypointList.setText(sb.toString());
    }

    private EditText buildEditorField(String hint, String value, boolean signed) {
        EditText et = new EditText(getContext());
        et.setHint(hint);
        et.setText(value);
        int inputType = android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL;
        if (signed) inputType |= android.text.InputType.TYPE_NUMBER_FLAG_SIGNED;
        et.setInputType(inputType);
        return et;
    }

    private void showToast(String message) {
        post(() -> Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show());
    }

    // =========================================================================
    // Lifecycle
    // =========================================================================

    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
        controller.onDetached();
        QrWaypointScanActivity.clearResultListener();
    }
}