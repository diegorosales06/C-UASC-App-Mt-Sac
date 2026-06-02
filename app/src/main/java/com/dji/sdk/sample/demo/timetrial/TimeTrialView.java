package com.dji.sdk.sample.demo.timetrial;

import android.app.AlertDialog;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Typeface;
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

import com.dji.sdk.sample.demo.virtualstickwaypoint.QrWaypointScanActivity;
import com.dji.sdk.sample.internal.view.PresentableView;

import java.util.List;

/**
 * TimeTrialView
 *
 * UI entry point for the Time Trial Mission — registered in DemoListView
 * under the Flight Controller group, directly below VirtualStickWaypointView.
 *
 * The time trial flies through waypoints as fast as possible with no dwell
 * stop at each gate. Split times and total elapsed time are displayed live.
 *
 * File map for this system:
 *   TimeTrialMissionController — control loop, gate detection, timing, RTH
 *   TimeTrialWaypointStore     — waypoint list, persistence, QR parsing
 *   TimeTrialTuningPanel       — PD parameter UI, SharedPrefs persistence
 *   TimeTrialCallback          — interface connecting controller → view
 *
 * Shared with waypoint system (no duplication):
 *   QrWaypointScanActivity     — same QR scanner Activity
 *   WaypointNavigationMath     — same distance/bearing/yaw math
 *   FlightLogger               — same CSV logger
 */
public class TimeTrialView extends LinearLayout
        implements PresentableView, TimeTrialCallback {

    private static final String TAG = "TimeTrialView";

    // ── Collaborators ─────────────────────────────────────────────────────────
    private TimeTrialWaypointStore     store;
    private TimeTrialTuningPanel       tuning;
    private TimeTrialMissionController controller;

    // ── Timing display widgets ────────────────────────────────────────────────
    private TextView tvElapsed;
    private TextView tvLastSplit;

    // ── Status / telemetry widgets ────────────────────────────────────────────
    private TextView tvStatus;
    private TextView tvCurrentGate;
    private TextView tvDronePos;
    private TextView tvDroneSpeed;

    // ── Waypoint management widgets ───────────────────────────────────────────
    private TextView   tvWaypointList;
    private EditText   etLat;
    private EditText   etLng;
    private EditText   etAlt;
    private Button     btnAddWaypoint;
    private Button     btnImportQr;
    private Button     btnEditWaypoint;
    private Button     btnClearWaypoints;

    // ── Mission control widgets ───────────────────────────────────────────────
    private Button     btnStart;
    private Button     btnStop;

    // ── Log ───────────────────────────────────────────────────────────────────
    private ScrollView scrollLog;
    private TextView   tvLog;

    // =========================================================================
    // Constructors
    // =========================================================================

    public TimeTrialView(Context context) {
        super(context);
        init(context);
    }

    public TimeTrialView(Context context, AttributeSet attrs) {
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
    // TimeTrialCallback implementation
    // All methods are already called on the main thread by the controller.
    // =========================================================================

    @Override
    public void onStatusChanged(String status) {
        tvStatus.setText(status);
    }

    @Override
    public void onTargetLabelChanged(String label) {
        tvCurrentGate.setText(label);
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
            tvCurrentGate.setText("Target: —");
        }
    }

    @Override
    public void onTelemetryUpdated(String positionLabel, String speedLabel) {
        tvDronePos.setText(positionLabel);
        tvDroneSpeed.setText(speedLabel);
    }

    @Override
    public void onTimingUpdated(String elapsed, String lastSplit) {
        tvElapsed.setText(elapsed);
        tvLastSplit.setText(lastSplit);
    }

    @Override
    public void onSplitRecorded(int wpNumber, String splitTime, String totalTime) {
        tvLastSplit.setText("Gate " + wpNumber + ": " + splitTime);
    }

    // =========================================================================
    // UI Construction
    // =========================================================================

    private void init(Context context) {
        setOrientation(VERTICAL);
        setPadding(24, 24, 24, 24);

        store      = new TimeTrialWaypointStore(context);
        tuning     = new TimeTrialTuningPanel(context);
        controller = new TimeTrialMissionController(context, store, tuning, this);

        // ── Timing display — prominent at the top ─────────────────────────────
        LinearLayout timingRow = new LinearLayout(context);
        timingRow.setOrientation(HORIZONTAL);
        timingRow.setBackgroundColor(Color.parseColor("#1A1A2E"));
        timingRow.setPadding(16, 12, 16, 12);

        LinearLayout elapsedBlock = new LinearLayout(context);
        elapsedBlock.setOrientation(VERTICAL);
        elapsedBlock.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));

        TextView elapsedLabel = new TextView(context);
        elapsedLabel.setText("ELAPSED");
        elapsedLabel.setTextSize(10f);
        elapsedLabel.setTextColor(Color.parseColor("#8888AA"));
        elapsedLabel.setLetterSpacing(0.15f);
        elapsedBlock.addView(elapsedLabel);

        tvElapsed = new TextView(context);
        tvElapsed.setText("--:--.--");
        tvElapsed.setTextSize(28f);
        tvElapsed.setTypeface(Typeface.MONOSPACE, Typeface.BOLD);
        tvElapsed.setTextColor(Color.parseColor("#00E5FF"));
        elapsedBlock.addView(tvElapsed);

        timingRow.addView(elapsedBlock);

        LinearLayout splitBlock = new LinearLayout(context);
        splitBlock.setOrientation(VERTICAL);
        splitBlock.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));

        TextView splitLabel = new TextView(context);
        splitLabel.setText("LAST SPLIT");
        splitLabel.setTextSize(10f);
        splitLabel.setTextColor(Color.parseColor("#8888AA"));
        splitLabel.setLetterSpacing(0.15f);
        splitBlock.addView(splitLabel);

        tvLastSplit = new TextView(context);
        tvLastSplit.setText("—");
        tvLastSplit.setTextSize(16f);
        tvLastSplit.setTypeface(Typeface.MONOSPACE);
        tvLastSplit.setTextColor(Color.parseColor("#69FF47"));
        splitBlock.addView(tvLastSplit);

        timingRow.addView(splitBlock);
        addView(timingRow);

        // ── Status / telemetry ────────────────────────────────────────────────
        tvStatus = new TextView(context);
        tvStatus.setText("Time Trial: IDLE");
        tvStatus.setTextSize(15f);
        tvStatus.setPadding(0, 10, 0, 4);
        addView(tvStatus);

        tvCurrentGate = new TextView(context);
        tvCurrentGate.setText("Target: —");
        tvCurrentGate.setTextSize(13f);
        tvCurrentGate.setPadding(0, 0, 0, 4);
        addView(tvCurrentGate);

        tvDronePos = new TextView(context);
        tvDronePos.setText("Drone: unknown");
        tvDronePos.setTextSize(13f);
        tvDronePos.setPadding(0, 0, 0, 2);
        addView(tvDronePos);

        tvDroneSpeed = new TextView(context);
        tvDroneSpeed.setText("Speed: — m/s");
        tvDroneSpeed.setTextSize(13f);
        tvDroneSpeed.setPadding(0, 0, 0, 12);
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

        // ── Add / Import QR buttons ───────────────────────────────────────────
        LinearLayout btnRow1 = new LinearLayout(context);
        btnRow1.setOrientation(HORIZONTAL);
        btnRow1.setPadding(0, 8, 0, 8);

        btnAddWaypoint = new Button(context);
        btnAddWaypoint.setText("Add Gate");
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
        btnImportQr.setOnClickListener(v -> onImportGatesQr());
        btnRow1.addView(btnImportQr);

        addView(btnRow1);

        // ── Edit / Clear buttons ──────────────────────────────────────────────
        LinearLayout btnRowEdit = new LinearLayout(context);
        btnRowEdit.setOrientation(HORIZONTAL);
        btnRowEdit.setPadding(0, 0, 0, 8);

        btnEditWaypoint = new Button(context);
        btnEditWaypoint.setText("Edit Gate");
        LinearLayout.LayoutParams bP3 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP3.setMarginEnd(8);
        btnEditWaypoint.setLayoutParams(bP3);
        btnEditWaypoint.setOnClickListener(v -> onEditGate());
        btnRowEdit.addView(btnEditWaypoint);

        btnClearWaypoints = new Button(context);
        btnClearWaypoints.setText("Clear All");
        btnClearWaypoints.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        btnClearWaypoints.setOnClickListener(v -> onClearGates());
        btnRowEdit.addView(btnClearWaypoints);

        addView(btnRowEdit);

        // ── Gate list display ─────────────────────────────────────────────────
        tvWaypointList = new TextView(context);
        tvWaypointList.setTextSize(12f);
        tvWaypointList.setPadding(0, 0, 0, 12);
        tvWaypointList.setOnClickListener(v -> onEditGate());
        addView(tvWaypointList);
        refreshGateListDisplay();

        // ── Start / Stop buttons ──────────────────────────────────────────────
        LinearLayout btnRow2 = new LinearLayout(context);
        btnRow2.setOrientation(HORIZONTAL);
        btnRow2.setPadding(0, 0, 0, 16);

        btnStart = new Button(context);
        btnStart.setText("Start Time Trial");
        LinearLayout.LayoutParams bP5 = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP5.setMarginEnd(8);
        btnStart.setLayoutParams(bP5);
        btnStart.setOnClickListener(v -> controller.startMission());
        btnRow2.addView(btnStart);

        btnStop = new Button(context);
        btnStop.setText("Stop");
        btnStop.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        btnStop.setEnabled(false);
        btnStop.setOnClickListener(v -> controller.stopMission());
        btnRow2.addView(btnStop);

        addView(btnRow2);

        // ── Tuning panel ──────────────────────────────────────────────────────
        tuning.setChangeListener((kp, kd, kpV, speed, decel, brakeDist) ->
                onLogMessage(String.format(
                        "TT Tuning: Kp=%.2f Kd=%.2f speed=%.1f decel=%.2f brakeD=%.1fm",
                        kp, kd, speed, decel, brakeDist)));
        addView(tuning.buildPanelView(context));

        // ── Scrollable log ────────────────────────────────────────────────────
        tvLog = new TextView(context);
        tvLog.setText("Log:\n");
        tvLog.setTextSize(11f);

        scrollLog = new ScrollView(context);
        scrollLog.setLayoutParams(new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 400));
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
            refreshGateListDisplay();
            onLogMessage(String.format("Added gate %d: (%.6f, %.6f) @ %.1fm",
                    store.size(), lat, lng, alt));
        } catch (NumberFormatException e) {
            showToast("Invalid coordinates.");
        } catch (IllegalArgumentException e) {
            showToast(e.getMessage());
        }
    }

    private void onImportGatesQr() {
        Context ctx = getContext();
        if (!(ctx instanceof Activity)) {
            showToast("QR scanner needs an activity context.");
            return;
        }
        QrWaypointScanActivity.setResultListener(
                payload -> post(() -> handleGateQrPayload(payload)));
        ctx.startActivity(new Intent(ctx, QrWaypointScanActivity.class));
        onLogMessage("Gate QR scanner opened.");
    }

    private void handleGateQrPayload(String payload) {
        final List<double[]> imported;
        try {
            imported = store.parseWaypointQrPayload(payload);
        } catch (IllegalArgumentException e) {
            showToast(e.getMessage());
            onLogMessage("QR import failed: " + e.getMessage());
            return;
        }

        StringBuilder preview = new StringBuilder();
        preview.append("Found ").append(imported.size()).append(" gate");
        if (imported.size() != 1) preview.append("s");
        preview.append(":\n\n");
        for (int i = 0; i < imported.size(); i++) {
            double[] wp = imported.get(i);
            preview.append(String.format("%d: %.6f, %.6f @ %.1fm\n",
                    i + 1, wp[0], wp[1], wp[2]));
        }

        new AlertDialog.Builder(getContext())
                .setTitle("Import Gates")
                .setMessage(preview.toString())
                .setPositiveButton("Append", (dialog, which) -> {
                    store.appendWaypoints(imported);
                    refreshGateListDisplay();
                    onLogMessage("Imported " + imported.size() + " gate(s) from QR (appended).");
                })
                .setNeutralButton("Replace", (dialog, which) -> {
                    store.replaceWaypoints(imported);
                    refreshGateListDisplay();
                    onLogMessage("Replaced gate list with " + imported.size() + " QR gate(s).");
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void onEditGate() {
        if (store.isEmpty()) {
            showToast("No gates to edit.");
            return;
        }

        List<double[]> waypoints = store.getWaypoints();
        String[] labels = new String[waypoints.size()];
        for (int i = 0; i < waypoints.size(); i++) {
            double[] wp = waypoints.get(i);
            labels[i] = String.format("Gate %d: %.6f, %.6f @ %.1fm",
                    i + 1, wp[0], wp[1], wp[2]);
        }

        new AlertDialog.Builder(getContext())
                .setTitle("Edit Gate")
                .setItems(labels, (dialog, which) -> showEditGateDialog(which))
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void showEditGateDialog(int index) {
        if (index < 0 || index >= store.size()) {
            showToast("Gate no longer exists.");
            return;
        }

        double[] wp = store.getWaypoint(index);

        LinearLayout editor = new LinearLayout(getContext());
        editor.setOrientation(VERTICAL);
        editor.setPadding(32, 16, 32, 0);

        EditText latInput = buildEditorField("Latitude",    String.format("%.7f", wp[0]), true);
        EditText lngInput = buildEditorField("Longitude",   String.format("%.7f", wp[1]), true);
        EditText altInput = buildEditorField("Altitude (m)", String.format("%.1f",  wp[2]), false);

        editor.addView(latInput);
        editor.addView(lngInput);
        editor.addView(altInput);

        AlertDialog dialog = new AlertDialog.Builder(getContext())
                .setTitle("Gate " + (index + 1))
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
                        refreshGateListDisplay();
                        onLogMessage(String.format("Updated gate %d: (%.6f, %.6f) @ %.1fm",
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

    private void onClearGates() {
        store.clearWaypoints();
        refreshGateListDisplay();
        onLogMessage("Gates cleared.");
    }

    // =========================================================================
    // UI helpers
    // =========================================================================

    private void refreshGateListDisplay() {
        if (store.isEmpty()) {
            tvWaypointList.setText("Gates: (none)");
            return;
        }
        StringBuilder sb = new StringBuilder("Gates:\n");
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