package com.dji.sdk.sample.demo.virtualstickwaypoint;

import android.content.Context;
import android.content.SharedPreferences;
import android.util.Log;
import android.view.Gravity;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

/**
 * TuningPanel
 *
 * Owns all five tunable PD controller parameters and builds the in-app
 * tuning UI (the row of +/− buttons). Parameter values survive app restarts
 * via SharedPreferences.
 *
 * How to use:
 *   1. Construct one TuningPanel in VirtualStickWaypointView.init().
 *   2. Call tuningPanel.buildPanelView(context) to get the LinearLayout to add to the view.
 *   3. Pass the TuningPanel reference to WaypointMissionController — the controller
 *      calls getters on every control loop tick to read the current values.
 *   4. Changes made via the +/− buttons are automatically persisted and the
 *      TuningChangeListener callback is fired so the controller can log the update.
 *
 * Parameters and their effect on the PD controller:
 *
 *   Kp_HORIZONTAL  — Proportional gain during the braking phase.
 *                    Higher = reacts more aggressively to remaining distance.
 *                    Increase if the drone stops too far short of the waypoint.
 *                    Decrease if the drone oscillates around the waypoint.
 *
 *   Kd_HORIZONTAL  — Derivative (braking) gain during the braking phase.
 *                    Higher = harder braking at high closure speed = less overshoot.
 *                    Increase if the drone overshoots the waypoint.
 *                    Decrease if the drone brakes too early.
 *
 *   Kp_VERTICAL    — Proportional gain for altitude control.
 *                    Higher = climbs/descends faster toward target altitude.
 *
 *   MAX_HORIZONTAL_SPEED — Cruise speed in m/s during the feedforward phase
 *                          (when the drone is farther than brakingDistance from the waypoint).
 *                          Mavic Air 2 max ~19 m/s; keep below 12 until field-verified.
 *
 *   DECELERATION   — Simulated deceleration rate in m/s² used to compute braking distance.
 *                    brakingDistance = maxSpeed² / (2 * deceleration)
 *                    At 8 m/s and 2.0 m/s² → 16 m braking distance.
 *                    Increase to start braking earlier (safer approach).
 *                    Decrease for a more aggressive, later brake.
 */
public class TuningPanel {

    private static final String TAG        = "TuningPanel";
    private static final String PREFS_NAME = "vsw_tuning_prefs";

    // SharedPreferences keys
    private static final String KEY_KP       = "kp_horizontal";
    private static final String KEY_KD       = "kd_horizontal";
    private static final String KEY_KP_VERT  = "kp_vertical";
    private static final String KEY_SPEED    = "max_horizontal_speed";
    private static final String KEY_DECEL    = "deceleration";

    // ---------------------------------------------------------------------------------
    // Default values — used on first launch or if prefs are cleared
    // ---------------------------------------------------------------------------------

    private static final float DEFAULT_KP       = 0.4f;
    private static final float DEFAULT_KD       = 0.3f;
    private static final float DEFAULT_KP_VERT  = 0.6f;
    private static final float DEFAULT_SPEED    = 8.0f;
    private static final float DEFAULT_DECEL    = 2.0f;

    // ---------------------------------------------------------------------------------
    // Live parameter values — read by WaypointMissionController each tick
    // ---------------------------------------------------------------------------------

    private float kpHorizontal;
    private float kdHorizontal;
    private float kpVertical;
    private float maxHorizontalSpeed;
    private float deceleration;

    // ---------------------------------------------------------------------------------
    // Callback
    // ---------------------------------------------------------------------------------

    /**
     * Fired on the main thread whenever any parameter is changed via the UI.
     * WaypointMissionController can implement this to log the new values.
     */
    public interface TuningChangeListener {
        void onTuningChanged(float kp, float kd, float kpVert,
                             float maxSpeed, float decel, float brakingDistM);
    }

    private TuningChangeListener changeListener;

    // ---------------------------------------------------------------------------------
    // UI label references (kept so refreshLabels() can update them after load)
    // ---------------------------------------------------------------------------------

    private TextView tvKp;
    private TextView tvKd;
    private TextView tvSpeed;
    private TextView tvBraking;

    // ---------------------------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------------------------

    /**
     * Creates a TuningPanel and immediately loads previously persisted values.
     * Falls back to defaults if no values have been saved.
     *
     * @param context Application or Activity context.
     */
    public TuningPanel(Context context) {
        loadFromPrefs(context);
    }

    // ---------------------------------------------------------------------------------
    // Listener wiring
    // ---------------------------------------------------------------------------------

    public void setChangeListener(TuningChangeListener listener) {
        this.changeListener = listener;
    }

    // ---------------------------------------------------------------------------------
    // Getters — called by WaypointMissionController each control loop tick
    // ---------------------------------------------------------------------------------

    public float getKpHorizontal()    { return kpHorizontal; }
    public float getKdHorizontal()    { return kdHorizontal; }
    public float getKpVertical()      { return kpVertical; }
    public float getMaxHorizontalSpeed() { return maxHorizontalSpeed; }
    public float getDeceleration()    { return deceleration; }

    /** Convenience: computes braking distance from current speed and deceleration. */
    public float getBrakingDistanceM() {
        return (maxHorizontalSpeed * maxHorizontalSpeed) / (2 * deceleration);
    }

    // ---------------------------------------------------------------------------------
    // UI construction
    // ---------------------------------------------------------------------------------

    /**
     * Builds and returns the full tuning panel LinearLayout to be added to the
     * parent view. Includes the section header and all four parameter rows.
     *
     * @param context Activity context for constructing widgets.
     * @return Fully wired LinearLayout ready to addView() into the parent.
     */
    public LinearLayout buildPanelView(Context context) {
        LinearLayout panel = new LinearLayout(context);
        panel.setOrientation(LinearLayout.VERTICAL);

        TextView header = new TextView(context);
        header.setText("── Tuning ──────────────────");
        header.setTextSize(12f);
        header.setPadding(0, 8, 0, 4);
        panel.addView(header);

        // Kp row
        tvKp = new TextView(context);
        tvKp.setTextSize(12f);
        panel.addView(buildTuningRow(context, "Kp (P gain)", tvKp,
                () -> { kpHorizontal = Math.max(0.05f, kpHorizontal - 0.05f); onChanged(context); },
                () -> { kpHorizontal = Math.min(2.0f,  kpHorizontal + 0.05f); onChanged(context); }));

        // Kd row
        tvKd = new TextView(context);
        tvKd.setTextSize(12f);
        panel.addView(buildTuningRow(context, "Kd (D gain)", tvKd,
                () -> { kdHorizontal = Math.max(0.0f, kdHorizontal - 0.05f); onChanged(context); },
                () -> { kdHorizontal = Math.min(2.0f, kdHorizontal + 0.05f); onChanged(context); }));

        // Max speed row
        tvSpeed = new TextView(context);
        tvSpeed.setTextSize(12f);
        panel.addView(buildTuningRow(context, "Max Speed (m/s)", tvSpeed,
                () -> { maxHorizontalSpeed = Math.max(1.0f, maxHorizontalSpeed - 0.5f); onChanged(context); },
                () -> { maxHorizontalSpeed = maxHorizontalSpeed + 0.5f;                 onChanged(context); }));

        // Deceleration row
        tvBraking = new TextView(context);
        tvBraking.setTextSize(12f);
        panel.addView(buildTuningRow(context, "Decel (m/s²)", tvBraking,
                () -> { deceleration = Math.max(0.5f, deceleration - 0.25f); onChanged(context); },
                () -> { deceleration = Math.min(5.0f, deceleration + 0.25f); onChanged(context); }));

        refreshLabels();
        return panel;
    }

    // ---------------------------------------------------------------------------------
    // Private helpers
    // ---------------------------------------------------------------------------------

    /**
     * Builds a single tuning row: [label]  [−]  [value]  [+]
     */
    private LinearLayout buildTuningRow(Context context, String label,
                                        TextView valueDisplay,
                                        Runnable onDecrement,
                                        Runnable onIncrement) {
        LinearLayout row = new LinearLayout(context);
        row.setOrientation(LinearLayout.HORIZONTAL);
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
        btnMinus.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.5f));
        btnMinus.setOnClickListener(v -> onDecrement.run());
        row.addView(btnMinus);

        LinearLayout.LayoutParams valP = new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.8f);
        valueDisplay.setLayoutParams(valP);
        valueDisplay.setGravity(Gravity.CENTER);
        row.addView(valueDisplay);

        Button btnPlus = new Button(context);
        btnPlus.setText("+");
        btnPlus.setTextSize(14f);
        btnPlus.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.5f));
        btnPlus.setOnClickListener(v -> onIncrement.run());
        row.addView(btnPlus);

        return row;
    }

    /** Refreshes all value labels to reflect current parameter values. */
    private void refreshLabels() {
        float brakingDist = getBrakingDistanceM();
        if (tvKp      != null) tvKp.setText(String.format("%.2f", kpHorizontal));
        if (tvKd      != null) tvKd.setText(String.format("%.2f", kdHorizontal));
        if (tvSpeed   != null) tvSpeed.setText(String.format("%.1f", maxHorizontalSpeed));
        if (tvBraking != null) tvBraking.setText(
                String.format("%.1f (→%.1fm)", deceleration, brakingDist));
    }

    /** Called after any parameter change: saves to prefs, refreshes labels, fires callback. */
    private void onChanged(Context context) {
        saveToPrefs(context);
        refreshLabels();
        float bd = getBrakingDistanceM();
        Log.d(TAG, String.format(
                "Tuning updated: Kp=%.2f Kd=%.2f KpV=%.2f speed=%.1f decel=%.2f brakeDist=%.1fm",
                kpHorizontal, kdHorizontal, kpVertical, maxHorizontalSpeed, deceleration, bd));
        if (changeListener != null) {
            changeListener.onTuningChanged(kpHorizontal, kdHorizontal, kpVertical,
                    maxHorizontalSpeed, deceleration, bd);
        }
    }

    // ---------------------------------------------------------------------------------
    // SharedPreferences persistence
    // ---------------------------------------------------------------------------------

    private void saveToPrefs(Context context) {
        SharedPreferences prefs = context.getApplicationContext()
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        prefs.edit()
                .putFloat(KEY_KP,      kpHorizontal)
                .putFloat(KEY_KD,      kdHorizontal)
                .putFloat(KEY_KP_VERT, kpVertical)
                .putFloat(KEY_SPEED,   maxHorizontalSpeed)
                .putFloat(KEY_DECEL,   deceleration)
                .apply();
    }

    private void loadFromPrefs(Context context) {
        SharedPreferences prefs = context.getApplicationContext()
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        kpHorizontal      = prefs.getFloat(KEY_KP,      DEFAULT_KP);
        kdHorizontal      = prefs.getFloat(KEY_KD,      DEFAULT_KD);
        kpVertical        = prefs.getFloat(KEY_KP_VERT, DEFAULT_KP_VERT);
        maxHorizontalSpeed = prefs.getFloat(KEY_SPEED,  DEFAULT_SPEED);
        deceleration      = prefs.getFloat(KEY_DECEL,   DEFAULT_DECEL);
    }
}