package com.dji.sdk.sample.demo.timetrial;

import android.content.Context;
import android.content.SharedPreferences;
import android.util.Log;
import android.view.Gravity;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

/**
 * TimeTrialTuningPanel
 *
 * Independent tuning panel for the time trial system. Uses its own
 * SharedPreferences key "vsw_timetrial_tuning_prefs" so time trial
 * parameters never interfere with the waypoint mission parameters.
 *
 * Default values are tuned for speed rather than precision:
 *   - Higher default cruise speed (12 m/s vs 8 m/s)
 *   - More aggressive deceleration (3.0 m/s² vs 2.0 m/s²)
 *   - Wider braking distance still allows PD to engage, but drone
 *     doesn't need to stop so the approach can be faster
 *
 * Structure is identical to TuningPanel — same +/- row UI, same
 * SharedPreferences persistence, same TuningChangeListener callback.
 */
public class TimeTrialTuningPanel {

    private static final String TAG        = "TTTuningPanel";
    private static final String PREFS_NAME = "vsw_timetrial_tuning_prefs";

    // SharedPreferences keys
    private static final String KEY_KP      = "tt_kp_horizontal";
    private static final String KEY_KD      = "tt_kd_horizontal";
    private static final String KEY_KP_VERT = "tt_kp_vertical";
    private static final String KEY_SPEED   = "tt_max_horizontal_speed";
    private static final String KEY_DECEL   = "tt_deceleration";
    private static final String KEY_KP_YAW  = "tt_kp_yaw";
    private static final String KEY_YAW_MAX = "tt_max_yaw_rate";

    // ── Default values — tuned for speed ─────────────────────────────────────
    private static final float DEFAULT_KP      = 0.4f;
    private static final float DEFAULT_KD      = 0.3f;
    private static final float DEFAULT_KP_VERT = 0.6f;
    private static final float DEFAULT_SPEED   = 12.0f; // faster than waypoint default
    private static final float DEFAULT_DECEL   = 3.0f;  // more aggressive braking
    private static final float DEFAULT_KP_YAW  = 2.0f;
    private static final float DEFAULT_YAW_MAX = 45.0f;

    // ── Live parameter values ─────────────────────────────────────────────────
    private float kpHorizontal;
    private float kdHorizontal;
    private float kpVertical;
    private float maxHorizontalSpeed;
    private float deceleration;
    private float kpYaw;
    private float maxYawRate;

    // ── Callback ──────────────────────────────────────────────────────────────
    public interface TuningChangeListener {
        void onTuningChanged(float kp, float kd, float kpVert,
                             float maxSpeed, float decel, float brakingDistM);
    }

    private TuningChangeListener changeListener;

    // ── UI label references ───────────────────────────────────────────────────
    private TextView tvKp;
    private TextView tvKd;
    private TextView tvSpeed;
    private TextView tvBraking;
    private TextView tvKpYaw;
    private TextView tvMaxYaw;

    // =========================================================================
    // Constructor
    // =========================================================================

    public TimeTrialTuningPanel(Context context) {
        loadFromPrefs(context);
    }

    // =========================================================================
    // Listener
    // =========================================================================

    public void setChangeListener(TuningChangeListener listener) {
        this.changeListener = listener;
    }

    // =========================================================================
    // Getters
    // =========================================================================

    public float getKpHorizontal()       { return kpHorizontal; }
    public float getKdHorizontal()       { return kdHorizontal; }
    public float getKpVertical()         { return kpVertical; }
    public float getMaxHorizontalSpeed() { return maxHorizontalSpeed; }
    public float getDeceleration()       { return deceleration; }
    public float getKpYaw()              { return kpYaw; }
    public float getMaxYawRate()         { return maxYawRate; }

    public float getBrakingDistanceM() {
        return (maxHorizontalSpeed * maxHorizontalSpeed) / (2 * deceleration);
    }

    // =========================================================================
    // UI construction
    // =========================================================================

    public LinearLayout buildPanelView(Context context) {
        LinearLayout panel = new LinearLayout(context);
        panel.setOrientation(LinearLayout.VERTICAL);

        TextView header = new TextView(context);
        header.setText("── Time Trial Tuning ───────");
        header.setTextSize(12f);
        header.setPadding(0, 8, 0, 4);
        panel.addView(header);

        tvSpeed = new TextView(context);
        tvSpeed.setTextSize(12f);
        panel.addView(buildTuningRow(context, "Max Speed (m/s)", tvSpeed,
                () -> { maxHorizontalSpeed = Math.max(1.0f,  maxHorizontalSpeed - 0.5f); onChanged(context); },
                // Cap at 15 m/s — the DJI virtual-stick velocity limit. Above this
                // the SDK rejects the command with "Param illegal" and the drone stops.
                () -> { maxHorizontalSpeed = Math.min(15.0f, maxHorizontalSpeed + 0.5f); onChanged(context); }));

        tvKp = new TextView(context);
        tvKp.setTextSize(12f);
        panel.addView(buildTuningRow(context, "Kp (P gain)", tvKp,
                () -> { kpHorizontal = Math.max(0.05f, kpHorizontal - 0.05f); onChanged(context); },
                () -> { kpHorizontal = Math.min(2.0f,  kpHorizontal + 0.05f); onChanged(context); }));

        tvKd = new TextView(context);
        tvKd.setTextSize(12f);
        panel.addView(buildTuningRow(context, "Kd (D gain)", tvKd,
                () -> { kdHorizontal = Math.max(0.0f, kdHorizontal - 0.05f); onChanged(context); },
                () -> { kdHorizontal = Math.min(2.0f, kdHorizontal + 0.05f); onChanged(context); }));

        tvBraking = new TextView(context);
        tvBraking.setTextSize(12f);
        panel.addView(buildTuningRow(context, "Decel (m/s²)", tvBraking,
                () -> { deceleration = Math.max(0.5f, deceleration - 0.25f); onChanged(context); },
                () -> { deceleration = Math.min(5.0f, deceleration + 0.25f); onChanged(context); }));

        tvKpYaw = new TextView(context);
        tvKpYaw.setTextSize(12f);
        panel.addView(buildTuningRow(context, "Kp Yaw (°/s/°)", tvKpYaw,
                () -> { kpYaw = Math.max(0.1f, kpYaw - 0.1f); onChanged(context); },
                () -> { kpYaw = Math.min(5.0f, kpYaw + 0.1f); onChanged(context); }));

        tvMaxYaw = new TextView(context);
        tvMaxYaw.setTextSize(12f);
        panel.addView(buildTuningRow(context, "Max Yaw (°/s)", tvMaxYaw,
                () -> { maxYawRate = Math.max(5.0f,  maxYawRate - 5.0f);  onChanged(context); },
                () -> { maxYawRate = Math.min(90.0f, maxYawRate + 5.0f);  onChanged(context); }));

        refreshLabels();
        return panel;
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

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
        lbl.setLayoutParams(new LinearLayout.LayoutParams(
                0, LinearLayout.LayoutParams.WRAP_CONTENT, 1.4f));
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

    private void refreshLabels() {
        float bd = getBrakingDistanceM();
        if (tvKp      != null) tvKp.setText(String.format("%.2f", kpHorizontal));
        if (tvKd      != null) tvKd.setText(String.format("%.2f", kdHorizontal));
        if (tvSpeed   != null) tvSpeed.setText(String.format("%.1f", maxHorizontalSpeed));
        if (tvBraking != null) tvBraking.setText(
                String.format("%.1f (→%.1fm)", deceleration, bd));
        if (tvKpYaw   != null) tvKpYaw.setText(String.format("%.1f", kpYaw));
        if (tvMaxYaw  != null) tvMaxYaw.setText(String.format("%.0f°/s", maxYawRate));
    }

    private void onChanged(Context context) {
        saveToPrefs(context);
        refreshLabels();
        float bd = getBrakingDistanceM();
        Log.d(TAG, String.format(
                "TT Tuning: Kp=%.2f Kd=%.2f KpV=%.2f speed=%.1f decel=%.2f brakeDist=%.1fm",
                kpHorizontal, kdHorizontal, kpVertical, maxHorizontalSpeed, deceleration, bd));
        if (changeListener != null) {
            changeListener.onTuningChanged(
                    kpHorizontal, kdHorizontal, kpVertical,
                    maxHorizontalSpeed, deceleration, bd);
        }
    }

    // =========================================================================
    // SharedPreferences persistence
    // =========================================================================

    private void saveToPrefs(Context context) {
        context.getApplicationContext()
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit()
                .putFloat(KEY_KP,      kpHorizontal)
                .putFloat(KEY_KD,      kdHorizontal)
                .putFloat(KEY_KP_VERT, kpVertical)
                .putFloat(KEY_SPEED,   maxHorizontalSpeed)
                .putFloat(KEY_DECEL,   deceleration)
                .putFloat(KEY_KP_YAW,  kpYaw)
                .putFloat(KEY_YAW_MAX, maxYawRate)
                .apply();
    }

    private void loadFromPrefs(Context context) {
        SharedPreferences prefs = context.getApplicationContext()
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        kpHorizontal       = prefs.getFloat(KEY_KP,      DEFAULT_KP);
        kdHorizontal       = prefs.getFloat(KEY_KD,      DEFAULT_KD);
        kpVertical         = prefs.getFloat(KEY_KP_VERT, DEFAULT_KP_VERT);
        maxHorizontalSpeed = prefs.getFloat(KEY_SPEED,   DEFAULT_SPEED);
        deceleration       = prefs.getFloat(KEY_DECEL,   DEFAULT_DECEL);
        kpYaw              = prefs.getFloat(KEY_KP_YAW,  DEFAULT_KP_YAW);
        maxYawRate         = prefs.getFloat(KEY_YAW_MAX, DEFAULT_YAW_MAX);
    }
}