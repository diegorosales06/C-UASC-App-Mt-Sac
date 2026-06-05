package com.dji.sdk.sample.demo.searchrecord;

import android.content.Context;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.text.InputType;
import android.util.AttributeSet;
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
import com.dji.sdk.sample.internal.view.PresentableView;

import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;

/**
 * SearchRecordView
 *
 * Simple autonomous "search and record" screen:
 *   - Live downward camera preview at the top.
 *   - Enter the 4 corners of the search box (in perimeter order).
 *   - Set the search altitude (≥ 25 ft / 7.62 m), max speed, and track spacing.
 *   - "Start" takes off, points the camera straight down, starts recording video
 *     to the drone's SD card, flies a lawnmower sweep over the box while logging
 *     position to a CSV, then returns home.
 *
 * All flight/camera logic lives in {@link SearchRecordController}; this class is
 * UI only. Target identification/localization is intentionally NOT included yet.
 */
public class SearchRecordView extends LinearLayout
        implements PresentableView, SearchRecordCallback, TextureView.SurfaceTextureListener {

    private SearchRecordController controller;

    // ── Live video preview ──────────────────────────────────────────────────────
    private TextureView videoTextureView;
    private DJICodecManager codecManager;
    private VideoFeeder.VideoDataListener videoDataListener;

    // ── Status / telemetry ──────────────────────────────────────────────────────
    private TextView tvStatus;
    private TextView tvTarget;
    private TextView tvDronePos;
    private TextView tvDroneSpeed;

    // ── Inputs ──────────────────────────────────────────────────────────────────
    private final EditText[] cornerLat = new EditText[4];
    private final EditText[] cornerLng = new EditText[4];
    private EditText etAltitude;
    private EditText etMaxSpeed;
    private EditText etSpacing;

    private Button btnStart;
    private Button btnStop;
    private Button btnReturnHome;
    private Button btnMark;

    private ScrollView controlsScroll;
    private TextView   tvLog;

    public SearchRecordView(Context context) {
        super(context);
        init(context);
    }

    public SearchRecordView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    // ── PresentableView ─────────────────────────────────────────────────────────

    @Override
    public int getDescription() {
        return R.string.search_record_title;
    }

    @NonNull
    @Override
    public String getHint() {
        return this.getClass().getSimpleName() + ".java";
    }

    // ── Lifecycle ───────────────────────────────────────────────────────────────

    @Override
    protected void onAttachedToWindow() {
        super.onAttachedToWindow();
        registerVideoFeed();
    }

    @Override
    protected void onDetachedFromWindow() {
        controller.onDetached();
        unregisterVideoFeed();
        if (codecManager != null) {
            codecManager.cleanSurface();
            codecManager.destroyCodec();
            codecManager = null;
        }
        super.onDetachedFromWindow();
    }

    // ── UI construction ─────────────────────────────────────────────────────────

    private void init(Context context) {
        setOrientation(VERTICAL);
        setPadding(24, 24, 24, 24);

        controller = new SearchRecordController(context, this);

        // ── Live camera preview ───────────────────────────────────────────────
        FrameLayout videoFrame = new FrameLayout(context);
        videoFrame.setBackgroundColor(Color.BLACK);
        addView(videoFrame, new LayoutParams(LayoutParams.MATCH_PARENT, dp(500)));

        videoTextureView = new TextureView(context);
        videoTextureView.setSurfaceTextureListener(this);
        videoFrame.addView(videoTextureView, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT));

        videoDataListener = (bytes, size) -> {
            if (codecManager != null) {
                codecManager.sendDataToDecoder(bytes, size);
            }
        };

        // ── Scrollable controls below the preview ─────────────────────────────
        controlsScroll = new ScrollView(context);
        addView(controlsScroll, new LayoutParams(LayoutParams.MATCH_PARENT, 0, 1f));

        LinearLayout controls = new LinearLayout(context);
        controls.setOrientation(VERTICAL);
        controls.setPadding(0, dp(8), 0, dp(8));
        controlsScroll.addView(controls, new ScrollView.LayoutParams(
                ScrollView.LayoutParams.MATCH_PARENT, ScrollView.LayoutParams.WRAP_CONTENT));

        tvStatus = makeText(context, "Search: IDLE", 16f);
        controls.addView(tvStatus);
        tvTarget = makeText(context, "Target: —", 13f);
        controls.addView(tvTarget);
        tvDronePos = makeText(context, "Drone: unknown", 13f);
        controls.addView(tvDronePos);
        tvDroneSpeed = makeText(context, "Speed: — m/s", 13f);
        controls.addView(tvDroneSpeed);

        TextView cornersHeader = makeText(context, "── Search box corners (perimeter order) ──", 13f);
        cornersHeader.setPadding(0, 16, 0, 4);
        controls.addView(cornersHeader);
        for (int i = 0; i < 4; i++) {
            controls.addView(buildCornerRow(context, i));
        }

        controls.addView(buildParamsRow(context));
        controls.addView(buildButtonRow(context));

        // ── Manual position mark — logs timestamp + GPS coords on demand ───────
        btnMark = new Button(context);
        btnMark.setText("📍 Log Position (timestamp + GPS)");
        btnMark.setOnClickListener(v -> controller.logManualMark());
        controls.addView(btnMark, new LinearLayout.LayoutParams(
                LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));

        TextView csvNote = makeText(context,
                "Video → drone SD card.  Position CSV → app files /search-record/ "
                        + "(full path printed in the log on Start).", 11f);
        csvNote.setTextColor(Color.parseColor("#8888AA"));
        csvNote.setPadding(0, 4, 0, 8);
        controls.addView(csvNote);

        tvLog = new TextView(context);
        tvLog.setText("Log:\n");
        tvLog.setTextSize(11f);
        controls.addView(tvLog);

        controller.initFlightController();
    }

    private LinearLayout buildCornerRow(Context context, int index) {
        LinearLayout row = new LinearLayout(context);
        row.setOrientation(HORIZONTAL);
        row.setPadding(0, 2, 0, 2);

        TextView label = new TextView(context);
        label.setText("C" + (index + 1));
        label.setTextSize(13f);
        label.setLayoutParams(new LinearLayout.LayoutParams(0, LayoutParams.WRAP_CONTENT, 0.4f));
        row.addView(label);

        cornerLat[index] = signedDecimalField(context, "Latitude");
        cornerLat[index].setLayoutParams(weighted(1f, 6));
        row.addView(cornerLat[index]);

        cornerLng[index] = signedDecimalField(context, "Longitude");
        cornerLng[index].setLayoutParams(weighted(1f, 0));
        row.addView(cornerLng[index]);

        return row;
    }

    private LinearLayout buildParamsRow(Context context) {
        LinearLayout row = new LinearLayout(context);
        row.setOrientation(HORIZONTAL);
        row.setPadding(0, 10, 0, 6);

        etAltitude = decimalField(context, "Alt (m)");
        etAltitude.setText("15");
        etAltitude.setLayoutParams(weighted(1f, 6));
        row.addView(etAltitude);

        etMaxSpeed = decimalField(context, "Speed (m/s)");
        etMaxSpeed.setText("6");
        etMaxSpeed.setLayoutParams(weighted(1f, 6));
        row.addView(etMaxSpeed);

        etSpacing = decimalField(context, "Spacing (m)");
        etSpacing.setText("8");
        etSpacing.setLayoutParams(weighted(1f, 0));
        row.addView(etSpacing);

        return row;
    }

    private LinearLayout buildButtonRow(Context context) {
        LinearLayout row = new LinearLayout(context);
        row.setOrientation(HORIZONTAL);
        row.setPadding(0, 6, 0, 6);

        btnStart = new Button(context);
        btnStart.setText("Start Search");
        btnStart.setLayoutParams(weighted(1f, 8));
        btnStart.setOnClickListener(v -> onStartClicked());
        row.addView(btnStart);

        btnStop = new Button(context);
        btnStop.setText("Stop");
        btnStop.setEnabled(false);
        btnStop.setLayoutParams(weighted(0.8f, 8));
        btnStop.setOnClickListener(v -> controller.stopMission());
        row.addView(btnStop);

        btnReturnHome = new Button(context);
        btnReturnHome.setText("RTH");
        btnReturnHome.setLayoutParams(weighted(0.7f, 0));
        btnReturnHome.setOnClickListener(v -> controller.requestReturnToHome());
        row.addView(btnReturnHome);

        return row;
    }

    private void onStartClicked() {
        double[][] corners = new double[4][2];
        try {
            for (int i = 0; i < 4; i++) {
                corners[i][0] = parseRequired(cornerLat[i], "Corner " + (i + 1) + " latitude");
                corners[i][1] = parseRequired(cornerLng[i], "Corner " + (i + 1) + " longitude");
            }
            float altitude = (float) parseRequired(etAltitude, "Altitude");
            float maxSpeed = (float) parseRequired(etMaxSpeed, "Max speed");
            double spacing = parseRequired(etSpacing, "Track spacing");
            controller.startMission(corners, altitude, maxSpeed, spacing);
        } catch (NumberFormatException e) {
            showToast(e.getMessage());
        }
    }

    private double parseRequired(EditText field, String label) {
        String s = field.getText().toString().trim();
        if (s.isEmpty()) {
            throw new NumberFormatException(label + " is required.");
        }
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException e) {
            throw new NumberFormatException(label + " is not a valid number.");
        }
    }

    // ── Video feed ──────────────────────────────────────────────────────────────

    private void registerVideoFeed() {
        try {
            VideoFeeder.VideoFeed primaryFeed = VideoFeeder.getInstance().getPrimaryVideoFeed();
            if (primaryFeed != null
                    && videoDataListener != null
                    && !primaryFeed.getListeners().contains(videoDataListener)) {
                primaryFeed.addVideoDataListener(videoDataListener);
                onLogMessage("Camera preview feed connected.");
            }
        } catch (Exception e) {
            onLogMessage("Unable to connect camera feed: " + e.getMessage());
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

    // ── SearchRecordCallback (always called on main thread) ─────────────────────

    @Override
    public void onStatusChanged(String status) {
        tvStatus.setText(status);
    }

    @Override
    public void onLogMessage(String message) {
        post(() -> {
            tvLog.append(message + "\n");
            controlsScroll.post(() -> controlsScroll.fullScroll(View.FOCUS_DOWN));
        });
    }

    @Override
    public void onTargetLabelChanged(String label) {
        tvTarget.setText(label);
    }

    @Override
    public void onTelemetryUpdated(String positionLabel, String speedLabel) {
        tvDronePos.setText(positionLabel);
        tvDroneSpeed.setText(speedLabel);
    }

    @Override
    public void onMissionActiveChanged(boolean missionActive) {
        btnStart.setEnabled(!missionActive);
        btnStop.setEnabled(missionActive);
        for (int i = 0; i < 4; i++) {
            cornerLat[i].setEnabled(!missionActive);
            cornerLng[i].setEnabled(!missionActive);
        }
        etAltitude.setEnabled(!missionActive);
        etMaxSpeed.setEnabled(!missionActive);
        etSpacing.setEnabled(!missionActive);
        if (!missionActive) {
            tvTarget.setText("Target: —");
        }
    }

    // ── Helpers ─────────────────────────────────────────────────────────────────

    private TextView makeText(Context context, String text, float size) {
        TextView tv = new TextView(context);
        tv.setText(text);
        tv.setTextSize(size);
        tv.setPadding(0, 0, 0, 2);
        return tv;
    }

    private EditText signedDecimalField(Context context, String hint) {
        EditText et = new EditText(context);
        et.setHint(hint);
        et.setTextSize(12f);
        et.setInputType(InputType.TYPE_CLASS_NUMBER
                | InputType.TYPE_NUMBER_FLAG_DECIMAL
                | InputType.TYPE_NUMBER_FLAG_SIGNED);
        return et;
    }

    private EditText decimalField(Context context, String hint) {
        EditText et = new EditText(context);
        et.setHint(hint);
        et.setTextSize(12f);
        et.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL);
        return et;
    }

    private LinearLayout.LayoutParams weighted(float weight, int marginEnd) {
        LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(
                0, LayoutParams.WRAP_CONTENT, weight);
        p.setMarginEnd(marginEnd);
        return p;
    }

    private int dp(int value) {
        return (int) (value * getResources().getDisplayMetrics().density);
    }

    private void showToast(String message) {
        post(() -> Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show());
    }
}
