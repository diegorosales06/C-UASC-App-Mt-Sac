package com.dji.sdk.sample.demo.camera;

import android.content.Context;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.os.Handler;
import android.os.Looper;
import android.view.Gravity;
import android.view.TextureView;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.annotation.NonNull;

import com.dji.sdk.sample.R;
import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.utils.ModuleVerificationUtil;
import com.dji.sdk.sample.internal.utils.ToastUtils;
import com.dji.sdk.sample.internal.view.PresentableView;

import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import dji.common.camera.SettingsDefinitions;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;

/**
 * Record-video screen with a tall, full-width live camera preview on top and the
 * Start / Stop Record controls below.
 *
 * Recording goes to the drone's SD card (same SDK calls as before). The preview
 * decodes the primary video feed into a TextureView via DJICodecManager — the
 * same approach used by the Search & Record and Target Localization screens.
 */
public class RecordVideoView extends LinearLayout
        implements PresentableView, TextureView.SurfaceTextureListener {

    // Height of the live preview. Increase this to make the video taller.
    private static final int PREVIEW_HEIGHT_DP = 480;

    private TextureView videoTextureView;
    private DJICodecManager codecManager;
    private VideoFeeder.VideoDataListener videoDataListener;

    private TextView tvTime;
    private Button btnStart;
    private Button btnStop;

    private Timer timer;
    private long timeCounterMs = 0;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    public RecordVideoView(Context context) {
        super(context);
        init(context);
    }

    // ── UI ──────────────────────────────────────────────────────────────────────

    private void init(Context context) {
        setOrientation(VERTICAL);

        // Tall, full-width live preview.
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

        tvTime = new TextView(context);
        tvTime.setText("00:00:00");
        tvTime.setTextSize(18f);
        tvTime.setGravity(Gravity.CENTER);
        tvTime.setPadding(0, dp(8), 0, dp(8));
        addView(tvTime);

        LinearLayout btnRow = new LinearLayout(context);
        btnRow.setOrientation(HORIZONTAL);
        btnRow.setPadding(dp(8), 0, dp(8), dp(8));

        btnStart = new Button(context);
        btnStart.setText(R.string.record_video_start_record);
        LinearLayout.LayoutParams lp1 = new LinearLayout.LayoutParams(
                0, LayoutParams.WRAP_CONTENT, 1f);
        lp1.setMarginEnd(dp(8));
        btnStart.setLayoutParams(lp1);
        btnStart.setOnClickListener(v -> startRecording());
        btnRow.addView(btnStart);

        btnStop = new Button(context);
        btnStop.setText(R.string.record_video_stop_record);
        btnStop.setLayoutParams(new LinearLayout.LayoutParams(
                0, LayoutParams.WRAP_CONTENT, 1f));
        btnStop.setOnClickListener(v -> stopRecording());
        btnRow.addView(btnStop);

        addView(btnRow);
    }

    // ── Lifecycle ───────────────────────────────────────────────────────────────

    @Override
    protected void onAttachedToWindow() {
        super.onAttachedToWindow();
        setCameraToRecordMode();
        registerVideoFeed();
    }

    @Override
    protected void onDetachedFromWindow() {
        unregisterVideoFeed();
        if (codecManager != null) {
            codecManager.cleanSurface();
            codecManager.destroyCodec();
            codecManager = null;
        }
        stopTimer();
        super.onDetachedFromWindow();
    }

    // ── Camera / recording ──────────────────────────────────────────────────────

    private Camera getCamera() {
        if (!ModuleVerificationUtil.isCameraModuleAvailable()) {
            return null;
        }
        return DJISampleApplication.getProductInstance().getCamera();
    }

    private void setCameraToRecordMode() {
        Camera camera = getCamera();
        if (camera == null) {
            return;
        }
        if (ModuleVerificationUtil.isMavicAir2()) {
            camera.setFlatMode(SettingsDefinitions.FlatCameraMode.VIDEO_NORMAL,
                    e -> ToastUtils.setResultToToast("SetCameraMode to recordVideo"));
        } else {
            camera.setMode(SettingsDefinitions.CameraMode.RECORD_VIDEO,
                    e -> ToastUtils.setResultToToast("SetCameraMode to recordVideo"));
        }
    }

    private void startRecording() {
        Camera camera = getCamera();
        if (camera == null) {
            ToastUtils.setResultToToast("Camera unavailable.");
            return;
        }
        camera.startRecordVideo(error -> {
            if (error == null) {
                ToastUtils.setResultToToast("Start record");
                startTimer();
            } else {
                ToastUtils.setResultToToast("Start record failed: " + error.getDescription());
            }
        });
    }

    private void stopRecording() {
        Camera camera = getCamera();
        if (camera == null) {
            return;
        }
        camera.stopRecordVideo(error -> {
            ToastUtils.setResultToToast(error == null
                    ? "StopRecord"
                    : "Stop record failed: " + error.getDescription());
            stopTimer();
        });
    }

    private void startTimer() {
        stopTimer();
        timeCounterMs = 0;
        timer = new Timer();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                timeCounterMs += 1000;
                long h = TimeUnit.MILLISECONDS.toHours(timeCounterMs);
                long m = TimeUnit.MILLISECONDS.toMinutes(timeCounterMs) - h * 60;
                long s = TimeUnit.MILLISECONDS.toSeconds(timeCounterMs) - (h * 3600 + m * 60);
                final String t = String.format(Locale.US, "%02d:%02d:%02d", h, m, s);
                mainHandler.post(() -> tvTime.setText(t));
            }
        }, 1000, 1000);
    }

    private void stopTimer() {
        if (timer != null) {
            timer.cancel();
            timer = null;
        }
        mainHandler.post(() -> tvTime.setText("00:00:00"));
    }

    // ── Live video feed ─────────────────────────────────────────────────────────

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

    // ── PresentableView ─────────────────────────────────────────────────────────

    @Override
    public int getDescription() {
        return R.string.camera_listview_record_video;
    }

    @NonNull
    @Override
    public String getHint() {
        return this.getClass().getSimpleName() + ".java";
    }

    private int dp(int value) {
        return (int) (value * getResources().getDisplayMetrics().density);
    }
}
