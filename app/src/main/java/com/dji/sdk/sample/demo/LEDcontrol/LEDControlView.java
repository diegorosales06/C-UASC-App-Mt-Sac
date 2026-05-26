package com.dji.sdk.sample.demo.LEDcontrol;

import android.content.Context;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;

import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.view.PresentableView;

import dji.common.error.DJIError;
import dji.common.flightcontroller.LEDsSettings;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import dji.common.flightcontroller.flightassistant.FillLightMode;

/**
 * LEDControlView
 *
 * Controls the auxiliary bottom LED on the DJI Mavic Air 2 via
 * FlightController.setLEDsEnabled(LEDsSettings, callback) in SDK V4.
 *
 * LEDsSettings controls front, back, and bottom LEDs independently.
 * This view only toggles the bottom auxiliary LED, leaving front and
 * back LEDs on.
 *
 * Package must match the folder name exactly: LEDcontrol (capital LED)
 * To register: add entry in DemoListView.java pointing to LEDControlView.class
 */
public class LEDControlView extends LinearLayout implements PresentableView {

    private static final String TAG = "LEDControlView";

    // Tracks whether the bottom auxiliary LED is currently on
    private boolean ledOn = false;

    // ---------------------------------------------------------------------------------
    // DJI
    // ---------------------------------------------------------------------------------
    private FlightController flightController;
    private dji.sdk.flightcontroller.FlightAssistant flightAssistant;


    // ---------------------------------------------------------------------------------
    // UI
    // ---------------------------------------------------------------------------------
    private TextView   tvStatus;
    private Button     btnToggle;
    private ScrollView scrollLog;
    private TextView   tvLog;

    // ---------------------------------------------------------------------------------
    // Constructors
    // ---------------------------------------------------------------------------------

    public LEDControlView(Context context) {
        super(context);
        init(context);
    }

    public LEDControlView(Context context, AttributeSet attrs) {
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

        tvStatus = new TextView(context);
        tvStatus.setTextSize(18f);
        tvStatus.setPadding(0, 0, 0, 24);
        addView(tvStatus);

        btnToggle = new Button(context);
        btnToggle.setTextSize(16f);
        LinearLayout.LayoutParams btnParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT);
        btnParams.setMargins(0, 0, 0, 24);
        btnToggle.setLayoutParams(btnParams);
        btnToggle.setOnClickListener(v -> onToggleLED());
        addView(btnToggle);

        tvLog = new TextView(context);
        tvLog.setText("Log:\n");
        tvLog.setTextSize(11f);

        scrollLog = new ScrollView(context);
        scrollLog.setLayoutParams(new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 500));
        scrollLog.addView(tvLog);
        addView(scrollLog);

        syncUI();
        initFlightController();
    }

    // ---------------------------------------------------------------------------------
    // DJI
    // ---------------------------------------------------------------------------------

    private void initFlightController() {
        if (!(DJISampleApplication.getProductInstance() instanceof Aircraft)) {
            appendLog("No aircraft connected.");
            return;
        }
        Aircraft aircraft = (Aircraft) DJISampleApplication.getProductInstance();
        flightController = aircraft.getFlightController();
        if (flightController != null) {
            appendLog("FlightController ready.");
            flightAssistant = flightController.getFlightAssistant();
        } else {
            appendLog("FlightController unavailable.");
        }


    }

    // ---------------------------------------------------------------------------------
    // Toggle
    // ---------------------------------------------------------------------------------

    private void onToggleLED() {
        if (flightController == null) {
            initFlightController();
            if (flightController == null) {
                showToast("Flight controller not available.");
                return;
            }
        }

        boolean newState = !ledOn;

        if (flightAssistant == null) {
            showToast("Flight Assistant not available.");
            appendLog("Flight Assistant not available.");
            return;
        }

        FillLightMode mode = newState ? FillLightMode.ON : FillLightMode.OFF;

        flightAssistant.setDownwardFillLightMode(mode, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                if (error == null) {
                    ledOn = newState;
                    Log.d(TAG, "Downward fill light set to: " + (ledOn ? "ON" : "OFF"));
                    post(() -> {
                        syncUI();
                        appendLog("Downward fill light is now " + (ledOn ? "ON" : "OFF") + ".");
                    });
                } else {
                    Log.e(TAG, "Downward fill light failed: " + error.getDescription());
                    post(() -> {
                        appendLog("Downward fill light failed: " + error.getDescription());
                        showToast("Fill light command failed — see log.");
                    });
                }
            }
        });
    }
    // ---------------------------------------------------------------------------------
    // UI helpers
    // ---------------------------------------------------------------------------------

    private void syncUI() {
        if (ledOn) {
            tvStatus.setText("LED: ON");
            btnToggle.setText("Turn LED OFF");
        } else {
            tvStatus.setText("LED: OFF");
            btnToggle.setText("Turn LED ON");
        }
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

    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
    }
}