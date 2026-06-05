package com.dji.sdk.sample.internal.utils;

import android.os.Handler;
import android.os.Looper;

import com.dji.sdk.sample.internal.controller.DJISampleApplication;

import java.util.concurrent.atomic.AtomicBoolean;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;

/**
 * Shared helper for issuing a manual Return-to-Home command from mission UIs.
 */
public final class ReturnHomeCommand {

    private static final Handler MAIN_HANDLER = new Handler(Looper.getMainLooper());
    private static final long VIRTUAL_STICK_DISABLE_TIMEOUT_MS = 2_000L;
    private static final int RTH_RETRIES = 1;

    public interface MessageCallback {
        void onMessage(String message);
    }

    private ReturnHomeCommand() {}

    public static void start(MessageCallback callback) {
        Aircraft aircraft = DJISampleApplication.getAircraftInstance();
        if (aircraft == null) {
            emitMessage(callback, "No aircraft connected.");
            return;
        }

        FlightController controller = aircraft.getFlightController();
        if (controller == null) {
            emitMessage(callback, "Flight controller not available.");
            return;
        }

        start(controller, callback);
    }

    public static void start(FlightController controller, MessageCallback callback) {
        if (controller == null) {
            emitMessage(callback, "Flight controller not available.");
            return;
        }

        emitMessage(callback, "RTH requested.");
        AtomicBoolean rthIssued = new AtomicBoolean(false);
        Runnable fallback = () -> {
            emitMessage(callback, "Virtual Stick disable callback delayed - trying RTH now.");
            startGoHomeOnce(controller, rthIssued, callback, RTH_RETRIES);
        };

        MAIN_HANDLER.postDelayed(fallback, VIRTUAL_STICK_DISABLE_TIMEOUT_MS);
        controller.setVirtualStickModeEnabled(false, error -> {
            MAIN_HANDLER.removeCallbacks(fallback);
            if (error != null) {
                emitMessage(callback, "Virtual Stick disable before RTH failed: "
                        + error.getDescription());
            }
            startGoHomeOnce(controller, rthIssued, callback, RTH_RETRIES);
        });
    }

    private static void startGoHomeOnce(FlightController controller,
                                        AtomicBoolean rthIssued,
                                        MessageCallback callback,
                                        int retriesRemaining) {
        if (!rthIssued.compareAndSet(false, true)) return;
        startGoHome(controller, callback, retriesRemaining);
    }

    private static void startGoHome(FlightController controller,
                                    MessageCallback callback,
                                    int retriesRemaining) {
        setGoHomeHeightToCurrentAltitude(controller, callback, () ->
                controller.startGoHome(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                if (djiError == null) {
                    emitMessage(callback, "RTH command accepted by flight controller.");
                    return;
                }

                emitMessage(callback, "RTH command failed: " + djiError.getDescription());
                if (retriesRemaining > 0) {
                    emitMessage(callback, "Retrying RTH command.");
                    MAIN_HANDLER.postDelayed(
                            () -> startGoHome(controller, callback, retriesRemaining - 1),
                            1_000L);
                }
            }
        }));
    }

    public static void setGoHomeHeightToCurrentAltitude(FlightController controller,
                                                        MessageCallback callback,
                                                        Runnable afterSetAttempt) {
        int heightMeters = getCurrentAltitudeMeters(controller);
        if (heightMeters <= 0) {
            emitMessage(callback, "Current altitude unavailable - RTH will use the aircraft's configured height.");
            runAfterSetAttempt(afterSetAttempt);
            return;
        }

        emitMessage(callback, "Setting RTH height to current altitude: " + heightMeters + "m.");
        controller.setGoHomeHeightInMeters(heightMeters, error -> {
            if (error == null) {
                emitMessage(callback, "RTH height set to " + heightMeters + "m.");
            } else {
                emitMessage(callback, "Could not set RTH height to current altitude: "
                        + error.getDescription()
                        + ". RTH will use the aircraft's configured height.");
            }
            runAfterSetAttempt(afterSetAttempt);
        });
    }

    private static int getCurrentAltitudeMeters(FlightController controller) {
        FlightControllerState state = controller.getState();
        if (state == null) {
            state = FlightControllerStateDispatcher.getLatestState();
        }
        if (state == null || state.getAircraftLocation() == null) return -1;

        LocationCoordinate3D location = state.getAircraftLocation();
        float altitude = location.getAltitude();
        if (Float.isNaN(altitude) || Float.isInfinite(altitude) || altitude <= 0.0f) {
            return -1;
        }
        return Math.max(1, (int) Math.ceil(altitude));
    }

    private static void runAfterSetAttempt(Runnable afterSetAttempt) {
        if (afterSetAttempt != null) afterSetAttempt.run();
    }

    private static void emitMessage(MessageCallback callback, String message) {
        if (callback != null) callback.onMessage(message);
    }
}
