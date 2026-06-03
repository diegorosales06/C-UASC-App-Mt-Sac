package com.dji.sdk.sample.internal.utils;

import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.concurrent.CopyOnWriteArrayList;

import dji.common.flightcontroller.FlightControllerState;
import dji.sdk.flightcontroller.FlightController;

/**
 * Multiplexes DJI FlightControllerState updates.
 *
 * DJI SDK V4 exposes a single FlightController.setStateCallback slot. Screens
 * that register directly replace each other, which can silently stop background
 * safety features such as geofence enforcement. This dispatcher owns that one
 * SDK callback and fans updates out to every active listener.
 */
public final class FlightControllerStateDispatcher {

    private static final String TAG = "FCStateDispatcher";

    public interface Listener {
        void onUpdate(@NonNull FlightControllerState state);
    }

    private static final CopyOnWriteArrayList<Listener> listeners = new CopyOnWriteArrayList<>();

    @Nullable
    private static FlightController flightController;

    @Nullable
    private static volatile FlightControllerState latestState;

    private static final FlightControllerState.Callback DISPATCH_CALLBACK = state -> {
        if (state == null) return;
        latestState = state;
        for (Listener listener : listeners) {
            try {
                listener.onUpdate(state);
            } catch (RuntimeException e) {
                Log.e(TAG, "Flight state listener failed", e);
            }
        }
    };

    private FlightControllerStateDispatcher() {
    }

    public static synchronized void addListener(@Nullable FlightController controller,
                                                @Nullable Listener listener) {
        if (controller == null || listener == null) return;
        attachTo(controller);
        listeners.addIfAbsent(listener);

        FlightControllerState currentState = controller.getState();
        if (currentState == null) {
            currentState = latestState;
        }
        if (currentState != null) {
            listener.onUpdate(currentState);
        }
    }

    public static synchronized void removeListener(@Nullable Listener listener) {
        if (listener == null) return;
        listeners.remove(listener);
        if (listeners.isEmpty() && flightController != null) {
            flightController.setStateCallback(null);
            flightController = null;
            latestState = null;
        }
    }

    @Nullable
    public static FlightControllerState getLatestState() {
        return latestState;
    }

    public static synchronized void ensureAttached(@Nullable FlightController controller) {
        if (controller != null && !listeners.isEmpty()) {
            attachTo(controller);
        }
    }

    private static void attachTo(@NonNull FlightController controller) {
        if (flightController != controller) {
            if (flightController != null) {
                flightController.setStateCallback(null);
            }
            flightController = controller;
        }
        controller.setStateCallback(DISPATCH_CALLBACK);
    }
}
