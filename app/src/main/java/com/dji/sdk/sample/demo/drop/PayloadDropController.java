package com.dji.sdk.sample.demo.drop;

import android.util.Log;

import dji.common.flightcontroller.flightassistant.FillLightMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;

/**
 * PayloadDropController
 *
 * Manages the payload drop state for PayloadDropMissionView.
 *
 * Responsibilities:
 *  - Tracks whether the payload has been dropped via hasDropped()
 *  - Executes the drop sequence via dropPayload():
 *      1. Sets the dropped flag to true
 *      2. Turns on the downward fill light via FlightAssistant.setDownwardFillLightMode()
 *         using the same logic confirmed working in LEDControlView.java
 *
 * Usage in PayloadDropMissionView:
 *   if (closeToDropTarget && highEnoughToDrop) {
 *       payloadDropController.dropPayload(flightController);
 *   }
 *   if (!payloadDropController.hasDropped()) { ... }
 */
public class PayloadDropController {

    private static final String TAG = "PayloadDropController";

    // Tracks whether the payload has been dropped this mission session
    private boolean dropped = false;

    // ---------------------------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------------------------

    /**
     * Returns true if the payload has already been dropped this session.
     * Used in the control loop to prevent triggering the drop more than once.
     *
     * @return true if dropPayload() has already been called successfully
     */
    public boolean hasDropped() {
        return dropped;
    }

    /**
     * Executes the payload drop sequence:
     *  1. Sets dropped = true immediately so the control loop stops checking
     *  2. Turns on the downward fill light via FlightAssistant using the same
     *     FillLightMode.ON approach confirmed working in LEDControlView.java
     *
     * The FlightController is passed in from PayloadDropMissionView rather than
     * stored in this class — keeps this class stateless with respect to DJI objects
     * and avoids holding a stale reference after the view is destroyed.
     *
     * @param flightController the active FlightController from PayloadDropMissionView
     */
    public void dropPayload(FlightController flightController) {
        // Mark as dropped immediately — prevents double-drop if control loop
        // ticks again before the async LED callback returns
        dropped = true;

        Log.d(TAG, "dropPayload() called — payload released.");

        if (flightController == null) {
            Log.e(TAG, "FlightController is null — cannot turn on fill light.");
            return;
        }

        // Get FlightAssistant from FlightController — same pattern as LEDControlView
        dji.sdk.flightcontroller.FlightAssistant flightAssistant =
                flightController.getFlightAssistant();

        if (flightAssistant == null) {
            Log.e(TAG, "FlightAssistant unavailable — fill light not turned on.");
            return;
        }

        // Turn on the downward fill light — same call confirmed working in LEDControlView
        flightAssistant.setDownwardFillLightMode(FillLightMode.ON,
                new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(dji.common.error.DJIError error) {
                        if (error == null) {
                            Log.d(TAG, "Downward fill light turned ON at drop.");
                        } else {
                            Log.e(TAG, "Fill light failed: " + error.getDescription());
                        }
                    }
                });
    }

    /**
     * Resets the drop state — call this when starting a new mission session
     * so hasDropped() returns false again and the drop can be triggered once more.
     */
    public void reset() {
        dropped = false;
        Log.d(TAG, "PayloadDropController reset.");
    }
}