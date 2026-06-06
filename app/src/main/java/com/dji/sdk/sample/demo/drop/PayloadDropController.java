package com.dji.sdk.sample.demo.drop;

import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import com.dji.sdk.sample.internal.utils.ReturnHomeCommand.MessageCallback;

import dji.common.error.DJIError;
import dji.common.flightcontroller.flightassistant.FillLightMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightAssistant;
import dji.sdk.flightcontroller.FlightController;

/**
 * PayloadDropController
 *
 * Actuates the payload release, which is wired to the downward auxiliary fill
 * light: the pin OPENS (drops) when the fill light is ON, and is closed/loaded
 * when it is OFF (confirmed via the LED Control screen).
 *
 * Two robustness measures vs. the previous version, because the release worked
 * from the LED screen but not mid-mission:
 *
 *  1. Forced OFF→ON transition. Some relay/servo releases only actuate on a
 *     state CHANGE, so issuing "ON" when the light is already ON is a silent
 *     no-op. We drive OFF first, then ON, guaranteeing a transition. The final
 *     state is ON (= open = released).
 *
 *  2. Verified result + retries. The SDK's actual success/failure is reported
 *     through a {@link MessageCallback} so it shows up on-screen (the old code
 *     logged only to Logcat and the UI always said "dropped" regardless). If the
 *     ON command is rejected it is retried a couple of times.
 */
public class PayloadDropController {

    private static final String TAG = "PayloadDropController";

    private static final int  ON_RETRIES         = 2;
    private static final long TRANSITION_DELAY_MS = 300L;
    private static final long RETRY_DELAY_MS      = 500L;

    private boolean dropped = false;
    private final Handler handler = new Handler(Looper.getMainLooper());

    public boolean hasDropped() {
        return dropped;
    }

    /** Backwards-compatible entry point (no on-screen logging). */
    public void dropPayload(FlightController flightController) {
        dropPayload(flightController, null);
    }

    /**
     * Releases the payload: forces a fill-light OFF→ON transition and verifies the
     * ON command succeeded, reporting the real result through {@code log}.
     */
    public void dropPayload(FlightController flightController, MessageCallback log) {
        dropped = true;
        Log.d(TAG, "dropPayload() called.");

        FlightAssistant fa = flightController == null ? null : flightController.getFlightAssistant();
        if (fa == null) {
            report(log, "Drop FAILED: Flight Assistant unavailable — pin not actuated.");
            return;
        }

        report(log, "Releasing payload — fill light OFF→ON...");
        // Step 1: ensure OFF (pin closed) so the following ON is always a transition.
        fa.setDownwardFillLightMode(FillLightMode.OFF, offError ->
                // Step 2: after a short settle, drive ON (pin open) with retries.
                handler.postDelayed(() -> setOn(fa, log, ON_RETRIES), TRANSITION_DELAY_MS));
    }

    private void setOn(FlightAssistant fa, MessageCallback log, int retriesLeft) {
        fa.setDownwardFillLightMode(FillLightMode.ON, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                if (error == null) {
                    report(log, "Pin OPEN — fill light ON. Payload released.");
                } else if (retriesLeft > 0) {
                    report(log, "Fill light ON rejected (" + error.getDescription()
                            + "); retrying (" + retriesLeft + " left)...");
                    handler.postDelayed(() -> setOn(fa, log, retriesLeft - 1), RETRY_DELAY_MS);
                } else {
                    report(log, "Drop FAILED: fill light ON rejected — " + error.getDescription());
                }
            }
        });
    }

    /** Closes the pin (fill light OFF) — use to reload between drops/tests. */
    public void closePin(FlightController flightController, MessageCallback log) {
        FlightAssistant fa = flightController == null ? null : flightController.getFlightAssistant();
        if (fa == null) {
            report(log, "Cannot close pin: Flight Assistant unavailable.");
            return;
        }
        fa.setDownwardFillLightMode(FillLightMode.OFF, error -> {
            if (error == null) {
                report(log, "Pin CLOSED — fill light OFF (ready to load).");
            } else {
                report(log, "Close pin failed: " + error.getDescription());
            }
        });
    }

    /** Resets the dropped flag so a new mission session can drop again. */
    public void reset() {
        dropped = false;
        Log.d(TAG, "PayloadDropController reset.");
    }

    private void report(MessageCallback log, String msg) {
        Log.d(TAG, msg);
        if (log != null) log.onMessage(msg);
    }
}
