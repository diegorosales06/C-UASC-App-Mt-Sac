package com.dji.sdk.sample.demo.geofencing;

import android.content.Context;
import android.content.SharedPreferences;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;

import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.view.PresentableView;

import java.util.ArrayList;
import java.util.List;

import dji.common.error.DJIError;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;

/**
 * GeofencingView
 *
 * Polygon-based containment geofencing using DJI Mobile SDK V4.
 * If the drone exits the polygon, RTH is triggered automatically.
 *
 * Waypoints persist across navigation (stored in static fields) until
 * the user explicitly presses "Clear All".
 *
 * FIXES vs original version:
 *  1. Removed Timer entirely. setStateCallback() is now registered ONCE in
 *     attachStateCallback(). The old code called setStateCallback() inside a
 *     TimerTask every 500ms, stacking dozens of overlapping callbacks that all
 *     fired simultaneously before GPS had stabilized — causing immediate false RTH.
 *  2. fenceVertices and fenceActive are now static, so they survive the view
 *     being destroyed when the user navigates to another menu and comes back.
 *  3. onDetachedFromWindow() no longer kills the callback if the fence is active,
 *     so enforcement continues even while the user is on a different screen.
 */
public class GeofencingView extends LinearLayout implements PresentableView {

    private static final String TAG = "GeofencingView";

    // SharedPreferences keys for persisting waypoints across app restarts
    private static final String PREFS_NAME      = "geofencing_prefs";
    private static final String PREFS_KEY_COUNT = "waypoint_count";
    private static final String PREFS_KEY_LAT   = "waypoint_lat_"; // + index
    private static final String PREFS_KEY_LNG   = "waypoint_lng_"; // + index

    // ---------------------------------------------------------------------------------
    // Static state — persists across view destruction/recreation within a session
    // ---------------------------------------------------------------------------------
    private static final List<double[]> fenceVertices = new ArrayList<>();
    private static boolean fenceActive    = false;
    private static boolean verticesLoaded = false; // ensures we only load from prefs once per session

    // Default hardcoded fence vertices (Mt. SAC area)
    // These are pre-loaded on first launch; user can add/clear from here
    private static final double[][] DEFAULT_VERTICES = {
    {  34.04617438715812, -117.84557606131044},
    {  34.046386000758645, -117.84524699712334 },
    {  34.046623412154894, -117.84562697335046 }
    };

    // ---------------------------------------------------------------------------------
    // UI components
    // ---------------------------------------------------------------------------------
    private TextView   tvStatus;
    private TextView   tvDronePos;
    private TextView   tvWaypointList;
    private EditText   etLat;
    private EditText   etLng;
    private Button     btnAddWaypoint;
    private Button     btnClearWaypoints;
    private Button     btnStartFence;
    private Button     btnStopFence;
    private ScrollView scrollLog;
    private TextView   tvLog;

    // ---------------------------------------------------------------------------------
    // DJI
    // ---------------------------------------------------------------------------------
    private FlightController flightController;

    // ---------------------------------------------------------------------------------
    // Logger
    // ---------------------------------------------------------------------------------
    private FlightLogger flightLogger;

    // ---------------------------------------------------------------------------------
    // Constructors
    // ---------------------------------------------------------------------------------

    public GeofencingView(Context context) {
        super(context);
        init(context);
    }

    public GeofencingView(Context context, AttributeSet attrs) {
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

        // --- Status ---
        tvStatus = new TextView(context);
        tvStatus.setTextSize(16f);
        tvStatus.setPadding(0, 0, 0, 8);
        addView(tvStatus);

        tvDronePos = new TextView(context);
        tvDronePos.setText("Drone position: unknown");
        tvDronePos.setTextSize(13f);
        tvDronePos.setPadding(0, 0, 0, 16);
        addView(tvDronePos);

        // --- Lat/Lng input row ---
        LinearLayout inputRow = new LinearLayout(context);
        inputRow.setOrientation(HORIZONTAL);

        etLat = new EditText(context);
        etLat.setHint("Latitude");
        etLat.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams etP1 = new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        etP1.setMarginEnd(8);
        etLat.setLayoutParams(etP1);
        inputRow.addView(etLat);

        etLng = new EditText(context);
        etLng.setHint("Longitude");
        etLng.setInputType(android.text.InputType.TYPE_CLASS_NUMBER
                | android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                | android.text.InputType.TYPE_NUMBER_FLAG_SIGNED);
        LinearLayout.LayoutParams etP2 = new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        etLng.setLayoutParams(etP2);
        inputRow.addView(etLng);

        addView(inputRow);

        // --- Add / Clear buttons ---
        LinearLayout btnRow1 = new LinearLayout(context);
        btnRow1.setOrientation(HORIZONTAL);
        btnRow1.setPadding(0, 8, 0, 8);

        btnAddWaypoint = new Button(context);
        btnAddWaypoint.setText("Add Point");
        LinearLayout.LayoutParams bP1 = new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP1.setMarginEnd(8);
        btnAddWaypoint.setLayoutParams(bP1);
        btnAddWaypoint.setOnClickListener(v -> onAddWaypoint());
        btnRow1.addView(btnAddWaypoint);

        btnClearWaypoints = new Button(context);
        btnClearWaypoints.setText("Clear All");
        LinearLayout.LayoutParams bP2 = new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        btnClearWaypoints.setLayoutParams(bP2);
        btnClearWaypoints.setOnClickListener(v -> onClearWaypoints());
        btnRow1.addView(btnClearWaypoints);

        addView(btnRow1);

        // --- Waypoint list ---
        tvWaypointList = new TextView(context);
        tvWaypointList.setTextSize(12f);
        tvWaypointList.setPadding(0, 0, 0, 12);
        addView(tvWaypointList);

        // --- Start / Stop fence ---
        LinearLayout btnRow2 = new LinearLayout(context);
        btnRow2.setOrientation(HORIZONTAL);
        btnRow2.setPadding(0, 0, 0, 16);

        btnStartFence = new Button(context);
        btnStartFence.setText("Start Fence");
        LinearLayout.LayoutParams bP3 = new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        bP3.setMarginEnd(8);
        btnStartFence.setLayoutParams(bP3);
        btnStartFence.setOnClickListener(v -> onStartFence());
        btnRow2.addView(btnStartFence);

        btnStopFence = new Button(context);
        btnStopFence.setText("Stop Fence");
        LinearLayout.LayoutParams bP4 = new LinearLayout.LayoutParams(0,
                LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        btnStopFence.setLayoutParams(bP4);
        btnStopFence.setOnClickListener(v -> onStopFence());
        btnRow2.addView(btnStopFence);

        addView(btnRow2);

        // --- Log ---
        tvLog = new TextView(context);
        tvLog.setText("Log:\n");
        tvLog.setTextSize(11f);

        scrollLog = new ScrollView(context);
        scrollLog.setLayoutParams(new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 400));
        scrollLog.addView(tvLog);
        addView(scrollLog);

        // Load persisted waypoints from SharedPreferences on first creation this session
        if (!verticesLoaded) {
            loadWaypointsFromPrefs(context);
            verticesLoaded = true;
        }

        // Restore UI to match persisted state
        refreshWaypointList();
        syncButtonState();

        // Grab the flight controller
        initFlightController();

        // If the fence was active before the user navigated away, re-attach the callback
        if (fenceActive) {
            attachStateCallback();
        }
    }

    // ---------------------------------------------------------------------------------
    // DJI initialisation
    // ---------------------------------------------------------------------------------

    private void initFlightController() {
        if (DJISampleApplication.getProductInstance() instanceof Aircraft) {
            Aircraft aircraft = (Aircraft) DJISampleApplication.getProductInstance();
            flightController = aircraft.getFlightController();
            if (flightController != null) {
                appendLog("FlightController ready.");
            } else {
                appendLog("FlightController unavailable.");
            }
        } else {
            appendLog("No aircraft connected.");
        }
    }

    // ---------------------------------------------------------------------------------
    // Button handlers
    // ---------------------------------------------------------------------------------

    private void onAddWaypoint() {
        String latStr = etLat.getText().toString().trim();
        String lngStr = etLng.getText().toString().trim();

        if (latStr.isEmpty() || lngStr.isEmpty()) {
            showToast("Please enter both latitude and longitude.");
            return;
        }

        double lat, lng;
        try {
            lat = Double.parseDouble(latStr);
            lng = Double.parseDouble(lngStr);
        } catch (NumberFormatException e) {
            showToast("Invalid coordinates.");
            return;
        }

        if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
            showToast("Coordinates out of valid range.");
            return;
        }

        fenceVertices.add(new double[]{lat, lng});
        saveWaypointsToPrefs(getContext());
        etLat.setText("");
        etLng.setText("");
        refreshWaypointList();
        appendLog(String.format("Added waypoint %d: (%.6f, %.6f)", fenceVertices.size(), lat, lng));
    }

    private void onClearWaypoints() {
        if (fenceActive) {
            showToast("Stop the fence before clearing waypoints.");
            return;
        }
        fenceVertices.clear();
        saveWaypointsToPrefs(getContext());
        refreshWaypointList();
        appendLog("Cleared all waypoints.");
    }

    private void onStartFence() {
        if (fenceVertices.size() < 3) {
            showToast("A polygon needs at least 3 waypoints.");
            return;
        }
        if (flightController == null) {
            initFlightController();
            if (flightController == null) {
                showToast("Flight controller not available.");
                return;
            }
        }

        fenceActive = true;
        syncButtonState();
        appendLog("Fence activated with " + fenceVertices.size() + " vertices.");

        flightLogger = new FlightLogger(getContext());
        flightLogger.start();
        appendLog("Logging to: " + flightLogger.getLogFilePath());

        attachStateCallback();
    }

    private void onStopFence() {
        fenceActive = false;
        detachStateCallback();
        if (flightLogger != null) {
            flightLogger.stop();
            appendLog("Log saved to: " + flightLogger.getLogFilePath());
        }
        syncButtonState();
        appendLog("Fence deactivated by user.");
    }

    // ---------------------------------------------------------------------------------
    // State callback — registered ONCE, not inside a timer loop
    // ---------------------------------------------------------------------------------

    private void attachStateCallback() {
        if (flightController == null) return;

        flightController.setStateCallback(state -> {
            LocationCoordinate3D location = state.getAircraftLocation();
            if (location == null) return;

            double droneLat = location.getLatitude();
            double droneLng = location.getLongitude();

            // Ignore readings with no GPS fix (DJI SDK returns 0,0 when no fix)
            if (droneLat == 0.0 && droneLng == 0.0) return;

            // Always update the position display
//            post(() -> tvDronePos.setText(
//                    String.format("Drone: (%.6f, %.6f)  alt: %.1fm",
//                            droneLat, droneLng, location.getAltitude())));

            // Only enforce the boundary if the fence is currently active
            if (!fenceActive) return;

            boolean inside = isPointInPolygon(droneLat, droneLng, fenceVertices);

            // Log position and inside flag to Logcat on every tick
            Log.d(TAG, "lat=" + droneLat + "  lng=" + droneLng + "  inside=" + inside);

            // Also print to the on-screen log
            post(() -> appendLog(String.format(
                    "lat=%.6f  lng=%.6f  inside=%s", droneLat, droneLng, inside)));

            // Write to CSV file
            if (flightLogger != null) {
                flightLogger.log(droneLat, droneLng, inside);
            }

            if (!inside) {
                fenceActive = false;

                // Stop the logger on breach
                if (flightLogger != null) {
                    flightLogger.stop();
                }

                post(() -> {
                    appendLog(String.format(
                            "BREACH at (%.6f, %.6f) — initiating RTH.", droneLat, droneLng));
                    syncButtonState();
                    tvStatus.setText("Fence: BREACH — RTH issued");
                });

                triggerReturnToHome();
            }
        });
    }

    private void detachStateCallback() {
        if (flightController != null) {
            flightController.setStateCallback(null);
        }
    }

    // ---------------------------------------------------------------------------------
    // Return-to-Home
    // ---------------------------------------------------------------------------------

    private void triggerReturnToHome() {
        if (flightController == null) return;

        flightController.startGoHome(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                if (djiError == null) {
                    post(() -> appendLog("RTH command accepted by flight controller."));
                } else {
                    post(() -> appendLog("RTH command failed: " + djiError.getDescription()));
                }
            }
        });
    }

    // ---------------------------------------------------------------------------------
    // SharedPreferences persistence — survives full app restarts
    // ---------------------------------------------------------------------------------

    /**
     * Serialises fenceVertices into SharedPreferences.
     * Called every time a waypoint is added or the list is cleared.
     * Each waypoint is stored as two separate double entries keyed by index:
     *   waypoint_lat_0, waypoint_lng_0, waypoint_lat_1, waypoint_lng_1, ...
     */
    private void saveWaypointsToPrefs(Context context) {
        SharedPreferences.Editor editor = context
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit();
        editor.putInt(PREFS_KEY_COUNT, fenceVertices.size());
        for (int i = 0; i < fenceVertices.size(); i++) {
            // Store double as raw long bits to preserve full precision
            editor.putLong(PREFS_KEY_LAT + i,
                    Double.doubleToRawLongBits(fenceVertices.get(i)[0]));
            editor.putLong(PREFS_KEY_LNG + i,
                    Double.doubleToRawLongBits(fenceVertices.get(i)[1]));
        }
        editor.apply();
        Log.d(TAG, "Saved " + fenceVertices.size() + " waypoints to SharedPreferences.");
    }

    /**
     * Restores fenceVertices from SharedPreferences.
     * Called once per app session the first time GeofencingView is created.
     * If no waypoints were previously saved, fenceVertices stays empty.
     */
    private void loadWaypointsFromPrefs(Context context) {
        SharedPreferences prefs = context
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        int count = prefs.getInt(PREFS_KEY_COUNT, -1);

        fenceVertices.clear();

        if (count <= 0) {
            // No saved waypoints — load the hardcoded defaults
            for (double[] vertex : DEFAULT_VERTICES) {
                fenceVertices.add(new double[]{vertex[0], vertex[1]});
            }
            Log.d(TAG, "No saved waypoints found — loaded " + fenceVertices.size() + " default vertices.");
        } else {
            for (int i = 0; i < count; i++) {
                double lat = Double.longBitsToDouble(prefs.getLong(PREFS_KEY_LAT + i, 0));
                double lng = Double.longBitsToDouble(prefs.getLong(PREFS_KEY_LNG + i, 0));
                fenceVertices.add(new double[]{lat, lng});
            }
            Log.d(TAG, "Loaded " + fenceVertices.size() + " waypoints from SharedPreferences.");
        }
    }

    // ---------------------------------------------------------------------------------
    // Geometry: Ray-Casting Point-in-Polygon
    // ---------------------------------------------------------------------------------

    /**
     * Returns true if (lat, lng) is inside the polygon defined by the vertex list.
     * Uses the standard ray-casting algorithm. Works for any simple
     * (non-self-intersecting) polygon regardless of winding order (CW or CCW).
     */
    static boolean isPointInPolygon(double lat, double lng, List<double[]> polygon) {
        int n = polygon.size();
        if (n < 3) return false;

        boolean inside = false;
        int j = n - 1;

        for (int i = 0; i < n; i++) {
            double xi = polygon.get(i)[0]; // lat of vertex i
            double yi = polygon.get(i)[1]; // lng of vertex i
            double xj = polygon.get(j)[0]; // lat of previous vertex
            double yj = polygon.get(j)[1]; // lng of previous vertex

            boolean intersect = ((yi > lng) != (yj > lng))
                    && (lat < (xj - xi) * (lng - yi) / (yj - yi) + xi);

            if (intersect) inside = !inside;
            j = i;
        }
        return inside;
    }

    // ---------------------------------------------------------------------------------
    // UI helpers
    // ---------------------------------------------------------------------------------

    /** Syncs button states and status label to the current value of fenceActive. */
    private void syncButtonState() {
        boolean active = fenceActive;
        if (btnStartFence     != null) btnStartFence.setEnabled(!active);
        if (btnStopFence      != null) btnStopFence.setEnabled(active);
        if (btnAddWaypoint    != null) btnAddWaypoint.setEnabled(!active);
        if (btnClearWaypoints != null) btnClearWaypoints.setEnabled(!active);
        if (tvStatus != null) {
            tvStatus.setText(active
                    ? "Fence: ACTIVE  (" + fenceVertices.size() + " vertices)"
                    : "Fence: INACTIVE");
        }
    }

    private void refreshWaypointList() {
        if (fenceVertices.isEmpty()) {
            tvWaypointList.setText("Waypoints: (none)");
            return;
        }
        StringBuilder sb = new StringBuilder("Waypoints:\n");
        for (int i = 0; i < fenceVertices.size(); i++) {
            sb.append(String.format("  %d: (%.6f, %.6f)\n",
                    i + 1, fenceVertices.get(i)[0], fenceVertices.get(i)[1]));
        }
        tvWaypointList.setText(sb.toString());
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

    // ---------------------------------------------------------------------------------
    // Lifecycle
    // ---------------------------------------------------------------------------------

    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
        // If the fence is still active, leave the callback registered so enforcement
        // continues while the user is on a different screen.
        // Only clean up if the fence is not running.
        if (!fenceActive) {
            detachStateCallback();
        }
    }
}