package com.dji.sdk.sample.demo.map;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.Handler;
import android.os.Looper;
import android.util.AttributeSet;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.view.PresentableView;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.model.LocationCoordinate2D;
import dji.keysdk.DJIKey;
import dji.keysdk.FlightControllerKey;
import dji.keysdk.KeyManager;
import dji.keysdk.callback.KeyListener;

/**
 * Standalone map screen that exercises the reusable free map add-in with
 * mock telemetry or DJI aircraft-location telemetry.
 */
public class MapTrackingView extends LinearLayout implements PresentableView {

    private static final String PREFS_NAME = "geofencing_prefs";
    private static final String PREFS_KEY_COUNT = "waypoint_count";
    private static final String PREFS_KEY_LAT = "waypoint_lat_";
    private static final String PREFS_KEY_LNG = "waypoint_lng_";

    private static final double[][] DEFAULT_FENCE_VERTICES = {
            {34.04618635227991, -117.84552701364355},
            {34.04632498120861, -117.84524530311664},
            {34.04658300697332, -117.84539805413424},
            {34.04646801720643, -117.84579969397946}
    };

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final List<MapPoint> fenceVertices = new ArrayList<>();
    private final List<MapPoint> virtualStickWaypoints = new ArrayList<>();
    private final List<MapPoint> mockRoute = new ArrayList<>();

    private MapAddInView mapAddInView;
    private TextView statusText;
    private TextView coordinateText;
    private Button telemetryModeButton;
    private Button reloadFenceButton;
    private Button dropPointButton;

    private boolean useMockTelemetry;
    private boolean started;
    private boolean showingDropPoint;
    private MapPoint currentDronePoint;
    private double currentHeadingDegrees;
    private int mockSegmentIndex;
    private double mockSegmentProgress;

    private DJIKey aircraftLocationKey;
    private KeyListener aircraftLocationListener;

    private final Runnable mockTick = new Runnable() {
        @Override
        public void run() {
            emitMockTelemetry();
            handler.postDelayed(this, 750);
        }
    };

    public MapTrackingView(Context context) {
        super(context);
        init(context);
    }

    public MapTrackingView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    @Override
    public int getDescription() {
        return 0;
    }

    @NonNull
    @Override
    public String getHint() {
        return this.getClass().getSimpleName() + ".java";
    }

    private void init(Context context) {
        setOrientation(VERTICAL);
        setPadding(dp(12), dp(12), dp(12), dp(12));

        loadFenceVertices(context);
        loadVirtualStickWaypoints(context);
        rebuildMockRoute();

        statusText = new TextView(context);
        statusText.setTextSize(16f);
        statusText.setText("Map telemetry: starting...");
        statusText.setPadding(0, 0, 0, dp(8));
        addView(statusText, new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));

        mapAddInView = new MapAddInView(context);
        LayoutParams mapParams = new LayoutParams(LayoutParams.MATCH_PARENT, 0, 1f);
        addView(mapAddInView, mapParams);

        coordinateText = new TextView(context);
        coordinateText.setTextSize(13f);
        coordinateText.setText("Drone position: waiting");
        coordinateText.setPadding(0, dp(8), 0, dp(8));
        addView(coordinateText, new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));

        LinearLayout buttonRow = new LinearLayout(context);
        buttonRow.setOrientation(HORIZONTAL);

        telemetryModeButton = makeButton(context, "Mock");
        telemetryModeButton.setOnClickListener(v -> toggleTelemetryMode());
        buttonRow.addView(telemetryModeButton, weightedParams(true));

        reloadFenceButton = makeButton(context, "Reload Map");
        reloadFenceButton.setOnClickListener(v -> reloadFence());
        buttonRow.addView(reloadFenceButton, weightedParams(true));

        dropPointButton = makeButton(context, "Drop Point");
        dropPointButton.setOnClickListener(v -> toggleDropPoint());
        buttonRow.addView(dropPointButton, weightedParams(false));

        addView(buttonRow, new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));

        useMockTelemetry = !isDjiTelemetryAvailable();
        updateModeButton();
        pushMissionDataToMap();
    }

    @Override
    protected void onAttachedToWindow() {
        super.onAttachedToWindow();
        mapAddInView.onResume();
        startTelemetry();
    }

    @Override
    protected void onDetachedFromWindow() {
        stopTelemetry();
        mapAddInView.onPause();
        mapAddInView.onDestroy();
        super.onDetachedFromWindow();
    }

    private void toggleTelemetryMode() {
        if (useMockTelemetry && !isDjiTelemetryAvailable()) {
            Toast.makeText(getContext(), "DJI telemetry is not available, staying in mock mode.", Toast.LENGTH_SHORT).show();
            return;
        }

        useMockTelemetry = !useMockTelemetry;
        currentDronePoint = null;
        mapAddInView.clearTrail();
        restartTelemetry();
    }

    private void reloadFence() {
        loadFenceVertices(getContext());
        loadVirtualStickWaypoints(getContext());
        rebuildMockRoute();
        showingDropPoint = false;
        mapAddInView.clearTrail();
        pushMissionDataToMap();
        Toast.makeText(getContext(), "Fence and VS waypoints reloaded.", Toast.LENGTH_SHORT).show();
    }

    private void toggleDropPoint() {
        showingDropPoint = !showingDropPoint;
        mapAddInView.setDropPoint(showingDropPoint ? makeDefaultDropPoint() : null);
        dropPointButton.setText(showingDropPoint ? "Hide Drop" : "Drop Point");
    }

    private void pushMissionDataToMap() {
        mapAddInView.setGeofence(fenceVertices);
        mapAddInView.setMissionWaypoints(virtualStickWaypoints);
        mapAddInView.setCircuitPoints(new ArrayList<MapPoint>());
        mapAddInView.setDropPoint(showingDropPoint ? makeDefaultDropPoint() : null);
    }

    private void updateModeButton() {
        if (telemetryModeButton != null) {
            telemetryModeButton.setText(useMockTelemetry ? "Mock" : "DJI");
        }
    }

    private void startTelemetry() {
        if (started) {
            return;
        }
        started = true;

        if (!useMockTelemetry && startDjiTelemetry()) {
            updateModeButton();
            return;
        }

        useMockTelemetry = true;
        startMockTelemetry();
        updateModeButton();
    }

    private void restartTelemetry() {
        stopTelemetry();
        startTelemetry();
    }

    private void stopTelemetry() {
        stopDjiTelemetry();
        handler.removeCallbacks(mockTick);
        started = false;
    }

    private boolean startDjiTelemetry() {
        KeyManager keyManager = KeyManager.getInstance();
        if (keyManager == null || !isDjiTelemetryAvailable()) {
            statusText.setText("Map telemetry: DJI unavailable, using mock data");
            return false;
        }

        aircraftLocationKey = FlightControllerKey.create(FlightControllerKey.AIRCRAFT_LOCATION);
        aircraftLocationListener = new KeyListener() {
            @Override
            public void onValueChange(@Nullable Object oldValue, @Nullable Object newValue) {
                if (newValue != null) {
                    post(() -> handleDjiLocation(newValue));
                }
            }
        };

        keyManager.addListener(aircraftLocationKey, aircraftLocationListener);
        Object currentLocation = keyManager.getValue(aircraftLocationKey);
        if (currentLocation != null) {
            handleDjiLocation(currentLocation);
        }
        statusText.setText("Map telemetry: DJI aircraft location");
        return true;
    }

    private void stopDjiTelemetry() {
        if (aircraftLocationListener != null && KeyManager.getInstance() != null) {
            KeyManager.getInstance().removeListener(aircraftLocationListener);
        }
        aircraftLocationListener = null;
        aircraftLocationKey = null;
    }

    private void handleDjiLocation(Object value) {
        MapPoint point = null;
        float altitude = 0f;

        if (value instanceof LocationCoordinate3D) {
            LocationCoordinate3D location = (LocationCoordinate3D) value;
            point = new MapPoint(location.getLatitude(), location.getLongitude());
            altitude = location.getAltitude();
        } else if (value instanceof LocationCoordinate2D) {
            LocationCoordinate2D location = (LocationCoordinate2D) value;
            point = new MapPoint(location.getLatitude(), location.getLongitude());
        }

        if (point == null || !isValidCoordinate(point)) {
            return;
        }

        double heading = currentDronePoint == null ? currentHeadingDegrees : bearingDegrees(currentDronePoint, point);
        updateTelemetry(point, altitude, heading, "DJI");
    }

    private void startMockTelemetry() {
        rebuildMockRoute();
        mockSegmentIndex = 0;
        mockSegmentProgress = 0.0;
        statusText.setText("Map telemetry: mock drone route");
        handler.removeCallbacks(mockTick);
        mockTick.run();
    }

    private void emitMockTelemetry() {
        if (mockRoute.size() < 2) {
            return;
        }

        MapPoint start = mockRoute.get(mockSegmentIndex);
        MapPoint end = mockRoute.get((mockSegmentIndex + 1) % mockRoute.size());
        MapPoint point = interpolate(start, end, mockSegmentProgress);
        double heading = bearingDegrees(start, end);
        float altitude = (float) (14.0 + Math.sin((mockSegmentIndex + mockSegmentProgress) * Math.PI) * 2.0);

        updateTelemetry(point, altitude, heading, "Mock");

        mockSegmentProgress += 0.07;
        if (mockSegmentProgress >= 1.0) {
            mockSegmentProgress = 0.0;
            mockSegmentIndex = (mockSegmentIndex + 1) % mockRoute.size();
        }
    }

    private void updateTelemetry(MapPoint point, float altitude, double headingDegrees, String source) {
        currentDronePoint = point;
        currentHeadingDegrees = headingDegrees;

        boolean inside = fenceVertices.size() >= 3 && isPointInPolygon(point, fenceVertices);
        statusText.setText(String.format(Locale.US,
                "Map telemetry: %s | fence %s",
                source,
                inside ? "INSIDE" : "OUTSIDE"));
        coordinateText.setText(String.format(Locale.US,
                "Drone: %.6f, %.6f  alt %.1fm  heading %.0f deg",
                point.latitude,
                point.longitude,
                altitude,
                headingDegrees));
        mapAddInView.updateDrone(point, altitude, headingDegrees, inside);
    }

    private boolean isDjiTelemetryAvailable() {
        return KeyManager.getInstance() != null && DJISampleApplication.isAircraftConnected();
    }

    private void loadFenceVertices(Context context) {
        fenceVertices.clear();
        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        int count = prefs.getInt(PREFS_KEY_COUNT, -1);

        if (count <= 0) {
            for (double[] vertex : DEFAULT_FENCE_VERTICES) {
                fenceVertices.add(new MapPoint(vertex[0], vertex[1]));
            }
            return;
        }

        for (int i = 0; i < count; i++) {
            double lat = Double.longBitsToDouble(prefs.getLong(PREFS_KEY_LAT + i, 0));
            double lng = Double.longBitsToDouble(prefs.getLong(PREFS_KEY_LNG + i, 0));
            MapPoint point = new MapPoint(lat, lng);
            if (isValidCoordinate(point)) {
                fenceVertices.add(point);
            }
        }

        if (fenceVertices.isEmpty()) {
            for (double[] vertex : DEFAULT_FENCE_VERTICES) {
                fenceVertices.add(new MapPoint(vertex[0], vertex[1]));
            }
        }
    }

    private void loadVirtualStickWaypoints(Context context) {
        virtualStickWaypoints.clear();
        virtualStickWaypoints.addAll(MissionMapDataStore.loadVirtualStickMapPoints(context));
    }

    private void rebuildMockRoute() {
        mockRoute.clear();
        if (!virtualStickWaypoints.isEmpty()) {
            mockRoute.addAll(virtualStickWaypoints);
            if (virtualStickWaypoints.size() == 1 && fenceVertices.size() >= 3) {
                mockRoute.add(centroid(fenceVertices));
            }
            return;
        }
        if (fenceVertices.size() < 3) {
            return;
        }

        MapPoint center = centroid(fenceVertices);
        mockRoute.add(center);
        for (MapPoint vertex : fenceVertices) {
            mockRoute.add(interpolate(center, vertex, 0.55));
        }
        mockRoute.add(projectBeyond(center, fenceVertices.get(0), 1.45));
        mockRoute.add(center);
    }

    private MapPoint makeDefaultDropPoint() {
        if (fenceVertices.isEmpty()) {
            return new MapPoint(DEFAULT_FENCE_VERTICES[0][0], DEFAULT_FENCE_VERTICES[0][1]);
        }
        return interpolate(centroid(fenceVertices), fenceVertices.get(0), 0.72);
    }

    private Button makeButton(Context context, String text) {
        Button button = new Button(context);
        button.setText(text);
        button.setAllCaps(false);
        button.setTextSize(12f);
        return button;
    }

    private LayoutParams weightedParams(boolean marginEnd) {
        LayoutParams params = new LayoutParams(0, LayoutParams.WRAP_CONTENT, 1f);
        if (marginEnd) {
            params.setMarginEnd(dp(6));
        }
        return params;
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private static boolean isValidCoordinate(MapPoint point) {
        return point.latitude >= -90.0
                && point.latitude <= 90.0
                && point.longitude >= -180.0
                && point.longitude <= 180.0
                && !(point.latitude == 0.0 && point.longitude == 0.0);
    }

    private static MapPoint centroid(List<MapPoint> points) {
        double lat = 0.0;
        double lng = 0.0;
        for (MapPoint point : points) {
            lat += point.latitude;
            lng += point.longitude;
        }
        return new MapPoint(lat / points.size(), lng / points.size());
    }

    private static MapPoint interpolate(MapPoint start, MapPoint end, double amount) {
        return new MapPoint(
                start.latitude + (end.latitude - start.latitude) * amount,
                start.longitude + (end.longitude - start.longitude) * amount);
    }

    private static MapPoint projectBeyond(MapPoint origin, MapPoint target, double scale) {
        return new MapPoint(
                origin.latitude + (target.latitude - origin.latitude) * scale,
                origin.longitude + (target.longitude - origin.longitude) * scale);
    }

    private static boolean isPointInPolygon(MapPoint point, List<MapPoint> polygon) {
        int n = polygon.size();
        if (n < 3) {
            return false;
        }

        boolean inside = false;
        int j = n - 1;
        for (int i = 0; i < n; i++) {
            double xi = polygon.get(i).latitude;
            double yi = polygon.get(i).longitude;
            double xj = polygon.get(j).latitude;
            double yj = polygon.get(j).longitude;

            boolean intersect = ((yi > point.longitude) != (yj > point.longitude))
                    && (point.latitude < (xj - xi) * (point.longitude - yi) / (yj - yi) + xi);
            if (intersect) {
                inside = !inside;
            }
            j = i;
        }
        return inside;
    }

    private static double bearingDegrees(MapPoint start, MapPoint end) {
        double lat1 = Math.toRadians(start.latitude);
        double lat2 = Math.toRadians(end.latitude);
        double lngDelta = Math.toRadians(end.longitude - start.longitude);
        double y = Math.sin(lngDelta) * Math.cos(lat2);
        double x = Math.cos(lat1) * Math.sin(lat2)
                - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lngDelta);
        return (Math.toDegrees(Math.atan2(y, x)) + 360.0) % 360.0;
    }
}
