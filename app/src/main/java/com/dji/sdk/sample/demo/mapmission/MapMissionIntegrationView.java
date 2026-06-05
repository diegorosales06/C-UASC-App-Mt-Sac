package com.dji.sdk.sample.demo.mapmission;

import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.os.Handler;
import android.os.Looper;
import android.util.AttributeSet;
import android.view.View;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import androidx.annotation.NonNull;

import com.dji.sdk.sample.R;
import com.dji.sdk.sample.demo.drop.DropTargetStore;
import com.dji.sdk.sample.demo.drop.PayloadDropMissionView;
import com.dji.sdk.sample.demo.geofencing.GeofencingView;
import com.dji.sdk.sample.demo.map.MapAddInView;
import com.dji.sdk.sample.demo.map.MapPoint;
import com.dji.sdk.sample.demo.timetrial.TimeTrialView;
import com.dji.sdk.sample.demo.timetrial.TimeTrialWaypointStore;
import com.dji.sdk.sample.demo.virtualstickwaypoint.VirtualStickWaypointView;
import com.dji.sdk.sample.demo.virtualstickwaypoint.WaypointStore;
import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.utils.FlightControllerStateDispatcher;
import com.dji.sdk.sample.internal.utils.ReturnHomeCommand;
import com.dji.sdk.sample.internal.view.PresentableView;

import java.util.ArrayList;
import java.util.List;

import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.model.LocationCoordinate2D;
import dji.keysdk.DJIKey;
import dji.keysdk.FlightControllerKey;
import dji.keysdk.KeyManager;
import dji.keysdk.callback.KeyListener;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;

/**
 * Single field-work screen for mission workflows that share the map workflow:
 * geofence/map points, virtual-stick waypoints, payload-drop target, and
 * circuit time-trial gates.
 */
public class MapMissionIntegrationView extends LinearLayout implements PresentableView {

    private static final String GEOFENCE_PREFS_NAME = "geofencing_prefs";
    private static final String GEOFENCE_PREFS_KEY_COUNT = "waypoint_count";
    private static final String GEOFENCE_PREFS_KEY_LAT = "waypoint_lat_";
    private static final String GEOFENCE_PREFS_KEY_LNG = "waypoint_lng_";

    private static final int MAP_HEIGHT_DP = 450;
    private static final long MAP_REFRESH_MS = 2500L;

    private static final double[][] DEFAULT_FENCE_VERTICES = {
            {34.04618635227991, -117.84552701364355},
            {34.04632498120861, -117.84524530311664},
            {34.04658300697332, -117.84539805413424},
            {34.04646801720643, -117.84579969397946}
    };

    private enum MissionMode {
        MAP_FENCE,
        VS_WAYPOINT,
        DROP_WAYPOINT,
        TIME_TRIAL
    }

    private TextView tvTitle;
    private TextView tvModeHint;
    private TextView tvSatelliteCount;
    private MapAddInView mapAddInView;
    private Button btnReloadMap;
    private Button btnSetHome;
    private Button btnReturnHome;
    private Button btnMapFence;
    private Button btnVsWaypoint;
    private Button btnDropWaypoint;
    private Button btnTimeTrial;
    private ScrollView contentScrollView;
    private FrameLayout contentFrame;
    private MissionMode currentMode;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final List<MapPoint> cachedFenceVertices = new ArrayList<>();
    private MapPoint latestAircraftLocation;
    private double latestHeadingDegrees;
    private DJIKey aircraftLocationKey;
    private KeyListener aircraftLocationListener;
    private final FlightControllerStateDispatcher.Listener flightStateListener =
            this::onFlightControllerState;

    private final Runnable mapRefreshRunnable = new Runnable() {
        @Override
        public void run() {
            refreshMapOverlays();
            mainHandler.postDelayed(this, MAP_REFRESH_MS);
        }
    };

    public MapMissionIntegrationView(Context context) {
        super(context);
        init(context);
    }

    public MapMissionIntegrationView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    @Override
    public int getDescription() {
        return R.string.map_mission_integration_title;
    }

    @NonNull
    @Override
    public String getHint() {
        return this.getClass().getSimpleName() + ".java";
    }

    private void init(Context context) {
        setOrientation(VERTICAL);
        setPadding(18, 18, 18, 18);

        tvTitle = new TextView(context);
        tvTitle.setText("Map Mission Integration");
        tvTitle.setTextSize(18f);
        tvTitle.setTextColor(Color.parseColor("#202124"));
        tvTitle.setPadding(0, 0, 0, 6);
        addView(tvTitle);

        tvModeHint = new TextView(context);
        tvModeHint.setTextSize(12f);
        tvModeHint.setTextColor(Color.parseColor("#5F6368"));
        tvModeHint.setPadding(0, 0, 0, 10);
        addView(tvModeHint);

        mapAddInView = new MapAddInView(context);
        addView(mapAddInView, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(MAP_HEIGHT_DP)));

        LinearLayout mapStatusRow = new LinearLayout(context);
        mapStatusRow.setOrientation(HORIZONTAL);
        mapStatusRow.setPadding(0, 8, 0, 4);
        addView(mapStatusRow);

        tvSatelliteCount = new TextView(context);
        tvSatelliteCount.setText("Sat: --");
        tvSatelliteCount.setTextSize(13f);
        tvSatelliteCount.setTextColor(Color.parseColor("#202124"));
        tvSatelliteCount.setBackgroundColor(Color.parseColor("#E8F0FE"));
        tvSatelliteCount.setGravity(android.view.Gravity.CENTER);
        tvSatelliteCount.setPadding(8, 0, 8, 0);
        LinearLayout.LayoutParams satParams = new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.MATCH_PARENT,
                0.35f);
        satParams.setMarginEnd(8);
        mapStatusRow.addView(tvSatelliteCount, satParams);

        btnSetHome = new Button(context);
        btnSetHome.setText("Set Home");
        btnSetHome.setAllCaps(false);
        btnSetHome.setOnClickListener(v -> setCurrentAircraftPositionAsHome());
        LinearLayout.LayoutParams setHomeParams = new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.WRAP_CONTENT,
                0.65f);
        setHomeParams.setMarginEnd(8);
        mapStatusRow.addView(btnSetHome, setHomeParams);

        btnReturnHome = new Button(context);
        btnReturnHome.setText("RTH");
        btnReturnHome.setAllCaps(false);
        btnReturnHome.setOnClickListener(v -> requestReturnToHome());
        mapStatusRow.addView(btnReturnHome, new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.WRAP_CONTENT,
                0.35f));

        LinearLayout mapActionRow = new LinearLayout(context);
        mapActionRow.setOrientation(HORIZONTAL);
        mapActionRow.setPadding(0, 0, 0, 8);
        addView(mapActionRow);

        btnReloadMap = new Button(context);
        btnReloadMap.setText("Reload Map Overlays");
        btnReloadMap.setAllCaps(false);
        btnReloadMap.setOnClickListener(v -> {
            refreshMapOverlays();
            mapAddInView.fitAllMapItems();
            mapAddInView.setStatusText("Leaflet overlays reloaded");
        });
        LinearLayout.LayoutParams reloadParams = new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.WRAP_CONTENT,
                1f);
        reloadParams.setMarginEnd(8);
        mapActionRow.addView(btnReloadMap, reloadParams);

        LinearLayout rowOne = new LinearLayout(context);
        rowOne.setOrientation(HORIZONTAL);
        rowOne.setPadding(0, 0, 0, 6);
        addView(rowOne);

        btnMapFence = buildModeButton(context, "Map / Fence", MissionMode.MAP_FENCE);
        btnVsWaypoint = buildModeButton(context, "VS Waypoints", MissionMode.VS_WAYPOINT);
        rowOne.addView(btnMapFence);
        rowOne.addView(btnVsWaypoint);

        LinearLayout rowTwo = new LinearLayout(context);
        rowTwo.setOrientation(HORIZONTAL);
        rowTwo.setPadding(0, 0, 0, 10);
        addView(rowTwo);

        btnDropWaypoint = buildModeButton(context, "Drop", MissionMode.DROP_WAYPOINT);
        btnTimeTrial = buildModeButton(context, "Time Trial", MissionMode.TIME_TRIAL);
        rowTwo.addView(btnDropWaypoint);
        rowTwo.addView(btnTimeTrial);

        contentScrollView = new ScrollView(context);
        contentScrollView.setFillViewport(true);
        contentScrollView.setLayoutParams(new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1f));
        addView(contentScrollView);

        contentFrame = new FrameLayout(context);
        contentScrollView.addView(contentFrame, new ScrollView.LayoutParams(
                ScrollView.LayoutParams.MATCH_PARENT,
                ScrollView.LayoutParams.WRAP_CONTENT));

        refreshMapOverlays();
        showMode(MissionMode.MAP_FENCE);
    }

    private Button buildModeButton(Context context, String label, MissionMode mode) {
        Button button = new Button(context);
        button.setText(label);
        button.setAllCaps(false);
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.WRAP_CONTENT,
                1f);
        params.setMarginEnd(6);
        button.setLayoutParams(params);
        button.setOnClickListener(v -> showMode(mode));
        return button;
    }

    private void showMode(MissionMode mode) {
        currentMode = mode;
        contentFrame.removeAllViews();
        refreshMapOverlays();
        mapAddInView.fitAllMapItems();

        View child;
        switch (mode) {
            case VS_WAYPOINT:
                child = new VirtualStickWaypointView(getContext());
                tvModeHint.setText("Virtual-stick waypoint mission: add/edit route points, tune controller values, then start after takeoff.");
                break;
            case DROP_WAYPOINT:
                child = new PayloadDropMissionView(getContext());
                tvModeHint.setText("Drop waypoint mission: set the drop target and altitude, then start after takeoff.");
                break;
            case TIME_TRIAL:
                child = new TimeTrialView(getContext());
                tvModeHint.setText("Circuit time trial: add gates, tune speed/control values, then run the timed route.");
                break;
            case MAP_FENCE:
            default:
                child = new GeofencingView(getContext());
                tvModeHint.setText("Map/fence workflow: manage boundary points and start containment monitoring.");
                break;
        }

        contentFrame.addView(child, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.WRAP_CONTENT));
        contentScrollView.post(() -> contentScrollView.scrollTo(0, 0));
        syncModeButtons();
    }

    public void requestReturnToHome() {
        View child = contentFrame == null || contentFrame.getChildCount() == 0
                ? null
                : contentFrame.getChildAt(0);

        if (child instanceof VirtualStickWaypointView) {
            ((VirtualStickWaypointView) child).requestReturnToHome();
            mapAddInView.setStatusText("RTH requested from VS waypoint mission");
            return;
        }
        if (child instanceof PayloadDropMissionView) {
            ((PayloadDropMissionView) child).requestReturnToHome();
            mapAddInView.setStatusText("RTH requested from drop mission");
            return;
        }
        if (child instanceof TimeTrialView) {
            ((TimeTrialView) child).requestReturnToHome();
            mapAddInView.setStatusText("RTH requested from time trial");
            return;
        }

        ReturnHomeCommand.start(message -> post(() -> {
            mapAddInView.setStatusText(message);
            tvModeHint.setText(message);
        }));
    }

    private void setCurrentAircraftPositionAsHome() {
        Aircraft aircraft = DJISampleApplication.getAircraftInstance();
        if (aircraft == null || aircraft.getFlightController() == null) {
            showMapStatus("Flight controller not available.");
            return;
        }

        FlightController flightController = aircraft.getFlightController();
        showMapStatus("Setting home to current drone position...");
        flightController.setHomeLocationUsingAircraftCurrentLocation(error -> {
            if (error == null) {
                showMapStatus("Home set to current drone position.");
            } else {
                showMapStatus("Set home failed: " + error.getDescription());
            }
        });
    }

    private void refreshMapOverlays() {
        if (mapAddInView == null) return;

        cachedFenceVertices.clear();
        cachedFenceVertices.addAll(loadGeofencePoints(getContext()));
        mapAddInView.setGeofence(cachedFenceVertices);
        mapAddInView.setMissionWaypoints(loadVirtualStickWaypointPoints(getContext()));
        mapAddInView.setCircuitPoints(loadTimeTrialGatePoints(getContext()));

        DropTargetStore.DropTarget dropTarget = DropTargetStore.load(getContext());
        mapAddInView.setDropPoint(new MapPoint(dropTarget.latitude, dropTarget.longitude));
    }

    private List<MapPoint> loadGeofencePoints(Context context) {
        SharedPreferences prefs = context.getApplicationContext()
                .getSharedPreferences(GEOFENCE_PREFS_NAME, Context.MODE_PRIVATE);
        int count = prefs.getInt(GEOFENCE_PREFS_KEY_COUNT, -1);
        List<MapPoint> points = new ArrayList<>();

        if (count <= 0) {
            for (double[] vertex : DEFAULT_FENCE_VERTICES) {
                points.add(new MapPoint(vertex[0], vertex[1]));
            }
            return points;
        }

        for (int i = 0; i < count; i++) {
            double lat = Double.longBitsToDouble(prefs.getLong(GEOFENCE_PREFS_KEY_LAT + i, 0L));
            double lng = Double.longBitsToDouble(prefs.getLong(GEOFENCE_PREFS_KEY_LNG + i, 0L));
            if (isValidCoordinate(lat, lng)) {
                points.add(new MapPoint(lat, lng));
            }
        }
        return points;
    }

    private List<MapPoint> loadVirtualStickWaypointPoints(Context context) {
        WaypointStore store = new WaypointStore(context);
        return toMapPoints(store.getWaypoints());
    }

    private List<MapPoint> loadTimeTrialGatePoints(Context context) {
        TimeTrialWaypointStore store = new TimeTrialWaypointStore(context);
        return toMapPoints(store.getWaypoints());
    }

    private List<MapPoint> toMapPoints(List<double[]> waypoints) {
        List<MapPoint> points = new ArrayList<>();
        for (double[] waypoint : waypoints) {
            if (waypoint.length >= 2 && isValidCoordinate(waypoint[0], waypoint[1])) {
                points.add(new MapPoint(waypoint[0], waypoint[1]));
            }
        }
        return points;
    }

    private void startAircraftLocationTelemetry() {
        KeyManager keyManager = KeyManager.getInstance();
        if (keyManager == null || aircraftLocationListener != null) {
            return;
        }

        aircraftLocationKey = FlightControllerKey.create(FlightControllerKey.AIRCRAFT_LOCATION);
        aircraftLocationListener = (oldValue, newValue) -> {
            if (newValue != null) {
                post(() -> handleAircraftLocation(newValue));
            }
        };
        keyManager.addListener(aircraftLocationKey, aircraftLocationListener);

        Object currentValue = keyManager.getValue(aircraftLocationKey);
        if (currentValue != null) {
            handleAircraftLocation(currentValue);
        }
    }

    private void stopAircraftLocationTelemetry() {
        if (aircraftLocationListener != null && KeyManager.getInstance() != null) {
            KeyManager.getInstance().removeListener(aircraftLocationListener);
        }
        aircraftLocationListener = null;
        aircraftLocationKey = null;
    }

    private void startFlightStateTelemetry() {
        Aircraft aircraft = DJISampleApplication.getAircraftInstance();
        if (aircraft == null || aircraft.getFlightController() == null) {
            updateSatelliteCount(null);
            return;
        }
        FlightControllerStateDispatcher.addListener(
                aircraft.getFlightController(),
                flightStateListener);
    }

    private void stopFlightStateTelemetry() {
        FlightControllerStateDispatcher.removeListener(flightStateListener);
    }

    private void onFlightControllerState(FlightControllerState state) {
        updateSatelliteCount(state);
    }

    private void updateSatelliteCount(FlightControllerState state) {
        if (tvSatelliteCount == null) return;
        String label = state == null
                ? "Sat: --"
                : "Sat: " + state.getSatelliteCount();
        post(() -> tvSatelliteCount.setText(label));
    }

    private void handleAircraftLocation(Object value) {
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

        if (point == null || !isValidCoordinate(point.latitude, point.longitude)) {
            return;
        }

        double heading = latestAircraftLocation == null
                ? latestHeadingDegrees
                : bearingDegrees(latestAircraftLocation, point);
        latestAircraftLocation = point;
        latestHeadingDegrees = heading;
        mapAddInView.updateDrone(point, altitude, heading,
                isInsideFence(point, cachedFenceVertices));
    }

    private void syncModeButtons() {
        syncModeButton(btnMapFence, MissionMode.MAP_FENCE);
        syncModeButton(btnVsWaypoint, MissionMode.VS_WAYPOINT);
        syncModeButton(btnDropWaypoint, MissionMode.DROP_WAYPOINT);
        syncModeButton(btnTimeTrial, MissionMode.TIME_TRIAL);
    }

    private void syncModeButton(Button button, MissionMode mode) {
        boolean selected = currentMode == mode;
        button.setEnabled(true);
        button.setSelected(selected);
        button.setTextColor(selected ? Color.WHITE : Color.parseColor("#202124"));
        button.setBackgroundColor(selected
                ? Color.parseColor("#1A73E8")
                : Color.parseColor("#E8EAED"));
    }

    private boolean isInsideFence(MapPoint point, List<MapPoint> polygon) {
        int n = polygon.size();
        if (n < 3) return true;

        boolean inside = false;
        int j = n - 1;
        for (int i = 0; i < n; i++) {
            double xi = polygon.get(i).latitude;
            double yi = polygon.get(i).longitude;
            double xj = polygon.get(j).latitude;
            double yj = polygon.get(j).longitude;
            boolean intersect = ((yi > point.longitude) != (yj > point.longitude))
                    && (point.latitude < (xj - xi) * (point.longitude - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
            j = i;
        }
        return inside;
    }

    private double bearingDegrees(MapPoint from, MapPoint to) {
        double lat1 = Math.toRadians(from.latitude);
        double lat2 = Math.toRadians(to.latitude);
        double dLng = Math.toRadians(to.longitude - from.longitude);
        double y = Math.sin(dLng) * Math.cos(lat2);
        double x = Math.cos(lat1) * Math.sin(lat2)
                - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLng);
        return (Math.toDegrees(Math.atan2(y, x)) + 360.0) % 360.0;
    }

    private boolean isValidCoordinate(double lat, double lng) {
        return lat >= -90.0
                && lat <= 90.0
                && lng >= -180.0
                && lng <= 180.0
                && !(lat == 0.0 && lng == 0.0);
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private void showMapStatus(String message) {
        post(() -> {
            if (mapAddInView != null) {
                mapAddInView.setStatusText(message);
            }
            if (tvModeHint != null) {
                tvModeHint.setText(message);
            }
        });
    }

    @Override
    protected void onAttachedToWindow() {
        super.onAttachedToWindow();
        mapAddInView.onResume();
        refreshMapOverlays();
        startAircraftLocationTelemetry();
        startFlightStateTelemetry();
        mainHandler.removeCallbacks(mapRefreshRunnable);
        mainHandler.postDelayed(mapRefreshRunnable, MAP_REFRESH_MS);
    }

    @Override
    protected void onDetachedFromWindow() {
        mainHandler.removeCallbacks(mapRefreshRunnable);
        stopAircraftLocationTelemetry();
        stopFlightStateTelemetry();
        if (mapAddInView != null) {
            mapAddInView.onPause();
            mapAddInView.onDestroy();
        }
        super.onDetachedFromWindow();
    }
}
