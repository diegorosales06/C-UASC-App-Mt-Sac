package com.dji.sdk.sample.demo.map;

import android.annotation.SuppressLint;
import android.content.Context;
import android.graphics.Color;
import android.os.Build;
import android.view.Gravity;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.annotation.Nullable;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Reusable free map add-in for mission screens.
 *
 * This uses a local Leaflet page inside WebView, OpenStreetMap for streets,
 * and Esri World Imagery for satellite-style aerial tiles.
 */
public class MapAddInView extends FrameLayout {

    private static final String MAP_ASSET_URL = "file:///android_asset/map_add_in.html";

    private final WebView webView;
    private final TextView statusView;
    private final Button mapTypeButton;
    private final Button followButton;
    private final Button fitButton;
    private final Button clearTrailButton;

    private final List<MapPoint> geofencePoints = new ArrayList<>();
    private final List<MapPoint> missionWaypoints = new ArrayList<>();
    private final List<MapPoint> circuitPoints = new ArrayList<>();
    private final List<MapPoint> trailPoints = new ArrayList<>();

    private boolean pageReady;
    private boolean imageryMap = true;
    private boolean followDrone = true;
    private MapPoint latestDronePoint;
    private double latestDroneHeading;
    private float latestDroneAltitude;
    private boolean latestDroneInside = true;
    private MapPoint dropPoint;

    public MapAddInView(Context context) {
        super(context);
        webView = new WebView(context);
        statusView = new TextView(context);
        mapTypeButton = makeButton(context, "Imagery");
        followButton = makeButton(context, "Follow");
        fitButton = makeButton(context, "Fit");
        clearTrailButton = makeButton(context, "Clear");
        init(context);
    }

    @SuppressLint("SetJavaScriptEnabled")
    private void init(Context context) {
        WebSettings settings = webView.getSettings();
        settings.setJavaScriptEnabled(true);
        settings.setDomStorageEnabled(true);
        settings.setDatabaseEnabled(true);
        settings.setLoadWithOverviewMode(true);
        settings.setUseWideViewPort(true);
        settings.setAllowFileAccess(true);
        settings.setAllowContentAccess(true);
        settings.setAllowFileAccessFromFileURLs(true);
        settings.setAllowUniversalAccessFromFileURLs(true);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            settings.setMixedContentMode(WebSettings.MIXED_CONTENT_ALWAYS_ALLOW);
        }

        webView.setWebViewClient(new WebViewClient() {
            @Override
            public void onPageFinished(WebView view, String url) {
                pageReady = true;
                renderAll();
                fitAllMapItems();
            }
        });

        addView(webView, new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.MATCH_PARENT));
        webView.loadUrl(MAP_ASSET_URL);

        statusView.setText("Free map loading...");
        statusView.setTextColor(Color.WHITE);
        statusView.setTextSize(12f);
        statusView.setGravity(Gravity.CENTER);
        statusView.setBackgroundColor(Color.argb(185, 30, 38, 48));
        statusView.setPadding(dp(10), dp(6), dp(10), dp(6));
        LayoutParams statusParams = new LayoutParams(LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT);
        statusParams.gravity = Gravity.TOP | Gravity.START;
        statusParams.setMargins(dp(10), dp(10), dp(10), dp(10));
        addView(statusView, statusParams);

        LinearLayout controls = new LinearLayout(context);
        controls.setOrientation(LinearLayout.HORIZONTAL);
        controls.setGravity(Gravity.CENTER);
        controls.setPadding(dp(6), dp(6), dp(6), dp(6));
        controls.setBackgroundColor(Color.argb(155, 30, 38, 48));

        controls.addView(mapTypeButton, weightedButtonParams());
        controls.addView(followButton, weightedButtonParams());
        controls.addView(fitButton, weightedButtonParams());
        controls.addView(clearTrailButton, weightedButtonParams());

        LayoutParams controlsParams = new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT);
        controlsParams.gravity = Gravity.BOTTOM;
        controlsParams.setMargins(dp(10), dp(10), dp(10), dp(10));
        addView(controls, controlsParams);

        mapTypeButton.setOnClickListener(v -> toggleMapType());
        followButton.setOnClickListener(v -> {
            followDrone = !followDrone;
            updateFollowButton();
            runJs("window.mapAddIn.setFollow(" + followDrone + ");");
        });
        fitButton.setOnClickListener(v -> fitAllMapItems());
        clearTrailButton.setOnClickListener(v -> clearTrail());
    }

    public void onResume() {
        webView.onResume();
    }

    public void onPause() {
        webView.onPause();
    }

    public void onDestroy() {
        webView.destroy();
    }

    public void onLowMemory() {
        // WebView handles its own memory pressure in this use case.
    }

    public void setGeofence(List<MapPoint> points) {
        geofencePoints.clear();
        geofencePoints.addAll(points);
        runJs("window.mapAddIn.setGeofence(" + toJsonArray(geofencePoints) + ");");
    }

    public void setMissionWaypoints(List<MapPoint> points) {
        missionWaypoints.clear();
        missionWaypoints.addAll(points);
        runJs("window.mapAddIn.setMissionWaypoints(" + toJsonArray(missionWaypoints) + ");");
    }

    public void setCircuitPoints(List<MapPoint> points) {
        circuitPoints.clear();
        circuitPoints.addAll(points);
        runJs("window.mapAddIn.setCircuitPoints(" + toJsonArray(circuitPoints) + ");");
    }

    public void setDropPoint(@Nullable MapPoint point) {
        dropPoint = point;
        runJs("window.mapAddIn.setDropPoint(" + toJsonPoint(dropPoint) + ");");
    }

    public void updateDrone(MapPoint point, float altitudeMeters, double headingDegrees, boolean insideGeofence) {
        latestDronePoint = point;
        latestDroneAltitude = altitudeMeters;
        latestDroneHeading = headingDegrees;
        latestDroneInside = insideGeofence;

        if (trailPoints.isEmpty() || distanceMeters(trailPoints.get(trailPoints.size() - 1), point) > 0.5) {
            trailPoints.add(point);
            while (trailPoints.size() > 500) {
                trailPoints.remove(0);
            }
        }

        runJs(String.format(Locale.US,
                "window.mapAddIn.updateDrone(%f,%f,%f,%f,%s);",
                point.latitude,
                point.longitude,
                altitudeMeters,
                headingDegrees,
                insideGeofence));
        updateMapStatus();
    }

    public void clearTrail() {
        trailPoints.clear();
        runJs("window.mapAddIn.clearTrail();");
    }

    public void setStatusText(String text) {
        statusView.setText(text);
    }

    public boolean hasDronePoint() {
        return latestDronePoint != null;
    }

    private void renderAll() {
        runJs("window.mapAddIn.setBaseLayer('" + (imageryMap ? "imagery" : "streets") + "');");
        runJs("window.mapAddIn.setFollow(" + followDrone + ");");
        runJs("window.mapAddIn.setGeofence(" + toJsonArray(geofencePoints) + ");");
        runJs("window.mapAddIn.setMissionWaypoints(" + toJsonArray(missionWaypoints) + ");");
        runJs("window.mapAddIn.setCircuitPoints(" + toJsonArray(circuitPoints) + ");");
        runJs("window.mapAddIn.setDropPoint(" + toJsonPoint(dropPoint) + ");");
        runJs("window.mapAddIn.setTrail(" + toJsonArray(trailPoints) + ");");
        if (latestDronePoint != null) {
            updateDrone(latestDronePoint, latestDroneAltitude, latestDroneHeading, latestDroneInside);
        }
        updateMapStatus();
    }

    private void toggleMapType() {
        imageryMap = !imageryMap;
        mapTypeButton.setText(imageryMap ? "Imagery" : "Streets");
        runJs("window.mapAddIn.setBaseLayer('" + (imageryMap ? "imagery" : "streets") + "');");
    }

    private void updateFollowButton() {
        followButton.setText(followDrone ? "Follow" : "Free");
    }

    private void updateMapStatus() {
        if (latestDronePoint == null) {
            statusView.setText(imageryMap ? "Esri imagery" : "OSM streets");
            return;
        }
        statusView.setText(String.format(Locale.US,
                "Drone %.6f, %.6f | %s",
                latestDronePoint.latitude,
                latestDronePoint.longitude,
                latestDroneInside ? "inside" : "outside"));
    }

    public void fitAllMapItems() {
        runJs("window.mapAddIn.fitAll();");
    }

    private void runJs(String script) {
        if (!pageReady) {
            return;
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.KITKAT) {
            webView.evaluateJavascript(script, null);
        } else {
            webView.loadUrl("javascript:" + script);
        }
    }

    private String toJsonArray(List<MapPoint> points) {
        JSONArray array = new JSONArray();
        for (MapPoint point : points) {
            try {
                array.put(toJsonObject(point));
            } catch (JSONException ignored) {
                // Coordinates are primitive doubles, so JSON construction should not fail.
            }
        }
        return array.toString();
    }

    private String toJsonPoint(@Nullable MapPoint point) {
        if (point == null) {
            return "null";
        }
        try {
            return toJsonObject(point).toString();
        } catch (JSONException ignored) {
            return "null";
        }
    }

    private JSONObject toJsonObject(MapPoint point) throws JSONException {
        JSONObject object = new JSONObject();
        object.put("lat", point.latitude);
        object.put("lng", point.longitude);
        return object;
    }

    private LinearLayout.LayoutParams weightedButtonParams() {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(0, LayoutParams.WRAP_CONTENT, 1f);
        params.setMargins(dp(3), 0, dp(3), 0);
        return params;
    }

    private Button makeButton(Context context, String text) {
        Button button = new Button(context);
        button.setText(text);
        button.setTextSize(11f);
        button.setAllCaps(false);
        button.setMinHeight(dp(36));
        button.setPadding(dp(4), 0, dp(4), 0);
        return button;
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private static double distanceMeters(MapPoint a, MapPoint b) {
        double latMeters = (b.latitude - a.latitude) * 110540.0;
        double lngMeters = (b.longitude - a.longitude)
                * Math.cos(Math.toRadians((a.latitude + b.latitude) / 2.0))
                * 111320.0;
        return Math.hypot(latMeters, lngMeters);
    }
}
