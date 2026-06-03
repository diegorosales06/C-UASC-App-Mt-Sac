package com.dji.sdk.sample.demo.map;

import android.content.Context;
import android.content.SharedPreferences;

import java.util.ArrayList;
import java.util.List;

public final class MissionMapDataStore {

    private static final String PREFS_NAME = "mission_map_prefs";
    private static final String VS_COUNT = "vs_waypoint_count";
    private static final String VS_LAT = "vs_waypoint_lat_";
    private static final String VS_LNG = "vs_waypoint_lng_";
    private static final String VS_ALT = "vs_waypoint_alt_";

    private static final double[][] DEFAULT_VS_WAYPOINTS = {
            {34.027154, -117.851337, 8.0},
            {34.027254, -117.851136, 8.0},
            {34.027455, -117.851226, 8.0}
    };

    private MissionMapDataStore() {
    }

    public static List<double[]> loadVirtualStickWaypoints(Context context) {
        SharedPreferences prefs = context.getApplicationContext()
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        int count = prefs.getInt(VS_COUNT, -1);
        List<double[]> waypoints = new ArrayList<>();

        if (count < 0) {
            for (double[] waypoint : DEFAULT_VS_WAYPOINTS) {
                waypoints.add(new double[]{waypoint[0], waypoint[1], waypoint[2]});
            }
            return waypoints;
        }

        for (int i = 0; i < count; i++) {
            double lat = Double.longBitsToDouble(prefs.getLong(VS_LAT + i, 0L));
            double lng = Double.longBitsToDouble(prefs.getLong(VS_LNG + i, 0L));
            double alt = Double.longBitsToDouble(prefs.getLong(VS_ALT + i, Double.doubleToRawLongBits(10.0)));
            if (isValidCoordinate(lat, lng)) {
                waypoints.add(new double[]{lat, lng, alt});
            }
        }

        return waypoints;
    }

    public static List<MapPoint> loadVirtualStickMapPoints(Context context) {
        List<double[]> waypoints = loadVirtualStickWaypoints(context);
        List<MapPoint> points = new ArrayList<>();
        for (double[] waypoint : waypoints) {
            points.add(new MapPoint(waypoint[0], waypoint[1]));
        }
        return points;
    }

    public static void saveVirtualStickWaypoints(Context context, List<double[]> waypoints) {
        SharedPreferences.Editor editor = context.getApplicationContext()
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit();
        editor.putInt(VS_COUNT, waypoints.size());
        for (int i = 0; i < waypoints.size(); i++) {
            double[] waypoint = waypoints.get(i);
            editor.putLong(VS_LAT + i, Double.doubleToRawLongBits(waypoint[0]));
            editor.putLong(VS_LNG + i, Double.doubleToRawLongBits(waypoint[1]));
            editor.putLong(VS_ALT + i, Double.doubleToRawLongBits(waypoint[2]));
        }
        editor.apply();
    }

    private static boolean isValidCoordinate(double lat, double lng) {
        return lat >= -90.0
                && lat <= 90.0
                && lng >= -180.0
                && lng <= 180.0
                && !(lat == 0.0 && lng == 0.0);
    }
}
