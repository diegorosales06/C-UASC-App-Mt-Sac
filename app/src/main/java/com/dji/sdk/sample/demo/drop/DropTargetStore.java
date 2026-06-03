package com.dji.sdk.sample.demo.drop;

import android.content.Context;
import android.content.SharedPreferences;

public final class DropTargetStore {

    private static final String PREFS_NAME = "payload_drop_target_prefs";
    private static final String KEY_LAT = "drop_lat";
    private static final String KEY_LNG = "drop_lng";
    private static final String KEY_ALT = "drop_alt";

    public static final double DEFAULT_DROP_LAT = 34.0272525;
    public static final double DEFAULT_DROP_LNG = -117.8511957;
    public static final float DEFAULT_DROP_ALT = 8.0f;

    private DropTargetStore() {
    }

    public static DropTarget load(Context context) {
        SharedPreferences prefs = context.getApplicationContext()
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        double lat = Double.longBitsToDouble(
                prefs.getLong(KEY_LAT, Double.doubleToRawLongBits(DEFAULT_DROP_LAT)));
        double lng = Double.longBitsToDouble(
                prefs.getLong(KEY_LNG, Double.doubleToRawLongBits(DEFAULT_DROP_LNG)));
        float alt = Float.intBitsToFloat(
                prefs.getInt(KEY_ALT, Float.floatToRawIntBits(DEFAULT_DROP_ALT)));

        if (!isValid(lat, lng) || Float.isNaN(alt) || Float.isInfinite(alt) || alt < 0f) {
            return new DropTarget(DEFAULT_DROP_LAT, DEFAULT_DROP_LNG, DEFAULT_DROP_ALT);
        }
        return new DropTarget(lat, lng, alt);
    }

    public static void save(Context context, double lat, double lng, float alt) {
        context.getApplicationContext()
                .getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit()
                .putLong(KEY_LAT, Double.doubleToRawLongBits(lat))
                .putLong(KEY_LNG, Double.doubleToRawLongBits(lng))
                .putInt(KEY_ALT, Float.floatToRawIntBits(alt))
                .apply();
    }

    private static boolean isValid(double lat, double lng) {
        return lat >= -90.0
                && lat <= 90.0
                && lng >= -180.0
                && lng <= 180.0;
    }

    public static final class DropTarget {
        public final double latitude;
        public final double longitude;
        public final float altitudeMeters;

        private DropTarget(double latitude, double longitude, float altitudeMeters) {
            this.latitude = latitude;
            this.longitude = longitude;
            this.altitudeMeters = altitudeMeters;
        }
    }
}
