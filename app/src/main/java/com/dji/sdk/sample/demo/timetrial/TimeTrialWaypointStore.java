package com.dji.sdk.sample.demo.timetrial;

import android.content.Context;
import android.content.SharedPreferences;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

/**
 * TimeTrialWaypointStore
 *
 * Independent waypoint store for the time trial system. Completely separate
 * from WaypointStore — uses its own SharedPreferences key so time trial
 * waypoints and mission waypoints never overwrite each other.
 *
 * Identical functionality to WaypointStore:
 *   - Owns and exposes List<double[]> of waypoints ({lat, lng, altMeters}).
 *   - Persists to "vsw_timetrial_waypoint_prefs" in JSON format.
 *   - On first launch falls back to 7 default CSULA field waypoints.
 *   - Full QR payload parsing (JSON object, JSON array, plain text).
 *   - Validates all incoming coordinate data.
 *
 * No UI dependencies. No DJI SDK dependencies.
 */
public class TimeTrialWaypointStore {

    // ── SharedPreferences — completely separate from WaypointStore ────────────
    private static final String PREFS_NAME    = "vsw_timetrial_waypoint_prefs";
    private static final String KEY_WAYPOINTS = "timetrial_waypoints_json";

    // ── State ─────────────────────────────────────────────────────────────────
    private final List<double[]> waypointList = new ArrayList<>();
    private final Context context;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Creates a TimeTrialWaypointStore and immediately loads persisted waypoints.
     * Falls back to default CSULA waypoints on first launch.
     *
     * @param context Application or Activity context.
     */
    public TimeTrialWaypointStore(Context context) {
        this.context = context.getApplicationContext();
        if (!loadFromPrefs()) {
            loadDefaults();
        }
    }

    // =========================================================================
    // Public list accessors
    // =========================================================================

    /** Returns an unmodifiable view of the current waypoint list. */
    public List<double[]> getWaypoints() {
        return java.util.Collections.unmodifiableList(waypointList);
    }

    /** Returns the waypoint at the given index, or null if out of range. */
    public double[] getWaypoint(int index) {
        if (index < 0 || index >= waypointList.size()) return null;
        return waypointList.get(index);
    }

    /** Returns the number of waypoints currently stored. */
    public int size() {
        return waypointList.size();
    }

    /** Returns true if there are no waypoints. */
    public boolean isEmpty() {
        return waypointList.isEmpty();
    }

    // =========================================================================
    // Public mutators — each one persists automatically
    // =========================================================================

    /**
     * Appends a validated waypoint and persists immediately.
     *
     * @throws IllegalArgumentException if coordinates are out of valid range.
     */
    public void addWaypoint(double lat, double lng, float alt) {
        waypointList.add(validateWaypoint(lat, lng, alt));
        saveToPrefs();
    }

    /**
     * Replaces the waypoint at the given index and persists.
     *
     * @throws IllegalArgumentException if index out of range or coordinates invalid.
     */
    public void editWaypoint(int index, double lat, double lng, float alt) {
        if (index < 0 || index >= waypointList.size()) {
            throw new IllegalArgumentException("Waypoint index out of range.");
        }
        waypointList.set(index, validateWaypoint(lat, lng, alt));
        saveToPrefs();
    }

    /**
     * Removes the waypoint at the given index and persists.
     *
     * @throws IllegalArgumentException if index is out of range.
     */
    public void removeWaypoint(int index) {
        if (index < 0 || index >= waypointList.size()) {
            throw new IllegalArgumentException("Waypoint index out of range.");
        }
        waypointList.remove(index);
        saveToPrefs();
    }

    /** Clears all waypoints and persists. */
    public void clearWaypoints() {
        waypointList.clear();
        saveToPrefs();
    }

    /**
     * Appends all waypoints from the given list and persists.
     * Used for QR "Append" action.
     */
    public void appendWaypoints(List<double[]> incoming) {
        waypointList.addAll(copyWaypoints(incoming));
        saveToPrefs();
    }

    /**
     * Replaces the entire waypoint list and persists.
     * Used for QR "Replace" action.
     */
    public void replaceWaypoints(List<double[]> incoming) {
        waypointList.clear();
        waypointList.addAll(copyWaypoints(incoming));
        saveToPrefs();
    }

    // =========================================================================
    // Persistence
    // =========================================================================

    /**
     * Serialises the waypoint list to JSON and writes to SharedPreferences.
     * Format: [{"lat":34.048510,"lng":-117.837831,"alt":3.0}, ...]
     */
    public void saveToPrefs() {
        try {
            JSONArray array = new JSONArray();
            for (double[] wp : waypointList) {
                JSONObject obj = new JSONObject();
                obj.put("lat", wp[0]);
                obj.put("lng", wp[1]);
                obj.put("alt", wp[2]);
                array.put(obj);
            }
            context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                    .edit()
                    .putString(KEY_WAYPOINTS, array.toString())
                    .apply();
        } catch (JSONException e) {
            // Not fatal — waypoints remain in memory.
        }
    }

    private boolean loadFromPrefs() {
        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        String json = prefs.getString(KEY_WAYPOINTS, null);
        if (json == null || json.isEmpty()) return false;
        try {
            JSONArray array = new JSONArray(json);
            List<double[]> loaded = new ArrayList<>();
            for (int i = 0; i < array.length(); i++) {
                JSONObject obj = array.getJSONObject(i);
                double lat = obj.getDouble("lat");
                double lng = obj.getDouble("lng");
                float  alt = (float) obj.getDouble("alt");
                loaded.add(validateWaypoint(lat, lng, alt));
            }
            if (loaded.isEmpty()) return false;
            waypointList.addAll(loaded);
            return true;
        } catch (JSONException | IllegalArgumentException e) {
            return false;
        }
    }

    // =========================================================================
    // Default waypoints
    // =========================================================================

    /**
     * Loads 7 default CSULA competition field waypoints.
     * Only called on first-ever launch when no saved data exists.
     */
    private void loadDefaults() {
        waypointList.clear();
        waypointList.add(new double[]{34.048510, -117.837831, 3});
        waypointList.add(new double[]{34.048414, -117.837468, 3});
        waypointList.add(new double[]{34.048140, -117.837227, 3});
        waypointList.add(new double[]{34.048121, -117.837787, 3});
        waypointList.add(new double[]{34.047934, -117.837956, 3});
        waypointList.add(new double[]{34.047717, -117.837637, 3});
        waypointList.add(new double[]{34.047729, -117.837246, 3});
        saveToPrefs();
    }

    // =========================================================================
    // QR Parsing
    // =========================================================================

    /**
     * Parses a raw QR code payload into a list of waypoints.
     * Supports JSON object, JSON array, and plain text formats.
     * Returns the list WITHOUT adding it to the store — caller confirms first.
     *
     * @throws IllegalArgumentException if payload is empty, malformed, or wrong type.
     */
    public List<double[]> parseWaypointQrPayload(String payload) {
        if (payload == null || payload.trim().isEmpty()) {
            throw new IllegalArgumentException("QR code was empty.");
        }
        String trimmed = payload.trim();
        try {
            if (trimmed.startsWith("{")) return parseWaypointJson(new JSONObject(trimmed));
            if (trimmed.startsWith("[")) return parseWaypointJsonArray(new JSONArray(trimmed));
        } catch (JSONException e) {
            throw new IllegalArgumentException("QR JSON was not valid.");
        }
        return parseWaypointText(trimmed);
    }

    private List<double[]> parseWaypointJson(JSONObject root) throws JSONException {
        String type = root.optString("type", root.optString("mission", ""));
        validateWaypointQrType(type);
        JSONArray points = root.optJSONArray("points");
        if (points == null) points = root.optJSONArray("waypoints");
        if (points != null) return parseWaypointJsonArray(points);
        List<double[]> result = new ArrayList<>();
        result.add(parseWaypointJsonObject(root));
        return result;
    }

    private List<double[]> parseWaypointJsonArray(JSONArray points) throws JSONException {
        List<double[]> result = new ArrayList<>();
        for (int i = 0; i < points.length(); i++) {
            Object value = points.get(i);
            if (!(value instanceof JSONObject)) {
                throw new IllegalArgumentException("Waypoint JSON points must be objects.");
            }
            result.add(parseWaypointJsonObject((JSONObject) value));
        }
        if (result.isEmpty()) throw new IllegalArgumentException("QR code did not contain waypoints.");
        return result;
    }

    private double[] parseWaypointJsonObject(JSONObject point) {
        double lat = readRequiredJsonDouble(point, "lat", "latitude");
        double lng = readRequiredJsonDouble(point, "lng", "lon", "longitude");
        float  alt = (float) readOptionalJsonDouble(point, 10.0, "alt", "altitude");
        return validateWaypoint(lat, lng, alt);
    }

    private List<double[]> parseWaypointText(String payload) {
        List<double[]> result = new ArrayList<>();
        String[] lines = payload.split("\\r?\\n");
        boolean sawDataLine = false;
        for (String rawLine : lines) {
            String line = rawLine.trim();
            if (line.isEmpty() || line.startsWith("#")) continue;
            if (!sawDataLine && !line.contains(",")) {
                validateWaypointQrType(line);
                sawDataLine = true;
                continue;
            }
            sawDataLine = true;
            result.add(parseWaypointTextLine(line));
        }
        if (result.isEmpty()) throw new IllegalArgumentException("QR code did not contain waypoints.");
        return result;
    }

    private double[] parseWaypointTextLine(String line) {
        String[] columns = line.split(",");
        for (int i = 0; i < columns.length; i++) columns[i] = columns[i].trim();
        if (columns.length < 2) {
            throw new IllegalArgumentException("Waypoint QR lines need latitude and longitude.");
        }
        int latIndex, lngIndex, altIndex;
        if (isNumeric(columns[0])) {
            latIndex = 0; lngIndex = 1; altIndex = 2;
        } else if (columns.length >= 4 && !isNumeric(columns[1])) {
            validateWaypointQrType(columns[1]);
            latIndex = 2; lngIndex = 3; altIndex = 4;
        } else {
            latIndex = 1; lngIndex = 2; altIndex = 3;
        }
        if (columns.length <= lngIndex) {
            throw new IllegalArgumentException("Waypoint QR lines need latitude and longitude.");
        }
        try {
            double lat = Double.parseDouble(columns[latIndex]);
            double lng = Double.parseDouble(columns[lngIndex]);
            float  alt = (columns.length > altIndex && !columns[altIndex].isEmpty())
                    ? Float.parseFloat(columns[altIndex]) : 10.0f;
            return validateWaypoint(lat, lng, alt);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("Waypoint QR contains invalid coordinates.");
        }
    }

    // =========================================================================
    // Validation
    // =========================================================================

    private void validateWaypointQrType(String type) {
        String t = type == null ? "" : type.trim().toUpperCase();
        if (t.isEmpty() || t.equals("WAYPOINTS") || t.equals("WAYPOINT")
                || t.equals("VS_WAYPOINTS") || t.equals("TIME_TRIAL")
                || t.equals("CIRCUIT_TIME_TRIAL")) return;
        if (t.equals("PACKAGE_DROP") || t.equals("DROP_TARGET")) {
            throw new IllegalArgumentException(
                    "This QR contains a package drop target, not waypoints.");
        }
        throw new IllegalArgumentException("This QR is not a waypoint QR.");
    }

    public static double[] validateWaypoint(double lat, double lng, float alt) {
        if (Double.isNaN(lat) || Double.isNaN(lng) || Float.isNaN(alt)
                || Double.isInfinite(lat) || Double.isInfinite(lng)
                || Float.isInfinite(alt)) {
            throw new IllegalArgumentException("Coordinates must be finite numbers.");
        }
        if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
            throw new IllegalArgumentException("Coordinates out of valid GPS range.");
        }
        if (alt < 0) {
            throw new IllegalArgumentException("Altitude must be 0m or higher.");
        }
        return new double[]{lat, lng, alt};
    }

    // =========================================================================
    // JSON helpers
    // =========================================================================

    private double readRequiredJsonDouble(JSONObject object, String... keys) {
        for (String key : keys) {
            if (object.has(key)) return object.optDouble(key, Double.NaN);
        }
        throw new IllegalArgumentException("Waypoint JSON is missing " + keys[0] + ".");
    }

    private double readOptionalJsonDouble(JSONObject object, double defaultValue,
                                          String... keys) {
        for (String key : keys) {
            if (object.has(key)) return object.optDouble(key, defaultValue);
        }
        return defaultValue;
    }

    private boolean isNumeric(String value) {
        if (value == null || value.trim().isEmpty()) return false;
        try { Double.parseDouble(value.trim()); return true; }
        catch (NumberFormatException e) { return false; }
    }

    private List<double[]> copyWaypoints(List<double[]> source) {
        List<double[]> copy = new ArrayList<>();
        for (double[] wp : source) copy.add(new double[]{wp[0], wp[1], wp[2]});
        return copy;
    }
}