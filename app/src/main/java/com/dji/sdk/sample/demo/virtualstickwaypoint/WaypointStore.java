package com.dji.sdk.sample.demo.virtualstickwaypoint;

import android.content.Context;
import android.content.SharedPreferences;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

/**
 * WaypointStore
 *
 * Single source of truth for the waypoint list.
 *
 * Responsibilities:
 *   - Owns and exposes the List<double[]> of waypoints (each entry is {lat, lng, altMeters}).
 *   - Persists waypoints to SharedPreferences so they survive app restarts.
 *   - On first launch (no saved data) loads a set of default CSULA field waypoints via loadDefaults().
 *   - Parses QR code payloads (JSON object, JSON array, or plain-text CSV) into waypoints.
 *   - Validates all incoming coordinate data before it enters the list.
 *
 * This class has no UI dependencies and no DJI SDK dependencies.
 * It can be instantiated by VirtualStickWaypointView and passed to WaypointMissionController.
 */
public class WaypointStore {

    // ---------------------------------------------------------------------------------
    // SharedPreferences keys
    // ---------------------------------------------------------------------------------

    private static final String PREFS_NAME     = "vsw_waypoint_prefs";
    private static final String KEY_WAYPOINTS  = "waypoints_json";

    // ---------------------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------------------

    private final List<double[]> waypointList = new ArrayList<>();
    private final Context context;

    // ---------------------------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------------------------

    /**
     * Creates a WaypointStore and immediately loads persisted waypoints.
     * If no waypoints have ever been saved, falls back to loadDefaults().
     *
     * @param context Application or Activity context used for SharedPreferences access.
     */
    public WaypointStore(Context context) {
        this.context = context.getApplicationContext();
        if (!loadFromPrefs()) {
            loadDefaults();
        }
    }

    // ---------------------------------------------------------------------------------
    // Public list accessors
    // ---------------------------------------------------------------------------------

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

    // ---------------------------------------------------------------------------------
    // Public mutators
    // ---------------------------------------------------------------------------------

    /**
     * Appends a validated waypoint to the list and persists immediately.
     *
     * @param lat Latitude  in degrees.
     * @param lng Longitude in degrees.
     * @param alt Altitude  in meters above takeoff point.
     * @throws IllegalArgumentException if coordinates are out of valid range.
     */
    public void addWaypoint(double lat, double lng, float alt) {
        waypointList.add(validateWaypoint(lat, lng, alt));
        saveToPrefs();
    }

    /**
     * Replaces the waypoint at waypointIndex with new coordinates and persists.
     *
     * @throws IllegalArgumentException if coordinates are invalid or index is out of range.
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

    /**
     * Removes all waypoints and persists (empty list written to prefs).
     */
    public void clearWaypoints() {
        waypointList.clear();
        saveToPrefs();
    }

    /**
     * Appends all waypoints from the given list to the current list and persists.
     * Used for QR "Append" action.
     *
     * @param incoming List returned by parseWaypointQrPayload().
     */
    public void appendWaypoints(List<double[]> incoming) {
        waypointList.addAll(copyWaypoints(incoming));
        saveToPrefs();
    }

    /**
     * Replaces the entire waypoint list with the given list and persists.
     * Used for QR "Replace" action.
     *
     * @param incoming List returned by parseWaypointQrPayload().
     */
    public void replaceWaypoints(List<double[]> incoming) {
        waypointList.clear();
        waypointList.addAll(copyWaypoints(incoming));
        saveToPrefs();
    }

    // ---------------------------------------------------------------------------------
    // Persistence
    // ---------------------------------------------------------------------------------

    /**
     * Serialises the current waypoint list to a JSON array and writes it to
     * SharedPreferences under KEY_WAYPOINTS.
     *
     * Format:
     * [
     *   {"lat": 34.048510, "lng": -117.837831, "alt": 3.0},
     *   ...
     * ]
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
            SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
            prefs.edit().putString(KEY_WAYPOINTS, array.toString()).apply();
        } catch (JSONException e) {
            // Serialisation failure — not fatal, waypoints remain in memory.
        }
    }

    /**
     * Loads the waypoint list from SharedPreferences.
     *
     * @return true if at least one waypoint was loaded successfully, false if
     *         the preferences entry was absent or could not be parsed.
     */
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
            // Corrupted prefs — fall through to loadDefaults()
            return false;
        }
    }

    /**
     * Populates the waypoint list with the default CSULA competition field waypoints.
     * Only called on first launch when no previously saved waypoints exist.
     * Persists immediately so these become the "last saved" state.
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

    // ---------------------------------------------------------------------------------
    // QR Parsing — public entry point
    // ---------------------------------------------------------------------------------

    /**
     * Parses a raw QR code payload string into a list of waypoints.
     *
     * Supports three formats:
     *   1. JSON object  — { "type": "WAYPOINTS", "points": [ {lat, lng, alt}, ... ] }
     *   2. JSON array   — [ {lat, lng, alt}, ... ]
     *   3. Plain text   — one "lat,lng[,alt]" pair per line, optional type header line
     *
     * The returned list is NOT automatically added to the store. The caller should
     * show a confirmation dialog and then call appendWaypoints() or replaceWaypoints().
     *
     * @param payload Raw string from the QR code scanner.
     * @return Parsed, validated list of waypoints.
     * @throws IllegalArgumentException if the payload is empty, malformed, or wrong type.
     */
    public List<double[]> parseWaypointQrPayload(String payload) {
        if (payload == null || payload.trim().isEmpty()) {
            throw new IllegalArgumentException("QR code was empty.");
        }

        String trimmed = payload.trim();
        try {
            if (trimmed.startsWith("{")) {
                return parseWaypointJson(new JSONObject(trimmed));
            }
            if (trimmed.startsWith("[")) {
                return parseWaypointJsonArray(new JSONArray(trimmed));
            }
        } catch (JSONException e) {
            throw new IllegalArgumentException("QR JSON was not valid.");
        }

        return parseWaypointText(trimmed);
    }

    // ---------------------------------------------------------------------------------
    // QR Parsing — private helpers
    // ---------------------------------------------------------------------------------

    private List<double[]> parseWaypointJson(JSONObject root) throws JSONException {
        String type = root.optString("type", root.optString("mission", ""));
        validateWaypointQrType(type);

        JSONArray points = root.optJSONArray("points");
        if (points == null) points = root.optJSONArray("waypoints");
        if (points != null) return parseWaypointJsonArray(points);

        // Single-waypoint object
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
        if (result.isEmpty()) {
            throw new IllegalArgumentException("QR code did not contain waypoints.");
        }
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

        if (result.isEmpty()) {
            throw new IllegalArgumentException("QR code did not contain waypoints.");
        }
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

    // ---------------------------------------------------------------------------------
    // Validation helpers
    // ---------------------------------------------------------------------------------

    private void validateWaypointQrType(String type) {
        String t = type == null ? "" : type.trim().toUpperCase();
        if (t.isEmpty() || t.equals("WAYPOINTS") || t.equals("WAYPOINT") || t.equals("VS_WAYPOINTS")) return;
        if (t.equals("PACKAGE_DROP") || t.equals("DROP_TARGET")) {
            throw new IllegalArgumentException("This QR contains a package drop target, not waypoints.");
        }
        if (t.equals("TIME_TRIAL") || t.equals("CIRCUIT_TIME_TRIAL")) {
            throw new IllegalArgumentException("This QR contains a time trial route, not waypoints.");
        }
        throw new IllegalArgumentException("This QR is not a waypoint QR.");
    }

    /**
     * Validates a single waypoint's coordinate values.
     *
     * @throws IllegalArgumentException if any value is NaN, infinite, or out of GPS range.
     */
    public static double[] validateWaypoint(double lat, double lng, float alt) {
        if (Double.isNaN(lat) || Double.isNaN(lng) || Float.isNaN(alt)
                || Double.isInfinite(lat) || Double.isInfinite(lng) || Float.isInfinite(alt)) {
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

    // ---------------------------------------------------------------------------------
    // JSON helpers
    // ---------------------------------------------------------------------------------

    private double readRequiredJsonDouble(JSONObject object, String... keys) {
        for (String key : keys) {
            if (object.has(key)) return object.optDouble(key, Double.NaN);
        }
        throw new IllegalArgumentException("Waypoint JSON is missing " + keys[0] + ".");
    }

    private double readOptionalJsonDouble(JSONObject object, double defaultValue, String... keys) {
        for (String key : keys) {
            if (object.has(key)) return object.optDouble(key, defaultValue);
        }
        return defaultValue;
    }

    // ---------------------------------------------------------------------------------
    // Misc helpers
    // ---------------------------------------------------------------------------------

    private boolean isNumeric(String value) {
        if (value == null || value.trim().isEmpty()) return false;
        try {
            Double.parseDouble(value.trim());
            return true;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    private List<double[]> copyWaypoints(List<double[]> source) {
        List<double[]> copy = new ArrayList<>();
        for (double[] wp : source) copy.add(new double[]{wp[0], wp[1], wp[2]});
        return copy;
    }
}