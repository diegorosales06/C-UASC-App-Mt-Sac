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
 *   - Owns and exposes the List<double[]> of waypoints ({lat, lng, altMeters}).
 *   - Persists waypoints to SharedPreferences so they survive app restarts.
 *   - On first launch, migrates any existing data from MissionMapDataStore's
 *     "mission_map_prefs" Long-bits format into this store's "vsw_waypoint_prefs"
 *     JSON format so no previously saved waypoints are lost during the transition.
 *   - If no saved or legacy data exists, loads 7 default CSULA field waypoints.
 *   - Parses QR code payloads (JSON object, JSON array, plain-text CSV).
 *   - Validates all incoming coordinate data before it enters the list.
 *
 * Load priority on construction:
 *   1. "vsw_waypoint_prefs" (JSON format)        — use if found
 *   2. "mission_map_prefs"  (MissionMapDataStore) — migrate if found
 *   3. loadDefaults()                             — 7 CSULA waypoints
 *
 * No UI dependencies. No DJI SDK dependencies.
 * Used by: VirtualStickWaypointView, WaypointMissionController, MapTrackingView.
 */
public class WaypointStore {

    // ── New SharedPreferences (JSON format) ───────────────────────────────────
    private static final String PREFS_NAME    = "vsw_waypoint_prefs";
    private static final String KEY_WAYPOINTS = "waypoints_json";

    // ── Legacy SharedPreferences keys (MissionMapDataStore format) ────────────
    // Read-only — used only during one-time migration. Never written by this class.
    private static final String LEGACY_PREFS_NAME = "mission_map_prefs";
    private static final String LEGACY_KEY_COUNT  = "vs_waypoint_count";
    private static final String LEGACY_KEY_LAT    = "vs_waypoint_lat_";
    private static final String LEGACY_KEY_LNG    = "vs_waypoint_lng_";
    private static final String LEGACY_KEY_ALT    = "vs_waypoint_alt_";

    // ── State ─────────────────────────────────────────────────────────────────
    private final List<double[]> waypointList = new ArrayList<>();
    private final Context context;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Creates a WaypointStore and immediately loads persisted waypoints.
     *
     * Load order:
     *   1. Try "vsw_waypoint_prefs" (this store's JSON format).
     *   2. Try migrating from "mission_map_prefs" (MissionMapDataStore legacy format).
     *   3. Fall back to loadDefaults() — 7 CSULA waypoints.
     *
     * @param context Application or Activity context.
     */
    public WaypointStore(Context context) {
        this.context = context.getApplicationContext();
        if (!loadFromPrefs()) {
            if (!migrateFromLegacyPrefs()) {
                loadDefaults();
            }
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
    // Persistence — new JSON format
    // =========================================================================

    /**
     * Serialises the waypoint list to a JSON array and writes to SharedPreferences.
     *
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
            // Serialisation failure — not fatal, waypoints remain in memory.
        }
    }

    /**
     * Loads waypoints from "vsw_waypoint_prefs" (JSON format).
     *
     * @return true if at least one waypoint loaded successfully.
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
            return false;
        }
    }

    // =========================================================================
    // Migration — from MissionMapDataStore's Long-bits format
    // =========================================================================

    /**
     * One-time migration from MissionMapDataStore's "mission_map_prefs" format.
     *
     * MissionMapDataStore encodes doubles as raw Long bits:
     *   Double.longBitsToDouble(prefs.getLong("vs_waypoint_lat_0", 0L))
     *
     * Reads those values, imports them into WaypointStore's JSON format, and
     * persists. Does NOT delete the legacy prefs — MissionMapDataStore may still
     * be used by other parts of the app.
     *
     * After this runs once, "vsw_waypoint_prefs" will be populated and
     * loadFromPrefs() will succeed on all future launches, so this method
     * will never be called again.
     *
     * @return true if at least one waypoint was migrated successfully.
     */
    private boolean migrateFromLegacyPrefs() {
        SharedPreferences legacyPrefs = context.getSharedPreferences(
                LEGACY_PREFS_NAME, Context.MODE_PRIVATE);
        int count = legacyPrefs.getInt(LEGACY_KEY_COUNT, -1);
        if (count <= 0) return false;

        List<double[]> migrated = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double lat  = Double.longBitsToDouble(
                    legacyPrefs.getLong(LEGACY_KEY_LAT + i, 0L));
            double lng  = Double.longBitsToDouble(
                    legacyPrefs.getLong(LEGACY_KEY_LNG + i, 0L));
            float  alt  = (float) Double.longBitsToDouble(
                    legacyPrefs.getLong(LEGACY_KEY_ALT + i,
                            Double.doubleToRawLongBits(10.0)));
            try {
                migrated.add(validateWaypoint(lat, lng, alt));
            } catch (IllegalArgumentException ignored) {
                // Skip any corrupt entries from the legacy store.
            }
        }

        if (migrated.isEmpty()) return false;

        waypointList.addAll(migrated);
        saveToPrefs(); // write into new JSON format immediately
        return true;
    }

    // =========================================================================
    // Default waypoints
    // =========================================================================

    /**
     * Loads 7 default CSULA competition field waypoints.
     * Only called on first-ever launch when no saved or legacy data exists.
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

    // =========================================================================
    // QR Parsing — public entry point
    // =========================================================================

    /**
     * Parses a raw QR code payload string into a list of waypoints.
     *
     * Supported formats:
     *   1. JSON object  — {"type":"WAYPOINTS","points":[{lat,lng,alt},...]}
     *   2. JSON array   — [{lat,lng,alt},...]
     *   3. Plain text   — one "lat,lng[,alt]" pair per line
     *
     * Returns the parsed list WITHOUT adding it to the store. Caller should
     * show a confirm dialog then call appendWaypoints() or replaceWaypoints().
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

    // =========================================================================
    // QR Parsing — private helpers
    // =========================================================================

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
                || t.equals("VS_WAYPOINTS")) return;
        if (t.equals("PACKAGE_DROP") || t.equals("DROP_TARGET")) {
            throw new IllegalArgumentException(
                    "This QR contains a package drop target, not waypoints.");
        }
        if (t.equals("TIME_TRIAL") || t.equals("CIRCUIT_TIME_TRIAL")) {
            throw new IllegalArgumentException(
                    "This QR contains a time trial route, not waypoints.");
        }
        throw new IllegalArgumentException("This QR is not a waypoint QR.");
    }

    /** Validates and returns the waypoint array, or throws IllegalArgumentException. */
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

    // =========================================================================
    // Misc helpers
    // =========================================================================

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