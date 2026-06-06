package com.dji.sdk.sample.demo.searchrecord;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Parses a QR payload into the 4 boundary corners of the search box.
 *
 * Unlike the waypoint QR parser, NO altitude is required or used — the
 * competition only provides the 4 boundary GPS points (latitude/longitude).
 * Any extra columns/keys (including altitude) are ignored.
 *
 * Accepted formats (must contain exactly 4 points):
 *   JSON array of objects : [{"lat":34.1,"lng":-117.8}, … x4]
 *   JSON array of pairs   : [[34.1,-117.8], … x4]
 *   JSON object           : {"type":"BOUNDARY","points":[ …4… ]}
 *                           ("points", "corners", or "boundary" array)
 *   Plain text            : one "lat,lng" per line; "#" comments and a
 *                           non-coordinate header line are ignored
 *
 * @return a {@code double[4][2]} of {lat, lng}, or throws IllegalArgumentException.
 */
public final class BoundaryQrParser {

    private BoundaryQrParser() {}

    public static double[][] parse(String payload) {
        if (payload == null || payload.trim().isEmpty()) {
            throw new IllegalArgumentException("QR code was empty.");
        }
        String trimmed = payload.trim();
        List<double[]> points;
        try {
            if (trimmed.startsWith("{")) {
                points = parseJsonObject(new JSONObject(trimmed));
            } else if (trimmed.startsWith("[")) {
                points = parseJsonArray(new JSONArray(trimmed));
            } else {
                points = parseText(trimmed);
            }
        } catch (JSONException e) {
            throw new IllegalArgumentException("QR JSON was not valid.");
        }
        if (points.size() != 4) {
            throw new IllegalArgumentException(
                    "Boundary QR must contain exactly 4 corner points (found "
                            + points.size() + ").");
        }
        return points.toArray(new double[0][]);
    }

    private static List<double[]> parseJsonObject(JSONObject root) throws JSONException {
        JSONArray pts = root.optJSONArray("points");
        if (pts == null) pts = root.optJSONArray("corners");
        if (pts == null) pts = root.optJSONArray("boundary");
        if (pts == null) {
            throw new IllegalArgumentException(
                    "Boundary QR object needs a \"points\" (or \"corners\") array.");
        }
        return parseJsonArray(pts);
    }

    private static List<double[]> parseJsonArray(JSONArray arr) throws JSONException {
        List<double[]> result = new ArrayList<>();
        for (int i = 0; i < arr.length(); i++) {
            Object value = arr.get(i);
            if (value instanceof JSONObject) {
                JSONObject o = (JSONObject) value;
                result.add(finite(readDouble(o, "lat", "latitude"),
                                  readDouble(o, "lng", "lon", "longitude")));
            } else if (value instanceof JSONArray) {
                JSONArray pair = (JSONArray) value;
                if (pair.length() < 2) {
                    throw new IllegalArgumentException("Each boundary point needs lat and lng.");
                }
                result.add(finite(pair.getDouble(0), pair.getDouble(1)));
            } else {
                throw new IllegalArgumentException(
                        "Boundary points must be objects or [lat,lng] pairs.");
            }
        }
        return result;
    }

    private static List<double[]> parseText(String payload) {
        List<double[]> result = new ArrayList<>();
        for (String raw : payload.split("\\r?\\n")) {
            String line = raw.trim();
            if (line.isEmpty() || line.startsWith("#")) continue;
            if (!line.contains(",")) continue; // skip a non-coordinate header line
            String[] cols = line.split(",");
            if (cols.length < 2) {
                throw new IllegalArgumentException("Boundary lines need latitude and longitude.");
            }
            try {
                result.add(finite(Double.parseDouble(cols[0].trim()),
                                  Double.parseDouble(cols[1].trim())));
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("Boundary QR has invalid coordinates: " + line);
            }
        }
        return result;
    }

    private static double readDouble(JSONObject o, String... keys) {
        for (String k : keys) {
            if (o.has(k)) {
                try {
                    return o.getDouble(k);
                } catch (JSONException ignored) {
                    // try next key
                }
            }
        }
        throw new IllegalArgumentException("Boundary point is missing its latitude/longitude.");
    }

    private static double[] finite(double lat, double lng) {
        if (Double.isNaN(lat) || Double.isNaN(lng)
                || Double.isInfinite(lat) || Double.isInfinite(lng)) {
            throw new IllegalArgumentException("Coordinates must be finite numbers.");
        }
        if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
            throw new IllegalArgumentException(String.format(Locale.US,
                    "Coordinate out of range: (%.6f, %.6f)", lat, lng));
        }
        return new double[]{lat, lng};
    }
}
