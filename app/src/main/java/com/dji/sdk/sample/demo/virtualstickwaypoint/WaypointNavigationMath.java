package com.dji.sdk.sample.demo.virtualstickwaypoint;

/**
 * WaypointNavigationMath
 *
 * Pure static navigation math — zero Android dependencies, zero DJI dependencies.
 * All constants and formulas used by the control loop live here.
 *
 * Because this class has no external dependencies it can be unit-tested directly
 * on a JVM without an emulator or device. If navigation ever behaves unexpectedly
 * in the field, start debugging here.
 *
 * Coordinate conventions used throughout:
 *   Latitude  — degrees, positive = North
 *   Longitude — degrees, positive = East
 *   Bearing   — radians, clockwise from true North, range [0, 2π)
 *   Distance  — meters, great-circle
 *   Altitude  — meters above takeoff point (relative, not AMSL)
 */
public final class WaypointNavigationMath {

    // ---------------------------------------------------------------------------------
    // Physical constants
    // ---------------------------------------------------------------------------------

    /** Mean radius of the Earth in meters (WGS-84 approximation). */
    public static final double EARTH_RADIUS_M = 6_371_000.0;

    // ---------------------------------------------------------------------------------
    // Acceptance thresholds
    // ---------------------------------------------------------------------------------

    /**
     * Horizontal acceptance radius in meters.
     * When haversineDistance to the target waypoint drops below this value the
     * drone is considered to have "reached" the waypoint horizontally.
     * Increase if GPS jitter causes the drone to miss waypoints; decrease for
     * tighter positional accuracy (requires better GPS and lower approach speed).
     */
    public static final double ACCEPTANCE_RADIUS_M = 5.0;

    /**
     * Vertical acceptance threshold in meters.
     * When |targetAlt - currentAlt| drops below this value the drone is considered
     * to be at the correct altitude for the current waypoint.
     */
    public static final double ALTITUDE_ACCEPTANCE_M = 0.5;

    // ---------------------------------------------------------------------------------
    // Offline debug helpers
    // ---------------------------------------------------------------------------------

    /**
     * Offset in meters used to place the simulated drone south-west of the first
     * waypoint when offline debug mode is active.
     */
    public static final double OFFLINE_START_OFFSET_M = 20.0;

    // Prevent instantiation — this is a pure utility class.
    private WaypointNavigationMath() {}

    // ---------------------------------------------------------------------------------
    // Distance
    // ---------------------------------------------------------------------------------

    /**
     * Haversine formula — computes the great-circle distance in meters between
     * two GPS coordinates.
     *
     * Accounts for Earth's curvature. At drone-scale distances (hundreds of meters)
     * a flat-Earth approximation would also work, but Haversine is used here for
     * correctness and because the cost is negligible.
     *
     * @param lat1 Current latitude  (degrees)
     * @param lng1 Current longitude (degrees)
     * @param lat2 Target latitude   (degrees)
     * @param lng2 Target longitude  (degrees)
     * @return Distance in meters between the two points.
     */
    public static double haversineDistance(double lat1, double lng1,
                                           double lat2, double lng2) {
        double dLat = Math.toRadians(lat2 - lat1);
        double dLng = Math.toRadians(lng2 - lng1);

        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(dLng / 2) * Math.sin(dLng / 2);

        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return EARTH_RADIUS_M * c;
    }

    // ---------------------------------------------------------------------------------
    // Bearing
    // ---------------------------------------------------------------------------------

    /**
     * Computes the initial bearing (heading) from point 1 to point 2.
     *
     * Returns a value in radians measured clockwise from true North:
     *   0       = North
     *   π/2     = East
     *   π       = South
     *   3π/2    = West
     *
     * This bearing is used by the control loop to decompose horizontal speed into
     * North (roll) and East (pitch) velocity components via cos() and sin().
     *
     * @param lat1 Current latitude  (degrees)
     * @param lng1 Current longitude (degrees)
     * @param lat2 Target latitude   (degrees)
     * @param lng2 Target longitude  (degrees)
     * @return Bearing in radians in the range [0, 2π).
     */
    public static double bearing(double lat1, double lng1,
                                 double lat2, double lng2) {
        double lat1Rad = Math.toRadians(lat1);
        double lat2Rad = Math.toRadians(lat2);
        double dLng    = Math.toRadians(lng2 - lng1);

        double y = Math.sin(dLng) * Math.cos(lat2Rad);
        double x = Math.cos(lat1Rad) * Math.sin(lat2Rad)
                - Math.sin(lat1Rad) * Math.cos(lat2Rad) * Math.cos(dLng);

        // atan2 returns [-π, π]; normalize to [0, 2π]
        return (Math.atan2(y, x) + 2 * Math.PI) % (2 * Math.PI);
    }

    // ---------------------------------------------------------------------------------
    // Coordinate conversion helpers (used by offline debug simulator)
    // ---------------------------------------------------------------------------------

    /**
     * Converts a north/south displacement in meters to a change in latitude degrees.
     *
     * @param meters Displacement in meters (positive = North).
     * @return Equivalent change in latitude degrees.
     */
    public static double metersToLatitudeDegrees(double meters) {
        return Math.toDegrees(meters / EARTH_RADIUS_M);
    }

    /**
     * Converts an east/west displacement in meters to a change in longitude degrees
     * at the given latitude.
     *
     * @param meters   Displacement in meters (positive = East).
     * @param latitude Current latitude in degrees (needed because longitude degrees
     *                 get shorter as you move away from the equator).
     * @return Equivalent change in longitude degrees.
     */
    public static double metersToLongitudeDegrees(double meters, double latitude) {
        double latitudeRadius = EARTH_RADIUS_M * Math.cos(Math.toRadians(latitude));
        if (Math.abs(latitudeRadius) < 1.0) {
            return 0.0;
        }
        return Math.toDegrees(meters / latitudeRadius);
    }
}