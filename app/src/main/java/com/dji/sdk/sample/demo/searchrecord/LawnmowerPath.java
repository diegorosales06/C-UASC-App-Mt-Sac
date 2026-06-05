package com.dji.sdk.sample.demo.searchrecord;

import com.dji.sdk.sample.demo.virtualstickwaypoint.WaypointNavigationMath;

import java.util.ArrayList;
import java.util.List;

/**
 * Generates a boustrophedon ("lawnmower") sweep over a quadrilateral search box.
 *
 * The four corners must be supplied in perimeter order — e.g. going around the
 * box corner1 → corner2 → corner3 → corner4 (clockwise or counter-clockwise,
 * either works as long as they are adjacent in order).
 *
 * Sweep lines run parallel to the corner1→corner2 edge and step across toward
 * the corner4→corner3 edge, spaced {@code trackSpacingM} metres apart. Every
 * other line is reversed so the resulting path is one continuous back-and-forth
 * route the drone can fly without backtracking.
 *
 *   c1 ────────────► c2      lane 0  (c1 → c2)
 *   │                 │       lane 1  (c2' ← c1')   reversed
 *   c4 ◄──────────── c3      ...
 *
 * Each returned element is {latitude, longitude}. Altitude is applied by the
 * controller, not here. Linear interpolation of lat/lng is accurate enough for
 * a competition-sized box (tens of metres).
 */
public final class LawnmowerPath {

    private LawnmowerPath() {}

    /**
     * @param c1,c2,c3,c4   corners in perimeter order, each {lat, lng}
     * @param trackSpacingM spacing between adjacent sweep lines, metres (min 1m)
     * @return ordered list of {lat, lng} waypoints forming the sweep
     */
    public static List<double[]> generate(double[] c1, double[] c2,
                                          double[] c3, double[] c4,
                                          double trackSpacingM) {
        // The "step" axis runs along the two side edges c1→c4 and c2→c3.
        double sideLen = 0.5 * (
                WaypointNavigationMath.haversineDistance(c1[0], c1[1], c4[0], c4[1])
              + WaypointNavigationMath.haversineDistance(c2[0], c2[1], c3[0], c3[1]));

        double spacing = Math.max(1.0, trackSpacingM);
        int steps = Math.max(1, (int) Math.ceil(sideLen / spacing));
        int lanes = steps + 1;

        List<double[]> path = new ArrayList<>();
        for (int i = 0; i < lanes; i++) {
            double t = (lanes == 1) ? 0.0 : (double) i / (lanes - 1);
            double[] left  = lerp(c1, c4, t); // point on the c1→c4 side
            double[] right = lerp(c2, c3, t); // point on the c2→c3 side
            if (i % 2 == 0) {
                path.add(left);
                path.add(right);
            } else {
                path.add(right);
                path.add(left);
            }
        }
        return path;
    }

    private static double[] lerp(double[] a, double[] b, double t) {
        return new double[] {
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t
        };
    }
}
