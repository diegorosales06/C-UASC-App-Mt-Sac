package com.dji.sdk.sample.demo.geofencing;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * FlightLogger
 *
 * Logs drone latitude, longitude, and inside-fence flag to a CSV file
 * stored on the device's external app files directory.
 *
 * Usage in GeofencingView:
 *
 *   // When fence starts:
 *   flightLogger = new FlightLogger(getContext());
 *   flightLogger.start();
 *
 *   // Inside the state callback on every tick:
 *   flightLogger.log(droneLat, droneLng, inside);
 *
 *   // When fence stops or breach occurs:
 *   flightLogger.stop();
 *
 * Pull the file after flight:
 *   adb pull /sdcard/Android/data/com.dji.sdk.sample/files/flight_log_<timestamp>.csv ~/Desktop/
 *
 * CSV format:
 *   timestamp_ms, latitude, longitude, inside
 *   1712345678901, 33.979028, -118.059903, true
 */
public class FlightLogger {

    private static final String TAG = "FlightLogger";

    // Single background thread for all file I/O — keeps disk writes off the callback thread
    private final ExecutorService ioExecutor = Executors.newSingleThreadExecutor();

    private final Context context;
    private BufferedWriter writer;
    private File           logFile;
    private boolean        running = false;

    public FlightLogger(Context context) {
        this.context = context.getApplicationContext();
    }

    // ---------------------------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------------------------

    /**
     * Creates a new timestamped CSV file and writes the header row.
     * Call this when the fence is activated.
     */
    public void start() {
        if (running) {
            Log.w(TAG, "Logger already running — ignoring start().");
            return;
        }

        String timestamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US)
                .format(new Date());
        String fileName = "flight_log_" + timestamp + ".csv";

        // Use the app's external files directory — no storage permission needed on Android 10+
        File dir = context.getExternalFilesDir(null);
        if (dir == null) {
            Log.e(TAG, "External storage unavailable — logging disabled.");
            return;
        }

        logFile = new File(dir, fileName);

        ioExecutor.execute(() -> {
            try {
                writer = new BufferedWriter(new FileWriter(logFile, false)); // false = overwrite
                writer.write("timestamp_ms,latitude,longitude,inside");
                writer.newLine();
                writer.flush();
                running = true;
                Log.d(TAG, "Flight log started: " + logFile.getAbsolutePath());
            } catch (IOException e) {
                Log.e(TAG, "Failed to create log file: " + e.getMessage());
            }
        });
    }

    /**
     * Appends one row to the CSV.
     * Safe to call from any thread at 10Hz — all I/O is dispatched to the background thread.
     *
     * @param lat    drone latitude
     * @param lng    drone longitude
     * @param inside true if drone is inside the geofence polygon
     */
    public void log(double lat, double lng, boolean inside) {
        if (!running) return;

        long now = System.currentTimeMillis();

        ioExecutor.execute(() -> {
            try {
                if (writer != null) {
                    writer.write(now + "," + lat + "," + lng + "," + inside);
                    writer.newLine();
                    // No flush here on every line — BufferedWriter batches automatically.
                    // Flushed on stop().
                }
            } catch (IOException e) {
                Log.e(TAG, "Write error: " + e.getMessage());
            }
        });
    }

    /**
     * Flushes and closes the file.
     * Call this when the fence is stopped or a breach occurs.
     */
    public void stop() {
        if (!running) return;
        running = false;

        ioExecutor.execute(() -> {
            try {
                if (writer != null) {
                    writer.flush();
                    writer.close();
                    writer = null;
                    Log.d(TAG, "Flight log saved: " + logFile.getAbsolutePath());
                    Log.d(TAG, "Pull with: adb pull " + logFile.getAbsolutePath());
                }
            } catch (IOException e) {
                Log.e(TAG, "Failed to close log file: " + e.getMessage());
            }
        });
    }

    /**
     * Returns the absolute path of the current log file, or null if not started.
     * Useful for displaying the save location to the user in the UI.
     */
    public String getLogFilePath() {
        return logFile != null ? logFile.getAbsolutePath() : null;
    }
}