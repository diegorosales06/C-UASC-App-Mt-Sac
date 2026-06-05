package com.dji.sdk.sample.demo.searchrecord;

import android.content.Context;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Writes the drone's position log for a Search &amp; Record run to a CSV file.
 *
 * One file is created per run. Each {@link #log} call appends one row:
 *
 *   timestamp_ms, iso_time, latitude, longitude, altitude_m, heading_deg, phase
 *
 * File location (Android external app storage, no permissions needed):
 *   &lt;externalFilesDir&gt;/search-record/search_log_&lt;yyyyMMdd_HHmmss&gt;.csv
 * which resolves on-device to:
 *   /storage/emulated/0/Android/data/&lt;package&gt;/files/search-record/
 *
 * Call {@link #getAbsolutePath()} to show the exact path in the UI/log.
 */
public class SearchPositionLogger {

    private static final String FOLDER_NAME       = "search-record";
    private static final String DEFAULT_PREFIX    = "search_log_";
    private static final String FILE_EXTENSION    = ".csv";

    private final File csvFile;
    private final SimpleDateFormat isoFormat =
            new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS", Locale.US);

    public SearchPositionLogger(Context context) throws IOException {
        this(context, DEFAULT_PREFIX);
    }

    public SearchPositionLogger(Context context, String filePrefix) throws IOException {
        File baseDir = context.getExternalFilesDir(null);
        if (baseDir == null) {
            baseDir = context.getFilesDir();
        }

        File dir = new File(baseDir, FOLDER_NAME);
        if (!dir.exists() && !dir.mkdirs()) {
            throw new IOException("Unable to create folder: " + dir.getAbsolutePath());
        }

        String ts = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        csvFile = new File(dir, filePrefix + ts + FILE_EXTENSION);

        try (FileWriter w = new FileWriter(csvFile, false)) {
            w.write("timestamp_ms,iso_time,latitude,longitude,altitude_m,heading_deg,phase\n");
        }
    }

    public synchronized void log(double lat, double lng, float altitudeM,
                                 float headingDeg, String phase) throws IOException {
        long now = System.currentTimeMillis();
        try (FileWriter w = new FileWriter(csvFile, true)) {
            w.write(String.format(Locale.US,
                    "%d,%s,%.8f,%.8f,%.2f,%.1f,%s\n",
                    now, isoFormat.format(new Date(now)),
                    lat, lng, altitudeM, headingDeg, phase));
        }
    }

    public File getCsvFile() {
        return csvFile;
    }

    public String getAbsolutePath() {
        return csvFile.getAbsolutePath();
    }
}
