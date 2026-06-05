package com.dji.sdk.sample.demo.targetlocalization;

import android.content.Context;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Handles manual drone GPS position recording for TargetLocalizationView.
 *
 * One instance of this class creates one fresh CSV file.
 * Each call to recordPosition(...) appends one row:
 *
 * latitude,longitude,tag
 */
public class DronePositionCsvRecorder {

    private static final String FOLDER_NAME = "target-localization";
    private static final String FILE_PREFIX = "manual_drone_positions_";
    private static final String FILE_EXTENSION = ".csv";

    private final File csvFile;

    public DronePositionCsvRecorder(Context context) throws IOException {
        File baseDir = context.getExternalFilesDir(null);
        if (baseDir == null) {
            baseDir = context.getFilesDir();
        }

        File dir = new File(baseDir, FOLDER_NAME);
        if (!dir.exists() && !dir.mkdirs()) {
            throw new IOException("Unable to create folder: " + dir.getAbsolutePath());
        }

        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        csvFile = new File(dir, FILE_PREFIX + timestamp + FILE_EXTENSION);

        writeHeader();
    }

    private void writeHeader() throws IOException {
        try (FileWriter writer = new FileWriter(csvFile, false)) {
            writer.write("latitude,longitude,tag\n");
        }
    }

    public void recordPosition(double latitude, double longitude, String tag) throws IOException {
        try (FileWriter writer = new FileWriter(csvFile, true)) {
            writer.write(String.format(
                    Locale.US,
                    "%.8f,%.8f,%s\n",
                    latitude,
                    longitude,
                    tag
            ));
        }
    }

    public File getCsvFile() {
        return csvFile;
    }

    private static String csv(String value) {
        if (value == null) {
            return "";
        }

        String escaped = value.replace("\"", "\"\"");
        return "\"" + escaped + "\"";
    }
}