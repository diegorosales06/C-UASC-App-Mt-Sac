package com.dji.sdk.sample.demo.virtualstickwaypoint;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.os.Bundle;
import android.util.SparseArray;
import android.view.Gravity;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.widget.FrameLayout;
import android.widget.TextView;

import com.google.android.gms.vision.CameraSource;
import com.google.android.gms.vision.Detector;
import com.google.android.gms.vision.barcode.Barcode;
import com.google.android.gms.vision.barcode.BarcodeDetector;

import java.io.IOException;

/**
 * QrWaypointScanActivity
 *
 * Full-screen QR code scanner Activity. Opened by VirtualStickWaypointView when
 * the user taps "Import QR". Scans using the device rear camera and delivers the
 * raw QR payload back to the View via QrResultListener.
 *
 * Integration with WaypointStore:
 *   This Activity delivers the raw string payload only — it does NOT parse
 *   waypoints itself. Parsing and validation are handled by WaypointStore in
 *   VirtualStickWaypointView.handleWaypointQrPayload(), which calls
 *   store.parseWaypointQrPayload(payload) and then shows the Append/Replace
 *   confirm dialog before writing to the store.
 *
 * Lifecycle contract:
 *   1. Caller sets a QrResultListener via setResultListener() before starting
 *      this Activity.
 *   2. On a successful scan, deliverResult() fires the listener on the main
 *      thread and calls finish().
 *   3. If the Activity is destroyed without scanning (e.g. user presses Back),
 *      the listener is cleared via clearResultListener() so no stale callback
 *      fires later.
 *
 * Permissions:
 *   android.permission.CAMERA is requested at runtime if not already granted.
 *   The CAMERA permission must also be declared in AndroidManifest.xml.
 *
 * Dependencies (add to build.gradle if not already present):
 *   implementation 'com.google.android.gms:play-services-vision:20.1.3'
 */
public class QrWaypointScanActivity extends Activity {

    // ── Intent extras ─────────────────────────────────────────────────────────
    /** Key used to retrieve the raw QR string from the result Intent. */
    public static final String EXTRA_QR_PAYLOAD =
            "com.dji.sdk.sample.extra.QR_PAYLOAD";

    private static final int REQUEST_CAMERA_PERMISSION = 9101;

    // =========================================================================
    // Result listener — static so it survives the Activity being created/destroyed
    // =========================================================================

    /**
     * Callback interface implemented anonymously in VirtualStickWaypointView.
     * Delivers the raw QR payload string back to the View on the main thread.
     */
    public interface QrResultListener {
        void onQrScanResult(String payload);
    }

    private static QrResultListener resultListener;

    /** Set by VirtualStickWaypointView before startActivity(). Thread-safe (main thread only). */
    public static void setResultListener(QrResultListener listener) {
        resultListener = listener;
    }

    /** Called in VirtualStickWaypointView.onDetachedFromWindow() to prevent stale callbacks. */
    public static void clearResultListener() {
        resultListener = null;
    }

    // =========================================================================
    // Instance state
    // =========================================================================

    private SurfaceView    cameraPreview;
    private TextView       statusText;
    private CameraSource   cameraSource;
    private BarcodeDetector barcodeDetector;

    /** Guards against delivering the result more than once if multiple frames decode. */
    private boolean hasDeliveredResult = false;

    // =========================================================================
    // Activity lifecycle
    // =========================================================================

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        buildUi();

        if (checkSelfPermission(Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(
                    new String[]{Manifest.permission.CAMERA},
                    REQUEST_CAMERA_PERMISSION);
            return;
        }

        createScanner();
    }

    @Override
    protected void onPause() {
        super.onPause();
        stopCameraPreview();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // If the user pressed Back without scanning, clear the listener so the
        // View doesn't wait for a callback that will never arrive.
        if (!hasDeliveredResult) {
            clearResultListener();
        }
        if (cameraSource != null) {
            cameraSource.release();
            cameraSource = null;
        }
        if (barcodeDetector != null) {
            barcodeDetector.release();
            barcodeDetector = null;
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String[] permissions,
                                           int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode != REQUEST_CAMERA_PERMISSION) return;

        if (grantResults.length > 0
                && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            createScanner();
        } else {
            statusText.setText("Camera permission is required to scan QR codes.");
        }
    }

    // =========================================================================
    // UI construction
    // =========================================================================

    private void buildUi() {
        FrameLayout root = new FrameLayout(this);

        cameraPreview = new SurfaceView(this);
        root.addView(cameraPreview, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT));

        statusText = new TextView(this);
        statusText.setText("Scan QR");
        statusText.setTextColor(Color.WHITE);
        statusText.setTextSize(18f);
        statusText.setGravity(Gravity.CENTER);
        statusText.setBackgroundColor(0x99000000);
        statusText.setPadding(24, 16, 24, 16);

        root.addView(statusText, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.WRAP_CONTENT,
                Gravity.BOTTOM));

        setContentView(root);
    }

    // =========================================================================
    // Scanner setup
    // =========================================================================

    private void createScanner() {
        barcodeDetector = new BarcodeDetector.Builder(this)
                .setBarcodeFormats(Barcode.QR_CODE)
                .build();

        if (!barcodeDetector.isOperational()) {
            statusText.setText("QR scanner is not ready on this tablet.");
            return;
        }

        cameraSource = new CameraSource.Builder(this, barcodeDetector)
                .setFacing(CameraSource.CAMERA_FACING_BACK)
                .setAutoFocusEnabled(true)
                .setRequestedPreviewSize(1280, 720)
                .build();

        barcodeDetector.setProcessor(new Detector.Processor<Barcode>() {
            @Override
            public void release() {
                // No-op.
            }

            @Override
            public void receiveDetections(Detector.Detections<Barcode> detections) {
                SparseArray<Barcode> detectedCodes = detections.getDetectedItems();
                if (detectedCodes.size() == 0 || hasDeliveredResult) return;

                Barcode barcode = detectedCodes.valueAt(0);
                if (barcode == null
                        || barcode.rawValue == null
                        || barcode.rawValue.trim().isEmpty()) {
                    return;
                }

                hasDeliveredResult = true;
                // receiveDetections fires on a background thread — post to main
                runOnUiThread(() -> deliverResult(barcode.rawValue));
            }
        });

        cameraPreview.getHolder().addCallback(new SurfaceHolder.Callback() {
            @Override
            public void surfaceCreated(SurfaceHolder holder) {
                startCameraPreview();
            }

            @Override
            public void surfaceChanged(SurfaceHolder holder, int format,
                                       int width, int height) {
                // No-op.
            }

            @Override
            public void surfaceDestroyed(SurfaceHolder holder) {
                stopCameraPreview();
            }
        });
    }

    // =========================================================================
    // Camera control
    // =========================================================================

    @SuppressLint("MissingPermission")
    private void startCameraPreview() {
        if (cameraSource == null) return;
        if (checkSelfPermission(Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        try {
            cameraSource.start(cameraPreview.getHolder());
        } catch (IOException | RuntimeException e) {
            statusText.setText("Could not start QR camera.");
        }
    }

    private void stopCameraPreview() {
        if (cameraSource != null) {
            cameraSource.stop();
        }
    }

    // =========================================================================
    // Result delivery
    // =========================================================================

    /**
     * Delivers the scanned QR payload back to VirtualStickWaypointView.
     *
     * Two delivery paths are used for robustness:
     *   1. setResult() + finish()    — covers the case where the caller used
     *                                  startActivityForResult() (future-proofing).
     *   2. QrResultListener callback — the primary path used by VirtualStickWaypointView.
     *      The callback fires on the main thread via runOnUiThread in the detector.
     *
     * After delivery the static listener is cleared to prevent it from being
     * accidentally called again.
     *
     * The payload is passed raw to the listener. Parsing and validation happen
     * in VirtualStickWaypointView → WaypointStore.parseWaypointQrPayload().
     *
     * @param payload Raw string content of the scanned QR code.
     */
    private void deliverResult(String payload) {
        // Path 1: Intent result (future-proofing for startActivityForResult callers)
        Intent data = new Intent();
        data.putExtra(EXTRA_QR_PAYLOAD, payload);
        setResult(RESULT_OK, data);

        // Path 2: direct callback to VirtualStickWaypointView
        QrResultListener listener = resultListener;
        resultListener = null; // clear before calling to avoid re-entrancy
        if (listener != null) {
            listener.onQrScanResult(payload);
        }

        finish();
    }
}
