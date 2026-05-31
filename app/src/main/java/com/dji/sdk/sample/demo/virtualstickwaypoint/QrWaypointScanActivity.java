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

public class QrWaypointScanActivity extends Activity {

    public static final String EXTRA_QR_PAYLOAD = "com.dji.sdk.sample.extra.QR_PAYLOAD";
    private static final int REQUEST_CAMERA_PERMISSION = 9101;

    public interface QrResultListener {
        void onQrScanResult(String payload);
    }

    private static QrResultListener resultListener;

    private SurfaceView cameraPreview;
    private TextView statusText;
    private CameraSource cameraSource;
    private BarcodeDetector barcodeDetector;
    private boolean hasDeliveredResult = false;

    public static void setResultListener(QrResultListener listener) {
        resultListener = listener;
    }

    public static void clearResultListener() {
        resultListener = null;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        buildUi();

        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.CAMERA}, REQUEST_CAMERA_PERMISSION);
            return;
        }

        createScanner();
    }

    private void buildUi() {
        FrameLayout root = new FrameLayout(this);

        cameraPreview = new SurfaceView(this);
        root.addView(cameraPreview, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT));

        statusText = new TextView(this);
        statusText.setText("Scan waypoint QR");
        statusText.setTextColor(Color.WHITE);
        statusText.setTextSize(18f);
        statusText.setGravity(Gravity.CENTER);
        statusText.setBackgroundColor(0x99000000);
        statusText.setPadding(24, 16, 24, 16);

        FrameLayout.LayoutParams statusParams = new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.WRAP_CONTENT,
                Gravity.BOTTOM);
        root.addView(statusText, statusParams);

        setContentView(root);
    }

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
                if (detectedCodes.size() == 0 || hasDeliveredResult) {
                    return;
                }

                Barcode barcode = detectedCodes.valueAt(0);
                if (barcode == null || barcode.rawValue == null || barcode.rawValue.trim().isEmpty()) {
                    return;
                }

                hasDeliveredResult = true;
                runOnUiThread(() -> deliverResult(barcode.rawValue));
            }
        });

        cameraPreview.getHolder().addCallback(new SurfaceHolder.Callback() {
            @Override
            public void surfaceCreated(SurfaceHolder holder) {
                startCameraPreview();
            }

            @Override
            public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
                // No-op.
            }

            @Override
            public void surfaceDestroyed(SurfaceHolder holder) {
                stopCameraPreview();
            }
        });
    }

    @SuppressLint("MissingPermission")
    private void startCameraPreview() {
        if (cameraSource == null) {
            return;
        }
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
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

    private void deliverResult(String payload) {
        Intent data = new Intent();
        data.putExtra(EXTRA_QR_PAYLOAD, payload);
        setResult(RESULT_OK, data);

        QrResultListener listener = resultListener;
        resultListener = null;
        if (listener != null) {
            listener.onQrScanResult(payload);
        }

        finish();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode != REQUEST_CAMERA_PERMISSION) {
            return;
        }
        if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            createScanner();
        } else {
            statusText.setText("Camera permission is required to scan waypoint QR codes.");
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        stopCameraPreview();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
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
}
