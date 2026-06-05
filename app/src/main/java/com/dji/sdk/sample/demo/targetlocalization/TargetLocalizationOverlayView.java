package com.dji.sdk.sample.demo.targetlocalization;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.View;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public class TargetLocalizationOverlayView extends View {

    private final Paint boxPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint bestBoxPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint crosshairPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF drawRect = new RectF();

    private List<TargetLocalizationDetector.Detection> detections = Collections.emptyList();
    private TargetLocalizationDetector.Detection bestDetection;
    private boolean centeringActive;

    public TargetLocalizationOverlayView(Context context) {
        this(context, null);
    }

    public TargetLocalizationOverlayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        setWillNotDraw(false);

        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setStrokeWidth(3f);
        boxPaint.setColor(Color.rgb(0, 210, 120));

        bestBoxPaint.setStyle(Paint.Style.STROKE);
        bestBoxPaint.setStrokeWidth(5f);
        bestBoxPaint.setColor(Color.rgb(255, 204, 64));

        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(26f);
        textPaint.setFakeBoldText(true);

        labelPaint.setStyle(Paint.Style.FILL);
        labelPaint.setColor(Color.argb(190, 0, 0, 0));

        crosshairPaint.setStyle(Paint.Style.STROKE);
        crosshairPaint.setStrokeWidth(2f);
        crosshairPaint.setColor(Color.argb(210, 255, 255, 255));
    }

    public void setDetections(List<TargetLocalizationDetector.Detection> newDetections,
                              TargetLocalizationDetector.Detection best) {
        if (newDetections == null || newDetections.isEmpty()) {
            detections = Collections.emptyList();
        } else {
            detections = new ArrayList<>(newDetections);
        }
        bestDetection = best;
        invalidate();
    }

    public void setCenteringActive(boolean active) {
        centeringActive = active;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        float width = getWidth();
        float height = getHeight();
        float centerX = width * 0.5f;
        float centerY = height * 0.5f;
        float crossSize = Math.min(width, height) * 0.08f;

        canvas.drawLine(centerX - crossSize, centerY, centerX + crossSize, centerY, crosshairPaint);
        canvas.drawLine(centerX, centerY - crossSize, centerX, centerY + crossSize, crosshairPaint);

        if (centeringActive) {
            Paint.Style oldStyle = crosshairPaint.getStyle();
            crosshairPaint.setStyle(Paint.Style.STROKE);
            canvas.drawCircle(centerX, centerY, crossSize * 0.95f, crosshairPaint);
            crosshairPaint.setStyle(oldStyle);
        }

        for (TargetLocalizationDetector.Detection detection : detections) {
            boolean isBest = detection == bestDetection;
            Paint paint = isBest ? bestBoxPaint : boxPaint;
            RectF n = detection.normalizedRect;
            drawRect.set(n.left * width, n.top * height, n.right * width, n.bottom * height);
            canvas.drawRect(drawRect, paint);

            String id = detection.targetId == null ? "?" : detection.targetId;
            String label = String.format(Locale.US, "ID %s  %.0f%%",
                    id, detection.confidence * 100f);
            float textWidth = textPaint.measureText(label);
            float labelHeight = textPaint.getTextSize() + 12f;
            float labelTop = Math.max(0f, drawRect.top - labelHeight);
            canvas.drawRect(drawRect.left,
                    labelTop,
                    Math.min(width, drawRect.left + textWidth + 18f),
                    labelTop + labelHeight,
                    labelPaint);
            canvas.drawText(label, drawRect.left + 8f,
                    labelTop + textPaint.getTextSize(), textPaint);
        }
    }
}
