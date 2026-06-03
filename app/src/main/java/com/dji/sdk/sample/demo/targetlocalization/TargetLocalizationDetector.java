package com.dji.sdk.sample.demo.targetlocalization;

import android.graphics.Bitmap;
import android.graphics.RectF;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Lightweight OpenCV-style target detector for the black/white localization
 * targets. The class is intentionally isolated so a native OpenCV Mat pipeline
 * can replace this implementation without touching the DJI view/control code.
 */
public final class TargetLocalizationDetector {

    private static final int MIN_COMPONENT_AREA_PX = 45;
    private static final int MAX_DETECTIONS = 8;
    private static final float MIN_CONFIDENCE = 0.48f;

    public List<Detection> detect(Bitmap frame) {
        if (frame == null || frame.isRecycled()) {
            return Collections.emptyList();
        }

        int width = frame.getWidth();
        int height = frame.getHeight();
        if (width < 40 || height < 40) {
            return Collections.emptyList();
        }

        int[] pixels = new int[width * height];
        int[] gray = new int[width * height];
        int[] histogram = new int[256];
        frame.getPixels(pixels, 0, width, 0, 0, width, height);

        long sum = 0;
        int min = 255;
        int max = 0;
        for (int i = 0; i < pixels.length; i++) {
            int color = pixels[i];
            int r = (color >> 16) & 0xFF;
            int g = (color >> 8) & 0xFF;
            int b = color & 0xFF;
            int y = (r * 299 + g * 587 + b * 114) / 1000;
            gray[i] = y;
            histogram[y]++;
            sum += y;
            if (y < min) min = y;
            if (y > max) max = y;
        }

        int contrast = max - min;
        if (contrast < 65) {
            return Collections.emptyList();
        }

        int mean = (int) (sum / gray.length);
        int p20 = percentile(histogram, gray.length, 0.20f);
        int p80 = percentile(histogram, gray.length, 0.80f);
        int darkThreshold = clampInt(Math.min(mean - 25, p20 + 18), 25, 135);
        int brightThreshold = clampInt(Math.max(mean + 25, p80 - 18), 120, 245);
        if (brightThreshold - darkThreshold < 55) {
            darkThreshold = clampInt(mean - 35, 25, 120);
            brightThreshold = clampInt(mean + 35, 135, 245);
        }

        boolean[] visited = new boolean[gray.length];
        int[] stack = new int[gray.length];
        List<IntRect> candidateRects = new ArrayList<>();

        for (int y = 0; y < height; y++) {
            int rowOffset = y * width;
            for (int x = 0; x < width; x++) {
                int index = rowOffset + x;
                if (visited[index] || gray[index] > darkThreshold) {
                    continue;
                }
                Component component = floodDarkComponent(gray, visited, stack, width, height,
                        x, y, darkThreshold);
                if (component.area < MIN_COMPONENT_AREA_PX) {
                    continue;
                }
                IntRect expanded = squareAround(component.minX, component.minY,
                        component.maxX, component.maxY, width, height);
                if (expanded != null) {
                    candidateRects.add(expanded);
                }
            }
        }

        List<Candidate> candidates = new ArrayList<>();
        for (IntRect rect : candidateRects) {
            Candidate candidate = scoreCandidate(gray, width, height, rect,
                    darkThreshold, brightThreshold);
            if (candidate != null && candidate.confidence >= MIN_CONFIDENCE) {
                candidates.add(candidate);
            }
        }

        Collections.sort(candidates, (a, b) -> Float.compare(b.confidence, a.confidence));

        List<Detection> detections = new ArrayList<>();
        for (Candidate candidate : candidates) {
            boolean overlaps = false;
            for (Detection existing : detections) {
                if (iou(candidate.rect, existing.pixelRect) > 0.35f) {
                    overlaps = true;
                    break;
                }
            }
            if (overlaps) {
                continue;
            }

            String targetId = inferTargetId(gray, width, height, candidate.rect, darkThreshold);
            RectF normalizedRect = new RectF(
                    candidate.rect.left / (float) width,
                    candidate.rect.top / (float) height,
                    candidate.rect.right / (float) width,
                    candidate.rect.bottom / (float) height);

            float centerX = (candidate.rect.left + candidate.rect.right) * 0.5f;
            float centerY = (candidate.rect.top + candidate.rect.bottom) * 0.5f;
            detections.add(new Detection(
                    new RectF(candidate.rect.left, candidate.rect.top,
                            candidate.rect.right, candidate.rect.bottom),
                    normalizedRect,
                    ((centerX / width) - 0.5f) * 2f,
                    ((centerY / height) - 0.5f) * 2f,
                    candidate.confidence,
                    targetId,
                    System.currentTimeMillis()));

            if (detections.size() >= MAX_DETECTIONS) {
                break;
            }
        }

        return detections;
    }

    private static Component floodDarkComponent(int[] gray,
                                                boolean[] visited,
                                                int[] stack,
                                                int width,
                                                int height,
                                                int startX,
                                                int startY,
                                                int darkThreshold) {
        int stackSize = 0;
        int startIndex = startY * width + startX;
        stack[stackSize++] = startIndex;
        visited[startIndex] = true;

        Component component = new Component(startX, startY);
        while (stackSize > 0) {
            int index = stack[--stackSize];
            int y = index / width;
            int x = index - y * width;
            component.include(x, y);

            if (x > 0) {
                stackSize = addDarkNeighbor(gray, visited, stack, stackSize,
                        index - 1, darkThreshold);
            }
            if (x < width - 1) {
                stackSize = addDarkNeighbor(gray, visited, stack, stackSize,
                        index + 1, darkThreshold);
            }
            if (y > 0) {
                stackSize = addDarkNeighbor(gray, visited, stack, stackSize,
                        index - width, darkThreshold);
            }
            if (y < height - 1) {
                stackSize = addDarkNeighbor(gray, visited, stack, stackSize,
                        index + width, darkThreshold);
            }
        }
        return component;
    }

    private static int addDarkNeighbor(int[] gray,
                                       boolean[] visited,
                                       int[] stack,
                                       int stackSize,
                                       int index,
                                       int darkThreshold) {
        if (!visited[index] && gray[index] <= darkThreshold) {
            visited[index] = true;
            stack[stackSize++] = index;
        }
        return stackSize;
    }

    private static IntRect squareAround(int minX, int minY, int maxX, int maxY,
                                        int frameWidth, int frameHeight) {
        int boxWidth = maxX - minX + 1;
        int boxHeight = maxY - minY + 1;
        if (boxWidth < 10 || boxHeight < 10) {
            return null;
        }

        int side = Math.max(boxWidth, boxHeight);
        side = (int) (side * 1.28f);
        int minSide = Math.min(frameWidth, frameHeight) / 14;
        int maxSide = Math.min(frameWidth, frameHeight) * 4 / 5;
        side = clampInt(side, Math.max(18, minSide), Math.max(20, maxSide));

        int centerX = (minX + maxX) / 2;
        int centerY = (minY + maxY) / 2;
        int left = clampInt(centerX - side / 2, 0, frameWidth - side);
        int top = clampInt(centerY - side / 2, 0, frameHeight - side);
        return new IntRect(left, top, left + side, top + side);
    }

    private static Candidate scoreCandidate(int[] gray,
                                            int width,
                                            int height,
                                            IntRect rect,
                                            int darkThreshold,
                                            int brightThreshold) {
        int rectWidth = rect.width();
        int rectHeight = rect.height();
        if (rectWidth < 18 || rectHeight < 18) {
            return null;
        }

        float aspect = rectWidth / (float) rectHeight;
        if (aspect < 0.70f || aspect > 1.35f) {
            return null;
        }

        int darkCount = 0;
        int brightCount = 0;
        long darkSum = 0;
        long brightSum = 0;
        int total = rectWidth * rectHeight;

        for (int y = rect.top; y < rect.bottom; y++) {
            int row = y * width;
            for (int x = rect.left; x < rect.right; x++) {
                int value = gray[row + x];
                if (value <= darkThreshold) {
                    darkCount++;
                    darkSum += value;
                } else if (value >= brightThreshold) {
                    brightCount++;
                    brightSum += value;
                }
            }
        }

        float darkRatio = darkCount / (float) total;
        float brightRatio = brightCount / (float) total;
        if (darkRatio < 0.12f || darkRatio > 0.72f || brightRatio < 0.10f) {
            return null;
        }

        float contrastScore = 0f;
        if (darkCount > 0 && brightCount > 0) {
            float darkAvg = darkSum / (float) darkCount;
            float brightAvg = brightSum / (float) brightCount;
            contrastScore = clamp01((brightAvg - darkAvg) / 170f);
        }

        float balanceScore = 1f - Math.abs(0.50f - darkRatio) * 1.6f;
        balanceScore = clamp01(balanceScore);

        float patternScore = patternScore(gray, width, rect);
        float borderScore = borderContrastScore(gray, width, height, rect);
        float confidence = 0.42f * contrastScore
                + 0.34f * patternScore
                + 0.14f * balanceScore
                + 0.10f * borderScore;

        return new Candidate(rect, clamp01(confidence));
    }

    private static float patternScore(int[] gray, int width, IntRect rect) {
        float top = cellDarkness(gray, width, rect, 2, 0);
        float bottom = cellDarkness(gray, width, rect, 2, 4);
        float left = cellDarkness(gray, width, rect, 0, 2);
        float right = cellDarkness(gray, width, rect, 4, 2);
        float center = cellDarkness(gray, width, rect, 2, 2);

        float verticalDark = (top + bottom + center) / 3f;
        float horizontalBright = ((1f - left) + (1f - right)) / 2f;
        float scoreA = (verticalDark + horizontalBright) / 2f;

        float horizontalDark = (left + right + center) / 3f;
        float verticalBright = ((1f - top) + (1f - bottom)) / 2f;
        float scoreB = (horizontalDark + verticalBright) / 2f;

        return clamp01(Math.max(scoreA, scoreB));
    }

    private static float borderContrastScore(int[] gray, int width, int height, IntRect rect) {
        int insetX = Math.max(1, rect.width() / 8);
        int insetY = Math.max(1, rect.height() / 8);
        int outsideX = Math.max(1, rect.width() / 16);
        int outsideY = Math.max(1, rect.height() / 16);

        float inside = averageGray(gray, width,
                rect.left + insetX,
                rect.top + insetY,
                rect.right - insetX,
                rect.bottom - insetY);
        float outsideTop = averageGray(gray, width,
                rect.left,
                Math.max(0, rect.top - outsideY),
                rect.right,
                rect.top);
        float outsideBottom = averageGray(gray, width,
                rect.left,
                rect.bottom,
                rect.right,
                Math.min(height, rect.bottom + outsideY));

        float outside = outsideTop >= 0 && outsideBottom >= 0
                ? (outsideTop + outsideBottom) * 0.5f
                : Math.max(outsideTop, outsideBottom);
        if (inside < 0 || outside < 0) {
            return 0.5f;
        }
        return clamp01(Math.abs(outside - inside) / 95f);
    }

    private static String inferTargetId(int[] gray, int width, int height,
                                        IntRect targetRect, int darkThreshold) {
        int rectWidth = targetRect.width();
        int rectHeight = targetRect.height();
        int left = clampInt(targetRect.left + rectWidth / 2, 0, width - 1);
        int top = clampInt(targetRect.top + rectHeight / 5, 0, height - 1);
        int right = clampInt(targetRect.right + rectWidth / 3, left + 1, width);
        int bottom = clampInt(targetRect.bottom - rectHeight / 8, top + 1, height);

        IntRect digitBounds = findDarkBounds(gray, width, left, top, right, bottom,
                darkThreshold, rectWidth * rectHeight / 80);
        if (digitBounds == null || digitBounds.height() < rectHeight * 0.18f) {
            return "?";
        }

        float aspect = digitBounds.width() / (float) Math.max(1, digitBounds.height());
        if (aspect < 0.34f) {
            return "1";
        }

        float centerDark = regionDarkRatio(gray, width, digitBounds,
                0.36f, 0.34f, 0.64f, 0.66f, darkThreshold);
        float topDark = regionDarkRatio(gray, width, digitBounds,
                0.18f, 0.00f, 0.82f, 0.24f, darkThreshold);
        float middleDark = regionDarkRatio(gray, width, digitBounds,
                0.18f, 0.38f, 0.82f, 0.62f, darkThreshold);
        float bottomDark = regionDarkRatio(gray, width, digitBounds,
                0.18f, 0.76f, 0.82f, 1.00f, darkThreshold);
        float upperRightDark = regionDarkRatio(gray, width, digitBounds,
                0.58f, 0.18f, 0.96f, 0.48f, darkThreshold);
        float lowerLeftDark = regionDarkRatio(gray, width, digitBounds,
                0.04f, 0.52f, 0.42f, 0.82f, darkThreshold);

        if (centerDark < 0.20f && topDark > 0.25f && bottomDark > 0.25f) {
            return "0";
        }
        if (topDark > 0.22f && middleDark > 0.18f && bottomDark > 0.22f
                && upperRightDark > 0.16f && lowerLeftDark > 0.14f) {
            return "2";
        }
        if (aspect < 0.52f || middleDark < 0.12f) {
            return "1";
        }
        return "?";
    }

    private static IntRect findDarkBounds(int[] gray,
                                          int width,
                                          int left,
                                          int top,
                                          int right,
                                          int bottom,
                                          int darkThreshold,
                                          int minDarkPixels) {
        int minX = right;
        int minY = bottom;
        int maxX = left;
        int maxY = top;
        int count = 0;

        for (int y = top; y < bottom; y++) {
            int row = y * width;
            for (int x = left; x < right; x++) {
                if (gray[row + x] <= darkThreshold) {
                    count++;
                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                }
            }
        }

        if (count < Math.max(12, minDarkPixels) || minX >= maxX || minY >= maxY) {
            return null;
        }
        return new IntRect(minX, minY, maxX + 1, maxY + 1);
    }

    private static float regionDarkRatio(int[] gray,
                                         int width,
                                         IntRect rect,
                                         float x0,
                                         float y0,
                                         float x1,
                                         float y1,
                                         int darkThreshold) {
        int left = rect.left + Math.round(rect.width() * x0);
        int top = rect.top + Math.round(rect.height() * y0);
        int right = rect.left + Math.round(rect.width() * x1);
        int bottom = rect.top + Math.round(rect.height() * y1);
        int total = 0;
        int dark = 0;
        for (int y = top; y < bottom; y++) {
            int row = y * width;
            for (int x = left; x < right; x++) {
                total++;
                if (gray[row + x] <= darkThreshold) {
                    dark++;
                }
            }
        }
        return total == 0 ? 0f : dark / (float) total;
    }

    private static float cellDarkness(int[] gray, int width, IntRect rect, int cellX, int cellY) {
        int left = rect.left + rect.width() * cellX / 5;
        int right = rect.left + rect.width() * (cellX + 1) / 5;
        int top = rect.top + rect.height() * cellY / 5;
        int bottom = rect.top + rect.height() * (cellY + 1) / 5;
        float average = averageGray(gray, width, left, top, right, bottom);
        if (average < 0) {
            return 0f;
        }
        return clamp01(1f - average / 255f);
    }

    private static float averageGray(int[] gray,
                                     int width,
                                     int left,
                                     int top,
                                     int right,
                                     int bottom) {
        if (right <= left || bottom <= top) {
            return -1f;
        }

        long sum = 0;
        int total = 0;
        for (int y = top; y < bottom; y++) {
            int row = y * width;
            for (int x = left; x < right; x++) {
                sum += gray[row + x];
                total++;
            }
        }
        return total == 0 ? -1f : sum / (float) total;
    }

    private static int percentile(int[] histogram, int total, float percentile) {
        int goal = Math.round(total * percentile);
        int count = 0;
        for (int i = 0; i < histogram.length; i++) {
            count += histogram[i];
            if (count >= goal) {
                return i;
            }
        }
        return histogram.length - 1;
    }

    private static float iou(IntRect a, RectF b) {
        float left = Math.max(a.left, b.left);
        float top = Math.max(a.top, b.top);
        float right = Math.min(a.right, b.right);
        float bottom = Math.min(a.bottom, b.bottom);
        float intersection = Math.max(0f, right - left) * Math.max(0f, bottom - top);
        float areaA = a.width() * a.height();
        float areaB = b.width() * b.height();
        float union = areaA + areaB - intersection;
        return union <= 0f ? 0f : intersection / union;
    }

    private static int clampInt(int value, int min, int max) {
        if (max < min) {
            return min;
        }
        return Math.max(min, Math.min(max, value));
    }

    private static float clamp01(float value) {
        return Math.max(0f, Math.min(1f, value));
    }

    public static final class Detection {
        public final RectF pixelRect;
        public final RectF normalizedRect;
        public final float centerXNorm;
        public final float centerYNorm;
        public final float confidence;
        public final String targetId;
        public final long timestampMs;

        Detection(RectF pixelRect,
                  RectF normalizedRect,
                  float centerXNorm,
                  float centerYNorm,
                  float confidence,
                  String targetId,
                  long timestampMs) {
            this.pixelRect = pixelRect;
            this.normalizedRect = normalizedRect;
            this.centerXNorm = centerXNorm;
            this.centerYNorm = centerYNorm;
            this.confidence = confidence;
            this.targetId = targetId;
            this.timestampMs = timestampMs;
        }

        public float offsetMagnitude() {
            return (float) Math.hypot(centerXNorm, centerYNorm);
        }
    }

    private static final class Component {
        int minX;
        int minY;
        int maxX;
        int maxY;
        int area;

        Component(int x, int y) {
            minX = maxX = x;
            minY = maxY = y;
        }

        void include(int x, int y) {
            area++;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
        }
    }

    private static final class Candidate {
        final IntRect rect;
        final float confidence;

        Candidate(IntRect rect, float confidence) {
            this.rect = rect;
            this.confidence = confidence;
        }
    }

    private static final class IntRect {
        final int left;
        final int top;
        final int right;
        final int bottom;

        IntRect(int left, int top, int right, int bottom) {
            this.left = left;
            this.top = top;
            this.right = right;
            this.bottom = bottom;
        }

        int width() {
            return right - left;
        }

        int height() {
            return bottom - top;
        }
    }
}
