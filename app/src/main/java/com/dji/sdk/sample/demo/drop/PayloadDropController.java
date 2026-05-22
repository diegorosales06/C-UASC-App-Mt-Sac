package com.dji.sdk.sample.demo.drop;

public class PayloadDropController {

    private boolean armed = false;
    private boolean alreadyDropped = false;

    public void armDrop() {
        armed = true;
    }

    public void dropPayload() {
        if (!armed) {
            return;
        }

        if (alreadyDropped) {
            return;
        }

        alreadyDropped = true;

        // Add gimbal movement later
    }

    public void resetDropSystem() {
        armed = false;
        alreadyDropped = false;
    }

    public boolean isArmed() {
        return armed;
    }

    public boolean hasDropped() {
        return alreadyDropped;
    }
}