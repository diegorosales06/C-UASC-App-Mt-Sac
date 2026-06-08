package com.dji.sdk.sample.internal.utils;

public final class OfflineDebugConfig {

    private OfflineDebugConfig() {
    }

    // Set to true to open and exercise debug-friendly screens without a drone/controller.
    // Set back to false before real flight so DJI hardware commands are enabled again.
    public static final boolean OFFLINE_DEBUG_MODE = false;
}
