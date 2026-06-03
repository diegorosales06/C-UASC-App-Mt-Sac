package com.dji.sdk.sample.internal.utils;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.util.Log;

import com.dji.sdk.sample.R;
import dji.common.error.DJIError;

/**
 * Created by dji on 2/3/16.
 */
public class DialogUtils {
    private static final String TAG = "DJIErrorDialog";

    public static void showDialog(Context ctx, String str) {
        AlertDialog.Builder builder = new AlertDialog.Builder(ctx, R.style.set_dialog);
        builder.setMessage(str);
        builder.setNeutralButton(android.R.string.ok, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
            }
        });
        builder.create().show();
    }

    public static void showDialog(Context ctx, int strId) {
        AlertDialog.Builder builder = new AlertDialog.Builder(ctx, R.style.set_dialog);
        builder.setMessage(strId);
        builder.setNeutralButton(android.R.string.ok, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
            }
        });
        builder.create().show();
    }

    public static void showConfirmationDialog(Context ctx, int strId,
                                              DialogInterface.OnClickListener onClickListener) {
        AlertDialog.Builder builder = new AlertDialog.Builder(ctx, R.style.set_dialog);
        builder.setMessage(strId);
        builder.setPositiveButton(android.R.string.ok, onClickListener);
        builder.setNegativeButton(android.R.string.cancel, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
            }
        });
        builder.create().show();
    }

    public static void showDialogBasedOnError(Context ctx, DJIError djiError) {
        if (null == djiError) {
            showDialog(ctx, R.string.success);
        } else {
            String message = formatDJIError(djiError);
            Log.e(TAG, message);
            showDialog(ctx, message);
        }
    }

    public static void logResult(String tag, String action, DJIError djiError) {
        if (djiError == null) {
            Log.i(tag, action + " succeeded.");
        } else {
            Log.e(tag, action + " failed.\n" + formatDJIError(djiError));
        }
    }

    public static String formatDJIError(DJIError djiError) {
        if (djiError == null) {
            return "Success";
        }

        return "DJI error"
                + "\nClass: " + djiError.getClass().getName()
                + "\nCode: " + djiError.getErrorCode()
                + "\nDescription: " + djiError.getDescription()
                + "\nRaw: " + djiError;
    }
}
