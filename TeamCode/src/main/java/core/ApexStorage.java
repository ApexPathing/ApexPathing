package core;

import android.os.Environment;

import java.io.File;

/** Resolves the folder used for Apex Pathing's persisted tuning data. */
public final class ApexStorage {
    private ApexStorage() { }

    public static File getDirectory() {
        return new File(Environment.getExternalStorageDirectory(), "FIRST/ApexPathing");
    }

    public static File getConstantsFile() {
        return new File(getDirectory(), "constants.json");
    }
}
