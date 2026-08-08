package core;

import android.os.Environment;

import java.io.File;

/** Resolves the folder used for Apex Pathing's persisted tuning data. */
public final class ApexStorage {
    /** JVM property used by desktop tools such as FTCodeSim. */
    public static final String DIRECTORY_PROPERTY = "apexpathing.storageDirectory";

    private ApexStorage() { }

    public static File getDirectory() {
        String desktopDirectory = System.getProperty(DIRECTORY_PROPERTY);
        if (desktopDirectory != null && !desktopDirectory.trim().isEmpty()) {
            return new File(desktopDirectory);
        }

        // This remains the normal Robot Controller location on Android.
        return new File(Environment.getExternalStorageDirectory(), "FIRST/ApexPathing");
    }

    public static File getConstantsFile() {
        return new File(getDirectory(), "constants.json");
    }
}
