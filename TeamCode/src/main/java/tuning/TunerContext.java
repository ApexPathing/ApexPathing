package tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;

import core.ApexStorage;
import core.Follower;
import core.FollowerConstants;

/**
 * Provides a context for the tuner phases to operate in, including access th the OpMode, telemetry,
 * and the follower instance.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class TunerContext {
    private static final long DEBUG_HOLD_NANOS = 1_500_000_000L;
    private static final DecimalFormat NORMAL_NUMBER_FORMAT = new DecimalFormat(
            "0.#####", DecimalFormatSymbols.getInstance(Locale.US));

    private final LinearOpMode opMode;
    private Follower follower;
    public FollowerConstants constants;
    private boolean debugMode;
    private boolean debugHoldHandled;
    private long debugHoldStartedNanos;

    public TunerContext(LinearOpMode opMode) { this.opMode = opMode; }

    public void setFollower(Follower follower) {
        this.follower = follower;
        this.constants = follower.getConstants();
    }

    public Follower getFollower() { return follower; }

    public Telemetry getTelemetry() { return opMode.telemetry; }

    boolean testButtonWasPressed() { return opMode.gamepad1.xWasPressed(); }

    boolean acceptButtonWasPressed() { return opMode.gamepad1.aWasPressed(); }

    boolean retuneButtonWasPressed() { return opMode.gamepad1.bWasPressed(); }

    public boolean isDebugMode() { return debugMode; }

    /** Keeps normal telemetry compact while preserving full precision in debug mode. */
    public String formatNumber(double value) {
        if (debugMode) { return Double.toString(value); }
        synchronized (NORMAL_NUMBER_FORMAT) {
            return NORMAL_NUMBER_FORMAT.format(value);
        }
    }

    /** Debug can be enabled only from the phase menu, but may be disabled from any screen. */
    public void updateDebugMode(boolean allowEnable) {
        boolean held = debugMode
                ? opMode.gamepad1.right_stick_button
                : allowEnable && opMode.gamepad1.left_stick_button;
        if (!held) {
            debugHoldStartedNanos = 0L;
            debugHoldHandled = false;
            return;
        }

        if (debugHoldHandled || (!allowEnable && !debugMode)) { return; }
        if (debugHoldStartedNanos == 0L) {
            debugHoldStartedNanos = System.nanoTime();
            return;
        }
        if (System.nanoTime() - debugHoldStartedNanos >= DEBUG_HOLD_NANOS) {
            debugMode = !debugMode;
            debugHoldHandled = true;
        }
    }

    /** Adds controls which must remain visible independently of the current phase. */
    public void addInterfaceHeader() {
        if (debugMode) {
            getTelemetry().addLine("DEBUG MODE");
            getTelemetry().addLine("Hold Right Stick Button to exit debug mode.");
        }
        if (debugMode) { getTelemetry().addLine(); }
    }

    public void saveConstants() {
        JSONObject constantsJSON = constants.toJson();
        try {
            File outputFolder = ApexStorage.getDirectory();

            boolean folderExists = outputFolder.exists();
            if (!folderExists) { folderExists = outputFolder.mkdirs(); }

            if (folderExists) {
                FileWriter fileWriter = new FileWriter(ApexStorage.getConstantsFile());
                fileWriter.write(constantsJSON.toString(4));
                fileWriter.close();
            } else {
                throw new IOException("Failed to create output folder");
            }
        } catch (Exception e) {
            getTelemetry().addLine("WARNING: Values were not saved successfully");
            getTelemetry().addLine("Error: " + e.getMessage());

            JSONArray keys = constantsJSON.names();
            if (keys != null) {
                try {
                    for (int i = 0; i < keys.length(); i++) {
                        String key = keys.getString(i);
                        getTelemetry().addData(key, constantsJSON.get(key));
                    }
                } catch (Exception ex) {
                    getTelemetry().addLine("Error displaying constants: " + ex.getMessage());
                }
            } else {
                getTelemetry().addLine("No constants were found to display.");
            }

            getTelemetry().update();
        }
    }
}
