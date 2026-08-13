package tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import core.ApexStorage;
import core.Follower;
import core.FollowerConstants;
import geometry.Pose;

/**
 * Provides a context for the tuner phases to operate in, including access th the OpMode, telemetry,
 * and the follower instance.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class TunerContext {
    private static final long DEBUG_HOLD_NANOS = 1_500_000_000L;

    private final LinearOpMode opMode;
    private Follower follower;
    public FollowerConstants constants;
    private boolean debugMode;
    private boolean debugHoldHandled;
    private long debugHoldStartedNanos;
    private boolean emergencyStopped;

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

    String control(String button) {
        if (!Boolean.getBoolean("apex.simulation.unlockTunerPhases")) { return button; }
        switch (button) {
            case "A": return "A [; key]";
            case "B": return "B [left-bracket key]";
            case "X": return "X [P key]";
            case "Y": return "Y [- key]";
            case "BACK": return "Back [Tab key]";
            default: return button;
        }
    }

    public boolean isDebugMode() { return debugMode; }

    public boolean isEmergencyStopped() { return emergencyStopped; }

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

    /** Stops all drivetrain output and terminates the tuner when Back is pressed. */
    public boolean checkEmergencyStop() {
        if (emergencyStopped) { return true; }
        if (!opMode.gamepad1.backWasPressed()) { return false; }

        emergencyStopped = true;
        if (follower != null) { follower.stop(); }
        getTelemetry().clearAll();
        getTelemetry().addLine("EMERGENCY STOP ACTIVATED");
        getTelemetry().addLine("All drivetrain output has been stopped.");
        getTelemetry().update();
        opMode.requestOpModeStop();
        return true;
    }

    /** Adds controls which must remain visible independently of the current phase. */
    public void addInterfaceHeader() {
        boolean simulation = Boolean.getBoolean("apex.simulation.unlockTunerPhases");
        getTelemetry().addLine("E-STOP: Back button" + (simulation ? " [Tab key]" : ""));
        if (debugMode) {
            getTelemetry().addLine("DEBUG MODE");
            getTelemetry().addLine("Hold Right Stick Button" +
                    (simulation ? " [comma key]" : "") + " to exit debug mode.");
        }
        getTelemetry().addLine();
    }

    /** Teleports only FTCodeSim; real hardware must still be positioned by its operator. */
    public void positionRobotForSimulation(Pose pose) {
        if (!Boolean.getBoolean("apex.simulation.unlockTunerPhases")) { return; }
        follower.stop();
        follower.setPose(pose);
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
