package tuning;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

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
    private final LinearOpMode opMode;
    private Follower follower;
    public FollowerConstants constants;

    public TunerContext(LinearOpMode opMode) { this.opMode = opMode; }

    public void setFollower(Follower follower) {
        this.follower = follower;
        this.constants = follower.getConstants();
    }

    public Follower getFollower() { return follower; }

    public Telemetry getTelemetry() { return opMode.telemetry; }

    public void saveConstants() {
        JSONObject constantsJSON = constants.toJson();
        try {
            File outputFolder = new File(
                    Environment.getExternalStorageDirectory().getPath() +
                            "/FIRST/ApexPathing/"
            );

            boolean folderExists = outputFolder.exists();
            if (!folderExists) { folderExists = outputFolder.mkdirs(); }

            if (folderExists) {
                FileWriter fileWriter = new FileWriter(new File(outputFolder, "constants.json"));
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
