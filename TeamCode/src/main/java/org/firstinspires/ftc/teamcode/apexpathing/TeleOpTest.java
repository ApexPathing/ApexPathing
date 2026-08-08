package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.Follower;
import geometry.Pose;

/**
 * Test OpMode for using Apex Pathing in TeleOp mode.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@TeleOp(name = "Apex TeleOp Test", group = "Apex Pathing")
public class TeleOpTest extends LinearOpMode {
    public double loops = 0, lastLoop = 0, loopTime = 0;
    Constants constants = new Constants();

    @Override
    public void runOpMode() {
        Follower follower = new Follower(constants, hardwareMap);

        telemetry.addLine("Use B to stop all robot movement");
        telemetry.addLine("Press Start to begin");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();

            if (gamepad1.b) { // Emergency stop
                follower.stop();
                telemetry.addLine("Follower stopped");
            } else {
                follower.manual(gamepad1);
            }

            loops++;

            if (loops > 15) {
                double now = System.currentTimeMillis();
                loopTime = (now - lastLoop) / loops;
                lastLoop = now;
                loops = 0;
            }

            telemetry.addData("Loop time (ms)", loopTime);
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y ", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.update();
        }
    }
}
