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
@TeleOp(name = "Apex TeleOp Test", group = "Apex Pathing Tests")
public class TeleOpTest extends LinearOpMode {
    Constants constants = new Constants();

    @Override
    public void runOpMode() {
        Follower follower = new Follower(constants, hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();

            if (gamepad1.left_trigger_pressed) { // Emergency stop
                follower.stop();
                telemetry.addLine("Follower stopped");
            } else {
                follower.teleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                );
            }

            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y ", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.update();
        }
    }
}