package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import core.Follower;
import geometry.GeometryFactory;
import geometry.Pose;
import paths.ExampleAutoPath;

/**
 * Test autonomous OpMode for Apex Pathing that uses the {@link ExampleAutoPath}. Make sure the
 * robot has been tuned with the {@link FollowerTuner} before running this OpMode. This OpMode will
 * first follow the test path, then follow the test turn, and finally stop.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@Autonomous(name = "Apex Auto Test", group = "Apex Pathing Tests")
public class AutoTest extends LinearOpMode {
    ExampleAutoPath path;
    AutoState currentState = AutoState.TEST_PATH;

    enum AutoState { TEST_PATH, TEST_TURN, COMPLETE }

    @Override
    public void runOpMode() {
        Follower follower = new Follower(new Constants(), hardwareMap);
        path = new ExampleAutoPath(follower, GeometryFactory.PoseMirror.NONE);

        telemetry.addLine("Robot initialized");
        telemetry.update();

        waitForStart();

        // TODO: Comment this when testing the second state machine method
        follower.follow(path.testPath);

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();

            switch (currentState) {
                case TEST_PATH:
                    if (!follower.isBusy()) {
                        follower.follow(path.testTurn);
                        currentState = AutoState.TEST_TURN;
                    }
                    break;
                case TEST_TURN:
                    if (!follower.isBusy()) {
                        currentState = AutoState.COMPLETE;
                    }
                    break;
                case COMPLETE:
                    telemetry.addLine("Auto Test Completed!");
                    break;
            }

            /* TODO: Test to make sure this version works
            switch (currentState) {
                case TEST_PATH:
                    if (!path.testPath.hasStarted()) follower.follow(path.testPath);
                    if (path.testPath.hasEnded()) currentState = AutoState.TEST_TURN;
                    break;

                case TEST_TURN:
                    if (!path.testTurn.hasStarted()) follower.follow(path.testTurn);
                    if (path.testTurn.hasEnded()) currentState = AutoState.COMPLETE;
                    break;

                case COMPLETE:
                    telemetry.addLine("Auto Test Completed!");
                    break;
            }
            */

            telemetry.addData("Current state:", currentState);
            telemetry.addData("Callback state:", path.callbackMessage);
            telemetry.addData("Follower is busy:", follower.isBusy());
            telemetry.addData("X (in):", pose.getX().getIn());
            telemetry.addData("Y (in):", pose.getY().getIn());
            telemetry.addData("Heading (deg):", pose.getHeading().getDeg());
            telemetry.update();
        }
    }
}