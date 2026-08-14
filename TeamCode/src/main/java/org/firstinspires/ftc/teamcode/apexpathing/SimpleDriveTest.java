package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import core.Follower;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

/**
 * Auto test for Apex that drives forward on a line.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@Autonomous(name = "Apex Line Test", group = "Apex Pathing")
public class SimpleDriveTest extends LinearOpMode {
    private AutoState currentState = AutoState.TEST_PATH;

    enum AutoState { TEST_PATH, COMPLETE }

    @Override
    public void runOpMode() {
        Follower follower = new Follower(new Constants(), hardwareMap);
        GeometryFactory factory = new GeometryFactory(follower).setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        Path path = factory.path(factory.pose(0, 0, 0), factory.pose(24, 0, 0))
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD)
                .quickBuild();

        telemetry.addLine("Press Start to begin");
        telemetry.update();

        waitForStart();

        follower.follow(path);

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();

            switch (currentState) {
                case TEST_PATH:
                    if (!follower.isBusy()) {
                        currentState = AutoState.COMPLETE;
                    }
                    break;
                case COMPLETE:
                    telemetry.addLine("Auto Test Completed!");
                    break;
            }

            telemetry.addData("Current state:", currentState);
            telemetry.addData("Follower is busy:", follower.isBusy());
            telemetry.addData("Follower T", follower.getBestT());
            telemetry.addData("X (in):", pose.getX().getIn());
            telemetry.addData("Y (in):", pose.getY().getIn());
            telemetry.addData("Heading (deg):", pose.getHeading().getDeg());
            telemetry.addData("Left Motor Power", follower.getDrivetrain().getLastFlPower());
            telemetry.addData("Right Motor Power", follower.getDrivetrain().getLastFrPower());
            telemetry.addData("Back Left Motor Power", follower.getDrivetrain().getLastBlPower());
            telemetry.addData("Back Right Motor Power", follower.getDrivetrain().getLastBrPower());
            telemetry.update();
        }
        follower.stop();
    }
}
