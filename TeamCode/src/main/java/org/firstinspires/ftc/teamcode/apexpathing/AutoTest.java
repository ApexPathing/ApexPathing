package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import core.Follower;
import geometry.Angle;
import geometry.AngleUnit;
import geometry.Dist;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.FollowerMovement;

/** Runs each Apex Pathing test movement in order. */
@Autonomous(name = "Apex Auto Test", group = "Apex Pathing")
public class AutoTest extends LinearOpMode {
    private static final Pose START_POSE = new Pose(Vector.zero(), Angle.fromDeg(90));

    private FollowerMovement[] movements;
    private int movementIndex;
    private String callbackMessage = "No callback triggered yet";


    private FollowerMovement[] buildMovements(GeometryFactory factory) {
        FollowerMovement testYMove = factory.path(START_POSE,
                        factory.pose(0, 30, 90))
                .interpolateWith(InterpolationStyle.CONSTANT_END_HEADING)
                .profiledBuild();

        FollowerMovement testXMove = factory.path(testYMove.getEndPose(),
                        factory.pose(30, 30, 90))
                .interpolateWith(InterpolationStyle.CONSTANT_END_HEADING)
                .quickBuild();

        FollowerMovement testTurn = factory.turn(testXMove.getEndPose())
                .turnTo(Angle.fromDeg(180))
                .addAngularCallback(Angle.fromDeg(45), this::exampleAngularCallback)
                .profiledBuild();

        FollowerMovement testArc = factory.path(testTurn.getEndPose(),
                        factory.pose(30,0),
                        factory.pose(0,0),
                        factory.pose(0, 30, 0))
                .interpolateWith(InterpolationStyle.CONSTANT_END_HEADING)
                .profiledBuild();
        FollowerMovement strafeBackPath = factory.path(testArc.getEndPose(),
                        factory.pose(30, 30),
                        factory.pose(30, 15),
                        factory.pose(0, 15, 90),
                        START_POSE)
                .interpolateWith(InterpolationStyle.TANGENT_BACKWARD)
                .setDistanceToStartFinalTurn(Dist.fromIn(0.0))
                .profiledBuild();

        return new FollowerMovement[] {
                testYMove,
                testXMove,
                testTurn,
                testArc,
                strafeBackPath
        };
    }

    @Override
    public void runOpMode() {
        Follower follower = new Follower(new Constants(), hardwareMap);
        GeometryFactory factory = new GeometryFactory(follower)
                .setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG)
                .setPoseMirror(GeometryFactory.PoseMirror.NONE);

        movements = buildMovements(factory);

        telemetry.addLine("Apex follower test: curve, turn, return, and strafe.");
        telemetry.addLine("Press Start to begin");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        follower.setPose(START_POSE);
        follower.follow(movements[movementIndex]);

        while (opModeIsActive() && movementIndex < movements.length) {
            follower.update();

            if (!follower.isBusy()) {
                movementIndex++;
                if (movementIndex < movements.length) {
                    follower.follow(movements[movementIndex]);
                }
            }

            Pose pose = follower.getPose();
            telemetry.addData("Movement", Math.min(movementIndex + 1, movements.length)
                    + " / " + movements.length);
            telemetry.addData("Follower busy", follower.isBusy());
            telemetry.addData("Callback", callbackMessage);
            telemetry.addData("X (in)", pose.getX().getIn());
            telemetry.addData("Y (in)", pose.getY().getIn());
            telemetry.addData("Heading (deg)", pose.getHeading().getDeg());
            telemetry.update();
        }

        follower.stop();
        telemetry.addLine("Apex follower test complete.");
        telemetry.update();
    }

    private void exampleDistanceCallback() {
        callbackMessage = "Outbound distance callback triggered.";
    }

    private void exampleAngularCallback() {
        callbackMessage = "Turn callback triggered.";
    }

    private void exampleReturnCallback() {
        callbackMessage = "Return distance callback triggered.";
    }
}
