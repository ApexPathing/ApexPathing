package org.firstinspires.ftc.teamcode.apexpathing;

import core.Follower;
import geometry.Pose;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import paths.movements.Turn;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;

public class ExampleAutoPath {
    private static final Pose startPose = Pose.zero();

    public GeometryFactory factory;
    public Path testPath;
    public Turn testTurn;
    public Path returnPath;
    public Path strafeOutPath;
    public Path strafeBackPath;
    public String callbackMessage = "Callback not triggered yet";
    public boolean outboundCallbackTriggered;
    public boolean turnCallbackTriggered;
    public boolean returnCallbackTriggered;

    public ExampleAutoPath(Follower follower, GeometryFactory.PoseMirror mirror) {
        factory = new GeometryFactory(follower)
                .setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG)
                .setPoseMirror(mirror);

        build();
    }

    public void exampleDistanceCallback() {
        outboundCallbackTriggered = true;
        callbackMessage = "Outbound distance callback triggered!";
    }

    public void exampleAngularCallback() {
        turnCallbackTriggered = true;
        callbackMessage = "Angular callback triggered!";
    }

    public void exampleReturnCallback() {
        returnCallbackTriggered = true;
        callbackMessage = "Return distance callback triggered!";
    }

    private void build() {
        testPath = factory.path(startPose, // Forward and left curve
                        factory.pose(20, 0),
                        factory.pose(40, 20),
                        factory.pose(45, 25, 120)
                )
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END)
                .addDistanceCallback(0.5, this::exampleDistanceCallback)
                .profiledBuild();

        testTurn = factory.turn(testPath.getEndPose())
                .turnTo(factory.angle(45))
                .addAngularCallback(factory.angle(90), this::exampleAngularCallback)
                .quickBuild();

        // Exercise reverse tangent following on a profiled curve and bring the robot home. This
        // catches backward-heading and terminal-profile regressions that the outbound path cannot.
        returnPath = factory.path(testTurn.getEndPose(),
                        factory.pose(28, 5),
                        startPose
                )
                .interpolateWith(InterpolationStyle.TANGENT_BACKWARD)
                .addDistanceCallback(0.5, this::exampleReturnCallback)
                .profiledBuild();

        // Finish with pure lateral travel in both directions. Curved paths can hide a broken
        // strafe sign or weak lateral controller because their forward component still progresses.
        Pose strafeEnd = factory.pose(0, 24, 0);
        strafeOutPath = factory.path(startPose, strafeEnd)
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING)
                .profiledBuild();
        strafeBackPath = factory.path(strafeEnd, startPose)
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING)
                .profiledBuild();
    }
}
