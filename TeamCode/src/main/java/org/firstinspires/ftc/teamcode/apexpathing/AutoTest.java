package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import core.Follower;
import geometry.GeometryFactory;
import geometry.Pose;
import paths.movements.FollowerMovement;

/**
 * Test autonomous OpMode for Apex Pathing that uses the {@link ExampleAutoPath}. Make sure the
 * robot has been tuned with the {@link FollowerTuner} before running this OpMode. This OpMode will
 * first follow the test path, then follow the test turn, and finally stop.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@Autonomous(name = "Apex Auto Test", group = "Apex Pathing")
public class AutoTest extends LinearOpMode {
    private static final double STAGE_TIMEOUT_SECONDS = 50.0;
    private static final double POSITION_TOLERANCE_INCHES = 3.0;
    private static final double HEADING_TOLERANCE_DEGREES = 5.0;
    // This is a coarse stress-test curve, not the endpoint acceptance tolerance. Leave enough
    // margin for normal transient error while still catching gross drift.
    private static final double CROSS_TRACK_TOLERANCE_INCHES = 15;

    private ExampleAutoPath path;
    private AutoState currentState = AutoState.OUTBOUND_CURVE;
    private final ElapsedTime stageTimer = new ElapsedTime();
    private String failureReason = "None";
    private int passedStages;
    private double lastPositionError;
    private double lastHeadingError;
    private double maximumCrossTrackError;
    private double lastMaximumCrossTrackError;

    private enum AutoState {
        OUTBOUND_CURVE,
        POINT_TURN,
        REVERSE_RETURN,
        STRAFE_OUT,
        STRAFE_BACK,
        COMPLETE,
        FAILED
    }

    @Override
    public void runOpMode() {
        Follower follower = new Follower(new Constants(), hardwareMap);
        path = new ExampleAutoPath(follower, GeometryFactory.PoseMirror.NONE);

        telemetry.addLine("Apex follower self-test: curve, turn, reverse return, and strafe.");
        telemetry.addLine("Press Start to begin");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) { return; }

        follower.setPose(Pose.zero());
        startStage(follower, currentState);

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();
            maximumCrossTrackError = Math.max(maximumCrossTrackError,
                    Math.abs(follower.getCrossTrackErrorIn()));

            if (!isTerminal(currentState)) {
                if (stageTimer.seconds() > STAGE_TIMEOUT_SECONDS) {
                    fail(follower, "Stage exceeded " + STAGE_TIMEOUT_SECONDS + " seconds");
                } else if (!follower.isBusy()) {
                    finishStage(follower, pose);
                }
            }

            if (currentState == AutoState.COMPLETE) {
                telemetry.addLine("PASS: all Apex follower checks completed.");
            } else if (currentState == AutoState.FAILED) {
                telemetry.addLine("FAIL: " + failureReason);
            }

            telemetry.addData("Current check", currentState);
            telemetry.addData("Passed checks", passedStages + " / 5");
            telemetry.addData("Stage time (s)", stageTimer.seconds());
            telemetry.addData("Follower busy", follower.isBusy());
            telemetry.addData("Callback state", path.callbackMessage);
            telemetry.addData("Last endpoint position error (in)", lastPositionError);
            telemetry.addData("Last endpoint heading error (deg)", lastHeadingError);
            telemetry.addData("Current maximum cross-track error (in)",
                    maximumCrossTrackError);
            telemetry.addData("Last maximum cross-track error (in)",
                    lastMaximumCrossTrackError);
            telemetry.addData("X (in)", pose.getX().getIn());
            telemetry.addData("Y (in)", pose.getY().getIn());
            telemetry.addData("Heading (deg)", pose.getHeading().getDeg());
            telemetry.update();
            sleep(20);
        }

        follower.stop();
    }

    private void finishStage(Follower follower, Pose actualPose) {
        FollowerMovement completed = movementFor(currentState);
        Pose expectedPose = completed.getEndPose();
        lastPositionError = actualPose.distanceTo(expectedPose).getIn();
        lastHeadingError = Math.abs(actualPose.getHeading()
                .getShortestAngleTo(expectedPose.getHeading()).getDeg());
        lastMaximumCrossTrackError = maximumCrossTrackError;

        if (!Double.isFinite(lastPositionError) || !Double.isFinite(lastHeadingError) ||
                !Double.isFinite(maximumCrossTrackError)) {
            fail(follower, "Non-finite localization/tracking result in " + currentState);
            return;
        }
        if (lastPositionError > POSITION_TOLERANCE_INCHES ||
                lastHeadingError > HEADING_TOLERANCE_DEGREES) {
            fail(follower, "Endpoint tolerance missed in " + currentState +
                    ": position=" + lastPositionError + " in, heading=" +
                    lastHeadingError + " deg");
            return;
        }
        if (currentState != AutoState.POINT_TURN &&
                maximumCrossTrackError > CROSS_TRACK_TOLERANCE_INCHES) {
            fail(follower, "Cross-track error exceeded " + CROSS_TRACK_TOLERANCE_INCHES +
                    " inches in " + currentState + ": " + maximumCrossTrackError + " in");
            return;
        }
        if (!requiredCallbackTriggered(currentState)) {
            fail(follower, "Expected callback did not run in " + currentState);
            return;
        }

        passedStages++;
        currentState = nextState(currentState);
        if (currentState == AutoState.COMPLETE) {
            follower.stop();
        } else {
            startStage(follower, currentState);
        }
    }

    private void startStage(Follower follower, AutoState state) {
        maximumCrossTrackError = 0.0;
        stageTimer.reset();
        follower.follow(movementFor(state));
    }

    private void fail(Follower follower, String reason) {
        follower.stop();
        failureReason = reason;
        currentState = AutoState.FAILED;
    }

    private FollowerMovement movementFor(AutoState state) {
        switch (state) {
            case OUTBOUND_CURVE: return path.testPath;
            case POINT_TURN: return path.testTurn;
            case REVERSE_RETURN: return path.returnPath;
            case STRAFE_OUT: return path.strafeOutPath;
            case STRAFE_BACK: return path.strafeBackPath;
            default: throw new IllegalArgumentException("No movement for terminal state " + state);
        }
    }

    private boolean requiredCallbackTriggered(AutoState state) {
        switch (state) {
            case OUTBOUND_CURVE: return path.outboundCallbackTriggered;
            case POINT_TURN: return path.turnCallbackTriggered;
            case REVERSE_RETURN: return path.returnCallbackTriggered;
            default: return true;
        }
    }

    private static AutoState nextState(AutoState state) {
        switch (state) {
            case OUTBOUND_CURVE: return AutoState.POINT_TURN;
            case POINT_TURN: return AutoState.REVERSE_RETURN;
            case REVERSE_RETURN: return AutoState.STRAFE_OUT;
            case STRAFE_OUT: return AutoState.STRAFE_BACK;
            case STRAFE_BACK: return AutoState.COMPLETE;
            default: return state;
        }
    }

    private static boolean isTerminal(AutoState state) {
        return state == AutoState.COMPLETE || state == AutoState.FAILED;
    }

}
