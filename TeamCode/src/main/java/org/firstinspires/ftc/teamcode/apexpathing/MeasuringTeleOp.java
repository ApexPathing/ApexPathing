package org.firstinspires.ftc.teamcode.apexpathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.Follower;

@TeleOp(name = "Apex Velocity Measuring TeleOp", group = "Apex Pathing")
public class MeasuringTeleOp extends LinearOpMode {
    Constants constants = new Constants();

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Follower follower = new Follower(constants, hardwareMap);

        telemetry.addLine("Connect to the robot Wi-Fi and navigate to 192.168.49.1:8080/dash");
        telemetry.addLine("Use B to stop all robot movement");
        telemetry.addLine("Press Start to begin");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            double velocity = follower.getVelocity().getVec().getMag().getIn();
            double acceleration = follower.getAcceleration().getVec().getMag().getIn();

            if (gamepad1.b) { // Emergency stop
                follower.stop();
                telemetry.addLine("Follower stopped");
            } else {
                follower.manual(gamepad1);
            }

            telemetry.addData("Velocity", velocity);
            telemetry.addData("Acceleration", acceleration);
            telemetry.update();
        }
    }
}
