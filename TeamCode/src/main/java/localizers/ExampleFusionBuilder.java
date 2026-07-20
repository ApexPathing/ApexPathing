package localizers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import core.ApexBuilder;
import drivetrains.constants.DrivetrainConstants;
import drivetrains.constants.MecanumConstants;
import followers.constants.BSplineFollowerConstants;
import followers.constants.FollowerConstants;
import localizers.constants.FusionConstants;
import localizers.constants.LocalizerConstants;

public class ExampleFusionBuilder extends ApexBuilder {

    @Override
    public DrivetrainConstants setDrivetrainConstants() {
        return new MecanumConstants()
                .setFrontLeftMotorName("front_left_drive")
                .setFrontRightMotorName("front_right_drive")
                .setBackLeftMotorName("back_left_drive")
                .setBackRightMotorName("back_right_drive");
    }

    @Override
    public LocalizerConstants setLocalizerConstants() {
        return new FusionConstants()
                .setPinpointDeviceName("pinpoint")
                .setCameraName("Webcam 1")
                .setXOffset(-4.0)
                .setYOffset(2.5)
                .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
                .setXDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .setYDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .setCameraResolution(640, 480)
                .setDrawTagOverlay(true)
                .setProcessNoise(0.05, 0.05, 0.005)
                .setVisionNoise(1.0, 1.0, 0.05)
                .setOutlierThresholds(24.0, Math.toRadians(45));
    }

    @Override
    public FollowerConstants setFollowerConstants() {
        return new BSplineFollowerConstants()
                .setTranslationP(0.1)
                .setHeadingP(0.4)
                .setVelocityFF(0.01)
                .setDistanceTolerance(0.5)
                .setHeadingTolerance(Math.toRadians(1.0));
    }
}