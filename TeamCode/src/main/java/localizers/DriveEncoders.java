package localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import geometry.Angle;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;

/**
 * A localizer that uses 4 drive encoders and an IMU
 *
 * @author Topher F. - 23571 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class DriveEncoders extends BaseLocalizer<DriveEncoders.Constants> {
    private final static GeometryFactory factory = new GeometryFactory()
            .setDistUnit(DistUnit.IN).setAngleUnit(geometry.AngleUnit.RAD);
    private final OdometryPod frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;

    private double correction = 0.0;

    public DriveEncoders(Constants constants, HardwareMap hardwareMap) {
        super(constants);

        frontLeft = new OdometryPod(hardwareMap, constants.frontLeftName, config.ticksPerInch);
        frontRight = new OdometryPod(hardwareMap, constants.frontRightName, config.ticksPerInch);
        if (constants.backLeftName != null && constants.backRightName != null) {
            backLeft = new OdometryPod(hardwareMap, constants.backLeftName, config.ticksPerInch);
            backRight = new OdometryPod(hardwareMap, constants.backRightName, config.ticksPerInch);
        } else {
            backLeft = null;
            backRight = null;
        }

        imu = hardwareMap.get(IMU.class, constants.imuName);
        imu.initialize(new IMU.Parameters(constants.hubOrientation));
    }

    @Override
    public void update() {
        double blDelta = 0;
        double brDelta = 0;
        if (backLeft != null) {
            blDelta = backLeft.getDeltaInches();
            brDelta = backRight.getDeltaInches();
        }

        double deltaY = (frontLeft.getDeltaInches() + frontRight.getDeltaInches() +
                blDelta + brDelta) / 4.0;
        double deltaX = (-frontLeft.getDeltaInches() + frontRight.getDeltaInches() +
                blDelta - brDelta) / 4.0;

        double oldYaw = pose.getHeading(geometry.AngleUnit.RAD);
        double currentYaw = Angle.normalize(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - correction
        );
        double deltaYaw = Angle.normalize(currentYaw - oldYaw);
        double avgYaw = oldYaw + deltaYaw / 2.0;

        factory.pose(
                pose.getX(DistUnit.IN) + (deltaX * Math.cos(avgYaw) - deltaY * Math.sin(avgYaw)),
                pose.getY(DistUnit.IN) + (deltaX * Math.sin(avgYaw) + deltaY * Math.cos(avgYaw)),
                currentYaw
        );

        calculate(UpdateType.BOTH);
    }

    @Override
    public void setPose(Pose newPose) {
        correction = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) -
                newPose.getHeading(geometry.AngleUnit.RAD);
        pose = newPose;
        frontLeft.reset();
        frontRight.reset();
        if (backLeft != null) {
            backLeft.reset();
            backRight.reset();
        }
    }

    public static class Constants extends BaseLocalizerConstants<Constants> {
        public String frontLeftName = null;
        public String frontRightName = null;
        public String backLeftName = null;
        public String backRightName = null;
        public String imuName = null;
        public RevHubOrientationOnRobot hubOrientation = null;
        public double ticksPerInch = 1.0;

        @Override
        public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            if (frontLeftName == null || frontRightName == null) {
                throw new IllegalArgumentException(
                        "You must call setFrontLeftName and setFrontRightName to set the names of the " +
                                "motors that the encoders are attached to. For drivetrains that are " +
                                "not 2 wheel tank, you must also call setBackLeftName and setBackRightName"
                );
            }

            if (imuName == null) {
                throw new IllegalArgumentException(
                        "You must call setIMUName to set the name of the control hub IMU."
                );
            }

            if (this.hubOrientation == null) {
                throw new IllegalArgumentException(
                        "You must call setHubOrientation to set the orientation of the control hub " +
                                "on the robot."
                );
            }

            return new DriveEncoders(this, hardwareMap);
        }

        /** Sets the name of the motor that the front left encoder is attached to. */
        public Constants setFrontLeftName(String name) {
            this.frontLeftName = name;
            return this;
        }

        /** Sets the name of the motor that the front right encoder is attached to. */
        public Constants setFrontRightName(String name) {
            this.frontRightName = name;
            return this;
        }

        /**
         * Sets the name of the motor that the back left encoder is attached to. For 2 wheel tank
         * drivetrains, don't set the back encoders, only the front.
         */
        public Constants setBackLeftName(String name) {
            this.backLeftName = name;
            return this;
        }

        /**
         * Sets the name of the motor that the back right encoder is attached to. For 2 wheel tank
         * drivetrains, don't set the back encoders, only the front.
         */
        public Constants setBackRightName(String name) {
            this.backRightName = name;
            return this;
        }

        /** Sets the name of the control hub IMU that is used to measure heading */
        public Constants setIMUName(String name) {
            this.imuName = name;
            return this;
        }

        /** Sets the orientation of the control hub on the robot. */
        public Constants setHubOrientation(RevHubOrientationOnRobot orientation) {
            this.hubOrientation = orientation;
            return this;
        }

        /** Sets the number of encoder ticks per inch of travel. */
        public Constants setTicksPerInch(double ticksPerInch) {
            this.ticksPerInch = ticksPerInch;
            return this;
        }
    }
}
