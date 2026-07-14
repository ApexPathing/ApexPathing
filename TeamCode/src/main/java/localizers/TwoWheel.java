package localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import geometry.Angle;
import geometry.GeometryFactory;
import geometry.Pose;
import geometry.Vector;
import geometry.DistUnit;

/**
 * A localizer that uses 2 dead wheel odometry pods (1 parallel and 1 perpendicular)
 *
 * @author Topher F. - 23571 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class TwoWheel extends BaseLocalizer<TwoWheel.Constants> {
    private final static GeometryFactory factory = new GeometryFactory()
            .setDistUnit(DistUnit.IN).setAngleUnit(geometry.AngleUnit.RAD);
    private final OdometryPod forwardPod, strafePod;
    private final IMU imu;
    private final double forwardOffsetIn, strafeOffsetIn;
    private double correction = 0.0;

    public TwoWheel(Constants constants, HardwareMap hardwareMap) {
        super(constants);

        this.strafePod = new OdometryPod(
                hardwareMap, constants.strafePodName, constants.ticksPerInch
        );
        this.forwardPod = new OdometryPod(
                hardwareMap, constants.forwardPodName, constants.ticksPerInch
        );
        this.imu = hardwareMap.get(IMU.class, constants.imuName);
        this.imu.initialize(new IMU.Parameters(constants.hubOrientation));

        this.forwardOffsetIn = constants.offsets.getX(DistUnit.IN);
        this.strafeOffsetIn = constants.offsets.getY(DistUnit.IN);
    }

    @Override
    public void update() {
        double oldYaw = pose.getHeading(geometry.AngleUnit.RAD);
        double currentYaw = Angle.normalize(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - correction
        );
        double deltaYaw = Angle.normalize(currentYaw - oldYaw);
        double avgYaw = oldYaw + deltaYaw / 2.0;
        double deltaX = forwardPod.getDeltaInches() - forwardOffsetIn * deltaYaw;
        double deltaY = strafePod.getDeltaInches() - strafeOffsetIn * deltaYaw;
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
        strafePod.reset();
        forwardPod.reset();
    }

    public static class Constants extends BaseLocalizerConstants<Constants> {
        public String forwardPodName = null;
        public String strafePodName = null;
        public String imuName = null;
        public RevHubOrientationOnRobot hubOrientation = null;
        public Vector offsets = Vector.zero();
        public double ticksPerInch = 1.0;

        @Override
        public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            if (this.forwardPodName == null || this.strafePodName == null) {
                throw new IllegalArgumentException(
                        "You must call setForwardPodName and setStrafePodName to set the names of " +
                                "the motor ports that hold the odometry pod encoders."
                );
            }

            if (this.imuName == null) {
                throw new IllegalArgumentException(
                        "You must call setIMUName to set the name of the control hub IMU"
                );
            }

            if (this.hubOrientation == null) {
                throw new IllegalArgumentException(
                        "You must call setHubOrientation to set the orientation of the control hub " +
                                "on the robot"
                );
            }

            return new TwoWheel(this, hardwareMap);
        }

        /** Sets the name of the motor that the forward pod encoder is attached to. */
        public TwoWheel.Constants setForwardPodName(String name) {
            this.forwardPodName = name;
            return this;
        }

        /** Sets the name of the motor that the strafe pod encoder is attached to. */
        public TwoWheel.Constants setStrafePodName(String name) {
            this.strafePodName = name;
            return this;
        }

        /** Sets the name of the control hub IMU that is used to measure heading */
        public TwoWheel.Constants setIMUName(String name) {
            this.imuName = name;
            return this;
        }

        /** Sets the orientation of the control hub on the robot. */
        public TwoWheel.Constants setHubOrientation(RevHubOrientationOnRobot orientation) {
            this.hubOrientation = orientation;
            return this;
        }

        /** Sets the offsets of the odometry pods from the robot center. */
        public TwoWheel.Constants setOffsets(Vector offsets) {
            this.offsets = offsets;
            return this;
        }

        /** Sets the number of encoder ticks per inch of travel. */
        public TwoWheel.Constants setTicksPerInch(double ticksPerInch) {
            this.ticksPerInch = ticksPerInch;
            return this;
        }
    }
}
