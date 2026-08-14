package localizers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;

public class BaseLocalizerTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void nativeVelocityUsesSharedVelocityAndAccelerationFilter() throws Exception {
        NativeVelocityLocalizer localizer = new NativeVelocityLocalizer();
        localizer.setVelocityFilterMode(BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE);

        localizer.setMeasuredVelocity(pose(0.0, 0.0, 0.0));
        localizer.update();
        Thread.sleep(5);
        localizer.setMeasuredVelocity(pose(14.0, 0.0, 0.0));
        localizer.update();

        assertEquals(14.0, localizer.getRawVel().getX().getIn(), EPSILON);
        assertEquals(7.0, localizer.getVel().getX().getIn(), EPSILON);
        assertTrue(localizer.getRawAccel().getX().getIn() > 0.0);
        assertTrue(localizer.getAccel().getX().getIn() > 0.0);
    }

    @Test
    public void adaptiveKalmanIsTheDefaultAndSupportsAngularOnlyMotion() throws Exception {
        NativeVelocityLocalizer localizer = new NativeVelocityLocalizer();
        assertEquals(BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN,
                localizer.getVelocityFilterMode());

        localizer.setMeasuredVelocity(pose(0.0, 0.0, 0.0));
        localizer.update();
        Thread.sleep(5);
        localizer.setMeasuredVelocity(pose(0.0, 0.0, 2.0));
        localizer.update();

        assertEquals(2.0, localizer.getRawVel().getHeading().getRad(), EPSILON);
        assertTrue(localizer.getVel().getHeading().getRad() > 0.0);
        assertEquals(0.0, localizer.getVel().getX().getIn(), EPSILON);
    }

    @Test
    public void kalmanTuningIsExplicitAndFrozenAfterOneTimeCalibration() {
        NativeVelocityLocalizer localizer = new NativeVelocityLocalizer();
        localizer.setIsTuning(true);
        assertTrue(localizer.isTuningVelocityFilter());
        localizer.setIsTuning(false);
        assertTrue(!localizer.isTuningVelocityFilter());
    }

    @Test
    public void poseResetClearsVelocityAccelerationAndFilterHistory() throws Exception {
        NativeVelocityLocalizer localizer = new NativeVelocityLocalizer();
        localizer.setMeasuredVelocity(pose(20.0, -4.0, 2.0));
        localizer.update();
        Thread.sleep(5);
        localizer.setMeasuredVelocity(pose(30.0, -8.0, 3.0));
        localizer.update();

        Pose stagedPose = pose(55.0, -12.0, 1.0);
        localizer.setPose(stagedPose);

        assertEquals(55.0, localizer.getPose().getX().getIn(), EPSILON);
        assertPoseIsZero(localizer.getVel());
        assertPoseIsZero(localizer.getRawVel());
        assertPoseIsZero(localizer.getAccel());
        assertPoseIsZero(localizer.getRawAccel());

        localizer.setMeasuredVelocity(Pose.zero());
        localizer.update();
        assertPoseIsZero(localizer.getVel());
        assertPoseIsZero(localizer.getAccel());
    }

    @Test
    public void poseDerivedLocalizerDoesNotInterpretResetAsMotion() throws Exception {
        PoseDerivedLocalizer localizer = new PoseDerivedLocalizer();
        localizer.update();
        Thread.sleep(5);
        localizer.setMeasuredPose(pose(1.0, 0.0, 0.0));
        localizer.update();

        localizer.setPose(pose(55.0, 0.0, 0.0));
        Thread.sleep(5);
        localizer.update();
        Thread.sleep(5);
        localizer.update();

        assertPoseIsZero(localizer.getRawVel());
        assertPoseIsZero(localizer.getVel());
        assertPoseIsZero(localizer.getRawAccel());
        assertPoseIsZero(localizer.getAccel());
    }

    private static void assertPoseIsZero(Pose pose) {
        assertEquals(0.0, pose.getX().getIn(), EPSILON);
        assertEquals(0.0, pose.getY().getIn(), EPSILON);
        assertEquals(0.0, pose.getHeading().getRad(), EPSILON);
    }

    private static Pose pose(double x, double y, double heading) {
        return new Pose(Vector.of(x, y, DistUnit.IN), Angle.fromRad(heading));
    }

    private static final class NativeVelocityLocalizer
            extends BaseLocalizer<TestConstants> {
        private Pose measuredVelocity = Pose.zero();

        NativeVelocityLocalizer() {
            super(new TestConstants());
        }

        void setMeasuredVelocity(Pose measuredVelocity) {
            this.measuredVelocity = measuredVelocity;
        }

        @Override
        public void update() {
            calculate(measuredVelocity);
        }

        @Override
        public void setPose(Pose newPose) {
            resetKinematicEstimate(newPose);
        }
    }

    private static final class PoseDerivedLocalizer
            extends BaseLocalizer<TestConstants> {
        private Pose measuredPose = Pose.zero();

        PoseDerivedLocalizer() {
            super(new TestConstants());
        }

        void setMeasuredPose(Pose measuredPose) {
            this.measuredPose = measuredPose;
        }

        @Override
        public void update() {
            pose = measuredPose;
            calculate(UpdateType.BOTH);
        }

        @Override
        public void setPose(Pose newPose) {
            measuredPose = newPose;
            resetKinematicEstimate(newPose);
        }
    }

    private static final class TestConstants
            implements BaseLocalizerConstants<TestConstants> {
        @Override
        public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            return new NativeVelocityLocalizer();
        }
    }
}
