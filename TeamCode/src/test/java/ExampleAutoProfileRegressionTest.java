import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.io.File;

import controllers.PDSController.PDSCoefficients;
import core.FollowerConstants;
import core.ApexStorage;
import feedforward.MotionParameters;
import feedforward.generators.MecanumProfileGenerator;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

/** Fixed reproducer for a long curve whose profile pinning used to erase its startup. */
public class ExampleAutoProfileRegressionTest {
    @Test
    public void initialAutoCurveHasUsableProfileStartup() {
        configureConstants();
        GeometryFactory factory = new GeometryFactory(true)
                .setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG);
        Path path = factory.path(Pose.zero(),
                        factory.arcPose(30, 0, 7),
                        factory.arcPose(30, -30, 7),
                        factory.arcPose(-30, -30, 7),
                        factory.arcPose(-30, 30, 7),
                        factory.pose(30, 30, -90))
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD)
                .quickBuild();

        MecanumProfileGenerator generator = new MecanumProfileGenerator(
                FollowerConstants.getInstance(), path);
        MotionParameters[] profile = generator.generate();
        int firstMoving = -1;
        for (int i = 0; i < profile.length; i++) {
            if (profile[i].getTangentialVel() > 1e-6 ||
                    Math.abs(profile[i].getTangentialAccel()) > 1e-6) {
                firstMoving = i;
                break;
            }
        }
        assertTrue("Profile should contain a moving sample", firstMoving >= 0);
        assertTrue("Profile must command motion at the first sample",
                Math.abs(profile[0].getTangentialVel()) > 1e-6 ||
                        Math.abs(profile[0].getTangentialAccel()) > 1e-6);
    }

    private static void configureConstants() {
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                new File("build/profile-tests").getAbsolutePath());
        FollowerConstants constants = FollowerConstants.getInstance();
        constants.angularCoeffs = new PDSCoefficients(0.80, 0.10, 0.23);
        constants.translationalCoeffs = new PDSCoefficients(0.12, 0.03, 0.23);
        constants.angularKV = 0.066;
        constants.angularKA = 0.043;
        constants.translationalKV = 0.0071;
        constants.translationalKA = 0.0047;
        constants.kCentripetal = 0.0061;
        constants.velocityFeedbackGain = 0.059;
        constants.angularVelocityFeedbackGain = 0.25;
        constants.forwardVelLimitIn = 64.7;
        constants.forwardAccelLimitIn = 105.7;
        constants.strafeVelLimitIn = 53.6;
        constants.strafeAccelLimitIn = 84.9;
        constants.angularVelLimitRad = 6.96;
        constants.angularAccelLimitRad = 12.0;
    }
}
