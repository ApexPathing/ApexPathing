import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;

import org.junit.Before;
import org.junit.Test;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.BitmapEncoder.BitmapFormat;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.XYSeries;
import org.knowm.xchart.style.Styler;

import controllers.PDSController.PDSCoefficients;
import core.ApexStorage;
import core.FollowerConstants;
import drivetrains.BaseDrivetrain.DrivetrainType;
import drivetrains.Mecanum;
import drivetrains.Mecanum.DirectionalLut.DirectionalKinematics;
import feedforward.MotionParameters;
import feedforward.generators.BaseProfileGenerator;
import feedforward.generators.MecanumProfileGenerator;
import feedforward.generators.SwerveProfileGenerator;
import feedforward.generators.TankProfileGenerator;
import geometry.Angle;
import geometry.AngleUnit;
import geometry.Dist;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.PathPoint;
import geometry.Vector;
import paths.builders.HolonomicPathBuilder;
import paths.builders.TankPathBuilder;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

/**
 * Motion-profile convergence tests recovered from commit 61c977 and updated for the current API.
 * The graph test writes three PNGs to the project directory for visual profile inspection.
 */
public class PathProfileConvergenceTest {
    private FollowerConstants constants;
    private GeometryFactory geometry;
    private Path path;
    private BaseProfileGenerator generator;

    @Before
    public void setUp() {
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                new File("build/profile-tests").getAbsolutePath());
        constants = FollowerConstants.getInstance();
        constants.drivetrainType = DrivetrainType.MECANUM;
        constants.angularCoeffs = new PDSCoefficients(0.8, 0.04, 0.0);
        constants.translationalCoeffs = new PDSCoefficients(1.3, 0.04, 0.0);
        constants.velocityFeedbackGain = 2.5;
        constants.translationalKV = 0.0135;
        constants.translationalKA = 0.0026;
        constants.angularKV = 0.011;
        constants.angularKA = 0.0021;
        constants.kCentripetal = 0.001;
        constants.forwardVelLimitIn = 72.5;
        constants.forwardAccelLimitIn = 62.5;
        constants.strafeVelLimitIn = 58.0;
        constants.strafeAccelLimitIn = 50.0;
        constants.angularVelLimitRad = Math.toRadians(291.0);
        constants.angularAccelLimitRad = Math.toRadians(2000.0);

        geometry = new GeometryFactory(true)
                .setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG);
        path = new HolonomicPathBuilder(
                geometry.pose(0, -50, 90),
                geometry.pose(0, 50, 0))
                .interpolateWith(InterpolationStyle.FACING_POINT, geometry.vector(10, 0))
                .quickBuild();
        generator = new MecanumProfileGenerator(constants, path);
    }

    @Test
    public void testProfileConvergenceAndGraph() throws Exception {
        MotionParameters[] profile = generator.generate();
        PathPoint[] points = path.getGeneratedPoints();
        double pathLength = path.getParametricPath().getLengthIn();

        double[] distance = new double[points.length];
        double[] velocity = new double[points.length];
        double[] acceleration = new double[points.length];
        double[] power = new double[points.length];
        double[] angularVelocity = new double[points.length];
        double[] angularAcceleration = new double[points.length];
        double[] velocityLimit = new double[points.length];
        double[] positiveAccelLimit = new double[points.length];
        double[] negativeAccelLimit = new double[points.length];
        double[] positiveAngularVelLimit = constantArray(points.length,
                constants.angularVelLimitRad);
        double[] negativeAngularVelLimit = constantArray(points.length,
                -constants.angularVelLimitRad);
        double[] positiveAngularAccelLimit = constantArray(points.length,
                constants.angularAccelLimitRad);
        double[] negativeAngularAccelLimit = constantArray(points.length,
                -constants.angularAccelLimitRad);
        double[] fullPower = constantArray(points.length, 1.0);
        double[] zeroLine = constantArray(points.length, 0.0);
        Mecanum.DirectionalLut limits = createMecanumLimits();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        for (int i = 0; i < points.length; i++) {
            PathPoint point = points[i];
            distance[i] = pathLength - point.getDistanceToEndIn();
            velocity[i] = profile[i].getTangentialVel();
            acceleration[i] = profile[i].getTangentialAccel();
            power[i] = profile[i].getMotorPower();
            angularVelocity[i] = profile[i].getAngularVel();
            angularAcceleration[i] = profile[i].getAngularAccel();

            Angle heading = path.getInterpolator().getHeadingTarg(
                    point.getDistanceToEndIn(), point.getFirstDerivative(), finalTangent);
            DirectionalKinematics directional = limits.getKinematics(
                    point.getFirstDerivative(), heading);
            velocityLimit[i] = directional.maxVel;
            positiveAccelLimit[i] = directional.maxAccel;
            negativeAccelLimit[i] = -directional.maxAccel;
        }

        XYChart translation = chart("Mecanum Profile - Translational State",
                "Arc Length s Along Path (in)");
        translation.setYAxisGroupTitle(0, "Velocity (in/s)");
        translation.setYAxisGroupTitle(1, "Tangential Accel (in/s^2)");
        translation.getStyler().setYAxisGroupPosition(1, Styler.YAxisPosition.Right);
        addSeries(translation, "Optimized velocity", distance, velocity, 0);
        addSeries(translation, "Directional velocity limit", distance, velocityLimit, 0);
        addSeries(translation, "Optimized tangential accel", distance, acceleration, 1);
        addSeries(translation, "+directional accel limit", distance, positiveAccelLimit, 1);
        addSeries(translation, "-directional accel limit", distance, negativeAccelLimit, 1);
        addSeries(translation, "Zero accel", distance, zeroLine, 1);

        XYChart angular = chart("Mecanum Profile - Heading State",
                "Arc Length s Along Path (in)");
        angular.setYAxisGroupTitle(0, "Angular Velocity omega (rad/s)");
        angular.setYAxisGroupTitle(1, "Angular Accel alpha (rad/s^2)");
        angular.getStyler().setYAxisGroupPosition(1, Styler.YAxisPosition.Right);
        addSeries(angular, "Optimized omega", distance, angularVelocity, 0);
        addSeries(angular, "+omega limit", distance, positiveAngularVelLimit, 0);
        addSeries(angular, "-omega limit", distance, negativeAngularVelLimit, 0);
        addSeries(angular, "Optimized alpha", distance, angularAcceleration, 1);
        addSeries(angular, "+alpha limit", distance, positiveAngularAccelLimit, 1);
        addSeries(angular, "-alpha limit", distance, negativeAngularAccelLimit, 1);
        addSeries(angular, "Zero alpha", distance, zeroLine, 1);

        XYChart utilization = chart(String.format(
                "Mecanum Profile - Power Utilization (avg %.3f, max %.3f)",
                average(power), max(power)), "Arc Length s Along Path (in)");
        utilization.setYAxisTitle("Normalized Motor Utilization");
        utilization.getStyler().setYAxisMin(0.0);
        utilization.getStyler().setYAxisMax(1.12);
        addSeries(utilization, "Optimized utilization", distance, power, 0);
        addSeries(utilization, "Full-power target", distance, fullPower, 0);

        System.out.println("--- GENERATOR DEBUG REPORT ---");
        System.out.println(generator.getLastDebugReport());
        BitmapEncoder.saveBitmap(translation, "Translational_Kinematics", BitmapFormat.PNG);
        BitmapEncoder.saveBitmap(angular, "Angular_Kinematics", BitmapFormat.PNG);
        BitmapEncoder.saveBitmap(utilization, "Power_Utilization", BitmapFormat.PNG);
    }

    @Test
    public void testMecanumAndTankProfilesGenerateFiniteOutput() {
        Path mecanumPath = new HolonomicPathBuilder(
                geometry.pose(0, -50, 90), geometry.arcPose(0, 50, 30),
                geometry.pose(100, 50, 0))
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD)
                .quickBuild();
        assertUsableProfile("mecanum", new MecanumProfileGenerator(constants, mecanumPath).generate());

        Path tankPath = new TankPathBuilder(
                geometry.pose(0, -50, 90), geometry.arcPose(0, 50, 30),
                geometry.pose(100, 50, 0))
                .quickBuild();
        assertUsableProfile("tank", new TankProfileGenerator(constants, tankPath).generate());
    }

    @Test
    public void testMecanumDirectionSpecificVelocityLimit() {
        double forwardMax = maxVelocity(generateStraightMecanumProfile(90));
        double diagonalMax = maxVelocity(generateStraightMecanumProfile(45));
        assertTrue("45-degree mecanum profile should have a lower peak velocity",
                diagonalMax < forwardMax - 1.0);
    }

    @Test
    public void testMecanumDirectionalKinematicsOrdering() {
        Mecanum.DirectionalLut limits = createMecanumLimits();
        DirectionalKinematics forward = limits.getKinematics(
                Vector.of(1, 0, DistUnit.IN), Angle.zero());
        DirectionalKinematics diagonal = limits.getKinematics(
                Vector.fromPolar(Dist.fromIn(1), Angle.fromDeg(45)), Angle.zero());
        DirectionalKinematics strafe = limits.getKinematics(
                Vector.of(0, 1, DistUnit.IN), Angle.zero());

        assertEquals(constants.forwardVelLimitIn, forward.maxVel, 1e-6);
        assertEquals(constants.forwardAccelLimitIn, forward.maxAccel, 1e-6);
        assertEquals(constants.strafeVelLimitIn, strafe.maxVel, 1e-6);
        assertEquals(constants.strafeAccelLimitIn, strafe.maxAccel, 1e-6);
        assertTrue(diagonal.maxVel < strafe.maxVel);
        assertTrue(diagonal.maxAccel < strafe.maxAccel);
    }

    @Test
    public void testMecanumDirectionalKinematicsInterpolatesBetweenDegrees() {
        Mecanum.DirectionalLut limits = createMecanumLimits();
        DirectionalKinematics below = limits.getKinematics(
                Vector.fromPolar(Dist.fromIn(1), Angle.fromDeg(44.49)), Angle.zero());
        DirectionalKinematics above = limits.getKinematics(
                Vector.fromPolar(Dist.fromIn(1), Angle.fromDeg(44.51)), Angle.zero());
        assertTrue(Math.abs(above.maxVel - below.maxVel) < 0.01);
        assertTrue(Math.abs(above.maxAccel - below.maxAccel) < 0.01);
    }

    @Test
    public void testProfileProgressionUsesDistanceTraveled() {
        MotionParameters[] profile = generator.generate();
        PathPoint[] points = path.getGeneratedPoints();
        double pathLength = path.getParametricPath().getLengthIn();
        assertEquals(points.length, profile.length);
        for (int i = 0; i < profile.length; i++) {
            assertEquals(pathLength - points[i].getDistanceToEndIn(),
                    profile[i].getProgression(), 1e-6);
            if (i > 0) {
                assertTrue(profile[i].getProgression() >= profile[i - 1].getProgression());
            }
        }
    }

    private MotionParameters[] generateStraightMecanumProfile(double headingDeg) {
        Path straight = new HolonomicPathBuilder(
                geometry.pose(0, 0, headingDeg), geometry.pose(60, 0, headingDeg))
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING)
                .quickBuild();
        return new MecanumProfileGenerator(constants, straight).generate();
    }

    private Mecanum.DirectionalLut createMecanumLimits() {
        return new Mecanum.DirectionalLut(
                constants.forwardVelLimitIn, constants.forwardAccelLimitIn,
                constants.strafeVelLimitIn, constants.strafeAccelLimitIn);
    }

    private XYChart chart(String title, String xAxisTitle) {
        XYChart chart = new XYChartBuilder().width(1000).height(460)
                .title(title).xAxisTitle(xAxisTitle).build();
        chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
        chart.getStyler().setMarkerSize(0);
        chart.getStyler().setPlotGridLinesVisible(true);
        chart.getStyler().setPlotContentSize(0.92);
        chart.getStyler().setChartPadding(12);
        chart.getStyler().setXAxisDecimalPattern("0");
        chart.getStyler().setYAxisDecimalPattern("0.###");
        chart.getStyler().setDefaultSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Line);
        chart.getStyler().setLegendSeriesLineLength(44);
        return chart;
    }

    private void addSeries(XYChart chart, String name, double[] x, double[] y, int axis) {
        chart.addSeries(name, x, y).setYAxisGroup(axis);
    }

    private double[] constantArray(int length, double value) {
        double[] values = new double[length];
        for (int i = 0; i < length; i++) { values[i] = value; }
        return values;
    }

    private double average(double[] values) {
        double total = 0.0;
        for (double value : values) { total += value; }
        return values.length == 0 ? 0.0 : total / values.length;
    }

    private double max(double[] values) {
        double result = 0.0;
        for (double value : values) { result = Math.max(result, value); }
        return result;
    }

    private double maxVelocity(MotionParameters[] profile) {
        double result = 0.0;
        for (MotionParameters point : profile) {
            result = Math.max(result, point.getTangentialVel());
        }
        return result;
    }

    private void assertUsableProfile(String name, MotionParameters[] profile) {
        assertTrue(name + " profile should contain samples", profile.length > 2);
        double maxVelocity = 0.0;
        double maxPower = 0.0;
        for (MotionParameters point : profile) {
            assertTrue(Double.isFinite(point.getTangentialVel()));
            assertTrue(Double.isFinite(point.getTangentialAccel()));
            assertTrue(Double.isFinite(point.getAngularVel()));
            assertTrue(Double.isFinite(point.getAngularAccel()));
            assertTrue(Double.isFinite(point.getMotorPower()));
            assertTrue(point.getTangentialVel() >= -1e-6);
            maxVelocity = Math.max(maxVelocity, point.getTangentialVel());
            maxPower = Math.max(maxPower, point.getMotorPower());
        }
        assertTrue(name + " should move", maxVelocity > 1.0);
        assertTrue(name + " should stay near normalized power", maxPower <= 1.05);
    }
}
