package org.codeblooded.ftcodesim.simulator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.psilynx.psikit.core.wpi.math.Pose2d;

import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import paths.builders.HolonomicPathBuilder;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

public class ApexAdvantageScopeLoggerTest {
    private static final double EPSILON = 1e-6;

    @Test
    public void convertsApexPoseToFtCodeSimFieldCoordinates() {
        Pose2d converted = ApexAdvantageScopeLogger.toFtcPose(new Pose(
                Vector.of(10.0, 20.0, DistUnit.IN),
                Angle.zero()
        ));

        assertEquals(-0.508, converted.getX(), EPSILON);
        assertEquals(0.254, converted.getY(), EPSILON);
        assertEquals(Math.PI / 2.0, converted.getRotation().getRadians(), EPSILON);
    }

    @Test
    public void convertsGeneratedPathToStructuredPoseArray() {
        Path path = new HolonomicPathBuilder(
                new Pose(Vector.zero(), Angle.zero()),
                new Pose(Vector.of(10.0, 0.0, DistUnit.IN), Angle.zero())
        ).interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild();

        Pose2d[] converted = ApexAdvantageScopeLogger.toFtcPath(path);

        assertTrue(converted.length > 2);
        assertEquals(0.0, converted[0].getX(), EPSILON);
        assertEquals(0.0, converted[0].getY(), EPSILON);
        assertEquals(0.0, converted[converted.length - 1].getX(), EPSILON);
        assertEquals(0.254, converted[converted.length - 1].getY(), EPSILON);
        assertEquals(Math.PI / 2.0,
                converted[converted.length - 1].getRotation().getRadians(), EPSILON);
    }
}
