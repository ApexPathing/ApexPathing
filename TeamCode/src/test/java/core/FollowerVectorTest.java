package core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import geometry.DistUnit;
import geometry.PathSegment;
import geometry.Vector;

public class FollowerVectorTest {
    @Test
    public void centripetalVectorPointsInsideForLeftAndRightCurves() {
        Vector tangent = Vector.of(1.0, 0.0, DistUnit.IN);

        Vector leftNormal = PathSegment.calculateArcNormal(
                tangent, Vector.of(0.0, 1.0, DistUnit.IN));
        Vector leftCorrection = Follower.calculateCentripetalCorrection(
                leftNormal, 20.0, 0.05, 0.01);
        assertEquals(0.0, leftCorrection.getX().getIn(), 1e-9);
        assertTrue(leftCorrection.getY().getIn() > 0.0);

        Vector rightNormal = PathSegment.calculateArcNormal(
                tangent, Vector.of(0.0, -1.0, DistUnit.IN));
        Vector rightCorrection = Follower.calculateCentripetalCorrection(
                rightNormal, 20.0, -0.05, 0.01);
        assertEquals(0.0, rightCorrection.getX().getIn(), 1e-9);
        assertTrue(rightCorrection.getY().getIn() < 0.0);
    }

    @Test
    public void centripetalMagnitudeDoesNotDependOnTurnDirection() {
        Vector left = Follower.calculateCentripetalCorrection(
                Vector.of(0.0, 1.0, DistUnit.IN), 30.0, 0.04, 0.002);
        Vector right = Follower.calculateCentripetalCorrection(
                Vector.of(0.0, -1.0, DistUnit.IN), 30.0, -0.04, 0.002);

        assertEquals(left.getMag().getIn(), right.getMag().getIn(), 1e-9);
        assertEquals(-left.getY().getIn(), right.getY().getIn(), 1e-9);
    }

    @Test
    public void crossTrackNormalExistsOnStraightSections() {
        Vector normal = PathSegment.calculateLeftNormal(
                Vector.of(4.0, 0.0, DistUnit.IN));

        assertEquals(0.0, normal.getX().getIn(), 1e-9);
        assertEquals(1.0, normal.getY().getIn(), 1e-9);
    }

    @Test
    public void crossTrackCorrectionDirectionIsIndependentOfWhichSideRobotIsOn() {
        Vector closestPoint = Vector.of(10.0, 0.0, DistUnit.IN);
        Vector leftNormal = PathSegment.calculateLeftNormal(
                Vector.of(1.0, 0.0, DistUnit.IN));

        Vector robotOnRight = Vector.of(10.0, -3.0, DistUnit.IN);
        double rightError = closestPoint.minus(robotOnRight).dot(leftNormal).getIn();
        Vector rightCorrection = leftNormal.times(rightError);
        assertTrue(rightCorrection.getY().getIn() > 0.0);

        Vector robotOnLeft = Vector.of(10.0, 3.0, DistUnit.IN);
        double leftError = closestPoint.minus(robotOnLeft).dot(leftNormal).getIn();
        Vector leftCorrection = leftNormal.times(leftError);
        assertTrue(leftCorrection.getY().getIn() < 0.0);
    }

    @Test
    public void profiledFeedforwardUsesAccelerationSignWhenStartingFromRest() {
        assertEquals(1.0, Follower.feedforwardMotionSign(0.0, 30.0), 1e-9);
        assertEquals(-1.0, Follower.feedforwardMotionSign(0.0, -30.0), 1e-9);
        assertEquals(1.0, Follower.feedforwardMotionSign(10.0, -30.0), 1e-9);
        assertEquals(0.0, Follower.feedforwardMotionSign(0.0, 0.0), 1e-9);
    }
}
