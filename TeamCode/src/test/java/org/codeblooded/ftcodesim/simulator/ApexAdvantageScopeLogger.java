package org.codeblooded.ftcodesim.simulator;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

import java.util.concurrent.atomic.AtomicReference;

import core.FollowerDiagnostics;
import geometry.Angle;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.PathPoint;
import geometry.Pose;
import geometry.Vector;
import paths.movements.Path;

/** Publishes Apex follower geometry in AdvantageScope's FTC center/rotated coordinates. */
final class ApexAdvantageScopeLogger implements FollowerDiagnostics {
    static final String ESTIMATED_POSE_KEY = "Apex/EstimatedPose";
    static final String CURRENT_PATH_KEY = "Apex/CurrentPath";
    private static final double METERS_PER_INCH = 0.0254;

    private volatile Pose2d latestPose;
    private final AtomicReference<Pose2d[]> pendingPath =
            new AtomicReference<Pose2d[]>();

    @Override
    public void recordPose(Pose pose) {
        latestPose = toFtcPose(pose);
    }

    @Override
    public void recordCurrentPath(Path path) {
        pendingPath.set(toFtcPath(path));
    }

    @Override
    public void clearCurrentPath() {
        pendingPath.set(new Pose2d[0]);
    }

    /**
     * Writes queued snapshots from FTCodeSim's event-loop thread.
     *
     * <p>PsiKit caches one mutable StructBuffer per geometry type. FTCodeSim also logs Pose2d
     * values during its hardware update, so writing directly from the LinearOpMode thread races
     * that shared buffer and can produce a BufferOverflowException.</p>
     */
    void flush() {
        Pose2d pose = latestPose;
        if (pose != null) {
            Logger.recordOutput(ESTIMATED_POSE_KEY, pose);
        }

        Pose2d[] path = pendingPath.getAndSet(null);
        if (path != null) {
            Logger.recordOutput(CURRENT_PATH_KEY, path);
        }
    }

    static Pose2d[] toFtcPath(Path path) {
        PathPoint[] points = path.getGeneratedPoints();
        Pose2d[] poses = new Pose2d[points.length];
        if (points.length == 0) { return poses; }

        Vector finalTangent = points[points.length - 1].getFirstDerivative().normalize();
        for (int i = 0; i < points.length; i++) {
            PathPoint point = points[i];
            Angle heading = path.getInterpolator().getHeadingTarg(
                    point.getDistanceToEndIn(),
                    point.getFirstDerivative(),
                    finalTangent
            );
            poses[i] = toFtcPose(
                    point.getLocation().getX(DistUnit.IN),
                    point.getLocation().getY(DistUnit.IN),
                    heading.get(AngleUnit.RAD)
            );
        }
        return poses;
    }

    static Pose2d toFtcPose(Pose pose) {
        return toFtcPose(
                pose.getX(DistUnit.IN),
                pose.getY(DistUnit.IN),
                pose.getHeading(AngleUnit.RAD)
        );
    }

    private static Pose2d toFtcPose(double apexXInches, double apexYInches,
                                    double apexHeadingRadians) {
        // FTCodeSim publishes its centered FTC coordinates as (-Apex Y, Apex X), with heading
        // rotated by +90 degrees. Using the same transform keeps the path and localized pose
        // aligned with FTCodeSim's ground-truth robot on both AdvantageScope field views.
        return new Pose2d(
                -apexYInches * METERS_PER_INCH,
                apexXInches * METERS_PER_INCH,
                Rotation2d.fromRadians(apexHeadingRadians + Math.PI / 2.0)
        );
    }
}
