package paths.builders;

import java.util.ArrayList;
import java.util.List;

import geometry.Angle;
import geometry.ArcPose;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import paths.Callback;
import paths.constraint.PathConstraint;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

/**
 * Provides shared functionality and structure for building paths for all drivetrain types.
 *
 * @author DrPixelCat - 7842 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class PathBuilder<T extends PathBuilder<T>> {
    protected Path path;
    protected InterpolationStyle style = InterpolationStyle.TANGENT_FORWARD;

    final Pose[] rawPoses;
    final List<Runnable> buildTasks = new ArrayList<>();

    /**
     * Creates a new PathBuilder using the provided poses.
     *
     * @param type The type of the path to be built.
     * @param poses A sequence of Pose objects defining the path. Must contain at least two poses.
     *              Endpoints cannot be ArcPoses.
     */
    public PathBuilder(Path.PathType type, Pose... poses) {
        this.rawPoses = poses;
        this.path = new Path(type);
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }
    }

    /**
     * Adds a kinematic constraint to the path at a specific distance percentage.
     * <p><strong> NOTE: Only velocity can be limited on a quickBuild </strong></p>
     *
     * @param constraint The {@link PathConstraint} to be added to the path
     * @return The current PathBuilder instance for method chaining.
     */
    @SuppressWarnings("unchecked")
    protected T addConstraint(PathConstraint constraint) {
        if (constraint.getS() >= 1.0 || constraint.getS() < 0.0) {
            constraint.setS(Math.min(Math.max(constraint.getS(), 0.0), 0.9));
            path.addWarning("s must be within [0, 1) bounds! Normalized to " + constraint.getS() +
                    " for safety.");
        }
        path.addConstraint(constraint);
        return (T) this;
    }

    /**
     * Overrides the default (SMOOTH_START_TO_END for holonomic, TANGENT_FORWARD for tank)
     * interpolation with a different {@link InterpolationStyle}. If a non-tangent interpolation
     * style is used for a tank drivetrain, an {@link IllegalArgumentException} will be thrown.
     *
     * @param style The interpolation style to apply.
     * @return The current PathBuilder instance for method chaining.
     */
    public abstract T interpolateWith(InterpolationStyle style);

    /**
     * Overrides the interpolation style, providing a custom angular offset. Used primarily for
     * {@link InterpolationStyle#TANGENT_CUSTOM}. This method is only available for holonomic
     * drivetrains.
     *
     * @param style The interpolation style to apply.
     * @param angleOffset The fixed angle to offset the calculation by.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder interpolateWith(InterpolationStyle style, Angle angleOffset) {
        throw new UnsupportedOperationException(
                "This method is only available for holonomic drivetrains!"
        ); // The HolonomicPathBuilder overrides this method.
    }

    /**
     * Overrides the interpolation style, providing a fixed field point to face. Used primarily for
     * {@link InterpolationStyle#FACING_POINT}. This method is only available for holonomic
     * drivetrains.
     *
     * @param style The interpolation style to apply.
     * @param pointToFace The field coordinate the robot should face.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder interpolateWith(InterpolationStyle style, Vector pointToFace) {
        throw new UnsupportedOperationException(
                "This method is only available for holonomic drivetrains!"
        ); // The HolonomicPathBuilder overrides this method.
    }

    /**
     * Overrides the interpolation style, providing a fixed field point and angular offset. Used
     * primarily for {@link InterpolationStyle#FACING_POINT}. This method is only available for
     * holonomic drivetrains.
     *
     * @param style The interpolation style to apply.
     * @param pointToFace The field coordinate the robot should face.
     * @param angleOffset The fixed angle to offset the facing direction by.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder interpolateWith(InterpolationStyle style, Vector pointToFace,
                                                Angle angleOffset) {
        throw new UnsupportedOperationException(
                "This method is only available for holonomic drivetrains!"
        ); // The HolonomicPathBuilder overrides this method.
    }

    /**
     * Adds a heading node for NODE_BASED interpolation. Automatically sets the interpolation style
     * to NODE_BASED. This method is only available for holonomic drivetrains.
     *
     * @param pct The distance percentage [0.0, 1.0].
     * @param target The target Angle at this point.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder addHeadingNode(double pct, Angle target) {
        throw new UnsupportedOperationException(
                "This method is only available for holonomic drivetrains!"
        ); // The HolonomicPathBuilder overrides this method.
    }

    /**
     * Sets how far from the end of the path the robot should start rotating to face its final
     * target direction. This method is only available for holonomic drivetrains.
     *
     * @param distanceFromEnd The distance away from the end of the path.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder setDistanceToStartFinalTurn(Dist distanceFromEnd) {
        throw new UnsupportedOperationException(
                "This method is only available for holonomic drivetrains!"
        ); // The HolonomicPathBuilder overrides this method.
    }

    /**
     * Attaches an executable callback based on the physical distance percentage.
     *
     * @param s The physical distance percentage [0.0, 1.0].
     * @param action The method to execute when the robot reaches the specified distance.
     * @return The current PathBuilder instance for method chaining.
     */
    @SuppressWarnings("unchecked")
    public T addDistanceCallback(double s, Runnable action) {
        this.buildTasks.add(() -> path.addCallback(new Callback(s, action)));
        return (T) this; // Safe cast because T is always a subclass of PathBuilder
    }

    /**
     * Attaches an executable callback based on the robot reaching a target heading.
     *
     * @param angle The Angle at which the callback should trigger.
     * @param action The method to execute when the robot reaches the specified heading.
     * @return The current PathBuilder instance for method chaining.
     */
    public abstract T addAngularCallback(Angle angle, Runnable action);

    /**
     * Builds the path geometry without generating a physical motion profile. The follower will
     * automatically use dynamic velocity-bounded feedback.
     *
     * @return The constructed Path.
     */
    public abstract Path quickBuild();

    /**
     * Builds the path geometry and solves a complete kinematically constrained feedforward
     * motion profile.
     *
     * @return The constructed Path with a fully optimized FeedforwardLut attached.
     */
    public abstract Path profiledBuild();
}