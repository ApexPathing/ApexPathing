package paths.movements;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import feedforward.FeedforwardLut;
import geometry.PathPoint;
import geometry.PathSegment;
import geometry.Pose;
import paths.Callback;
import paths.constraint.PathConstraint;
import paths.constraint.PathConstraint.ConstraintType;
import paths.constraint.TranslationalConstraint;
import paths.heading.HeadingInterpolator;

/**
 * Represents a complete, navigable geometric route for the robot to follow.
 *
 * <p>
 * A {@code Path} encapsulates a continuous parametric curve (e.g., a B-Spline),
 * its associated heading interpolation strategy, and any scheduled mechanical
 * callbacks triggered along the route.
 * <p>
 *
 * @author DrPixelCat - 7842 alum
 * @author Sohum Arora - 22985 Paraducks
 */
public class Path extends FollowerMovement {
    private final List<String> buildWarnings = new ArrayList<>();
    private final ArrayList<Callback> callbacks = new ArrayList<>();
    private final ArrayList<PathConstraint> constraints = new ArrayList<>();

    private PathSegment parametricPath;
    private HeadingInterpolator interpolator;
    private Pose endPose;
    private FeedforwardLut feedforwardLut;
    private boolean isAccelBoosted = false;

    public enum PathType {
        HOLONOMIC,
        TANK
    }

    private final PathType pathType;

    /**
     * Creates a path object for the robot to follow
     *
     * @param pathType {@link PathType}: HOLONOMIC, or TANK
     */
    public Path(PathType pathType) { this.pathType = pathType; }

    /** Attaches an executable action to this path. */
    public void addCallback(Callback callback) { callbacks.add(callback); }

    /** @return An array of callbacks scheduled along the path. */
    public Callback[] getCallbacks() { return callbacks.toArray(new Callback[0]); }

    /** @param constraint The kinematic constraint to apply */
    public void addConstraint(PathConstraint constraint) { constraints.add(constraint); }

    /** @return the path's kinematic constraints */
    public PathConstraint[] getConstraints() { return constraints.toArray(new PathConstraint[0]); }

    /**
     * Evaluates the active velocity constraints for unprofiled quick builds. Acts as a step
     * function: the velocity limit changes when the robot crosses a constraint's 't' threshold.
     *
     * @param t The current path percentage [0.0, 1.0].
     * @param defaultLimit The hardware maximum velocity from FollowerConstants.
     * @return The active velocity limit in inches per second.
     */
    public double getQuickVelocityLimit(double t, double defaultLimit) {
        double currentLimit = defaultLimit;
        double highestS = -1.0;

        for (PathConstraint baseConstraint : constraints) {
            if (baseConstraint instanceof TranslationalConstraint) {
                TranslationalConstraint constraint = (TranslationalConstraint) baseConstraint;
                if (constraint.getType() == ConstraintType.VELOCITY) {
                    if (t >= constraint.getS() && constraint.getS() > highestS) {
                        currentLimit = constraint.getValueIn();
                        highestS = constraint.getS();
                    }
                }
            }
        }
        return currentLimit;
    }

    /** @param endPose The final target pose of this path. */
    public void setEndPose(Pose endPose) {this.endPose = endPose;}

    /** @return The final target pose of this path. */
    public Pose getEndPose() { return endPose; }

    /** @return The generated LUT points from the ParametricPath */
    public PathPoint[] getGeneratedPoints() { return parametricPath.getPointLUT().clone(); }

    /**
     * Injects the calculated geometric curve (e.g., a B-Spline) that defines
     * the physical route the robot will drive.
     *
     * @param path The compiled path segment.
     */
    public void setParametricPath(PathSegment path) { this.parametricPath = path; }

    /**
     * Retrieves the geometric curve defining the physical route.
     *
     * @return The compiled path segment.
     */
    public PathSegment getParametricPath() { return parametricPath; }

    /** @param interpolator The heading interpolator to use */
    public void setInterpolator(HeadingInterpolator interpolator) {
        this.interpolator = interpolator;
    }

    /** @return The heading interpolator. */
    public HeadingInterpolator getInterpolator() { return interpolator; }

    /** @return The type of path: HOLONOMIC, or TANK */
    public PathType getPathType() { return pathType; }

    /** @return The feedforward look-up table */
    public FeedforwardLut getFeedforwardLut() { return feedforwardLut; }

    /** @param feedforwardLut The path's motion profile as a {@link FeedforwardLut} */
    public void setFeedforwardLut(FeedforwardLut feedforwardLut) {
        this.feedforwardLut = feedforwardLut;
    }

    /** @return true if built with profiledBuild(), false if built with quickBuild() */
    public boolean isProfiled() { return feedforwardLut != null; }

    /** Determines if this path should be followed with boosted acceleration. */
    public void useBoostedAccel() { isAccelBoosted = true; }

    /** @return Whether the path should be followed with boosted acceleration or not */
    public boolean isAccelBoosted() { return isAccelBoosted; }

    /**
     * Logs a non-fatal warning generated during the path building process (e.g., missing headings,
     * ignored waypoints. Duplicate warnings are ignored to prevent telemetry spam on the driver
     * station.
     *
     * @param warning The warning string.
     */
    public void addWarning(String warning) {
        if (!buildWarnings.contains(warning)) {
            buildWarnings.add(warning);
        }
    }

    /** @return A read-only list of warning strings. */
    public List<String> getWarnings() { return Collections.unmodifiableList(buildWarnings); }
}