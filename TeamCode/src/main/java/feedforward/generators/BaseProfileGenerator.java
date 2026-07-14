package feedforward.generators;

import java.util.ArrayList;
import java.util.List;

import core.FollowerConstants;
import feedforward.FeedforwardLut;
import feedforward.MotionParameters;
import geometry.PathPoint;
import geometry.Vector;
import paths.constraint.AngularConstraint;
import paths.constraint.PathConstraint;
import paths.constraint.PathConstraint.ConstraintType;
import paths.constraint.TranslationalConstraint;
import paths.movements.FollowerMovement;
import paths.movements.Path;

/**
 * Shared path-parameterized profile generator.
 *
 * <p>
 * Subclasses provide the drivetrain-specific power model through {@link #evaluatePoint}.
 * The shared algorithm does the drivetrain-independent work: sample the path, apply velocity
 * ceilings, run acceleration/deceleration sweeps, then iteratively lower any samples that still
 * exceed normalized power utilization.
 * </p>
 *
 * @author DrPixelCat - 7842 alum
 */
public abstract class BaseProfileGenerator {
    /** Normalized full power. Values above this mean the model predicts saturation. */
    private static final double UTILIZATION_LIMIT = 1.0;
    /** Small allowance so floating point noise does not create endless pinning. */
    private static final double UTILIZATION_TOLERANCE = 1e-3;
    /** Shared tiny value used to avoid divide-by-zero and unstable comparisons. */
    private static final double EPSILON = 1e-6;
    /** Binary-search depth for local velocity caps. */
    private static final int PIN_SEARCH_ITERATIONS = 20;

    /** Movement being profiled. This base class currently expects a {@link Path}. */
    protected FollowerMovement path;
    /** Debug information from the most recent full generation run. */
    private DebugReport lastReport;

    /**
     * @return diagnostics from the last call to {@link #generate()}
     */
    public DebugReport getLastDebugReport() {
        return lastReport;
    }

    // region Abstract Methods

    /**
     * Calculates the velocity ceiling at one path sample.
     * <p>
     * Subclasses fold in drivetrain geometry here. For example, mecanum uses direction-dependent
     * limits while tank only has forward and heading demand.
     *
     * @param point path sample being evaluated
     * @param path path that owns the sample
     * @param maxAngVel active angular velocity constraint
     * @param maxAngAccel active angular acceleration constraint
     * @return maximum tangential velocity allowed at this sample
     */
    protected abstract double calculateMaxTangentialVelocity(PathPoint point, Path path,
                                                             double maxAngVel, double maxAngAccel);

    /**
     * Evaluates normalized drivetrain utilization for the segment ending at {@code current}.
     * <p>
     * Inputs are the local kinematic state. Outputs split utilization into translational,
     * lateral/centripetal, and heading terms so the debug report can explain what saturated.
     */
    protected abstract void evaluatePoint(
            Path path, PathPoint prev, PathPoint current,
            double v_prev, double v, double a_t,
            EvaluationResult outResult
    );

    /**
     * Computes allowed braking acceleration for the backward pass.
     */
    protected abstract double getMaxTangentialAccel(double currentVel, PathPoint point, Path path
            , double maxAngAccel);

    /**
     * Computes allowed positive acceleration for the forward pass.
     */
    protected abstract double calculateDynamicMaxAccel(double currentVel, PathPoint point,
                                                       Path path, double maxAngAccel);

    // region Master Loop

    /**
     * Builds the first pass profile before the iterative pinning loop.
     * Useful for comparing the raw constraint sweep against {@link #generate()}.
     */
    public MotionParameters[] generateInitialProfile() {
        Path path = (Path) this.path;
        PathPoint[] points = path.getGeneratedPoints();

        MotionParameters[] outputParams = generateBasePass(points, path);
        runBackwardPass(outputParams, points, path);
        runForwardPass(outputParams, points, path);

        populateKinematicsAndPower(outputParams, points, path);
        return outputParams;
    }

    /**
     * Generates a velocity profile and tightens any point that exceeds drivetrain utilization.
     * <p>
     * The paper's central idea is that each target state {@code [v, a, omega, alpha]} should get
     * close to full normalized utilization without asking for more than the motors can provide.
     * The first sweep handles obvious velocity/acceleration constraints; the pinning loop cleans
     * up coupled cases where translation, centripetal force, and heading demand add to more than 1.
     */
    public MotionParameters[] generate() {
        if (!(path instanceof Path)) {
            throw new IllegalArgumentException("BaseFeedforwardGen only handles Path movements.");
        }

        Path path = (Path) this.path;
        PathPoint[] points = path.getGeneratedPoints();
        if (points == null || points.length == 0) {
            throw new IllegalStateException("Points must be set before generating.");
        }

        lastReport = new DebugReport();

        // Start with each point's local velocity ceiling, then enforce reachability both ways.
        MotionParameters[] outputParams = generateBasePass(points, path);
        runBackwardPass(outputParams, points, path);
        runForwardPass(outputParams, points, path);

        // Fill acceleration, omega, alpha, and power after the sweeps settle.
        ProfileEvaluation profileEval = populateKinematicsAndPower(outputParams, points, path);
        int maxIterations = Math.max(25, points.length * 2);
        int iterations = 0;

        // If any point still saturates, lower one nearby velocity and let both passes smooth it.
        while (profileEval.maxUtilization > UTILIZATION_LIMIT + UTILIZATION_TOLERANCE
                && iterations < maxIterations) {
            int checkIndex = Math.max(1, profileEval.worstIndex);
            int pinIndex = choosePinIndex(outputParams, checkIndex);

            // Reduce the closest useful velocity sample, then rerun both sweeps so the change
            // spreads through neighboring accel/decel constraints instead of making a hard notch.
            double previousVelocity = outputParams[pinIndex].getTangentialVel();
            double pinnedVelocity = findPinnedVelocity(
                    outputParams, points, path, pinIndex, checkIndex, previousVelocity
            );

            if (previousVelocity - pinnedVelocity < EPSILON) {
                pinnedVelocity = previousVelocity * 0.98;
            }

            outputParams[pinIndex].setTangentialVel(Math.max(0.0, pinnedVelocity));

            IterationLog log = new IterationLog();
            log.iteration = iterations + 1;
            log.pinnedIndex = pinIndex;
            log.previousVelocity = previousVelocity;
            log.newVelocity = outputParams[pinIndex].getTangentialVel();
            log.maxUtilization = profileEval.maxUtilization;
            log.totalPower = profileEval.totalPower;
            log.pForward = profileEval.pForward;
            log.pLateral = profileEval.pLateral;
            log.pHeading = profileEval.pHeading;
            lastReport.logs.add(log);

            runBackwardPass(outputParams, points, path);
            runForwardPass(outputParams, points, path);
            profileEval = populateKinematicsAndPower(outputParams, points, path);
            iterations++;
        }

        lastReport.iterationsRun = Math.max(1, iterations + 1);
        lastReport.finalMaxUtilization = profileEval.maxUtilization;
        lastReport.converged =
                profileEval.maxUtilization <= UTILIZATION_LIMIT + UTILIZATION_TOLERANCE;

        return outputParams;
    }

    /**
     * Calculates and populates the remaining kinematic variables (acceleration, angular velocity, power)
     * based on the final velocity sweep.
     * <p>
     * Main formulas:
     * {@code a = (v^2 - v_prev^2) / (2 * ds)},
     * {@code omega = dtheta/ds * v}, and
     * {@code alpha = d2theta/ds2 * v^2 + dtheta/ds * a}.
     * The last formula is the chain rule: heading changes with path distance, and path distance
     * changes with time.
     */
    private ProfileEvaluation populateKinematicsAndPower(MotionParameters[] lut, PathPoint[] points,
                                                         Path path) {
        EvaluationResult currentEval = new EvaluationResult();
        ProfileEvaluation profileEval = new ProfileEvaluation();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);
        lut[0].setTangentialAccel(0.0);
        lut[0].setAngularVel(0.0);
        lut[0].setAngularAccel(0.0);

        for (int i = 1; i < points.length; i++) {
            double ds = Math.abs(points[i].getDistanceToEndIn() - points[i - 1].getDistanceToEndIn());
            double v = lut[i].getTangentialVel();
            double v_prev = lut[i - 1].getTangentialVel();
            // Constant-acceleration kinematics in path-distance space.
            double a_t = (ds < EPSILON) ? 0.0 : ((v * v) - (v_prev * v_prev)) / (2.0 * ds);

            lut[i].setTangentialAccel(a_t);

            double s = points[i].getDistanceToEndIn();
            double kappa = points[i].getSignedCurvature();
            double dKappa = points[i].getCurvatureDerivative();

            double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
            double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);

            // theta_dot = dtheta/ds * ds/dt. theta_ddot also carries the path acceleration term.
            lut[i].setAngularVel(fPrime * v);
            lut[i].setAngularAccel((fDoublePrime * (v * v)) + (fPrime * a_t));

            evaluatePoint(path, points[i - 1], points[i], v_prev, v, a_t, currentEval);
            lut[i].setMotorPower(currentEval.totalPower);

            // Track the worst point so the pinning loop knows where to reduce speed.
            if (currentEval.maxUtilization > profileEval.maxUtilization) {
                profileEval.worstIndex = i;
                profileEval.maxUtilization = currentEval.maxUtilization;
                profileEval.totalPower = currentEval.totalPower;
                profileEval.pForward = currentEval.pForward;
                profileEval.pLateral = currentEval.pLateral;
                profileEval.pHeading = currentEval.pHeading;
            }
        }

        lut[0].setMotorPower(lut.length > 1 ? lut[1].getMotorPower() : 0.0);
        return profileEval;
    }

    /**
     * Generates a simple profile using only global forward acceleration limits.
     * <p>
     * This is cheaper than {@link #generate()} but does not try to solve the full coupled
     * utilization problem.
     *
     * @param config follower constants containing global limits
     * @return quick feedforward lookup table
     */
    public FeedforwardLut generateQuick(FollowerConstants config) {
        Path path = (Path) this.path;
        PathPoint[] points = path.getGeneratedPoints();
        MotionParameters[] lut = new MotionParameters[points.length];

        // Base pass: Max velocity everywhere
        for (int i = 0; i < points.length; i++) {
            lut[i] = new MotionParameters();
            lut[i].setTangentialVel(config.forwardVelLimitIn);
            lut[i].setDistAlongCurve(
                    path.getParametricPath().getLengthIn() - points[i].getDistanceToEndIn()
            );
        }

        // Backward pass: Naive deceleration
        // If boosted, relax the boundary condition so the path ends at cruising speed
        if (!path.isAccelBoosted()) {
            lut[points.length - 1].setTangentialVel(0.0);
        }

        for (int i = points.length - 2; i >= 0; i--) {
            double ds =
                    Math.abs(points[i + 1].getDistanceToEndIn() - points[i].getDistanceToEndIn());
            double nextVel = lut[i + 1].getTangentialVel();
            double maxReachableVel =
                    Math.sqrt((nextVel * nextVel) + (2.0 * config.forwardAccelLimitIn * ds));
            lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), maxReachableVel));
        }

        // Forward pass: Naive acceleration and populate angular targets
        // If boosted, relax the boundary condition so the path begins at cruising speed
        if (!path.isAccelBoosted()) {
            lut[0].setTangentialVel(0.0);
        }
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        for (int i = 1; i < points.length; i++) {
            double ds =
                    Math.abs(points[i].getDistanceToEndIn() - points[i - 1].getDistanceToEndIn());
            double prevVel = lut[i - 1].getTangentialVel();
            double maxReachableVel =
                    Math.sqrt((prevVel * prevVel) + (2.0 * config.forwardAccelLimitIn * ds));

            double v = Math.min(lut[i].getTangentialVel(), maxReachableVel);
            lut[i].setTangentialVel(v);

            double a_t = (ds < EPSILON) ? 0.0 : ((v * v) - (prevVel * prevVel)) / (2.0 * ds);
            lut[i].setTangentialAccel(a_t);

            double s = points[i].getDistanceToEndIn();
            double kappa = points[i].getSignedCurvature();
            double dKappa = points[i].getCurvatureDerivative();

            double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa,
                    finalTangent);
            double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                    finalTangent);

            lut[i].setAngularVel(fPrime * v);
            lut[i].setAngularAccel((fDoublePrime * (v * v)) + (fPrime * a_t));
        }

        return new FeedforwardLut(lut);
    }

    // region Fwd/Bkwd Passes

    /**
     * Creates the first velocity row for every sampled path point.
     * <p>
     * Path constraints are treated as stepwise: once the path progression passes a constraint's
     * {@code s}, that constraint remains active until another one overrides it.
     */
    private MotionParameters[] generateBasePass(PathPoint[] points, Path path) {
        MotionParameters[] lut = new MotionParameters[points.length];
        double pathLength_in = Math.max(path.getParametricPath().getLengthIn(), EPSILON);
        PathConstraint[] constraints = path.getConstraints();

        for (int i = 0; i < points.length; i++) {
            // Constraints are stepwise: the latest constraint whose s has been reached is active.
            double pctCompleted = 1.0 - (points[i].getDistanceToEndIn() / pathLength_in);
            double currentMaxVel = Double.MAX_VALUE;
            double currentMaxAngVel = Double.MAX_VALUE;
            double currentMaxAngAccel = Double.MAX_VALUE;
            for (PathConstraint constraint : constraints) {
                if (constraint.getS() <= pctCompleted) {
                    if (constraint instanceof TranslationalConstraint && constraint.getType() == ConstraintType.VELOCITY)  {
                        currentMaxVel = ((TranslationalConstraint) constraint).getValueIn();
                    } else if (constraint instanceof AngularConstraint && constraint.getType() == ConstraintType.VELOCITY) {
                        currentMaxAngVel = ((AngularConstraint) constraint).getValueRad();
                    } else if (constraint instanceof AngularConstraint && constraint.getType() == ConstraintType.ACCELERATION) {
                        currentMaxAngAccel = ((AngularConstraint) constraint).getValueRad();
                    }
                }
            }

            // Let the drivetrain-specific subclass translate heading/curvature demand into a
            // local top speed, then apply any explicit translational velocity constraint.
            double maxVel = calculateMaxTangentialVelocity(points[i], path, currentMaxAngVel, currentMaxAngAccel);
            if (currentMaxVel != Double.MAX_VALUE && currentMaxVel > 0.0) {
                maxVel = Math.min(maxVel, currentMaxVel);
            }

            lut[i] = new MotionParameters().setTangentialVel(maxVel);
            lut[i].setDistAlongCurve(
                    path.getParametricPath().getLengthIn() - points[i].getDistanceToEndIn()
            );
        }
        return lut;
    }

    /**
     * Sweeps from end to start so every point can decelerate into the next point.
     * <p>
     * The reachability equation is {@code v0 = sqrt(v1^2 + 2 * a * ds)}. After that normal
     * kinematic cap, the code optionally binary-searches the previous velocity against the full
     * power model because braking plus turning can still saturate.
     */
    private void runBackwardPass(MotionParameters[] lut, PathPoint[] points, Path path) {
        if (!path.isAccelBoosted()) {
            lut[points.length - 1].setTangentialVel(0.0);
        }

        double pathLength_in = Math.max(path.getParametricPath().getLengthIn(), EPSILON);
        PathConstraint[] constraints = path.getConstraints();

        for (int i = points.length - 2; i >= 0; i--) {
            double ds =
                    Math.abs(points[i + 1].getDistanceToEndIn() - points[i].getDistanceToEndIn());
            double nextVel = lut[i + 1].getTangentialVel();
            if (ds <= EPSILON) {
                lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), nextVel));
                continue;
            }

            double pctCompleted = 1.0 - (points[i + 1].getDistanceToEndIn() / pathLength_in);
            double currentMaxAccel = Double.MAX_VALUE;
            double currentMaxAngAccel = Double.MAX_VALUE;

            for (PathConstraint constraint : constraints) {
                if (constraint.getS() <= pctCompleted) {
                    if (constraint instanceof TranslationalConstraint &&
                            constraint.getType() == ConstraintType.ACCELERATION) {
                        currentMaxAccel = ((TranslationalConstraint) constraint).getValueIn();
                    } else if (constraint instanceof AngularConstraint &&
                            constraint.getType() == ConstraintType.ACCELERATION) {
                        currentMaxAngAccel = ((AngularConstraint) constraint).getValueRad();
                    }
                }
            }

            double maxAccel = getMaxTangentialAccel(nextVel, points[i + 1], path,
                    currentMaxAngAccel);
            if (currentMaxAccel != Double.MAX_VALUE && currentMaxAccel > 0.0) {
                maxAccel = Math.min(maxAccel, currentMaxAccel);
            }
            maxAccel = sanitizeNonNegative(maxAccel);

            double maxReachableVel = safeReachableVelocity(nextVel, maxAccel, ds);
            double cappedVel = Math.min(lut[i].getTangentialVel(), maxReachableVel);
            if (ds > EPSILON && cappedVel > nextVel + EPSILON) {
                // The kinematic cap can still overuse power once rotation/lateral load is included.
                cappedVel = findMaxPreviousVelocity(
                        path, points[i], points[i + 1], nextVel, cappedVel, maxAccel, ds
                );
            }
            lut[i].setTangentialVel(cappedVel);
        }
    }

    /**
     * Sweeps from start to end so every point can accelerate from the previous point.
     * <p>
     * This is the forward companion to {@link #runBackwardPass}. Together the two passes create a
     * profile that respects both start/end boundary conditions and local acceleration limits.
     */
    private void runForwardPass(MotionParameters[] lut, PathPoint[] points, Path path) {
        /* If boosted, relax the boundary condition so the path begins at cruising speed so the
        feedback controller can provide maximum +/- power */
        if (!path.isAccelBoosted()) {
            lut[0].setTangentialVel(0.0);
        }

        double pathLength_in = Math.max(path.getParametricPath().getLengthIn(), EPSILON);
        PathConstraint[] constraints = path.getConstraints();

        for (int i = 1; i < points.length; i++) {
            double ds =
                    Math.abs(points[i].getDistanceToEndIn() - points[i - 1].getDistanceToEndIn());
            double prevVel = lut[i - 1].getTangentialVel();
            if (ds <= EPSILON) {
                lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), prevVel));
                continue;
            }

            double pctCompleted = 1.0 - (points[i].getDistanceToEndIn() / pathLength_in);
            double currentMaxAccel = Double.MAX_VALUE;
            double currentMaxAngAccel = Double.MAX_VALUE;

            for (PathConstraint constraint : constraints) {
                if (constraint.getS() <= pctCompleted) {
                    if (constraint instanceof TranslationalConstraint && constraint.getType() == ConstraintType.ACCELERATION) {
                        currentMaxAccel = ((TranslationalConstraint) constraint).getValueIn();
                    } else if (constraint instanceof AngularConstraint && constraint.getType() == ConstraintType.ACCELERATION) {
                        currentMaxAngAccel = ((AngularConstraint) constraint).getValueRad();
                    }
                }
            }

            double dynamicAccel = calculateDynamicMaxAccel(prevVel, points[i], path,
                    currentMaxAngAccel);
            if (currentMaxAccel != Double.MAX_VALUE && currentMaxAccel > 0.0) {
                dynamicAccel = Math.min(dynamicAccel, currentMaxAccel);
            }
            dynamicAccel = sanitizeNonNegative(dynamicAccel);

            double maxReachableVel = safeReachableVelocity(prevVel, dynamicAccel, ds);
            double cappedVel = Math.min(lut[i].getTangentialVel(), maxReachableVel);
            if (ds > EPSILON && cappedVel > prevVel + EPSILON) {
                // Re-check the candidate segment against the full drivetrain power equation.
                cappedVel = findMaxNextVelocity(
                        path, points[i - 1], points[i], prevVel, cappedVel, dynamicAccel, ds
                );
            }
            lut[i].setTangentialVel(cappedVel);
        }
    }

    /**
     * Chooses which velocity sample to lower when a later sample is over-utilized.
     * <p>
     * During braking, lowering the point before the saturated sample is usually more useful than
     * lowering the already-braking sample itself.
     */
    private int choosePinIndex(MotionParameters[] lut, int worstIndex) {
        // If the worst point is braking, the velocity before it is usually the useful handle.
        int preferredIndex = worstIndex;
        if (worstIndex > 0 && lut[worstIndex].getTangentialAccel() < -EPSILON) {
            preferredIndex = worstIndex - 1;
        } else if (lut[worstIndex].getTangentialVel() <= EPSILON && worstIndex > 0) {
            preferredIndex = worstIndex - 1;
        }

        if (lut[preferredIndex].getTangentialVel() > EPSILON) {
            return preferredIndex;
        }

        for (int i = worstIndex - 1; i >= 0; i--) {
            if (lut[i].getTangentialVel() > EPSILON) {
                return i;
            }
        }

        for (int i = worstIndex + 1; i < lut.length; i++) {
            if (lut[i].getTangentialVel() > EPSILON) {
                return i;
            }
        }

        return preferredIndex;
    }

    /**
     * Binary-searches a pinned velocity that makes the checked point feasible after both sweeps.
     * <p>
     * A single point's velocity is not meaningful by itself: changing it can affect neighboring
     * samples through acceleration limits. That is why each candidate is tested after rerunning
     * the backward and forward passes.
     */
    private double findPinnedVelocity(MotionParameters[] currentProfile, PathPoint[] points,
                                      Path path, int pinIndex, int checkIndex,
                                      double currentVelocity) {
        if (currentVelocity <= EPSILON) {
            return 0.0;
        }

        double low = 0.0;
        double high = currentVelocity;
        double bestVelocity = 0.0;
        int boundedCheckIndex = Math.max(1, Math.min(checkIndex, currentProfile.length - 1));

        for (int i = 0; i < PIN_SEARCH_ITERATIONS; i++) {
            double candidate = (low + high) / 2.0;
            MotionParameters[] trialProfile = copyProfile(currentProfile);
            trialProfile[pinIndex].setTangentialVel(candidate);

            // Test the candidate in-context; a local velocity is only valid after both sweeps settle.
            runBackwardPass(trialProfile, points, path);
            runForwardPass(trialProfile, points, path);
            populateKinematicsAndPower(trialProfile, points, path);

            if (trialProfile[boundedCheckIndex].getMotorPower()
                    <= UTILIZATION_LIMIT + UTILIZATION_TOLERANCE) {
                bestVelocity = trialProfile[pinIndex].getTangentialVel();
                low = candidate;
            } else {
                high = candidate;
            }
        }

        return Math.min(bestVelocity, currentVelocity);
    }

    /**
     * Makes a mutable copy so binary-search candidates do not disturb the real profile.
     */
    private MotionParameters[] copyProfile(MotionParameters[] profile) {
        MotionParameters[] copy = new MotionParameters[profile.length];
        for (int i = 0; i < profile.length; i++) {
            copy[i] = new MotionParameters()
                    .setTangentialVel(profile[i].getTangentialVel())
                    .setTangentialAccel(profile[i].getTangentialAccel())
                    .setAngularVel(profile[i].getAngularVel())
                    .setAngularAccel(profile[i].getAngularAccel());
            copy[i].setMotorPower(profile[i].getMotorPower());
            copy[i].setDistAlongCurve(profile[i].getDistAlongCurve());
        }
        return copy;
    }

    /**
     * Finds the highest previous velocity that can decelerate into {@code nextVel} without
     * exceeding acceleration or power limits.
     */
    private double findMaxPreviousVelocity(Path path, PathPoint prevPoint,
                                           PathPoint currentPoint, double nextVel,
                                           double upperVel, double maxDecel, double ds) {
        if (upperVel <= nextVel + EPSILON) {
            return upperVel;
        }

        double low = Math.min(nextVel, upperVel);
        double high = upperVel;
        if (isBackwardTransitionFeasible(path, prevPoint, currentPoint, nextVel, high,
                maxDecel, ds)) {
            return high;
        }

        for (int i = 0; i < PIN_SEARCH_ITERATIONS; i++) {
            double candidate = (low + high) / 2.0;
            if (isBackwardTransitionFeasible(path, prevPoint, currentPoint, nextVel, candidate,
                    maxDecel, ds)) {
                low = candidate;
            } else {
                high = candidate;
            }
        }

        return low;
    }

    /**
     * Finds the highest next velocity that can be reached from {@code prevVel} without exceeding
     * acceleration or power limits.
     */
    private double findMaxNextVelocity(Path path, PathPoint prevPoint, PathPoint currentPoint,
                                       double prevVel, double upperVel, double maxAccel,
                                       double ds) {
        if (upperVel <= prevVel + EPSILON) {
            return upperVel;
        }

        double low = Math.min(prevVel, upperVel);
        double high = upperVel;
        if (isForwardTransitionFeasible(path, prevPoint, currentPoint, high, prevVel,
                maxAccel, ds)) {
            return high;
        }

        for (int i = 0; i < PIN_SEARCH_ITERATIONS; i++) {
            double candidate = (low + high) / 2.0;
            if (isForwardTransitionFeasible(path, prevPoint, currentPoint, candidate, prevVel,
                    maxAccel, ds)) {
                low = candidate;
            } else {
                high = candidate;
            }
        }

        return low;
    }

    /**
     * Checks a braking transition from {@code prevVel} to {@code nextVel}.
     */
    private boolean isBackwardTransitionFeasible(Path path, PathPoint prevPoint,
                                                 PathPoint currentPoint, double nextVel,
                                                 double prevVel, double maxDecel, double ds) {
        double accel = calculateTangentialAccel(prevVel, nextVel, ds);
        if (accel < -maxDecel - EPSILON) {
            return false;
        }
        return isPowerFeasible(path, prevPoint, currentPoint, prevVel, nextVel, accel);
    }

    /**
     * Checks an accelerating transition from {@code prevVel} to {@code nextVel}.
     */
    private boolean isForwardTransitionFeasible(Path path, PathPoint prevPoint,
                                                PathPoint currentPoint, double nextVel, double prevVel,
                                                double maxAccel, double ds) {
        double accel = calculateTangentialAccel(prevVel, nextVel, ds);
        if (accel > maxAccel + EPSILON) {
            return false;
        }
        return isPowerFeasible(path, prevPoint, currentPoint, prevVel, nextVel, accel);
    }

    /**
     * Runs the subclass power model and compares it against normalized full power.
     */
    private boolean isPowerFeasible(Path path, PathPoint prevPoint, PathPoint currentPoint,
                                    double prevVel, double nextVel, double accel) {
        EvaluationResult result = new EvaluationResult();
        evaluatePoint(path, prevPoint, currentPoint, prevVel, nextVel, accel, result);
        return result.maxUtilization <= UTILIZATION_LIMIT + UTILIZATION_TOLERANCE;
    }

    /**
     * Calculates constant tangential acceleration from two velocities over distance.
     */
    private double calculateTangentialAccel(double prevVel, double nextVel, double ds) {
        if (ds <= EPSILON) {
            return 0.0;
        }
        return ((nextVel * nextVel) - (prevVel * prevVel)) / (2.0 * ds);
    }

    /**
     * Computes {@code sqrt(v0^2 + 2*a*ds)} while guarding against tiny negative roundoff.
     */
    private double safeReachableVelocity(double startVelocity, double accel, double ds) {
        if (ds <= EPSILON) {
            return startVelocity;
        }

        double velocitySq = (startVelocity * startVelocity) + (2.0 * accel * ds);
        return Math.sqrt(Math.max(0.0, velocitySq));
    }

    /**
     * Converts invalid or negative acceleration caps into a safe zero cap.
     */
    private double sanitizeNonNegative(double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            return 0.0;
        }
        return value;
    }

    // region Logging and CSV Export

    /**
     * Output container for one drivetrain power evaluation.
     * <p>
     * {@code totalPower} and {@code maxUtilization} are normalized, where 1.0 means full available
     * power. The component fields are diagnostic terms.
     */
    public static class EvaluationResult {
        public double totalPower = 0.0;
        public double maxUtilization = 0.0;
        public double pForward = 0.0;
        public double pLateral = 0.0;
        public double pHeading = 0.0;

        /**
         * Copies another result into this object to avoid repeated allocations in hot loops.
         */
        public void copyFrom(EvaluationResult other) {
            this.totalPower = other.totalPower;
            this.maxUtilization = other.maxUtilization;
            this.pForward = other.pForward;
            this.pLateral = other.pLateral;
            this.pHeading = other.pHeading;
        }
    }

    /**
     * Summary of the iterative pinning phase from a generated profile.
     */
    public static class DebugReport {
        public int iterationsRun = 0;
        public boolean converged = false;
        public double finalMaxUtilization = 0.0;
        public List<IterationLog> logs = new ArrayList<>();

        /**
         * @return compact human-readable convergence summary
         */
        public String getSummary() {
            return String.format("Converged: %b | Iterations: %d | Final Max Util: %.3f",
                    converged, iterationsRun, finalMaxUtilization);
        }
    }

    /**
     * One iteration of the pinning loop.
     */
    public static class IterationLog {
        public int iteration;
        public int pinnedIndex;
        public double previousVelocity, newVelocity;
        public double maxUtilization, totalPower;
        public double pForward, pLateral, pHeading;
    }

    /**
     * Internal aggregate used to find the worst point in the current profile.
     */
    private static class ProfileEvaluation {
        int worstIndex = 0;
        double maxUtilization = 0.0;
        double totalPower = 0.0;
        double pForward = 0.0;
        double pLateral = 0.0;
        double pHeading = 0.0;
    }
}