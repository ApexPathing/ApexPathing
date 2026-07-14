package paths.builders;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import core.FollowerConstants;
import drivetrains.BaseDrivetrain;
import feedforward.FeedforwardLut;
import feedforward.generators.MecanumProfileGenerator;
import feedforward.generators.SwerveProfileGenerator;
import geometry.Angle;
import geometry.ArcPose;
import geometry.BSpline;
import geometry.CubicSpline1D;
import geometry.Dist;
import geometry.PathPoint;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.Callback;
import paths.heading.HeadingNode;
import paths.heading.InterpolationStyle;
import paths.heading.HolonomicInterpolator;
import paths.movements.Path;

/**
 * A builder class designed to construct a {@link Path} fluently for holonomic drivetrains.
 * This class captures path configurations (waypoints, interpolators, callbacks) in any order and
 * defers geometric and kinematic compilation until a build method is called.
 *
 * @author DrPixelCat - 7842 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class HolonomicPathBuilder extends PathBuilder<HolonomicPathBuilder> {
    private static final double EPSILON = 1e-6;

    private final Pose startPose;
    private final Pose expectedEndPose;
    private Dist blendWindow = null;

    private Angle customOffset = null;
    private Vector facingPoint = null;

    private final List<HeadingNode> headingNodes = new ArrayList<>();

    /**
     * Creates a new HolonomicPathBuilder using the provided poses.
     *
     * @param poses A sequence of Pose objects defining the path. Must contain at least two poses.
     *              Endpoints cannot be ArcPoses.
     */
    public HolonomicPathBuilder(Pose... poses) {
        super(Path.PathType.HOLONOMIC, poses);

        this.startPose = poses[0];
        this.expectedEndPose = poses[poses.length - 1];
    }

    @Override
    public HolonomicPathBuilder interpolateWith(InterpolationStyle style) {
        this.style = style;
        return this;
    }

    @Override
    public HolonomicPathBuilder interpolateWith(InterpolationStyle style, Angle angleOffset) {
        this.style = style;
        this.customOffset = angleOffset;
        return this;
    }

    @Override
    public HolonomicPathBuilder interpolateWith(InterpolationStyle style, Vector pointToFace) {
        return interpolateWith(style, pointToFace, Angle.zero());
    }

    @Override
    public HolonomicPathBuilder interpolateWith(InterpolationStyle style, Vector pointToFace,
                                                Angle angleOffset) {
        this.style = style;
        this.facingPoint = pointToFace != null ? pointToFace.copy() : null;
        this.customOffset = angleOffset != null ? angleOffset : Angle.zero();
        return this;
    }

    @Override
    public HolonomicPathBuilder addHeadingNode(double pct, Angle target) {
        this.style = InterpolationStyle.NODE_BASED;
        this.headingNodes.add(new HeadingNode(pct, target));
        return this;
    }

    @Override
    public HolonomicPathBuilder setDistanceToStartFinalTurn(Dist distanceFromEnd) {
        this.blendWindow = distanceFromEnd;
        return this;
    }

    @Override
    public HolonomicPathBuilder addAngularCallback(Angle angle, Runnable action) {
        buildTasks.add(() -> {
            if (style == InterpolationStyle.SMOOTH_START_TO_END) {
                Angle startRad = startPose.getHeading();
                Angle endRad = expectedEndPose.getHeading();

                if (Double.isFinite(startRad.getRad()) && Double.isFinite(endRad.getRad())) {
                    double totalDiff = startRad.getShortestAngleTo(endRad).getRad();
                    double targetDiff = startRad.getShortestAngleTo(angle).getRad();

                    if (Math.abs(totalDiff) < 1e-6) {
                        if (Math.abs(targetDiff) > 1e-6) {
                            throw new IllegalArgumentException("Angular callback out of bounds: " +
                                    "The path's target heading is constant.");
                        }
                    } else if ((totalDiff * targetDiff < 0) ||
                            (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                        throw new IllegalArgumentException("Angular callback is outside the sweep" +
                                " range of the start and end headings.");
                    }
                }
            }
            path.addCallback(new Callback(angle, action));
        });
        return this;
    }

    private CubicSpline1D buildHeadingSpline(List<HeadingNode> nodes, String interpolationName) {
        Collections.sort(nodes);

        if (nodes.size() < 2) {
            throw new IllegalStateException(interpolationName + " interpolation requires at least" +
                    " a start and end heading.");
        }

        double[] x = new double[nodes.size()];
        double[] y = new double[nodes.size()];

        x[0] = nodes.get(0).pct;
        y[0] = nodes.get(0).target.getRad();

        // Unwrap shortest delta to ensure smooth continuous math across 2-PI bounds
        for (int i = 1; i < nodes.size(); i++) {
            x[i] = nodes.get(i).pct;
            double shortestDelta =
                    Angle.fromRad(y[i - 1]).getShortestAngleTo(nodes.get(i).target).getRad();
            y[i] = y[i - 1] + shortestDelta;
        }

        return new CubicSpline1D(x, y);
    }

    private void appendFacingSample(ArrayList<Double> pcts, ArrayList<Angle> headings,
                                    double pct, Angle heading) {
        double clampedPct = Math.min(Math.max(pct, 0.0), 1.0);

        if (!pcts.isEmpty()) {
            int last = pcts.size() - 1;
            double lastPct = pcts.get(last);
            if (clampedPct < lastPct + EPSILON) {
                if (Math.abs(clampedPct - lastPct) < EPSILON &&
                        (heading != null || headings.get(last) == null)) {
                    pcts.set(last, clampedPct);
                    headings.set(last, heading);
                }
                return;
            }
        }

        pcts.add(clampedPct);
        headings.add(heading);
    }

    private List<HeadingNode> buildFacingPointNodes(PathSegment curve, Vector pointToFace,
                                                    Angle angleOffset) {
        double pathLength = curve.getLengthIn();
        if (pathLength < EPSILON) {
            throw new IllegalStateException("FACING_POINT interpolation requires a non-zero " +
                    "path length.");
        }

        PathPoint[] samples = curve.getPointLUT();
        ArrayList<Double> pcts = new ArrayList<>(samples.length);
        ArrayList<Angle> headings = new ArrayList<>(samples.length);
        Angle offset = angleOffset != null ? angleOffset : Angle.zero();

        for (PathPoint sample : samples) {
            double pct = 1.0 - (sample.getDistanceToEndIn() / pathLength);
            Vector toPoint = pointToFace.minus(sample.getLocation());
            Angle heading = null;
            if (toPoint.getMag().getIn() > EPSILON) {
                heading = toPoint.getTheta().plus(offset);
            }
            appendFacingSample(pcts, headings, pct, heading);
        }

        Angle lastValid = null;
        boolean hasValid = false;
        for (int i = 0; i < headings.size(); i++) {
            if (headings.get(i) != null) {
                lastValid = headings.get(i);
                hasValid = true;
            } else if (lastValid != null) {
                headings.set(i, lastValid);
            }
        }

        if (!hasValid) {
            throw new IllegalStateException("FACING_POINT interpolation cannot face a point that " +
                    "matches every sampled path position.");
        }

        Angle nextValid = null;
        for (int i = headings.size() - 1; i >= 0; i--) {
            if (headings.get(i) != null) {
                nextValid = headings.get(i);
            } else if (nextValid != null) {
                headings.set(i, nextValid);
            }
        }

        ArrayList<HeadingNode> nodes = new ArrayList<>(pcts.size());
        for (int i = 0; i < pcts.size(); i++) {
            nodes.add(new HeadingNode(pcts.get(i), headings.get(i)));
        }
        return nodes;
    }

    /**
     * Internal method to compile the B-Spline geometry, process arc poses, and initialize the
     * interpolator.
     */
    private void compileGeometry() {
        ArrayList<Pose> processedPoses = new ArrayList<>(rawPoses.length * 2);
        processedPoses.add(rawPoses[0]);

        boolean intermediateWarningSent = false;

        for (int i = 1; i < rawPoses.length - 1; i++) {
            Pose currentPose = rawPoses[i];

            if (!intermediateWarningSent && Double.isFinite(currentPose.getHeading().getRad())) {
                String headingSource = style == InterpolationStyle.FACING_POINT ?
                        "FACING_POINT derives headings from the point to face." :
                        "Only the final factory heading controls the end heading.";
                path.addWarning("APEX WARNING: Intermediate B-Spline headings are currently " +
                        "ignored! " + headingSource);
                intermediateWarningSent = true;
            }

            if (currentPose instanceof ArcPose) {
                ArcPose arcPose = (ArcPose) currentPose;
                double radius = arcPose.getRadius().getIn();

                if (radius < 2.0) {
                    throw new IllegalArgumentException("ArcPose radius must be at least 2.0 " +
                            "inches.");
                }

                Pose prevPose = rawPoses[i - 1];
                Pose nextPose = rawPoses[i + 1];

                Vector vecToLast = prevPose.getVec().minus(arcPose.getVec());
                Vector vecToNext = nextPose.getVec().minus(arcPose.getVec());

                double distToLast = vecToLast.getMag().getIn();
                double distToNext = vecToNext.getMag().getIn();

                if (radius > distToLast || radius > distToNext) {
                    throw new IllegalArgumentException("ArcPose radius exceeds distance to " +
                            "adjacent control points.");
                }

                Vector p1Vec = arcPose.getVec().plus(vecToLast.times(radius / distToLast));
                Vector p2Vec = arcPose.getVec().plus(vecToNext.times(radius / distToNext));

                processedPoses.add(new Pose(p1Vec, arcPose.getHeading()));
                processedPoses.add(currentPose);
                processedPoses.add(new Pose(p2Vec, arcPose.getHeading()));
            } else {
                processedPoses.add(currentPose);
            }
        }

        processedPoses.add(rawPoses[rawPoses.length - 1]);

        Vector[] vectors = new Vector[processedPoses.size()];
        for (int i = 0; i < processedPoses.size(); i++) {
            vectors[i] = processedPoses.get(i).getVec();
        }

        PathSegment curve = new PathSegment(new BSpline(vectors));
        path.setParametricPath(curve);

        Angle startH = startPose.getHeading();
        Angle endH = expectedEndPose.getHeading();

        CubicSpline1D spline = null;
        boolean missingParams =
                (style == InterpolationStyle.CONSTANT_START_HEADING && !startH.isFinite() ||
                        (style == InterpolationStyle.CONSTANT_END_HEADING && !endH.isFinite()) ||
                        (style == InterpolationStyle.TANGENT_CUSTOM &&
                                (customOffset == null || !customOffset.isFinite())) ||
                        (style == InterpolationStyle.FACING_POINT && (!facingPoint.isFinite() ||
                                (customOffset != null && !customOffset.isFinite())))) ||
                        (style == InterpolationStyle.SMOOTH_START_TO_END && (!startH.isFinite() ||
                                !endH.isFinite()));

        if (missingParams) {
            path.addWarning("APEX WARNING: " + style.name() + " is missing required " +
                    "parameters! Falling back to TANGENT_FORWARD.");
            style = InterpolationStyle.TANGENT_FORWARD;
        }

        if (style == InterpolationStyle.TANGENT_OPTIMAL) {
            if (!Double.isFinite(startH.getRad())) {
                style = InterpolationStyle.TANGENT_FORWARD;
            } else {
                // Resolve once here so the runtime interpolator only handles concrete headings.
                Vector startTangent = curve.getFirstDerivative(0.0);
                double fwdError =
                        Math.abs(startH.getShortestAngleTo(startTangent.getTheta()).getRad());
                double bwdError =
                        Math.abs(startH.getShortestAngleTo(
                                startTangent.getTheta().plus(Angle.fromRad(Math.PI))).getRad());
                if (bwdError < fwdError) {
                    style = InterpolationStyle.TANGENT_CUSTOM;
                    customOffset = Angle.fromRad(Math.PI);
                } else {
                    style = InterpolationStyle.TANGENT_FORWARD;
                }
            }
        }

        if (style == InterpolationStyle.NODE_BASED) {
            ArrayList<HeadingNode> nodes = new ArrayList<>(headingNodes);

            // Automatically inject path boundary headings if the user didn't explicitly define them
            boolean hasStart = false;
            boolean hasEnd = false;
            for (HeadingNode node : nodes) {
                if (Math.abs(node.pct - 0.0) < EPSILON) hasStart = true;
                if (Math.abs(node.pct - 1.0) < EPSILON) hasEnd = true;
            }

            if (!hasStart && Double.isFinite(startH.getRad())) {
                nodes.add(new HeadingNode(0.0, startH));
            }
            if (!hasEnd && Double.isFinite(endH.getRad())) {
                nodes.add(new HeadingNode(1.0, endH));
            }

            spline = buildHeadingSpline(nodes, "NODE_BASED");
        } else if (style == InterpolationStyle.FACING_POINT) {
            Angle facingOffset = customOffset != null ? customOffset : Angle.zero();
            List<HeadingNode> facingNodes = buildFacingPointNodes(curve, facingPoint, facingOffset);
            spline = buildHeadingSpline(facingNodes, "FACING_POINT");
        }

        HolonomicInterpolator interpolator = new HolonomicInterpolator(style, startH, endH
                , customOffset, spline);
        interpolator.setPathLength(curve.getLengthIn());
        if (blendWindow != null) {
            interpolator.setBlendWindow(blendWindow.getIn());
        }
        path.setInterpolator(interpolator);
        path.setEndPose(expectedEndPose);

        for (Runnable task : buildTasks) {
            task.run();
        }
    }

    @Override
    public Path quickBuild() {
        compileGeometry();
        path.setFeedforwardLut(null);
        return path;
    }

    @Override
    public Path profiledBuild() {
        compileGeometry();
        FollowerConstants constants = FollowerConstants.getInstance();

        if (constants.drivetrainType == BaseDrivetrain.DrivetrainType.COAXIAL_SWERVE) {
            SwerveProfileGenerator generator = new SwerveProfileGenerator(constants, path);
            path.setFeedforwardLut(new FeedforwardLut(generator.generate()));
        } else {
            MecanumProfileGenerator generator = new MecanumProfileGenerator(constants, path);
            path.setFeedforwardLut(new FeedforwardLut(generator.generate()));
        }

        return path;
    }
}