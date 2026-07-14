package paths.movements;

import java.util.ArrayList;
import java.util.List;

import feedforward.FeedforwardLut;
import geometry.Angle;
import geometry.Pose;
import paths.Callback;

/**
 * Represents a stationary point-turn movement. The robot will remain at its starting (x, y)
 * coordinates and rotate to the specified target heading.
 *
 * @author DrPixelCat - 7842 alum
 * @author Sohum Arora - 22985 Paraducks
 */
public class Turn extends FollowerMovement {
    private final Pose startPose;
    private final Pose endPose;
    private FeedforwardLut feedforwardLut = null;
    private final List<Callback> callbacks = new ArrayList<>();

    /**
     * Constructs a Turn movement.
     *
     * @param startPose The robot's state at the beginning of the turn.
     * @param targetHeading The final angle the robot should face.
     */
    public Turn(Pose startPose, Angle targetHeading) {
        this.startPose = startPose;
        // The end factory shares the same X/Y, but updates the heading
        this.endPose = new Pose(startPose.getVec(), targetHeading);
    }

    public void addCallback(Callback callback) { callbacks.add(callback); }

    public Callback[] getCallbacks() { return callbacks.toArray(new Callback[0]); }

    public Pose getStartPose() { return startPose; }

    public FeedforwardLut getFeedforwardLut() { return feedforwardLut; }

    public void setFeedforwardLut(FeedforwardLut feedforwardLut) {
        this.feedforwardLut = feedforwardLut;
    }

    @Override
    public Pose getEndPose() { return endPose; }
}