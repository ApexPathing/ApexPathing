package paths.movements;

import java.util.ArrayList;
import java.util.List;

import feedforward.FFLut;
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
    private FFLut FFLut;
    private final List<Callback> callbacks = new ArrayList<Callback>();

    /**
     * Constructs a Turn movement.
     *
     * @param startPose The robot's state at the beginning of the turn.
     * @param targetHeading The final angle the robot should face.
     */
    public Turn(Pose startPose, Angle targetHeading) {
        this.startPose = startPose;
        this.endPose = new Pose(startPose.getVec(), targetHeading);
    }

    public void addCallback(Callback callback) { callbacks.add(callback); }

    public Callback[] getCallbacks() { return callbacks.toArray(new Callback[0]); }

    public Pose getStartPose() { return startPose; }

    public FFLut getFeedforwardLut() { return FFLut; }

    public void setFeedforwardLut(FFLut FFLut) { this.FFLut = FFLut; }
}