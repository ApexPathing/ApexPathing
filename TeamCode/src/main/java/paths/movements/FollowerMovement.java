package paths.movements;

import geometry.Pose;

/**
 * Base class for {@link Path} and {@link Turn} movements.
 *
 * @author Sohum Arora - 22985 Paraducks
 */
public abstract class FollowerMovement {
    private boolean started = false;
    private boolean ended = false;

    /**
     * Gets the expected final pose of the robot after this movement is completed. Generally, this
     * is used to get the start pose of the next movement in a sequence.
     *
     * @return the final Pose.
     */
    public abstract Pose getEndPose();

    public boolean hasStarted() { return started; }

    public boolean hasEnded() { return ended; }

    public void setStarted(boolean started) { this.started = started; }

    public void setEnded(boolean ended) { this.ended = ended; }

    public Path toPath() { return (Path) this; }

    public Turn toTurn() { return (Turn) this; }
}