package paths.builders;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import core.FollowerConstants;
import feedforward.generators.TurnProfileGenerator;
import geometry.Angle;
import geometry.Pose;
import paths.Callback;
import paths.movements.Turn;

/**
 * A builder class designed to construct a {@link Turn} fluently.
 *
 * <p>
 * This handles stationary point-turns, allowing users to inject mechanical
 * callbacks at specific angles during the rotation.
 * </p>
 *
 * @author DrPixelCat
 */
public class TurnBuilder {
    private final Pose startPose;
    private Angle targetHeading = null;
    private final FollowerConstants constants;

    private double angularVelLimitRad;
    private double angularAccelLimitRad;

    private final List<Consumer<Turn>> buildTasks = new ArrayList<>();

    /**
     * Initializes the TurnBuilder with the robot's starting state and loads global limits.
     *
     * @param startPose The Pose of the robot before the turn begins.
     */
    public TurnBuilder(Pose startPose) {
        this.startPose = startPose;
        this.constants = FollowerConstants.getInstance();
        this.angularVelLimitRad = constants.angularVelLimitRad;
        this.angularAccelLimitRad = constants.angularAccelLimitRad;
    }

    /**
     * Defines the target angle for the point turn.
     *
     * @param targetHeading The Angle to rotate to.
     * @return The current TurnBuilder instance.
     */
    public TurnBuilder turnTo(Angle targetHeading) {
        this.targetHeading = targetHeading;
        return this;
    }

    /**
     * Attaches an executable callback to trigger when the robot passes a specific angle during
     * the turn.
     *
     * @param angle  The angle at which the callback should trigger.
     * @param action The code to execute.
     * @return The current TurnBuilder instance for method chaining.
     */
    public TurnBuilder addAngularCallback(Angle angle, Runnable action) {
        buildTasks.add(turn -> {
            Angle startRad = turn.getStartPose().getHeading();
            Angle endRad = turn.getEndPose().getHeading();

            double totalDiff = startRad.getShortestAngleTo(endRad).getRad();
            double targetDiff = startRad.getShortestAngleTo(angle).getRad();

            if (Math.abs(totalDiff) < 1e-6) {
                if (Math.abs(targetDiff) > 1e-6) {
                    throw new IllegalArgumentException("Callback out of bounds: The turn has no " +
                            "rotational distance.");
                }
            } else if ((totalDiff * targetDiff < 0) || (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                throw new IllegalArgumentException("Angular callback is outside the sweep range " +
                        "of this turn.");
            }

            turn.addCallback(new Callback(angle, action));
        });

        return this;
    }

    /**
     * Sets a custom angular velocity limit for this specific turn.
     *
     * @param limit The maximum angular velocity.
     * @return The current TurnBuilder instance for method chaining.
     */
    public TurnBuilder setAngularVelocityLimit(Angle limit) {
        if (limit.getRad() > constants.angularVelLimitRad) {
            throw new IllegalStateException("The angular velocity limit must be <= the " +
                    "drivetrain's max angular velocity constraint!");
        }
        this.angularVelLimitRad = limit.getRad();
        return this;
    }

    /**
     * Sets a custom angular acceleration limit for this specific turn.
     *
     * @param limit The maximum angular acceleration.
     * @return The current TurnBuilder instance for method chaining.
     */
    public TurnBuilder setAngularAccelerationLimit(Angle limit) {
        if (limit.getRad() > constants.angularAccelLimitRad) {
            throw new IllegalStateException("The angular acceleration limit must be <= the " +
                    "drivetrain's max angular acceleration constraint!");
        }
        this.angularAccelLimitRad = limit.getRad();
        return this;
    }

    /** Internal method to compile the turn and execute callback bounds checks. */
    private Turn compileTurn() {
        if (targetHeading == null) {
            throw new IllegalStateException("Cannot build Turn: No target heading was specified! " +
                    "Use .turnTo().");
        }

        Turn turn = new Turn(startPose, targetHeading);

        for (Consumer<Turn> task : buildTasks) {
            task.accept(turn);
        }

        return turn;
    }

    /**
     * Compiles the turn, verifies callback bounds, and returns the executable Turn movement without
     * motion profiling.
     *
     * @return The fully constructed {@link Turn}.
     */
    public Turn quickBuild() { return compileTurn(); }

    /**
     * Compiles the turn, verifies callback bounds, and returns the executable, profiled Turn
     * movement. It is recommended to use .quickBuild() instead for faster turns and generation
     * time. Profiles are not needed as much for {@link Turn} movements so much as Path movements.
     *
     * @return The fully constructed {@link Turn} with an attached feedforward profile.
     */
    public Turn profiledBuild() {
        Turn finalTurn = compileTurn();

        TurnProfileGenerator motionGen = new TurnProfileGenerator(
                angularVelLimitRad,
                angularAccelLimitRad,
                constants
        );

        // Ensure TurnProfileGenerator generates a compatible LUT for the Turn object
        finalTurn.setFeedforwardLut(motionGen.generate(finalTurn));

        return finalTurn;
    }
}
