package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import controllers.PDSController.PDSCoefficients;
import controllers.PDSController;
import geometry.Pose;

/**
 * Base class for an Apex Pathing tuning phase.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class TuningPhase {
    private State state = State.INIT;
    protected final TunerContext context;
    private boolean manualMode;

    protected TuningPhase(TunerContext context) { this.context = context; }

    enum State { INIT, RUNNING, COMPLETE }

    /**
     * Updates the tuning phase. This method should be called repeatedly until it returns true.
     */
    public final boolean update(boolean aWasPressed, boolean bPressed) {
        switch (state) {
            case INIT:
                boolean initialized = initLoop(aWasPressed, bPressed);
                if (initialized) {
                    state = State.RUNNING;
                }
                break;
            case RUNNING:
                boolean complete;
                if (manualMode) {
                    complete = manualUpdate();
                } else {
                    complete = automaticUpdate();
                }

                if (complete) {
                    state = State.COMPLETE;
                    context.getFollower().stop();
                }
                break;
            case COMPLETE:
                endLoop();
                if (bPressed) {
                    return true;
                }
                break;
        }

        return false;
    }

    /**
     * @return true when initialization is complete and the tuning phase should begin running.
     */
    private boolean initLoop(boolean aWasPressed, boolean bPressed) {
        context.getTelemetry().addLine(getPhaseName() + " phase initialized");

        if (manualTuneIsPossible() && autoTuneIsPossible()) {
            context.getTelemetry().addData("Selected Mode:", manualMode ? "Manual" : "Automatic");
            context.getTelemetry().addLine("A - Toggle mode");

            if (aWasPressed) {
                manualMode = !manualMode;
            }
        }

        context.getTelemetry().addLine("B - Start tuning");
        context.getTelemetry().update();

        if (bPressed) {
            init();
            return true;
        }

        return false;
    }

    private void endLoop() {
        context.getTelemetry().addLine(getPhaseName() + " phase complete with results:");
        reportResults();
        context.getTelemetry().addLine("B - Exit");
        context.getTelemetry().update();
    }

    /**
     * @return the name of this phase as a string for display purposes.
     */
    protected abstract String getPhaseName();

    /**
     * @return true if manual tuning is possible for this phase, false otherwise.
     */
    protected abstract boolean manualTuneIsPossible();

    /**
     * @return true if automatic tuning is possible for this phase, false otherwise.
     */
    protected abstract boolean autoTuneIsPossible();

    /** Initializes the tuning phase (for automatic or manual) */
    protected abstract void init();

    /**
     * This method should perform manual tuning updates. It will be called repeatedly until it
     * returns true.
     * @return true if the manual tuning is complete, false otherwise
     */
    protected abstract boolean manualUpdate();

    /**
     * This method should perform automatic tuning updates. It will be called repeatedly until it
     * returns true.
     *
     * @return true if the automatic tuning is complete, false otherwise
     */
    protected abstract boolean automaticUpdate();

    /**
     * This method should use the telemetry (context.getTelemetry()) to report the results of
     * the tuning phase. It will be called repeatedly until the user exits the phase.
     */
    protected abstract void reportResults();

    protected static class BinarySearch {
        private double maxGuess;
        private double minGuess;
        private final double convergenceThreshold;
        private double guess;

        /**
         * Creates a new BinarySearch instance with the given parameters.
         * @param minGuess Minimum guess value for the search (it will not go below this value)
         * @param maxGuess Maximum guess value for the search (it will not go above this value)
         * @param convergenceThreshold The threshold for convergence. If the difference between
         *                             the last guess and the current guess is less than or equal
         *                             to this value, the search is considered converged.
         */
        public BinarySearch(double minGuess, double maxGuess, double convergenceThreshold) {
            this.maxGuess = maxGuess;
            this.minGuess = minGuess;
            this.convergenceThreshold = convergenceThreshold;
            this.guess = (maxGuess + minGuess) / 2;
        }

        /** @return false if the guess has converged and the search should stop, true otherwise */
        public boolean updateGuess(boolean increase) {
            double lastGuess = guess;
            if (increase) {
                minGuess = guess;
            } else {
                maxGuess = guess;
            }
            guess = (maxGuess + minGuess) / 2;

            return !(Math.abs(lastGuess - guess) <= convergenceThreshold);
        }

        /** @return the current guess value */
        public double getGuess() { return guess; }
    }

    protected static class PDSRoutine {
        final double HAS_MOVED_THRESHOLD = 0.05; // Inches
        final double HAS_MOVED_THRESHOLD_HEADING = 0.02; // Radians
        final double TIME_PER_GUESS_MS = 1500;
        final double SETTLING_DURATION_MS = 750;
        final double PD_TUNER_DURATION = 2000;

        public enum State { TUNING_KS, SETTLING_BETWEEN_KS, SETTLING_FOR_PD, TUNING_PD }
        private State currentState = State.TUNING_KS;

        private final Axis axis;
        private final ElapsedTime timer = new ElapsedTime();
        private final PDSController controller;
        private final BinarySearch kSSearch;
        private final double threshold;

        private double startTime = 0;
        private double maxAccel = 0;
        private double velAtTimestamp = 0;
        private double timestamp = 0;

        public enum Axis { DRIVE, STRAFE, HEADING }

        public PDSRoutine(TunerContext context, Axis axis) {
            this.kSSearch = new BinarySearch(0, 0.4, 0.01);
            this.axis = axis;
            context.getFollower().disableControllers();
            context.getFollower().setPose(Pose.zero());

            this.controller = new PDSController(new PDSCoefficients());
            if (this.axis == Axis.HEADING) { this.controller.setAngularController(); }

            this.threshold = (axis == Axis.HEADING) ?
                    HAS_MOVED_THRESHOLD_HEADING : HAS_MOVED_THRESHOLD;
        }

        public void start() {
            timer.reset();
            controller.getCoefficients().setkS(kSSearch.getGuess());
            currentState = State.TUNING_KS;
        }

        private void moveInDirection(TunerContext context, double power) {
            switch (axis) {
                case DRIVE:
                    context.getFollower().getDrivetrain().moveWithVectors(power, 0, 0);
                    break;
                case STRAFE:
                    context.getFollower().getDrivetrain().moveWithVectors(0, power, 0);
                    break;
                case HEADING:
                    context.getFollower().getDrivetrain().moveWithVectors(0, 0, power);
                    break;
            }
        }

        public double getPoseAxis(Pose pose) {
            switch (axis) {
                case DRIVE:
                    return pose.getX().getIn();
                case STRAFE:
                    return pose.getY().getIn();
                case HEADING:
                    return pose.getHeading().getRad();
                default:
                    return 0;
            }
        }

        public boolean update(TunerContext context) {
            switch (currentState) {
                case TUNING_KS:
                    moveInDirection(context, kSSearch.getGuess());
                    if (timer.milliseconds() >= TIME_PER_GUESS_MS) {
                        Pose pose = context.getFollower().getPose();
                        double currentMovement = Math.abs(getPoseAxis(pose));

                        boolean keepTuning = kSSearch.updateGuess(currentMovement <= threshold);

                        if (keepTuning) {
                            currentState = State.SETTLING_BETWEEN_KS;
                        } else {
                            controller.getCoefficients().setkS(kSSearch.getGuess()); // Save final kS
                            currentState = State.SETTLING_FOR_PD;
                        }

                        timer.reset();
                    }
                    break;
                case SETTLING_BETWEEN_KS:
                    context.getFollower().stop();
                    if (timer.milliseconds() >= SETTLING_DURATION_MS) {
                        currentState = State.TUNING_KS;
                        timer.reset();
                        context.getFollower().setPose(Pose.zero());
                    }
                    break;

                case SETTLING_FOR_PD:
                    context.getFollower().stop();
                    if (timer.milliseconds() >= SETTLING_DURATION_MS) {
                        currentState = State.TUNING_PD;
                        timer.reset();
                        startTime = System.nanoTime();
                    }
                    break;

                case TUNING_PD:
                    moveInDirection(context, 1.0);
                    double accel = getPoseAxis(context.getFollower().getAcceleration());

                    if (accel > maxAccel) {
                        maxAccel = accel;
                        timestamp = (System.nanoTime() - startTime) / 1.0e9;
                        velAtTimestamp = getPoseAxis(context.getFollower().getVelocity());
                    }

                    if (timer.milliseconds() >= PD_TUNER_DURATION) {
                        context.getFollower().stop();

                        if (maxAccel <= 0.001) {
                            throw new RuntimeException(
                                    "Max acceleration is zero or too low during tuning. check that " +
                                            "your localizer is working properly.");
                        }

                        // Derive kP and kD using Ziegler-Nichols formulas
                        double L = timestamp - (velAtTimestamp / maxAccel); // Delay time
                        if (L <= 0.001) L = 0.001;

                        controller.getCoefficients().setkP(1.2 / (L * maxAccel));
                        controller.getCoefficients().setkD(0.6 / maxAccel);
                        return true;
                    }
                    break;
            }
            return false;
        }

        public PDSCoefficients getCoefficients() {
            return controller.getCoefficients();
        }
    }
}