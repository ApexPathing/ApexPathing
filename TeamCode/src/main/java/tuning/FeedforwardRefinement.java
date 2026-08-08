package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import geometry.AngleUnit;

/**
 * Tunes the feedforward coefficients (kV and kA) for both the angular (heading) and
 * translational (drive) controllers. Users can manually tune using the gamepad or run
 * the automatic tuning routine which employs a binary search to find optimal values.
 *
 * @author Joel H - 7842a
 */
public class FeedforwardRefinement extends TuningPhase {
    private enum Coefficient { ANGULAR_KV, ANGULAR_KA, TRANSLATIONAL_KV, TRANSLATIONAL_KA }

    enum FFSteps { ANGULAR, DRIVE }
    private FFSteps steps = FFSteps.ANGULAR;
    BinarySearch searcher;
    private final ElapsedTime timer;
    private final TrapezoidProfile driveProfile;
    private final TrapezoidProfile angularProfile;

    private Coefficient selected = Coefficient.ANGULAR_KV;

    private enum AutoTuneState { ANGULAR_KV, ANGULAR_KA, TRANSLATIONAL_KV, TRANSLATIONAL_KA, DONE }
    private AutoTuneState autoState = AutoTuneState.ANGULAR_KV;
    private double evalTarget = 0.0;
    private double evalActual = 0.0;
    private boolean waitingForStart = false;
    private boolean isForward = true;

    public FeedforwardRefinement(TunerContext context) {
        super(context);
        final double EPSILON = 0.2;

        // Target half of our physical limits to ensure we don't saturate the motors during tuning
        double omegaTarget = context.getFollower().getConstants().angularVelLimitRad / 2;
        double alphaTarget = context.getFollower().getConstants().angularAccelLimitRad / 2;
        double vTarget = context.getFollower().getConstants().forwardVelLimitIn / 2;
        double aTarget = context.getFollower().getConstants().forwardAccelLimitIn / 2;

        timer = new ElapsedTime();
        angularProfile = new TrapezoidProfile(omegaTarget, alphaTarget);
        driveProfile = new TrapezoidProfile(vTarget, aTarget);
    }

    @Override
    protected String getPhaseName() {
        return "Feedforward Refinement";
    }

    @Override
    protected boolean manualTuneIsPossible() {
        return true;
    }

    @Override
    protected boolean autoTuneIsPossible() {
        return true;
    }

    @Override
    protected void init() {
        timer.reset();
        if (!manualMode) {
            autoState = AutoTuneState.ANGULAR_KV;
            waitingForStart = false;
            isForward = true;
            setupSearcherForState();
        }
    }

    /**
     * Initializes the binary search boundaries based on the current coefficient being tuned.
     */
    private void setupSearcherForState() {
        switch(autoState) {
            case ANGULAR_KV:
                // Angular limits are typically larger (e.g. 0.1 to 0.5)
                searcher = new BinarySearch(0.0001, 0.2, 0.0001);
                break;
            case ANGULAR_KA:
                searcher = new BinarySearch(0.0001, 0.2, 0.0001);
                break;
            case TRANSLATIONAL_KV:
                // Translational limits are usually very small (e.g. 0.015) because they multiply by inches/sec
                searcher = new BinarySearch(0.0001, 0.2, 0.00001);
                break;
            case TRANSLATIONAL_KA:
                searcher = new BinarySearch(0.0001, 0.2, 0.00001);
                break;
            case DONE:
                break;
        }
        applyGuess();
    }

    /**
     * Applies the current guess from the binary searcher to the actual robot constants.
     */
    private void applyGuess() {
        if (searcher == null) { return; }
        double guess = searcher.getGuess();
        switch (autoState) {
            case ANGULAR_KV: context.getFollower().getConstants().angularKV = guess; break;
            case ANGULAR_KA: context.getFollower().getConstants().angularKA = guess; break;
            case TRANSLATIONAL_KV: context.getFollower().getConstants().translationalKV = guess; break;
            case TRANSLATIONAL_KA: context.getFollower().getConstants().translationalKA = guess; break;
            default: break;
        }
    }

    /**
     * Generates a 1-second cruise trapezoidal motion profile to evaluate feedforward consistency.
     */
    public static class TrapezoidProfile {
        private final double accel;
        private final double vel;
        private final double tAccelEnd;
        private final double tCruiseEnd;
        private final double tEnd;

        public TrapezoidProfile(double vel, double accel) {
            this.vel = Math.abs(vel);
            this.accel = Math.abs(accel);

            // Phase 1: Ramping up (v = at -> t = v/a)
            this.tAccelEnd = this.vel / this.accel;
            // Phase 2: Cruising for exactly 1.0 second
            this.tCruiseEnd = this.tAccelEnd + 1.0;
            // Phase 3: Ramping down (takes the same time as ramping up)
            this.tEnd = this.tCruiseEnd + this.tAccelEnd;
        }

        public double getAccel(double t) {
            if (t < 0 || t >= tEnd) {
                return 0.0;
            }
            if (t < tAccelEnd) {
                return accel;
            }
            if (t < tCruiseEnd) {
                return 0.0;
            }
            return -accel;
        }

        public double getVel(double t) {
            if (t < 0 || t >= tEnd) {
                return 0.0;
            }
            if (t < tAccelEnd) {
                return accel * t;
            }
            if (t < tCruiseEnd) {
                return vel;
            }
            // Deceleration: Max velocity minus what has been lost over time
            return vel - accel * (t - tCruiseEnd);
        }

        public double getTotalTime() {
            return tEnd;
        }

        public double getAccelEnd() {
            return tAccelEnd;
        }

        public double getCruiseEnd() {
            return tCruiseEnd;
        }

        public double getTotalDistance() {
            return vel * tCruiseEnd;
        }
    }

    @Override
    protected boolean manualTuned() {
        if (opMode.gamepad1.leftBumperWasPressed()) {
            selected = selected == Coefficient.ANGULAR_KV ? Coefficient.TRANSLATIONAL_KA :
                    Coefficient.values()[selected.ordinal() - 1];
        }
        if (opMode.gamepad1.rightBumperWasPressed()) {
            selected = selected == Coefficient.TRANSLATIONAL_KA ? Coefficient.ANGULAR_KV :
                    Coefficient.values()[selected.ordinal() + 1];
        }

        double change = manualChange();
        if (change != 0.0) {
            if (selected == Coefficient.ANGULAR_KV) {
                context.getFollower().getConstants().angularKV = Math.max(
                        0.0, context.getFollower().getConstants().angularKV + change
                );
            } else if (selected == Coefficient.ANGULAR_KA) {
                context.getFollower().getConstants().angularKA = Math.max(
                        0.0, context.getFollower().getConstants().angularKA + change
                );
            } else if (selected == Coefficient.TRANSLATIONAL_KV) {
                context.getFollower().getConstants().translationalKV = Math.max(
                        0.0, context.getFollower().getConstants().translationalKV + change
                );
            } else if (selected == Coefficient.TRANSLATIONAL_KA) {
                context.getFollower().getConstants().translationalKA = Math.max(
                        0.0, context.getFollower().getConstants().translationalKA + change
                );
            }
        }

        if (opMode.gamepad1.aWasPressed()) {
            return true;
        }

        context.getFollower().getLocalizer().update();
        double angularVel = context.getFollower().getLocalizer().getVel().getHeading(AngleUnit.RAD);
        double driveVel = context.getFollower().getLocalizer().getVel().getVec().getMag().getIn();

        // Ensure we only run the profile within its bounds
        boolean working = steps == FFSteps.ANGULAR && timer.time(TimeUnit.SECONDS) <= angularProfile.getTotalTime()
                || steps == FFSteps.DRIVE && timer.time(TimeUnit.SECONDS) <= driveProfile.getTotalTime();

        if (opMode.gamepad1.xWasPressed() && !working) {
            steps = FFSteps.ANGULAR;
            timer.reset();
        } else if (opMode.gamepad1.yWasPressed() && !working) {
            steps = FFSteps.DRIVE;
            timer.reset();
        }

        double time_sec = timer.time(TimeUnit.SECONDS);
        double targetVel = 0.0;
        double currentVel = 0.0;

        switch (steps) {
            case ANGULAR:
                targetVel = angularProfile.getVel(time_sec);
                currentVel = angularVel;
                double angularPow =
                        context.getFollower().getConstants().angularKV * targetVel +
                                context.getFollower().getConstants().angularKA * angularProfile.getAccel(time_sec) +
                                context.getFollower().getConstants().angularCoeffs.kS;
                context.getFollower().getDrivetrain().drive(0.0,0.0, angularPow);
                break;
            case DRIVE:
                targetVel = driveProfile.getVel(time_sec);
                currentVel = driveVel;
                double drivePow =
                        context.getFollower().getConstants().translationalKV * targetVel +
                                context.getFollower().getConstants().translationalKA * driveProfile.getAccel(time_sec) +
                                context.getFollower().getConstants().translationalCoeffs.kS;
                context.getFollower().getDrivetrain().drive(drivePow, 0.0, 0.0);
                break;
        }

        if (opMode.gamepad1.xWasPressed() && timer.time(TimeUnit.SECONDS) > angularProfile.getTotalTime()) {
            timer.reset();
        }

        context.getTelemetry().addData("Selected", selected.toString());
        reportResults();
        context.getTelemetry().addData("Increment", increment);
        context.getTelemetry().addLine("Dpad Up/Down: Change value");
        context.getTelemetry().addLine("Dpad Left/Right: Change increment");
        context.getTelemetry().addLine("LB/RB: select Value to tune");
        context.getTelemetry().addLine("Target Vel: " + targetVel);
        context.getTelemetry().addLine("Current Vel: " + currentVel + (steps == FFSteps.ANGULAR ? " rad/s" : " in/s"));
        context.getTelemetry().addLine("Current velocity " + ((currentVel >= targetVel) ? "> target" : "< target"));
        context.getTelemetry().addLine("X: Run and edit angular routine");
        context.getTelemetry().addLine("Y: Run and edit drive routine");
        context.getTelemetry().addLine("A: Save");
        context.getTelemetry().update();
        return false;
    }

    @Override
    protected boolean autoTuned() {
        if (autoState == AutoTuneState.DONE) {
            return true;
        }

        // We pause between iterations for translational movements so the user can verify
        // the robot will not hit a wall. Angular tuning simply spins in place and doesn't wait.
        if (waitingForStart) {
            double safeDist = driveProfile.getTotalDistance() + 24.0;
            context.getTelemetry().addLine("Point the " + (isForward ? "front" : "back") +
                    " of your robot towards " + String.format("%.1f", safeDist) + " inches of clearance.");
            context.getTelemetry().addLine("Press A to begin drive tuning.");
            context.getTelemetry().update();

            if (opMode.gamepad1.aWasPressed()) {
                waitingForStart = false;
                timer.reset();
            }
            return false;
        }

        context.getFollower().getLocalizer().update();
        double time_sec = timer.time(TimeUnit.SECONDS);

        boolean isAngular = autoState == AutoTuneState.ANGULAR_KV || autoState == AutoTuneState.ANGULAR_KA;
        TrapezoidProfile profile = isAngular ? angularProfile : driveProfile;

        // Use absolute values so comparisons hold regardless of whether we are driving forward or backwards
        double currentVel = isAngular ?
                Math.abs(context.getFollower().getLocalizer().getVel().getHeading(AngleUnit.RAD)) :
                Math.abs(context.getFollower().getLocalizer().getVel().getVec().getMag().getIn());

        double targetVel = profile.getVel(time_sec);

        // Sampling Logic: We continuously overwrite evalActual and evalTarget as long as we are in the
        // correct phase. The final values grabbed right before transitioning out of the phase are what
        // we use to evaluate the binary search.
        if (autoState == AutoTuneState.ANGULAR_KV || autoState == AutoTuneState.TRANSLATIONAL_KV) {
            // kV targets steady-state velocity, so we sample during the cruise phase
            if (time_sec >= profile.getAccelEnd() && time_sec < profile.getCruiseEnd()) {
                evalActual = currentVel;
                evalTarget = targetVel;
            }
        } else {
            // kA targets acceleration, so we sample during the ramping up phase
            if (time_sec > 0 && time_sec < profile.getAccelEnd()) {
                evalActual = currentVel;
                evalTarget = targetVel;
            }
        }

        // Active profile execution
        if (time_sec <= profile.getTotalTime()) {
            if (isAngular) {
                double angularPow =
                        context.getFollower().getConstants().angularKV * targetVel +
                                context.getFollower().getConstants().angularKA * profile.getAccel(time_sec) +
                                context.getFollower().getConstants().angularCoeffs.kS;
                context.getFollower().getDrivetrain().drive(0.0, 0.0, angularPow);
            } else {
                double drivePow =
                        context.getFollower().getConstants().translationalKV * targetVel +
                                context.getFollower().getConstants().translationalKA * profile.getAccel(time_sec) +
                                context.getFollower().getConstants().translationalCoeffs.kS;

                // Multiply by direction to physically alternate driving forward/backward
                double direction = isForward ? 1.0 : -1.0;
                context.getFollower().getDrivetrain().drive(drivePow * direction, 0.0, 0.0);
            }
        } else {
            // Profile complete, kill power
            context.getFollower().getDrivetrain().drive(0.0, 0.0, 0.0);

            // Wait an extra 0.5 seconds for the physical robot to stop oscillating before evaluating
            if (time_sec > profile.getTotalTime() + 0.5) {

                // If actual velocity missed the target velocity, we need a higher feedforward constant
                boolean increase = evalActual < evalTarget;

                // updateGuess returns false when the threshold is met
                boolean converged = !searcher.updateGuess(increase);

                if (converged) {
                    // Move to the next tuning state
                    switch (autoState) {
                        case ANGULAR_KV:
                            autoState = AutoTuneState.ANGULAR_KA;
                            break;
                        case ANGULAR_KA:
                            autoState = AutoTuneState.TRANSLATIONAL_KV;
                            isForward = true; // reset direction for drive phase
                            break;
                        case TRANSLATIONAL_KV:
                            autoState = AutoTuneState.TRANSLATIONAL_KA;
                            isForward = true; // reset direction for drive phase
                            break;
                        case TRANSLATIONAL_KA:
                            autoState = AutoTuneState.DONE;
                            break;
                        default: break;
                    }

                    if (autoState != AutoTuneState.DONE) {
                        setupSearcherForState();
                    }
                } else {
                    applyGuess();

                    // Toggle drive direction for the next iteration if we're tuning translation
                    if (!isAngular) {
                        isForward = !isForward;
                    }
                }

                // Require user prompt on next frame if we are in a translational state
                waitingForStart = autoState == AutoTuneState.TRANSLATIONAL_KV || autoState == AutoTuneState.TRANSLATIONAL_KA;
                timer.reset();
            }
        }

        context.getTelemetry().addData("AutoTuning State", autoState);
        context.getTelemetry().addData("Current Guess", searcher != null ? searcher.getGuess() : 0.0);
        context.getTelemetry().addData("Eval Target", evalTarget);
        context.getTelemetry().addData("Eval Actual", evalActual);
        reportResults();
        context.getTelemetry().update();

        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Angular KV", context.getFollower().getConstants().angularKV);
        context.getTelemetry().addData("Angular  KA", context.getFollower().getConstants().angularKA);
        context.getTelemetry().addData("Translational KV", context.getFollower().getConstants().translationalKV);
        context.getTelemetry().addData("Translational KA", context.getFollower().getConstants().translationalKA);
    }
}
