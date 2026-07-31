package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import geometry.Pose;

/**
 * A routine that tunes kP, kD, and kS for a specific axis (drive, strafe, or heading) by first
 * tuning the kS value and then tuning the kP and kD values based on the maximum acceleration
 * observed during a movement.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Sohum Arora - 22985 Paraducks
 */
public class PDSRoutine {
    enum Axis {
        DRIVE,
        STRAFE,
        HEADING
    }

    enum PDSState {
        TUNING_KS,
        SETTLING_BETWEEN_KS,
        SETTLING_FOR_PD,
        TUNING_PD
    }

    private static final double MOVEMENT_THRESHOLD = 0.05;
    private static final double HEADING_THRESHOLD = 0.02;
    private static final double GUESS_TIME = 1500;
    private static final double SETTLING_TIME = 750;
    private static final double TUNING_TIME = 1250;

    private final Axis axis;
    private final ElapsedTime timer = new ElapsedTime();
    private final PDSController controller;
    private final BinarySearch search;
    private final double threshold;

    private PDSState state = PDSState.TUNING_KS;
    private double startTime;
    private double maxAcceleration;
    private double velocityAtMaxAcceleration;
    private double maxAccelerationTime;

    PDSRoutine(TunerContext context, Axis axis) {
        search = new BinarySearch(0.0, 0.4, 0.01);
        this.axis = axis;
        context.getFollower().disableControllers();
        context.getFollower().setPose(Pose.zero());
        controller = new PDSController(new PDSCoefficients());
        if (axis == Axis.HEADING) {
            controller.setAngularController();
        }
        threshold = axis == Axis.HEADING ? HEADING_THRESHOLD : MOVEMENT_THRESHOLD;
    }

    void start() {
        timer.reset();
        controller.getCoefficients().setkS(search.getGuess());
        state = PDSState.TUNING_KS;
    }

    private void move(TunerContext context, double power) {
        switch (axis) {
            case DRIVE:
                context.getFollower().getDrivetrain().moveWithVectors(power, 0.0, 0.0);
                break;
            case STRAFE:
                context.getFollower().getDrivetrain().moveWithVectors(0.0, power, 0.0);
                break;
            case HEADING:
                context.getFollower().getDrivetrain().moveWithVectors(0.0, 0.0, power);
                break;
        }
    }

    private double getValue(Pose pose) {
        switch (axis) {
            case DRIVE:
                return pose.getX().getIn();
            case STRAFE:
                return pose.getY().getIn();
            case HEADING:
                return pose.getHeading().getRad();
            default:
                return 0.0;
        }
    }

    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    boolean update(TunerContext context) {
        switch (state) {
            case TUNING_KS:
                move(context, search.getGuess());
                if (timer.milliseconds() >= GUESS_TIME) {
                    double movement = Math.abs(getValue(context.getFollower().getPose()));
                    boolean keepTuning = search.updateGuess(movement <= threshold);
                    state = keepTuning ? PDSState.SETTLING_BETWEEN_KS : PDSState.SETTLING_FOR_PD;
                    if (!keepTuning) {
                        controller.getCoefficients().setkS(search.getGuess());
                    }
                    timer.reset();
                }
                break;
            case SETTLING_BETWEEN_KS:
                context.getFollower().stop();
                if (timer.milliseconds() >= SETTLING_TIME) {
                    state = PDSState.TUNING_KS;
                    timer.reset();
                    context.getFollower().setPose(Pose.zero());
                }
                break;
            case SETTLING_FOR_PD:
                context.getFollower().stop();
                if (timer.milliseconds() >= SETTLING_TIME) {
                    state = PDSState.TUNING_PD;
                    timer.reset();
                    startTime = System.nanoTime();
                }
                break;
            case TUNING_PD:
                move(context, 1.0);
                double acceleration = getValue(context.getFollower().getAcceleration());
                if (acceleration > maxAcceleration) {
                    maxAcceleration = acceleration;
                    maxAccelerationTime = (System.nanoTime() - startTime) / 1.0e9;
                    velocityAtMaxAcceleration = getValue(context.getFollower().getVelocity());
                }
                if (timer.milliseconds() >= TUNING_TIME) {
                    context.getFollower().stop();
                    if (maxAcceleration <= 0.001) {
                        throw new IllegalStateException(
                                "Max acceleration was too low during tuning."
                        );
                    }
                    double delay = Math.max(
                            0.001, maxAccelerationTime - velocityAtMaxAcceleration / maxAcceleration
                    );
                    controller.getCoefficients().setkP(1.2 / (delay * maxAcceleration));
                    controller.getCoefficients().setkD(0.6 / maxAcceleration);
                    return true;
                }
                break;
        }
        return false;
    }

    PDSCoefficients getCoefficients() { return controller.getCoefficients(); }
}
