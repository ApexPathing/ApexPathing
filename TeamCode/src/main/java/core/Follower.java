package core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import controllers.DriveController;
import controllers.DriveController.AllocatedCommand;
import controllers.TurnController;
import controllers.PDSController;
import drivetrains.BaseDrivetrain;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.DualActuated;
import drivetrains.Mecanum;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.Dist;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.Callback;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/**
 * Apex Pathing's main Follower class. Handles the execution of generated paths and turns using
 * kinematic feedforward and feedback controllers.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author DrPixelCat - 7842 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Xander Haemel - 31616 404 Not Found
 */
public class Follower {
    private final FollowerConstants constants;
    private final BaseDrivetrain<?> drivetrain;
    private final BaseLocalizer<?> localizer;

    private enum HolonomicDriveModel { ANISOTROPIC, ISOTROPIC }

    private final double headingTol; // Radians
    private final double distanceTol; // Inches

    private double lastS = -1.0;
    private long lastNano = -1;
    private Angle lastHeading = null; // Tracks heading between ticks for angular callback sweeps

    private final PDSController headingController;
    private final TurnController turnController;
    private final DriveController driveController;
    private final double velocityFeedbackGain;

    private FollowerMovement currentMovement = null;
    private boolean paused = false;

    private boolean headingControllerEnabled = true;
    private boolean driveControllerEnabled = true;

    PathSegment segment;
    Angle targetHeading;
    Vector targetTurnPoseVec;
    double turnDirection;
    double turnTotalDisplacement;

    /** Constructs the drivetrain, localizer, and follower from the given {@link ApexConstants}. */
    public Follower(ApexConstants constants, HardwareMap hardwareMap) {
        BaseDrivetrainConstants<?> drivetrainConstants = constants.drivetrainConstants();

        this.drivetrain = drivetrainConstants.build(hardwareMap);
        this.localizer = constants.localizerConstants().build(hardwareMap);
        this.constants = FollowerConstants.getInstance();

        this.headingTol = drivetrainConstants.headingTolerance.getRad();
        this.distanceTol = drivetrainConstants.distanceTolerance.getIn();

        this.headingController = new PDSController(this.constants.headingCoeffs);
        this.headingController.setAngularController();

        this.turnController = new TurnController(
                this.constants.headingCoeffs,
                this.constants.angularKV,
                this.constants.angularKA,
                this.constants.angularVelocityFeedbackGain
        );

        this.driveController = new DriveController(
                Dist.fromIn(this.constants.forwardVelLimitIn),
                Dist.fromIn(this.constants.strafeVelLimitIn),
                this.constants.translationalCoeffs,
                Dist.fromIn(0.25),
                drivetrain instanceof Mecanum || drivetrain instanceof DualActuated
        );
        velocityFeedbackGain = this.constants.velocityFeedbackGain;
    }

    // region Callbacks

    /**
     * Evaluates all callbacks attached to the current movement and executes them if their
     * conditions are met.
     * * @param t The current geometric progression percentage [0.0, 1.0]. Pass -1.0 for turns.
     *
     * @param currentHeading The robot's current field orientation.
     */
    private void processCallbacks(double s, Angle currentHeading) {
        Callback[] callbacks = null;
        if (currentMovement instanceof Path) {
            callbacks = ((Path) currentMovement).getCallbacks();
        } else if (currentMovement instanceof Turn) {
            callbacks = ((Turn) currentMovement).getCallbacks();
        }

        if (callbacks != null) {
            for (Callback cb : callbacks) {
                if (cb.isTriggered()) continue;

                boolean shouldTrigger = false;

                if (cb.getType() == Callback.CallbackType.DISTANCE) {
                    if (s >= cb.getS() && s >= 0.0) {
                        shouldTrigger = true;
                    }
                } else if (cb.getType() == Callback.CallbackType.ANGLE) {
                    double error =
                            Math.abs(currentHeading.getShortestAngleTo(cb.getTheta()).getRad());

                    // Trigger if resting within 1 degree of target
                    if (error < Math.toRadians(1.0)) {
                        shouldTrigger = true;
                    }
                    // Trigger if the target angle was swept past between the last tick and this
                    // tick
                    else if (lastHeading != null) {
                        double tickSweep = lastHeading.getShortestAngleTo(currentHeading).getRad();
                        double targetSweep = lastHeading.getShortestAngleTo(cb.getTheta()).getRad();

                        // If sweeps are in the same direction AND the tick sweep is larger, it
                        // was crossed
                        if (Math.signum(tickSweep) == Math.signum(targetSweep) && Math.abs(targetSweep) <= Math.abs(tickSweep)) {
                            shouldTrigger = true;
                        }
                    }
                }

                if (shouldTrigger) {
                    cb.getAction().run();
                    cb.setTriggered(true);
                }
            }
        }
        lastHeading = currentHeading;
    }

    // region General methods
    /**
     * The main execution loop of the follower.
     * Must be called continuously during the active OpMode loop to drive the robot along the path.
     */
    public void update() {
        localizer.update();

        // Exit early if nothing is running
        if (currentMovement == null || paused) {
            return;
        }

        Pose current = getPose();
        Vector currentPos = current.getVec();
        Angle currentHeading = current.getHeading();

        long currentNano = System.nanoTime();

        // Calculate delta time for velocity feedback
        double deltaT_seconds = (lastNano != -1) ? (currentNano - lastNano) / 1e9 : 0.0;
        lastNano = currentNano;

        // region Turn Execution
        if (currentMovement instanceof Turn) {
            Turn turn = (Turn) currentMovement;
            double headingError = targetHeading.getRad() - currentHeading.getRad();

            // Process angular callbacks
            processCallbacks(-1.0, currentHeading);

            // Require both positional accuracy and low angular velocity to prevent momentum
            // overshoot
            double currentAngularVel = localizer.getVel().getHeading().getRad();
            if (Math.abs(headingError) < headingTol && Math.abs(currentAngularVel) < 0.05) {
                this.stop();
                return;
            }

            double totalTurnPower;
            if (turn.getFeedforwardLut() == null || turnTotalDisplacement < 1e-9) {
                totalTurnPower = turnController.calculateQuick(headingError);
            } else {
                double signedTravel = turn.getStartPose().getHeading()
                        .getShortestAngleTo(currentHeading).getRad() * turnDirection;
                double angularDisplacement = Range.clip(
                        signedTravel, 0.0, turnTotalDisplacement);
                MotionParameters turnTargets = turn.getFeedforwardLut()
                        .getFeedforwardParams(angularDisplacement);
                totalTurnPower = turnController.calculateProfiled(
                        headingError,
                        turnDirection,
                        turnTargets,
                        currentAngularVel
                );
            }

            Vector error = targetTurnPoseVec.minus(currentPos);
            double errorMag = error.getMag().getIn();

            // Hold xy position actively while turning
            if (drivetrain.isHolonomic() && errorMag > distanceTol) {
                Vector fieldFeedback = driveController.calculatePointToPoint(
                        targetTurnPoseVec, currentPos);
                AllocatedCommand positionHold = allocateHolonomicStage(
                        fieldFeedback,
                        currentHeading,
                        1.0 - Math.abs(totalTurnPower),
                        getActiveHolonomicDriveModel()
                );
                Vector robotCommand = positionHold.getRobotCommand();
                drivetrain.drive(robotCommand.getX().getIn(),
                        robotCommand.getY().getIn(), totalTurnPower);
            } else {
                drivetrain.drive(0, 0, totalTurnPower);
            }

        } else if (segment == null) {
            this.stop();
            // region Holonomic Following
        } else if (drivetrain.isHolonomic()) {
            // Retrieve path geometry at closest point
            double t = segment.getBestT(currentPos);

            Vector targetPoseVec = segment.getPosition(t);
            double s = segment.getDistanceToEndIn(targetPoseVec, t);
            Vector velVec = segment.getFirstDerivative(t);
            Vector accelVec = segment.getSecondDerivative(t);
            Vector normal = PathSegment.calculateArcNormal(velVec, accelVec);
            Vector unitTangent = velVec.normalize();
            Vector endTangent = segment.getFirstDerivative(1.0).normalize();

            // Process scheduled distance and angular callbacks
            processCallbacks(s / segment.getLengthIn(), currentHeading);

            Vector robotVel = localizer.getVel().getVec();
            double distanceRemaining = segment.getDistanceToEndIn(targetPoseVec, t);
            double kappa = segment.getSignedCurvature(t);
            double dKappa = segment.getCurvatureDerivative(t);

            Path path = (Path) currentMovement;
            boolean isProfiled = path.isProfiled();
            double distanceTraveled = path.getParametricPath().getLengthIn() - s;
            MotionParameters targets = isProfiled ?
                    path.getFeedforwardLut().getFeedforwardParams(distanceTraveled) : null;

            HolonomicDriveModel driveModel = getActiveHolonomicDriveModel();

            double robotTangentialVel = (deltaT_seconds > 1e-6 && lastS >= 0.0) ?
                    (lastS - s) / deltaT_seconds : 0.0;
            lastS = s;

            // Calculate heading power allocation
            Angle headingTarg = path.getInterpolator().getHeadingTarg(s, velVec, endTangent);
            double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, endTangent);
            double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                    endTangent);

            double headingFF = 0.0;
            if (isProfiled) {
                double omegaTarget = fPrime * robotTangentialVel;
                double alphaTarget = (fDoublePrime * (robotTangentialVel * robotTangentialVel)) +
                        (fPrime * targets.getTangentialAccel());

                headingFF = (omegaTarget * constants.angularKV) + (alphaTarget * constants.angularKA);
                if (Math.abs(omegaTarget) > 1e-6) {
                    headingFF += Math.signum(omegaTarget) * constants.headingCoeffs.kS;
                }
            }

            double headingFeedback = headingControllerEnabled
                    ? headingController.calculate(
                    headingTarg.getRad() - currentHeading.getRad()) : 0.0;
            double turnPow = Range.clip(headingFeedback + headingFF, -1.0, 1.0);
            double availableMotorPower = 1.0 - Math.abs(turnPow);

            // Calculate lateral cross track power allocation
            Vector positionalError = targetPoseVec.minus(currentPos);
            double crossTrackError = positionalError.dot(normal).getIn();
            double lateralFeedbackMag = driveControllerEnabled
                    ? driveController.calculateCrossTrack(crossTrackError) : 0.0;

            double requiredLateralAccel = (robotTangentialVel * robotTangentialVel) * kappa;
            double centripetalMag = requiredLateralAccel * constants.Kcentripetal;

            Vector requestedLateralField = normal.times(
                    centripetalMag + lateralFeedbackMag
            );
            AllocatedCommand lateralCommand = allocateHolonomicStage(
                    requestedLateralField,
                    currentHeading,
                    availableMotorPower,
                    driveModel
            );

            // Charge the corrected lateral demand before allocating tangent power. Mecanum uses
            // wheel-space L1 demand; isotropic drives combine orthogonal translation by magnitude.
            double tangentBudget;
            if (driveModel == HolonomicDriveModel.ISOTROPIC) {
                tangentBudget = Math.sqrt(Math.max(0.0,
                        (availableMotorPower * availableMotorPower) -
                                Math.pow(lateralCommand.getPowerDemand(), 2)));
            } else {
                tangentBudget = Math.max(0.0,
                        availableMotorPower - lateralCommand.getPowerDemand());
            }

            double totalTangentPower;
            if (t < 1.0) {
                if (isProfiled) {
                    double feedforward = (constants.translationalKV * targets.getTangentialVel()) +
                            (constants.translationalKA * targets.getTangentialAccel()) +
                            (Math.signum(targets.getTangentialVel()) * constants.translationalCoeffs.kS);

                    // TODO: Verify p only feedback performance, compare to SquID
                    totalTangentPower = ((targets.getTangentialVel() - robotTangentialVel) *
                            velocityFeedbackGain) + feedforward;

                    if (path.isAccelBoosted()) {
                        totalTangentPower = Math.min(
                                totalTangentPower,
                                driveController.calculateEndDistance(distanceRemaining));
                    }
                } else {
                    double decelPower =
                            driveController.calculateEndDistance(distanceRemaining);
                    double percentage = 1.0 - (s / path.getParametricPath().getLengthIn());
                    double percentageClipped = Math.min(Math.max(percentage, 0.0), 1.0);
                    double maxVel = path.getQuickVelocityLimit(percentageClipped,
                            constants.forwardVelLimitIn);
                    double velError = maxVel - robotTangentialVel;
                    double accelPower = (maxVel * constants.translationalKV)
                            + (Math.signum(maxVel) * constants.translationalCoeffs.kS)
                            + (velError * velocityFeedbackGain);
                    totalTangentPower = Math.min(accelPower, decelPower);
                }
            } else {
                // Apply reverse feedback if robot drifts past the final point
                double distancePastEnd = currentPos.minus(targetPoseVec).dot(endTangent).getIn();
                totalTangentPower =
                        driveController.calculateEndDistance(-distancePastEnd);
            }

            Vector requestedTangentField = unitTangent.times(totalTangentPower);
            AllocatedCommand tangentCommand = allocateHolonomicStage(
                    requestedTangentField,
                    currentHeading,
                    tangentBudget,
                    driveModel
            );
            Vector finalDriveOutput = lateralCommand.getRobotCommand()
                    .plus(tangentCommand.getRobotCommand());

            // Check stop condition and drive hardware
            // TODO: Maybe don't hardcode this 25
            if (distanceRemaining < distanceTol && robotVel.getMagSq().getIn() < 25) {
                stop();
                return;
            }

            drivetrain.drive(
                    finalDriveOutput.getX().getIn(), finalDriveOutput.getY().getIn(), turnPow
            );
            // region Tank Following
        } else {
            // Process tank driving via Ramsete controller
            double t = segment.getBestT(currentPos);

            // Process scheduled distance and angular callbacks
            processCallbacks(t, currentHeading);

            Vector targetPoseVec = segment.getPosition(t);
            double s = segment.getDistanceToEndIn(targetPoseVec, t);
            Vector velVec = segment.getFirstDerivative(t);
            Vector robotVel = localizer.getVel().getVec();

            Path path = (Path) currentMovement;
            Angle headingTarg = path.getInterpolator().getHeadingTarg(s, velVec,
                    segment.getFirstDerivative(1.0));
            double distanceTraveled = path.getParametricPath().getLengthIn() - s;
            MotionParameters targets =
                    path.getFeedforwardLut().getFeedforwardParams(distanceTraveled);

            double v_d = targets.getTangentialVel();
            double a_d = targets.getTangentialAccel();
            double omega_d = targets.getAngularVel();
            double alpha_d = targets.getAngularAccel();

            // Transform global error to robot local frame
            Vector globalError = targetPoseVec.minus(currentPos);
            Vector localError = globalError.rotate(Angle.fromRad(-currentHeading.getRad()));

            double e_x = localError.getX().getIn();
            double e_y = localError.getY().getIn();
            double e_theta = currentHeading.getShortestAngleTo(headingTarg).getRad();

            // Calculate non linear Ramsete gains
            double b = 2.0;
            double zeta = 0.7;
            double k = 2.0 * zeta * Math.sqrt(Math.pow(omega_d, 2) + b * Math.pow(v_d, 2));
            double sinc = (Math.abs(e_theta) < 1e-6) ? 1.0 : (Math.sin(e_theta) / e_theta);

            double v_cmd = v_d * Math.cos(e_theta) + k * e_x;
            double w_cmd = omega_d + k * e_theta + b * v_d * sinc * e_y;

            // Convert velocity commands to motor power using feedforward constants
            double totalTangentPower = (v_cmd * constants.translationalKV) +
                    (a_d * constants.translationalKA) + (Math.signum(v_cmd) *
                    constants.translationalCoeffs.kS);
            double turnPow = (w_cmd * constants.angularKV) + (alpha_d * constants.angularKA);
            turnPow += Math.signum(turnPow) * constants.headingCoeffs.kS;

            double availableMotorPower = 1.0;
            turnPow = Range.clip(turnPow, -availableMotorPower, availableMotorPower);
            availableMotorPower -= Math.abs(turnPow);
            totalTangentPower = Range.clip(totalTangentPower, -availableMotorPower,
                    availableMotorPower);

            if (s < distanceTol && robotVel.getMagSq().getIn() < 16) {
                stop();
                return;
            }

            drivetrain.drive(totalTangentPower, 0.0, turnPow);
        }
    }

    private HolonomicDriveModel getActiveHolonomicDriveModel() {
        if (drivetrain instanceof Mecanum) return HolonomicDriveModel.ANISOTROPIC;
        if (drivetrain instanceof DualActuated) {
            if (!drivetrain.isHolonomic()) {
                throw new IllegalStateException(
                        "Dual-actuated drivetrain is not in its holonomic state."
                );
            }
            return HolonomicDriveModel.ANISOTROPIC;
        }
        if (!drivetrain.isHolonomic()) {
            throw new IllegalStateException(
                    "A holonomic allocation was requested while the drivetrain was non-holonomic."
            );
        }
        return HolonomicDriveModel.ISOTROPIC;
    }

    private AllocatedCommand allocateHolonomicStage(Vector fieldCommand, Angle currentHeading,
                                                     double availablePower,
                                                     HolonomicDriveModel driveModel) {
        if (driveModel == HolonomicDriveModel.ANISOTROPIC) {
            return driveController.allocateMecanum(
                    fieldCommand, currentHeading, availablePower);
        }
        return driveController.allocateIsotropic(fieldCommand, currentHeading, availablePower);
    }

    //region Initialize Sequence

    /**
     * Starts following the given movement.
     *
     * @param movement The movement object to be executed.
     * @throws IllegalStateException if the follower is already busy executing a movement.
     */
    public void follow(FollowerMovement movement) {
        if (isBusy()) {
            throw new IllegalStateException(
                    "Cannot execute a new movement while another movement is still in progress " +
                            "Tip use follower.isBusy() to check if the follower is currently " +
                            "executing a movement before starting a new one."
            );
        }

        this.currentMovement = movement;
        this.currentMovement.setStarted(true);
        this.currentMovement.setEnded(false);
        this.targetHeading = movement.getEndPose().getHeading();

        if (movement instanceof Turn) {
            Turn turn = (Turn) currentMovement;
            this.targetTurnPoseVec = turn.getStartPose().getVec();
            double signedTurn = turn.getStartPose().getHeading()
                    .getShortestAngleTo(turn.getEndPose().getHeading()).getRad();
            this.turnDirection = Math.signum(signedTurn);
            this.turnTotalDisplacement = Math.abs(signedTurn);
        } else if (movement instanceof Path) {
            Path pathSegmentMove = (Path) currentMovement;
            this.segment = pathSegmentMove.getParametricPath();
            if (drivetrain instanceof DualActuated) {
                if (pathSegmentMove.getPathType() == Path.PathType.HOLONOMIC) {
                    ((DualActuated) drivetrain).activateHolonomicState();
                } else {
                    ((DualActuated) drivetrain).activateTractionState();
                }
            }
        }

        headingController.reset();
        turnController.reset();
        driveController.reset();
        lastS = -1.0;
        lastNano = -1;
        paused = false;

        // Reset sweeping tracker for angular callbacks so it doesn't instantly trigger on path
        // start
        lastHeading = null;
    }

    // region Public Methods

    /** Instantly stops the drivetrain and ends any ongoing movement. */
    public void stop() {
        if (this.currentMovement != null) {
            this.currentMovement.setEnded(true);
        }

        this.currentMovement = null;
        this.segment = null;
        this.targetHeading = null;
        this.targetTurnPoseVec = null;
        this.turnDirection = 0.0;
        this.turnTotalDisplacement = 0.0;

        this.drivetrain.stop();
    }

    /** Halts the current movement temporarily without clearing the target state */
    public void pause() {
        this.paused = true;
        this.drivetrain.stop();
    }

    /** Resumes a paused movement from the robots current location. */
    public void resume() {
        if (this.paused) {
            this.paused = false;
            this.lastNano = -1;
        }
    }

    /**
     * Drives the robot using joystick inputs adjusted for field centric control.
     * Stops any active autonomous movement.
     * Drives the robot using the provided  inputs. The joystick inputs are adjusted for
     * field-centric or robot-centric control based on the constants. This method  will stop the
     * current movement if one is in progress, as manual control takes priority over following a path.
     *
     * @param x left/right joystick input (positive for right, negative for left)
     * @param y forward/backward joystick input (positive for forward, negative for backward)
     * @param turn rotation joystick input (positive for clockwise, negative for counterclockwise)
     */
    public void teleOpDrive(double x, double y, double turn) {
        if (isBusy()) { stop(); }
        drivetrain.drive(x, y, turn, this.getPose().getHeading().getRad());
    }

    /**
     * Drives the robot using standard gamepad inputs. The left stick controls translation (x and y),
     * and the right stick controls rotation (turn). The joystick inputs are adjusted for
     * field-centric or robot-centric control based on the constants. This method will stop the
     * current movement if one is in progress, as manual control takes priority over following a path.
     *
     * @param gamepad the gamepad to read inputs from
     */
    public void teleOpDrive(Gamepad gamepad) {
        teleOpDrive(-gamepad.left_stick_x, -gamepad.left_stick_y, -gamepad.right_stick_x);
    }
    // endregion

    public void disableHeadingController() {
        this.headingControllerEnabled = false;
    }
    public void disableDriveController() {
        this.driveControllerEnabled = false;
    }
    public void disableControllers() {
        disableHeadingController();
        disableDriveController();
    }

    /**
     * Retrieves the robots current pose estimate from the localizer.
     *
     * @return The current global position and heading.
     */
    public Pose getPose() { return localizer.getPose(); }

    /**
     * Checks if the follower is currently executing a movement.
     *
     * @return true if a movement is in progress false otherwise.
     */
    public boolean isBusy() { return currentMovement != null; }

    /**
     * Forcibly overrides the localizers current pose estimate.
     *
     * @param pose The new global position and heading.
     */
    public void setPose(Pose pose) { localizer.setPose(pose); }

    /**
     * Retrieves the robots current velocity estimate from the localizer.
     *
     * @return The current velocity expressed in robot local frame.
     */
    public Pose getVelocity() { return localizer.getVel(); }

    /**
     * Retrieves the robots current acceleration estimate from the localizer.
     *
     * @return The current acceleration expressed in robot local frame.
     */
    public Pose getAcceleration() { return localizer.getAccel(); }

    /**
     * DO NOT USE THIS METHOD UNLESS YOU KNOW WHAT YOU ARE DOING.
     * It is intended for internal use only.
     */
    public BaseLocalizer<?> getLocalizer() { return localizer; }

    /**
     * DO NOT USE THIS METHOD UNLESS YOU KNOW WHAT YOU ARE DOING.
     * It is intended for internal use only.
     */
    public BaseDrivetrain<?> getDrivetrain() { return drivetrain; }

    public FollowerConstants getConstants() { return constants; }
    // endregion
}
