package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import geometry.Angle;
import geometry.Vector;

/**
 * Mecanum drivetrain controller
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author DrPixelCat - 7842 alum
 */
public class Mecanum extends BaseDrivetrain<Mecanum.Constants> {
    public Mecanum(Constants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap, DrivetrainType.MECANUM);

        if (constants.blMotorConfig == null || constants.brMotorConfig == null) {
            throw new IllegalArgumentException(
                    "Back left and right motor configurations must be provided for a mecanum drivetrain"
            );
        }
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        // Mecanum kinematics explanation: https://www.youtube.com/watch?v=gnSW2QpkGXQ
        setPowers(x - y - turn, x + y + turn,
                x + y - turn, x - y + turn
        );
    }

    /** Configuration class for Mecanum drivetrain. */
    public static class Constants extends BaseDrivetrainConstants<Constants> {
        @Override
        public Mecanum build(HardwareMap hardwareMap) {
            return new Mecanum(this, hardwareMap);
        }

        /** Sets the front left motor configuration. */
        public Constants setFrontLeftMotor(Motor Motor) {
            this.flMotorConfig = Motor;
            return this;
        }

        /** Sets the front right motor configuration. */
        public Constants setFrontRightMotor(Motor Motor) {
            this.frMotorConfig = Motor;
            return this;
        }

        /** Sets the back left motor configuration. */
        public Constants setBackLeftMotor(Motor Motor) {
            this.blMotorConfig = Motor;
            return this;
        }

        /** Sets the back right motor configuration. */
        public Constants setBackRightMotor(Motor Motor) {
            this.brMotorConfig = Motor;
            return this;
        }
    }

    public static class DirectionalLut {
        public static class DirectionalKinematics {
            public final double maxVel;
            public final double maxAccel;
            public final double velMultiplier;
            public final double accelMultiplier;

            public DirectionalKinematics(double maxVel, double maxAccel, double velMultiplier,
                                         double accelMultiplier) {
                this.maxVel = maxVel;
                this.maxAccel = maxAccel;
                this.velMultiplier = velMultiplier;
                this.accelMultiplier = accelMultiplier;
            }
        }

        private final DirectionalKinematics[] lut = new DirectionalKinematics[360];

        /**
         * Precomputes the entire 360-degree kinematic capability of the mecanum drive.
         *
         * @param maxFwdVel Absolute maximum forward velocity (in/s)
         * @param maxFwdAccel Absolute maximum forward acceleration (in/s^2)
         * @param maxSfeVel Absolute maximum strafing velocity (in/s)
         * @param maxSfeAccel Absolute maximum strafing acceleration (in/s^2)
         */
        public DirectionalLut(double maxFwdVel, double maxFwdAccel, double maxSfeVel,
                                     double maxSfeAccel) {
            for (int i = 0; i < 360; i++) {
                double theta = Math.toRadians(i);

                double absForward = Math.abs(Math.cos(theta));
                double absStrafe = Math.abs(Math.sin(theta));

                double maxVel = 1.0 / ((absForward / maxFwdVel) + (absStrafe / maxSfeVel));
                double maxAccel = 1.0 / ((absForward / maxFwdAccel) + (absStrafe / maxSfeAccel));

                double velMultiplier = maxFwdVel / maxVel;
                double accelMultiplier = maxFwdAccel / maxAccel;

                lut[i] = new DirectionalKinematics(maxVel, maxAccel, velMultiplier,
                        accelMultiplier);
            }
        }

        public DirectionalKinematics getKinematics(Vector globalDriveVector, Angle currentHeading) {
            if (globalDriveVector.getMagSq().getIn() < 1e-9) {
                return lut[0]; // Default to forward limits
            }

            Vector localVector = globalDriveVector.rotate(currentHeading.times(-1.0));
            double degrees = Math.toDegrees(localVector.getTheta().getRad());
            degrees = ((degrees % 360.0) + 360.0) % 360.0;

            int lowIndex = (int) Math.floor(degrees);
            int highIndex = (lowIndex + 1) % 360;
            double t = degrees - lowIndex;

            if (t < 1e-9) {
                return lut[lowIndex];
            }

            DirectionalKinematics low = lut[lowIndex];
            DirectionalKinematics high = lut[highIndex];

            return new DirectionalKinematics(
                    interpolate(low.maxVel, high.maxVel, t),
                    interpolate(low.maxAccel, high.maxAccel, t),
                    interpolate(low.velMultiplier, high.velMultiplier, t),
                    interpolate(low.accelMultiplier, high.accelMultiplier, t)
            );
        }

        private double interpolate(double low, double high, double t) {
            return low + ((high - low) * t);
        }
    }
}