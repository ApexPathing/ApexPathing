package localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.nio.ByteBuffer;

import geometry.Pose;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;

/**
 * SparkFun OTOS (Optical Tracking Odometry Sensor) localizer with a custom driver class
 * optimized for Apex Pathing.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class OTOS extends BaseLocalizer<OTOS.Constants> {
    private final Driver otos;

    public OTOS(Constants config, HardwareMap hardwareMap) {
        super(config);

        otos = hardwareMap.get(OTOS.Driver.class, config.name);
        otos.calibrateAndReset();
        otos.setOffset(config.offset);
        otos.setLinearScalar(config.linearScalar);
        otos.setAngularScalar(config.angularScalar);
    }

    @Override
    public void update() {
        otos.update();
        pose = otos.getPose();
        velocity = otos.getVel();
        acceleration = otos.getAccel();
    }

    @Override
    public void setPose(Pose newPose) { otos.setPosition(newPose); }

    /** Configuration class for the Sparkfun OTOS localizer. */
    public static class Constants extends BaseLocalizerConstants<Constants> {
        public String name = null;
        public Pose offset = Pose.zero();
        public double linearScalar = 1.0;
        public double angularScalar = 1.0;

        @Override
        public OTOS build(HardwareMap hardwareMap) {
            if (this.name == null) {
                throw new IllegalArgumentException("OTOS name is not set in the localizer constants.");
            }
            return new OTOS(this, hardwareMap);
        }

        /** Sets the name of the OTOS in the hardware map. */
        public Constants setName(String name) {
            this.name = name;
            return this;
        }

        /** Sets the offset of the OTOS from the center of the robot. */
        public Constants setOffset(Pose offset) {
            this.offset = offset;
            return this;
        }

        /**
         * Sets the linear scalar for the OTOS. This is a multiplier applied to the linear position
         * and velocity estimates from the OTOS to correct for any systematic errors in the sensor.
         * The value must be between 0.872 and 1.127, which corresponds to a correction
         * of +/- 12.7%.
         */
        public Constants setLinearScalar(double linearScalar) {
            if (linearScalar < Driver.MIN_SCALAR || linearScalar > Driver.MAX_SCALAR) {
                throw new IllegalArgumentException("Linear scalar must be between " + Driver.MIN_SCALAR + " and " + Driver.MAX_SCALAR);
            }
            this.linearScalar = linearScalar;
            return this;
        }

        /**
         * Sets the angular scalar for the OTOS. This is a multiplier applied to the angular
         * position and velocity estimates from the OTOS to correct for any systematic errors in the
         * sensor. The value must be between 0.872 and 1.127, which corresponds to a correction
         * of +/- 12.7%.
         */
        public Constants setAngularScalar(double angularScalar) {
            if (angularScalar < Driver.MIN_SCALAR || angularScalar > Driver.MAX_SCALAR) {
                throw new IllegalArgumentException("Angular scalar must be between " +
                        Driver.MIN_SCALAR + " and " + Driver.MAX_SCALAR);
            }
            this.angularScalar = angularScalar;
            return this;
        }
    }

    /**
     * Driver class for the SparkFun OTOS. This code is a modified version of the original driver
     * created by SparkFun. It has been modified to better suit the needs of Apex Pathing. The
     * original code can be found on the SparkFun
     * <a href="https://github.com/sparkfun/SparkFun_Qwiic_OTOS_FTC_Java_Library/">GitHub</a>.
     *
     * @author SparkFun Electronics
     * @author Dylan B. - 18597 RoboClovers Delta
     */
    @I2cDeviceType
    @DeviceProperties(
            name = "SparkFun OTOS",
            xmlTag = "SparkFunOTOS",
            description = "SparkFun Qwiic Optical Tracking Odometry Sensor, optimized for Apex Pathing"
    )
    private static class Driver extends I2cDeviceSynchDevice<I2cDeviceSynch> {
        private final GeometryFactory factory = new GeometryFactory()
                .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.RAD);

        public static final byte DEFAULT_ADDRESS = 0x17; // Default I2C addresses of the Qwiic OTOS
        public static final double MIN_SCALAR = 0.872; // Minimum scalar value for the linear and angular scalars
        public static final double MAX_SCALAR = 1.127; // Maximum scalar value for the linear and angular scalars

        // OTOS register map (not all are listed here because they aren't needed)
        protected static final byte REG_PRODUCT_ID = 0x00;
        protected static final byte REG_SCALAR_LINEAR = 0x04;
        protected static final byte REG_SCALAR_ANGULAR = 0x05;
        protected static final byte REG_IMU_CALIB = 0x06;
        protected static final byte REG_OFF_XL = 0x10;
        protected static final byte REG_POS_XL = 0x20;

        // Product ID register value
        protected static final byte PRODUCT_ID = 0x5F;

        // Conversion factors
        protected static final double DEGREE_TO_RADIAN = Math.PI / 180.0;
        protected static final double METER_TO_INCH = 1.0 / 0.0254;

        // Conversion factor for the linear position registers. 16-bit signed
        // registers with a max value of 10 meters (~393.7 inches) gives a resolution
        // of about 0.012 inches
        protected static final double INCH_TO_INT16 = 32768.0 / (10.0 * METER_TO_INCH);
        protected static final double INT16_TO_INCH = 1.0 / INCH_TO_INT16;

        // Conversion factor for the linear velocity registers. 16-bit signed
        // registers with a max value of 5 mps (~196.85 ips) gives a resolution of about
        // 0.006 ips
        protected static final double IPS_TO_INT16 = 32768.0 / (5.0 * METER_TO_INCH);
        protected static final double INT16_TO_IPS = 1.0 / IPS_TO_INT16;

        // Conversion factor for the linear acceleration registers. 16-bit signed
        // registers with a max value of 157 mps^2 (16 g / ~6177.16 ipss) gives a
        // resolution of about 0.19 ipss
        protected static final double IPSS_TO_INT16 = 32768.0 / (16.0 * 9.80665 * METER_TO_INCH);
        protected static final double INT16_TO_IPSS = 1.0 / IPSS_TO_INT16;

        // Conversion factor for the angular position registers. 16-bit signed
        // registers with a max value of pi radians (180 degrees) gives a resolution
        // of about 0.00096 radians (0.0055 degrees)
        protected static final double RAD_TO_INT16 = 32768.0 / Math.PI;
        protected static final double INT16_TO_RAD = 1.0 / RAD_TO_INT16;

        // Conversion factor for the angular velocity registers. 16-bit signed
        // registers with a max value of 34.9 rps (2000 dps) gives a resolution of
        // about 0.0011 rps (0.061 degrees per second)
        protected static final double RPS_TO_INT16 = 32768.0 / (2000.0 * DEGREE_TO_RADIAN);
        protected static final double INT16_TO_RPS = 1.0 / RPS_TO_INT16;

        // Conversion factor for the angular acceleration registers. 16-bit signed
        // registers with a max value of 3141 rps^2 (180000 dps^2) gives a
        // resolution of about 0.096 rps^2 (5.5 dps^2)
        protected static final double RPSS_TO_INT16 = 32768.0 / (Math.PI * 1000.0);
        protected static final double INT16_TO_RPSS = 1.0 / RPSS_TO_INT16;

        private Pose pose, velocity, acceleration;

        public Driver(I2cDeviceSynch deviceClient) {
            super(deviceClient, true);
            deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
            super.registerArmingStateCallback(false);
            this.deviceClient.engage();
        }

        @Override
        protected boolean doInitialize() { return isConnected();}

        @Override
        public Manufacturer getManufacturer() { return Manufacturer.SparkFun; }

        @Override
        public String getDeviceName() { return "SparkFun Qwiic Optical Tracking Odometry Sensor"; }

        /** @return true if the OTOS is connected, false otherwise */
        public boolean isConnected() { return deviceClient.read8(REG_PRODUCT_ID) == PRODUCT_ID; }

        /**
         * Calibrates the IMU on the OTOS, which removes the accelerometer and gyroscope offsets.
         * This will do the full 255 samples and wait until he calibration is done, which takes
         * about 612ms as of firmware v1.0).
         */
        public void calibrate() {
            // Write 255 to the calibration register (take 255 samples)
            deviceClient.write8(REG_IMU_CALIB, 255);

            // Wait 1 sample period (2.4ms) to ensure the register updates
            try {
                Thread.sleep(3);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }

            for (int numAttempts = 255; numAttempts > 0; numAttempts--) {
                // Read the gyro calibration register value to check completion
                if (deviceClient.read8(REG_IMU_CALIB) == 0) { return; }

                // Give a short delay between reads. As of firmware v1.0, samples take
                // 2.4ms each, so 3ms should guarantee the next sample is done. This
                // also ensures the max attempts is not exceeded in normal operation
                try {
                    Thread.sleep(3);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        /** Resets the position tracker to zero. */
        public void resetTracking() { deviceClient.write(REG_POS_XL, new byte[6]); }

        /** Calibrates the IMU and resets the tracking frame. */
        public void calibrateAndReset() {
            calibrate();
            resetTracking();
        }

        /** @param scalar linear scalar, must be between 0.872 and 1.127 */
        public void setLinearScalar(double scalar) {
            if (scalar < MIN_SCALAR || scalar > MAX_SCALAR) { return; }

            // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
            byte rawScalar = (byte) ((scalar - 1.0) * 1000 + 0.5);
            deviceClient.write8(REG_SCALAR_LINEAR, rawScalar);
        }

        /** @param scalar angular scalar, must be between 0.872 and 1.127 */
        public void setAngularScalar(double scalar) {
            if (scalar < MIN_SCALAR || scalar > MAX_SCALAR) { return; }

            // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
            byte rawScalar = (byte) ((scalar - 1.0) * 1000 + 0.5);
            deviceClient.write8(REG_SCALAR_ANGULAR, rawScalar);
        }

        public void writePose(Pose pose, byte reg) {
            byte[] rawData = new byte[6]; // Store raw data in a temporary buffer

            // Convert factory units to raw data
            short rawX = (short) (pose.getX().getIn() * INCH_TO_INT16);
            short rawY = (short) (pose.getY().getIn() * INCH_TO_INT16);
            short rawH = (short) (pose.getHeading().getRad() * RAD_TO_INT16);

            // Store raw data in buffer
            rawData[0] = (byte) (rawX & 0xFF);
            rawData[1] = (byte) ((rawX >> 8) & 0xFF);
            rawData[2] = (byte) (rawY & 0xFF);
            rawData[3] = (byte) ((rawY >> 8) & 0xFF);
            rawData[4] = (byte) (rawH & 0xFF);
            rawData[5] = (byte) ((rawH >> 8) & 0xFF);

            deviceClient.write(reg, rawData); // Write the raw data to the device
        }

        /** @param pose The new position estimate for the robot */
        public void setPosition(Pose pose) {
            writePose(pose, REG_POS_XL);
        }

        /** @param pose Offset of the sensor relative to the center of the robot */
        public void setOffset(Pose pose) {
            writePose(pose, REG_OFF_XL);
        }

        /**
         * Updates the position, velocity, and acceleration estimates from the OTOS. This should be
         * called regularly in a loop to get the latest estimates from the sensor.
         */
        public void update() {
            byte[] rawData = deviceClient.read(REG_POS_XL, 18);
            ByteBuffer data = ByteBuffer.wrap(rawData);
            data.order(java.nio.ByteOrder.LITTLE_ENDIAN);

            pose = factory.pose(
                    data.getShort(0) * INT16_TO_INCH,
                    data.getShort(2) * INT16_TO_INCH,
                    data.getShort(4) * INT16_TO_RAD
            );

            velocity = factory.pose(
                    data.getShort(6) * INT16_TO_IPS,
                    data.getShort(8) * INT16_TO_IPS,
                    data.getShort(10) * INT16_TO_RPS
            );

            acceleration = factory.pose(
                    data.getShort(12) * INT16_TO_IPSS,
                    data.getShort(14) * INT16_TO_IPSS,
                    data.getShort(16) * INT16_TO_RPSS
            );
        }

        /** @return the current pose estimate of the robot from the OTOS */
        public Pose getPose() { return pose; }

        /** @return the current velocity estimate of the robot from the OTOS */
        public Pose getVel() { return velocity; }

        /** @return the current acceleration estimate of the robot from the OTOS */
        public Pose getAccel() { return acceleration; }
    }
}