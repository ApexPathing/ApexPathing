package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.codeblooded.ftcodesim.hardware.SimHardwareMap;
import org.codeblooded.ftcodesim.hardware.devices.SimHardwareDevice;
import org.codeblooded.ftcodesim.hardware.devices.SimHardwareMechanism;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Set;
import java.util.function.LongSupplier;

/**
 * FTCodeSim hardware map that advances the motor model in bounded physics steps.
 *
 * <p>The upstream hardware map passes the entire wall-clock interval to an explicit-Euler motor
 * and drivetrain update. A telemetry, logger, or desktop scheduling pause can therefore turn one
 * delayed frame into an unrealistic velocity/pose jump. Substepping preserves elapsed simulation
 * time while preventing that numerical instability. A long debugger pause is capped so resuming
 * does not make the robot attempt to catch up several seconds in one update.</p>
 */
final class StableSimHardwareMap extends SimHardwareMap {
    static final double MAX_PHYSICS_STEP_SECONDS = 0.010;
    static final double MAX_CATCH_UP_SECONDS = 0.250;

    private final Set<SimHardwareDevice> devices = Collections.newSetFromMap(
            new IdentityHashMap<SimHardwareDevice, Boolean>());
    private final List<SimHardwareMechanism> stableMechanisms =
            new ArrayList<SimHardwareMechanism>();
    private final LongSupplier clock;
    private long previousTimeNanos;

    StableSimHardwareMap() {
        this(System::nanoTime);
    }

    StableSimHardwareMap(LongSupplier clock) {
        super();
        this.clock = clock;
        previousTimeNanos = clock.getAsLong();
    }

    @Override
    public <T extends HardwareDevice> T register(String name, T device) {
        T registered = super.register(name, device);
        if (registered instanceof SimHardwareDevice) {
            devices.add((SimHardwareDevice) registered);
        }
        return registered;
    }

    @Override
    public SimHardwareMechanism register(SimHardwareMechanism mechanism) {
        SimHardwareMechanism registered = super.register(mechanism);
        stableMechanisms.add(registered);
        return registered;
    }

    @Override
    public void update() {
        long now = clock.getAsLong();
        double elapsedSeconds = (now - previousTimeNanos) * 1e-9;
        previousTimeNanos = now;
        if (!Double.isFinite(elapsedSeconds) || elapsedSeconds <= 0.0) { return; }

        elapsedSeconds = Math.min(elapsedSeconds, MAX_CATCH_UP_SECONDS);
        int stepCount = Math.max(1,
                (int) Math.ceil(elapsedSeconds / MAX_PHYSICS_STEP_SECONDS));
        double stepSeconds = elapsedSeconds / stepCount;

        for (int step = 0; step < stepCount; step++) {
            // Motors create wheel velocity, mechanisms turn wheel velocity into chassis motion,
            // and sensors observe the resulting pose. This also removes the one-frame sensor lag
            // in the upstream all-devices-before-mechanisms update order.
            for (SimHardwareDevice device : devices) {
                if (device instanceof SimMotor) { device.update(stepSeconds); }
            }
            for (SimHardwareMechanism mechanism : stableMechanisms) {
                mechanism.update(stepSeconds);
            }
            for (SimHardwareDevice device : devices) {
                if (!(device instanceof SimMotor)) { device.update(stepSeconds); }
            }
        }
    }
}
