package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.codeblooded.ftcodesim.hardware.devices.SimHardwareMechanism;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.function.LongSupplier;

public class StableSimHardwareMapTest {
    @Test
    public void delayedFrameIsSplitIntoBoundedPhysicsSteps() {
        MutableClock clock = new MutableClock();
        StableSimHardwareMap hardwareMap = new StableSimHardwareMap(clock);
        RecordingMechanism mechanism = new RecordingMechanism();
        hardwareMap.register(mechanism);

        clock.advanceSeconds(0.243);
        hardwareMap.update();

        assertEquals(25, mechanism.steps.size());
        assertEquals(0.243, mechanism.totalSeconds(), 1e-9);
        for (double step : mechanism.steps) {
            assertTrue(step <= StableSimHardwareMap.MAX_PHYSICS_STEP_SECONDS + 1e-12);
        }
    }

    @Test
    public void debuggerPauseDoesNotTriggerUnboundedCatchUp() {
        MutableClock clock = new MutableClock();
        StableSimHardwareMap hardwareMap = new StableSimHardwareMap(clock);
        RecordingMechanism mechanism = new RecordingMechanism();
        hardwareMap.register(mechanism);

        clock.advanceSeconds(5.0);
        hardwareMap.update();

        assertEquals(StableSimHardwareMap.MAX_CATCH_UP_SECONDS,
                mechanism.totalSeconds(), 1e-9);
    }

    private static final class MutableClock implements LongSupplier {
        private long nanos;

        @Override
        public long getAsLong() { return nanos; }

        void advanceSeconds(double seconds) {
            nanos += Math.round(seconds * 1e9);
        }
    }

    private static final class RecordingMechanism implements SimHardwareMechanism {
        final List<Double> steps = new ArrayList<Double>();

        @Override
        public void update(double deltaTime) { steps.add(deltaTime); }

        double totalSeconds() {
            double total = 0.0;
            for (double step : steps) { total += step; }
            return total;
        }
    }
}
