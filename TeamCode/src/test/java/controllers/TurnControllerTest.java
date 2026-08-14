package controllers;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurnControllerTest {
    @Test
    public void stalledProfiledMotionRetainsBreakawayAuthorityDuringDeceleration() {
        assertEquals(0.26375, TurnController.ensureProfiledMotionBreakaway(
                0.20, 3.6, 0.0, 0.24375), 1e-9);
        assertEquals(-0.26375, TurnController.ensureProfiledMotionBreakaway(
                -0.20, -3.6, 0.0, 0.24375), 1e-9);
    }

    @Test
    public void lowSpeedProfileDoesNotStopBeforeEndpointRecovery() {
        assertEquals(0.26375, TurnController.ensureProfiledMotionBreakaway(
                0.04, 0.30, 0.10, 0.24375), 1e-9);
        assertEquals(-0.26375, TurnController.ensureProfiledMotionBreakaway(
                -0.04, -0.30, -0.10, 0.24375), 1e-9);
    }

    @Test
    public void breakawayGuardDoesNotAlterMovingOrStoppedProfileCommands() {
        assertEquals(0.20, TurnController.ensureProfiledMotionBreakaway(
                0.20, 3.6, 0.5, 0.24375), 1e-9);
        assertEquals(0.0, TurnController.ensureProfiledMotionBreakaway(
                0.0, 0.0, 0.0, 0.24375), 1e-9);
        assertEquals(0.40, TurnController.ensureProfiledMotionBreakaway(
                0.40, 3.6, 0.0, 0.24375), 1e-9);
    }

    @Test
    public void breakawayGuardCorrectsACommandOpposingRequestedMotion() {
        assertEquals(0.26375, TurnController.ensureProfiledMotionBreakaway(
                -0.10, 3.6, 0.0, 0.24375), 1e-9);
    }

    @Test
    public void endpointCaptureBlendsSmoothlyBeforeProfileVelocityReachesZero() {
        assertEquals(0.20, TurnController.blendEndpointCapturePower(
                0.20, 0.60, 1.0, 1.0), 1e-9);
        assertEquals(0.40, TurnController.blendEndpointCapturePower(
                0.20, 0.60, 0.5, 1.0), 1e-9);
        assertEquals(0.60, TurnController.blendEndpointCapturePower(
                0.20, 0.60, 0.0, 1.0), 1e-9);
    }
}
