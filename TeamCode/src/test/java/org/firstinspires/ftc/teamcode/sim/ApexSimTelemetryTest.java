package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class ApexSimTelemetryTest {
    @Test
    public void includesDataAndLinesInDriverStationFrame() {
        List<String> frames = new ArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);

        telemetry.addData("Selected", "HEADING");
        telemetry.addLine("Press A to run this phase.");

        assertTrue(telemetry.update());
        assertEquals(1, frames.size());
        assertEquals(
                "Selected HEADING\nPress A to run this phase.\n",
                frames.get(0)
        );
    }

    @Test
    public void coalescesFastTunerUpdatesInsteadOfFloodingDriverStation() {
        List<String> frames = new ArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(10_000);

        telemetry.addLine("old frame");
        assertTrue(telemetry.update());

        telemetry.addLine("intermediate frame");
        assertFalse(telemetry.update());
        telemetry.addLine("latest frame");
        assertFalse(telemetry.update());

        telemetry.setMsTransmissionInterval(0);
        assertTrue(telemetry.update());
        assertEquals(2, frames.size());
        assertEquals("latest frame\n", frames.get(1));
    }

    @Test
    public void eventLoopUpdateDoesNotSplitAUserTelemetryFrame() throws Exception {
        List<String> frames = new ArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        telemetry.addLine("first line");
        Thread eventLoop = new Thread(telemetry::update);
        eventLoop.start();
        eventLoop.join();
        telemetry.addLine("second line");

        assertTrue(telemetry.update());
        assertEquals(1, frames.size());
        assertEquals("first line\nsecond line\n", frames.get(0));
    }
}
