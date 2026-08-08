package org.codeblooded.ftcodesim.simulator;

import org.codeblooded.ftcodesim.hardware.devices.SimTelemetry;

/** Installs project-specific telemetry into FTCodeSim and its selected OpMode lifecycle. */
public final class FTCodeSimTelemetryInstaller {
    private FTCodeSimTelemetryInstaller() { }

    public static void install(FTCodeSim simulator, SimTelemetry telemetry) {
        simulator.telemetry = telemetry;

        Thread synchronizer = new Thread(() -> {
            while (simulator.state != null && simulator.windowIsRunning()) {
                OpModeLifecycle lifecycle = simulator.opModeLifecycle;
                if (lifecycle != null) {
                    lifecycle.telemetry = telemetry;
                    lifecycle.opMode.telemetry = telemetry;
                }

                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }, "Apex telemetry installer");
        synchronizer.setDaemon(true);
        synchronizer.start();
    }
}
