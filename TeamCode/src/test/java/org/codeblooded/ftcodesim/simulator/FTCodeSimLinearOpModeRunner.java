package org.codeblooded.ftcodesim.simulator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.psilynx.psikit.core.Logger;

import java.util.concurrent.atomic.AtomicBoolean;

/** Runs FTCodeSim with the FTC SDK's real LinearOpMode lifecycle. */
public final class FTCodeSimLinearOpModeRunner {
    private FTCodeSimLinearOpModeRunner() { }

    public static void run(FTCodeSim simulator) throws InterruptedException {
        try {
            while (simulator.windowIsRunning()) {
                OpModeLifecycle lifecycle = simulator.opModeLifecycle;
                if (lifecycle == null) {
                    Thread.sleep(simulator.config.loopTimeMs);
                    continue;
                }

                if (lifecycle.opMode instanceof LinearOpMode) {
                    runLinearOpMode(lifecycle);
                } else {
                    lifecycle.runOpMode();
                }

                if (simulator.opModeLifecycle == lifecycle) {
                    simulator.opModeLifecycle = null;
                }
            }
        } finally {
            simulator.close();
        }
    }

    static void runLinearOpMode(OpModeLifecycle lifecycle) throws InterruptedException {
        LinearOpMode opMode = (LinearOpMode) lifecycle.opMode;
        opMode.telemetry = lifecycle.telemetry;
        opMode.hardwareMap = lifecycle.simHardwareMap;
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();

        SimFtcLogger ftcLog = new SimFtcLogger();
        ftcLog.start(opMode, 5800, "", true, "sim-logs");
        Logger.setSimulation(true);
        long start = System.nanoTime();
        Logger.setTimeSource(() -> (System.nanoTime() - start) * 1e-9);

        AtomicBoolean userRequestedStop = new AtomicBoolean(false);
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(
                opMode,
                () -> userRequestedStop.set(true)
        );

        try {
            while (!lifecycle.isStarted
                    && !lifecycle.isStopped
                    && !userRequestedStop.get()) {
                runEventLoopIteration(lifecycle, session);
            }

            if (lifecycle.isStarted
                    && !lifecycle.isStopped
                    && !userRequestedStop.get()) {
                SimLinearOpModeBridge.start(session);
            }

            while (!lifecycle.isStopped && !userRequestedStop.get()) {
                runEventLoopIteration(lifecycle, session);
            }
        } finally {
            SimLinearOpModeBridge.stop(session);
            Logger.end();
        }
    }

    private static void runEventLoopIteration(
            OpModeLifecycle lifecycle,
            SimLinearOpModeBridge.Session session
    ) throws InterruptedException {
        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();
        gamepad1.fromByteArray(lifecycle.latestGamepad1Data);
        gamepad2.fromByteArray(lifecycle.latestGamepad2Data);

        lifecycle.wrap(() -> SimLinearOpModeBridge.eventLoopIteration(
                session,
                gamepad1,
                gamepad2
        ));
    }
}
