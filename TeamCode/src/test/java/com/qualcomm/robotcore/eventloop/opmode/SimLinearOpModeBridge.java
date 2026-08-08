package com.qualcomm.robotcore.eventloop.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Test-only access to the FTC SDK lifecycle hooks that FTCodeSim cannot call directly.
 *
 * <p>The SDK deliberately makes these hooks package-private. Keeping this bridge in the SDK
 * package lets the simulator use the real LinearOpMode state machine instead of trying to imitate
 * {@code waitForStart()}, {@code opModeInInit()}, and stop behavior.</p>
 */
public final class SimLinearOpModeBridge {
    private SimLinearOpModeBridge() { }

    public static Session initialize(LinearOpMode opMode, Runnable stopRequest) {
        opMode.internalOpModeServices = new OpModeServices() {
            @Override
            public void refreshUserTelemetry(TelemetryMessage telemetry, double intervalSeconds) {
                // FTCodeSim's SimTelemetry sends its own packets to the desktop Driver Station.
            }

            @Override
            public void requestOpModeStop(OpMode requestedOpMode) {
                if (requestedOpMode == opMode) {
                    stopRequest.run();
                }
            }
        };

        opMode.isStarted = false;
        opMode.stopRequested = false;
        opMode.gamepad1.resetEdgeDetection();
        opMode.gamepad2.resetEdgeDetection();

        AtomicReference<Throwable> failure = new AtomicReference<>();
        Thread worker = new Thread(() -> {
            try {
                opMode.internalRunOpMode();
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            } catch (Throwable throwable) {
                if (!isAndroidRobotLogOnLinearOpModeExit(throwable)) {
                    failure.set(throwable);
                }
            } finally {
                stopRequest.run();
            }
        }, "OpModeThread");
        worker.setDaemon(true);

        Session session = new Session(opMode, worker, failure);
        worker.start();
        return session;
    }

    public static void start(Session session) {
        session.opMode.stopRequested = false;
        session.opMode.isStarted = true;
        session.opMode.internalOnStart();
    }

    public static void eventLoopIteration(
            Session session,
            Gamepad gamepad1,
            Gamepad gamepad2
    ) {
        LinearOpMode opMode = session.opMode;
        opMode.newGamepadDataAvailable(gamepad1, gamepad2);
        opMode.internalOnEventLoopIteration();
        throwIfFailed(session);
    }

    public static void stop(Session session) {
        session.opMode.stopRequested = true;
        session.opMode.internalOnStart();
        session.worker.interrupt();

        try {
            session.worker.join(1_000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException("Interrupted while stopping simulated OpMode", e);
        }

        if (session.worker.isAlive()) {
            throw new IllegalStateException("Simulated LinearOpMode did not stop within one second");
        }
        throwIfFailed(session);
    }

    private static void throwIfFailed(Session session) {
        Throwable failure = session.failure.get();
        if (failure == null) {
            return;
        }
        if (failure instanceof RuntimeException) {
            throw (RuntimeException) failure;
        }
        if (failure instanceof Error) {
            throw (Error) failure;
        }
        throw new RuntimeException("Simulated LinearOpMode failed", failure);
    }

    private static boolean isAndroidRobotLogOnLinearOpModeExit(Throwable throwable) {
        if (!(throwable instanceof RuntimeException)
                || throwable.getMessage() == null
                || !throwable.getMessage().contains("android.util.Log not mocked")) {
            return false;
        }

        for (StackTraceElement element : throwable.getStackTrace()) {
            if (LinearOpMode.class.getName().equals(element.getClassName())
                    && "internalRunOpMode".equals(element.getMethodName())) {
                return true;
            }
        }
        return false;
    }

    public static final class Session {
        private final LinearOpMode opMode;
        private final Thread worker;
        private final AtomicReference<Throwable> failure;

        private Session(
                LinearOpMode opMode,
                Thread worker,
                AtomicReference<Throwable> failure
        ) {
            this.opMode = opMode;
            this.worker = worker;
            this.failure = failure;
        }
    }
}
