package org.firstinspires.ftc.teamcode.sim;

import org.codeblooded.ftcodesim.hardware.devices.SimTelemetry;
import org.codeblooded.ftcodesim.simulator.FTCodeSim;
import org.firstinspires.ftc.robotcore.external.Func;

import java.util.function.Consumer;

/** Adds the addData support that is currently missing from FTCodeSim's SimTelemetry. */
final class ApexSimTelemetry extends SimTelemetry {
    private final Consumer<String> telemetrySink;
    private final StringBuilder currentFrame = new StringBuilder();

    private String queuedFrame;
    private Thread frameOwner;
    private int transmissionIntervalMs = 100;
    private long lastTransmissionNanos;
    private boolean hasTransmitted;

    ApexSimTelemetry(FTCodeSim simulator) {
        super(simulator);
        this.telemetrySink = simulator::sendTelemetry;
    }

    ApexSimTelemetry(Consumer<String> telemetrySink) {
        super(null);
        this.telemetrySink = telemetrySink;
    }

    @Override
    public synchronized Item addData(String caption, String format, Object... args) {
        return addData(caption, String.format(format, args));
    }

    @Override
    public synchronized Item addData(String caption, Object value) {
        claimFrame();
        currentFrame.append(caption).append(' ').append(String.valueOf(value)).append('\n');
        return null;
    }

    @Override
    public synchronized <T> Item addData(String caption, Func<T> valueProducer) {
        return addData(caption, valueProducer.value());
    }

    @Override
    public synchronized <T> Item addData(String caption, String format,
                                         Func<T> valueProducer) {
        return addData(caption, format, valueProducer.value());
    }

    @Override
    public synchronized Line addLine() {
        claimFrame();
        currentFrame.append('\n');
        return null;
    }

    @Override
    public synchronized Line addLine(String lineCaption) {
        claimFrame();
        currentFrame.append(lineCaption).append('\n');
        return null;
    }

    @Override
    public synchronized void clear() {
        currentFrame.setLength(0);
        frameOwner = null;
    }

    @Override
    public synchronized void clearAll() {
        clear();
        queuedFrame = null;
    }

    @Override
    public synchronized int getMsTransmissionInterval() {
        return transmissionIntervalMs;
    }

    @Override
    public synchronized void setMsTransmissionInterval(int msTransmissionInterval) {
        transmissionIntervalMs = Math.max(0, msTransmissionInterval);
    }

    @Override
    public synchronized boolean update() {
        if (currentFrame.length() > 0 && frameOwner == Thread.currentThread()) {
            // Retain only the newest complete telemetry frame while rate-limited. FollowerTuner's
            // init selector calls update in a tight loop, so appending every frame would flood the
            // socket and Swing event queue.
            queuedFrame = currentFrame.toString();
            currentFrame.setLength(0);
            frameOwner = null;
        }

        if (queuedFrame == null) {
            return false;
        }

        long now = System.nanoTime();
        long intervalNanos = transmissionIntervalMs * 1_000_000L;
        if (hasTransmitted && now - lastTransmissionNanos < intervalNanos) {
            return false;
        }

        telemetrySink.accept(queuedFrame);
        queuedFrame = null;
        lastTransmissionNanos = now;
        hasTransmitted = true;
        return true;
    }

    private void claimFrame() {
        if (frameOwner == null) {
            frameOwner = Thread.currentThread();
        }
    }
}
