package tuning;

import controllers.PDSController.PDSCoefficients;

public class HeadingPhase extends TuningPhase {
    private PDSRoutine routine;

    public HeadingPhase(TunerContext context) { super(context); }

    @Override
    protected String getPhaseName() { return "Heading PDS"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void init() {
        routine = new PDSRoutine(context, PDSRoutine.Axis.HEADING);
        routine.start();
    }

    @Override
    protected boolean manualUpdate() {
        return false; // TODO: Implement manual tuning
    }

    @Override
    protected boolean automaticUpdate() {
        boolean done = routine.update(context);
        if (done) { context.constants.headingCoeffs = routine.getCoefficients(); }
        return done;
    }

    @Override
    protected void reportResults() {
        PDSCoefficients coefficients = context.constants.headingCoeffs;
        context.getTelemetry().addData(
                "Heading PDS Coefficients",
                "P: %.3f, D: %.3f, S: %.3f",
                coefficients.kP, coefficients.kD, coefficients.kS
        );
    }
}
