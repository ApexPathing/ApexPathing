package tuning;

import controllers.PDSController.PDSCoefficients;

public class TranslationalPhase extends TuningPhase {
    private PDSRoutine routine;

    public TranslationalPhase(TunerContext context) { super(context); }

    @Override
    protected String getPhaseName() { return "Translational PDS"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void init() {
        // Moving the robot forward because this controller does all positional movement
        routine = new PDSRoutine(context, PDSRoutine.Axis.DRIVE);
        routine.start();
    }

    @Override
    protected boolean manualUpdate() {
        return false; // TODO: Implement manual tuning
    }

    @Override
    protected boolean automaticUpdate() {
        boolean done = routine.update(context);
        if (done) { context.constants.translationalCoeffs = routine.getCoefficients(); }
        return done;
    }

    @Override
    protected void reportResults() {
        PDSCoefficients coefficients = context.constants.translationalCoeffs;
        context.getTelemetry().addData(
                "Translational PDS Coefficients",
                "P: %.3f, D: %.3f, S: %.3f",
                coefficients.kP, coefficients.kD, coefficients.kS
        );
    }
}
