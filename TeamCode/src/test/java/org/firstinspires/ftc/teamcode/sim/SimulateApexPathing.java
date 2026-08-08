package org.firstinspires.ftc.teamcode.sim;

import org.codeblooded.ftcodesim.simulator.FTCodeSimLinearOpModeRunner;
import org.junit.Test;

/**
 * Interactive FTCodeSim entry point. Select one enabled TeamCode OpMode in the simulated Driver
 * Station; only the selected OpMode is initialized and run.
 */
public class SimulateApexPathing {
    @Test
    public void selectAndRunOpMode() throws Exception {
        FTCodeSimLinearOpModeRunner.run(ApexSimulation.createSimulator());
    }
}
