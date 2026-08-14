package org.firstinspires.ftc.teamcode.sim;

import org.codeblooded.ftcodesim.simulator.FTCodeSimLinearOpModeRunner;
import org.junit.Assume;
import org.junit.Test;

/**
 * Interactive FTCodeSim entry point. Select one enabled TeamCode OpMode in the simulated Driver
 * Station; only the selected OpMode is initialized and run.
 */
public class SimulateApexPathing {
    /** Set true when intentionally launching the interactive FTCodeSim window. */
    private static final boolean RUN_INTERACTIVE_SIMULATOR = false;

    @Test
    public void selectAndRunOpMode() throws Exception {
        // This entry point intentionally remains open until the simulator windows are closed.
        // Keep ordinary unit-test/build runs finite and require an explicit source-level opt-in.
        Assume.assumeTrue("Set RUN_INTERACTIVE_SIMULATOR to true to launch FTCodeSim",
                RUN_INTERACTIVE_SIMULATOR);
        FTCodeSimLinearOpModeRunner.run(ApexSimulation.createSimulator());
    }
}
