package org.firstinspires.ftc.teamcode.apexpathing;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class AutoTestSequenceTest {
    @Test
    public void autoTestRunsAllFollowerChecksInOrder() {
        assertEquals(AutoTest.AutoState.POINT_TURN,
                AutoTest.nextState(AutoTest.AutoState.OUTBOUND_CURVE));
        assertEquals(AutoTest.AutoState.REVERSE_RETURN,
                AutoTest.nextState(AutoTest.AutoState.POINT_TURN));
        assertEquals(AutoTest.AutoState.STRAFE_OUT,
                AutoTest.nextState(AutoTest.AutoState.REVERSE_RETURN));
        assertEquals(AutoTest.AutoState.STRAFE_BACK,
                AutoTest.nextState(AutoTest.AutoState.STRAFE_OUT));
        assertEquals(AutoTest.AutoState.COMPLETE,
                AutoTest.nextState(AutoTest.AutoState.STRAFE_BACK));
    }

    @Test
    public void terminalAutoStatesCannotAdvance() {
        assertEquals(AutoTest.AutoState.COMPLETE,
                AutoTest.nextState(AutoTest.AutoState.COMPLETE));
        assertEquals(AutoTest.AutoState.FAILED,
                AutoTest.nextState(AutoTest.AutoState.FAILED));
        assertEquals(AutoTest.AutoState.ABORTED,
                AutoTest.nextState(AutoTest.AutoState.ABORTED));
    }
}
