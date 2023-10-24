package org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest;

import org.junit.Test;
import org.junit.Assert;

import org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest.states.*;

import java.util.HashMap;

public class StateMachineOpModeTest {
    @Test
    public void currentStateIsNonexistentInLoop() {
        Exception exception = Assert.assertThrows(RuntimeException.class, () -> {
            StateMachineOpMode stateMachine = new StateMachineOpMode();
            stateMachine.init();
            stateMachine.states = new HashMap<>();
            stateMachine.loop();
        });
        Assert.assertTrue(exception.getMessage().contains("Current state nonexistent: "));
    }

    @Test
    public void currentStateIsNullInLoop() {
        Exception exception = Assert.assertThrows(RuntimeException.class, () -> {
            StateMachineOpMode stateMachine = new StateMachineOpMode();
            stateMachine.states = new HashMap<>();
            stateMachine.loop();
        });
        Assert.assertTrue(exception.getMessage().contains("Current state is null."));
    }

    @Test
    public void checkingEdgesOfNonexistentStateInRunState() {
        Exception exception = Assert.assertThrows(RuntimeException.class, () -> {
            StateMachineOpMode stateMachine = new StateMachineOpMode();
            stateMachine.init();
            stateMachine.states = new HashMap<>();
            stateMachine.runState(new DrivingState());
        });
        Assert.assertTrue(exception.getMessage().contains("Attempting to check edges of nonexistent state: "));
    }

    /* A RuntimeException is thrown in runState() when checkEdges() points to a nonexistent state.
     *
     * But I'm not sure how to test this, so I didn't write a test for it to prove that it works yet.
     * Growth mindset. */
}