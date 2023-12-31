package org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest;

public interface StateInterface {
    /* Run before stateAction() to check to change states. */
    Class<? extends StateInterface> checkEdges();

    /* The "do stuff" of a state. */
    void stateAction();
}
