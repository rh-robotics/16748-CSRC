package org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest.states;

import org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest.StateInterface;

public class IntakeState implements StateInterface {
    @Override
    public Class<? extends StateInterface> checkEdges() {
        return IntakeState.class;
    }

    @Override
    public void stateAction() {

    }
}
