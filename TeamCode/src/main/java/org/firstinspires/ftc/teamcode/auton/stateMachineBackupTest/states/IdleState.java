package org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest.states;

import org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest.StateInterface;

public class IdleState implements StateInterface {
    @Override
    public Class<? extends StateInterface> checkEdges() {
        return IdleState.class;
    }

    @Override
    public void stateAction() {

    }
}
