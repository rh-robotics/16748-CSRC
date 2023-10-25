package org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest.states;

import org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest.StateInterface;

public class ScoringState implements StateInterface {
    @Override
    public Class<? extends StateInterface> checkEdges() {
        return ScoringState.class;
    }

    @Override
    public void stateAction() {

    }
}
