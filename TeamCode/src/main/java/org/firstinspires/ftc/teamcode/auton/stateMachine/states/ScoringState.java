package org.firstinspires.ftc.teamcode.auton.stateMachine.states;

import org.firstinspires.ftc.teamcode.auton.stateMachine.StateInterface;

public class ScoringState implements StateInterface {
    @Override
    public Class<? extends StateInterface> checkEdges() {
        return ScoringState.class;
    }

    @Override
    public void stateAction() {

    }
}
