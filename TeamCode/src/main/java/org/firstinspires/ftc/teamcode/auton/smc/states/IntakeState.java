package org.firstinspires.ftc.teamcode.auton.smc.states;

import org.firstinspires.ftc.teamcode.auton.smc.StateInterface;

public class IntakeState implements StateInterface {
    @Override
    public Class<? extends StateInterface> checkEdges() {
        return IntakeState.class;
    }

    @Override
    public void stateAction() {

    }
}
