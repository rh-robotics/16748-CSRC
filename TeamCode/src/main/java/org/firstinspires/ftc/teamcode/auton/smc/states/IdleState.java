package org.firstinspires.ftc.teamcode.auton.smc.states;

import org.firstinspires.ftc.teamcode.auton.smc.StateInterface;

public class IdleState implements StateInterface {
    @Override
    public Class<? extends StateInterface> checkEdges() {
        return IdleState.class;
    }

    @Override
    public void stateAction() {

    }
}
