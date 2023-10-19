package org.firstinspires.ftc.teamcode.auton.smc.states;

import org.firstinspires.ftc.teamcode.auton.smc.StateInterface;

public class DrivingState implements StateInterface {
    @Override
    public Class<? extends StateInterface> checkEdges() {
        return DrivingState.class;
    }

    @Override
    public void stateAction() {

    }
}
