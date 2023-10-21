package org.firstinspires.ftc.teamcode.subsystems.stateMachineController;

import java.util.ArrayList;

public interface State {
    /* Write the state's "do stuff" here. */
    void loop();
    /* Write the state's edges here. */
    ArrayList<Edge> getEdges();

}