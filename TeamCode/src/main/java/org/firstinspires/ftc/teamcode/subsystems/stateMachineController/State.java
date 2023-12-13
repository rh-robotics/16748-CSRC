package org.firstinspires.ftc.teamcode.subsystems.stateMachineController;

import java.util.ArrayList;

/**
 * When running our state machines, States are used to store different actions and methods
 * separately, which allows us to organize actions. Moving between States happens through the
 * use of Edges
 */
public interface State {
    /* Write the state's "do stuff" here. */
    void loop();
    /* Write the state's edges here. */
    ArrayList<Edge> getEdges();

}