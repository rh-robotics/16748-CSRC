package org.firstinspires.ftc.teamcode.subsystems.stateMachineController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.ArrayList;

public class StateMachine {
    /* Storing states in <Class, Instance of Class> pairs.*/
    public HashMap<Class<? extends State>, State> states;
    public State currentState;

    public void addState(State state) {
        this.states.put(state.getClass(), state);
    }

    public void loop(Telemetry telemetry) {
        if (states.isEmpty()) {
            throw new RuntimeException("No states defined in State Machine.");
        } else if (!states.containsKey(currentState.getClass())) {
            throw new RuntimeException("Current state not defined in state machine.");
        } else if (currentState == null) {
            throw new RuntimeException("Current state is null. Check if initial state was set.");
        }

        checkEdges(telemetry);
        currentState.loop();
    }

    public void checkEdges(Telemetry telemetry) {
        ArrayList<Edge> edgeCallbacks = new ArrayList();

        for (Edge edge : currentState.getEdges()) {
            if (edge.getCallback()) {
                edgeCallbacks.add(edge);
            }
        }
        if (edgeCallbacks.size() > 1) {
            telemetry.addLine("Multiple edge callbacks");
        }
        currentState = states.get(edgeCallbacks.get(0).to);
    }
}
