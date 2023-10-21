package org.firstinspires.ftc.teamcode.subsystems.stateMachineController;

import java.util.HashMap;
import java.util.ArrayList;

public class StateMachine {
    /* Storing states in <Class, Instance of Class> pairs.*/
    HashMap<Class <? extends State>, State> states;
    State currentState;

    public void addState(State state) {
        this.states.put(state.getClass(), state);
    }

    public void loop() {
        if (states.isEmpty()) {
            throw new RuntimeException("No states defined in State Machine.");
        }
        else if (!states.containsKey(currentState.getClass())) {
            throw new RuntimeException("Current state not defined in state machine.");
        }
        else if (currentState == null){
            throw new RuntimeException("Current state is null. Check if initial state was set.");
        }

        checkEdges();
        currentState.loop();
    }

    public void checkEdges() {
        for (Edge edge : currentState.getEdges()) {
            if(edge.getCallback()) {
                currentState = states.get(edge.to);
            }
        }
    }
}
