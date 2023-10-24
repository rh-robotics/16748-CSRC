package org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.stateMachineBackupTest.states.*;

import java.util.HashMap;

/* Autonomous State Machine Backup (Without Use of Controller).  */
@Autonomous(name = "State Machine Backup")
public class StateMachineOpMode extends OpMode {
    private final ElapsedTime time = new ElapsedTime();

    /* Holds the key to the entry of the current state in states HashMap. */
    Class<? extends StateInterface> currentState;
    DrivingState drivingState = new DrivingState();
    ScoringState scoringState = new ScoringState();
    IntakeState intakeState = new IntakeState();
    IdleState idleState = new IdleState();
    Class<? extends StateInterface> initialState = DrivingState.class;
    public HashMap<Class<? extends StateInterface>, StateInterface> states = new HashMap<>();

    public StateMachineOpMode() {
        states.put(drivingState.getClass(), drivingState);
        states.put(scoringState.getClass(), scoringState);
        states.put(intakeState.getClass(), intakeState);
        states.put(idleState.getClass(), idleState);
    }

    public void init() {
        /* Checks if initial state is a valid state. */
        if (!states.containsKey(initialState)) {
            throw new RuntimeException("Nonexistent initial state: '" + initialState.getSimpleName() + "'.");
        } else {
            currentState = initialState;
        }
    }

    public void start() {
        time.reset();
    }

    public void loop() {
        /* Checks if current state is a real state before running.
         * No feasible way for this exception to be thrown... */
        if (!states.containsKey(currentState)) {
            if (currentState != null) {
                throw new RuntimeException("Current state nonexistent: '" + currentState.getSimpleName() + "'.");
            } else {
                throw new RuntimeException("Current state is null. Check if currentState was initialized.");
            }
        } else {
            StateInterface state = states.get(currentState);
            assert (state != null);

            runState(state);
        }
    }

    public void runState(StateInterface state) {
        /* Checks if state we're checking edges of a nonexistent state.
         *  No feasible way for this exception to be thrown unless called outside of loop. */
        if (!states.containsKey(state.getClass())) {
            throw new RuntimeException("Attempting to check edges of nonexistent state: '" +
                    state.getClass().getSimpleName() + "'.");
        }

        Class<? extends StateInterface> newStateType = state.checkEdges();
        /* Checks if the state returned by checkEdges() is a nonexistent state. */
        if (!states.containsKey(newStateType)) {
            throw new RuntimeException("Edge directing to nonexistent state: '" +
                    state.checkEdges() + "'.");
        } else {
            StateInterface newState = states.get(newStateType);
            /* As defined in StateInterface, state.checkEdges() cannot return null. */
            assert (newState != null);

            /* Already confirmed that the state directed to from checkEdges() is a real state. */
            currentState = newState.getClass();
        }

        state.stateAction();
    }
}