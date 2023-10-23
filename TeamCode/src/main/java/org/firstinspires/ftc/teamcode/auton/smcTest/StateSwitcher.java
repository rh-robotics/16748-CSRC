package org.firstinspires.ftc.teamcode.auton.smcTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auton.stateMachine.states.IdleState;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.StateMachine;

@Autonomous(name = "State Machine Switcher")
public class StateSwitcher extends OpMode {

    StateMachine states;
    @Override
    public void init() {
        states = new StateMachine();
        states.addState(new DrivingState);
        states.addState(new IdleState);
        states.addState(new ScoringState);
    }

    public void changeStateDriving() {
        states.currentState = states.states.get(DrivingState.class);
        telemetry.addData("State", "Driving State");
    }
    public void changeStateIdle() {
        states.currentState = states.states.get(IdleState.class);
        telemetry.addData("State", "Idle State");
    }
    public void changeStateScoring() {
        states.currentState = states.states.get(ScoringState.class);
        telemetry.addData("States", "Scoring State");
    }





}
