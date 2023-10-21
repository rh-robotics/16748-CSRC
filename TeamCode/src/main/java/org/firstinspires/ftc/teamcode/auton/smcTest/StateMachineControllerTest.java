package org.firstinspires.ftc.teamcode.auton.smcTest;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.*;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.StateMachine;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

@Autonomous(name = "SMC Test")
public class StateMachineControllerTest extends OpMode {
    public static Gamepad gamepad;
    public static Telemetry pTelemetry;

    StateMachine stateMachine;

    @Override
    public void init() {
        gamepad = gamepad1;
        pTelemetry = telemetry;

        stateMachine = new StateMachine();
        stateMachine.addState(new DrivingState());
        stateMachine.addState(new ScoringState());

        stateMachine.currentState = stateMachine.states.get(DrivingState.class);
    }
    @Override
    public void loop() {
        stateMachine.loop(telemetry);
    }
}

class DrivingState implements State {
    ArrayList<Edge> edges = new ArrayList();
    public ArrayList<Edge> getEdges() {

        // currentState will switch to ScoringState when gamepad.a is pressed.
        edges.add(new Edge(ScoringState.class, () -> StateMachineControllerTest.gamepad.a));

        return edges;
    }
    public void loop() {
        telemetry.addData("State", "Driving State");
    }
}

class ScoringState implements State {
    ArrayList<Edge> edges = new ArrayList();

    public ArrayList<Edge> getEdges() {
        // currentState will switch to DrivingState when gamepad.b is pressed.
        edges.add(new Edge(DrivingState.class, () -> StateMachineControllerTest.gamepad.b));

        return edges;
    }
    public void loop() {
        telemetry.addData("State", "Driving State");
    }
}