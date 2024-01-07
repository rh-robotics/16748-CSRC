package org.firstinspires.ftc.teamcode.auton.stateMachine;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Edge;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.State;

import java.util.ArrayList;

class IntakeState implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList<Edge>();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.context.getPixelsInControl() == 2));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Intake");

        intakePixel();
    }

    public void intakePixel() {
        telemetry.addLine("Intaking Pixel");

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpMode.context.setPixelsInControl((byte) (StateMachineOpMode.context.getPixelsInControl() + 1));
    }
}