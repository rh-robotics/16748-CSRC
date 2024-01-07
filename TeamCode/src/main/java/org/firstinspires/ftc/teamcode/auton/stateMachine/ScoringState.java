package org.firstinspires.ftc.teamcode.auton.stateMachine;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Edge;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.State;

import java.util.ArrayList;

class ScoringState implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList<Edge>();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.context.getPixelsInControl() <= 0));

        edges.add(new Edge(TeamPropLineScoring.class, () -> !StateMachineOpMode.context.getTeamPropLineScored()));
        edges.add(new Edge(TeamPropBackdropScoring.class, () -> StateMachineOpMode.context.getTeamPropLineScored() && !StateMachineOpMode.context.getTeamPropBackdropScored()));
        edges.add(new Edge(ScoringOnBackdrop.class, () -> StateMachineOpMode.context.getTeamPropLineScored() && StateMachineOpMode.context.getTeamPropBackdropScored()));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Scoring");

        /* State should never run loop. Should always redirect on an edge before looping. */
        telemetry.addLine("Scoring state failing to redirect.");
    }
}

class TeamPropLineScoring implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList<Edge>();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.context.pixelsInControl <= 0));

        return edges;
    }

    public void loop() {
        telemetry.addLine("Scoring Pixel on Line according to Team Prop.");

        // TODO: Robot should detect which line the pixel is on and score accordingly. */

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpMode.context.setPixelsInControl((byte) (StateMachineOpMode.context.getPixelsInControl() - 1));
    }
}

class TeamPropBackdropScoring implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList<Edge>();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.context.pixelsInControl <= 0));

        return edges;
    }

    public void loop() {
        telemetry.addLine("Scoring Pixel on Backdrop according to Team Prop.");

        // TODO: Robot should score on backdrop accordingly to previously-stored information.

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system.
        StateMachineOpMode.context.setPixelsInControl((byte) (StateMachineOpMode.context.getPixelsInControl() - 1));
    }
}

class ScoringOnBackdrop implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList<Edge>();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.context.getPixelsInControl() <= 0));

        return edges;
    }

    public void loop() {
        telemetry.addLine("Scoring Pixel on Backdrop.");

        // TODO: Robot should analyze board for where to place a pixel and then score on backboard. */
        /* Consider keeping a list (that resets before we switch to scoring state) of "best moves"
         *
         * Could potentially cause problems if allied robot is scoring simultaneously with us to
         * have "predetermined" moves from when we started scoring, but also would most likely be
         * the most reasonable and efficient method of scoring. */

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpMode.context.setPixelsInControl((byte) (StateMachineOpMode.context.getPixelsInControl() - 1));
    }
}