package org.firstinspires.ftc.teamcode.auton.stateMachine;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Edge;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.State;

import java.util.ArrayList;

class IdleState implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList<Edge>();
        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Idle");

        /* TODO: Consider if we actually want it to cut out after 30 sec. Maybe make it a manual
         *  transfer instead to prevent the robot cutting out if something went wrong.
         */

        if (StateMachineOpMode.elapsedTime.time() >= 30) {
            telemetry.addLine("Switch to Driver Control TeleOp.");
        }
    }
}