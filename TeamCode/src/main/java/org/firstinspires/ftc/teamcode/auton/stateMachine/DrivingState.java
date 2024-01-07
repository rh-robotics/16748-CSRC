package org.firstinspires.ftc.teamcode.auton.stateMachine;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Edge;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.State;

import java.util.ArrayList;

/**
 * The Driving State is the "Default" state that most things point back to. This state handles
 * everything to do with getting from point A to point B. All control flow of our Autonomous OpMode
 * is handled within this state.
 */
class DrivingState implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList<Edge>();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.context.getRobotParked()));
        edges.add(new Edge(IntakeState.class, () -> StateMachineOpMode.context.getInIntakePosition()));
        edges.add(new Edge(ScoringState.class, () -> StateMachineOpMode.context.getInScoringPosition()));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Driving");

        /* Edges are always checked before loop is run. */
        StateMachineOpMode.context.setInScoringPosition(false);
        StateMachineOpMode.context.setInIntakePosition(false);
        StateMachineOpMode.context.setRobotParked(false);

        /* Main autonomous state machine control flow here. */

        /* For the sake of clarity within the code, we are grouping these together in an
        if-statement as the initial "Team Prop" scoring opportunities. */
        if (!StateMachineOpMode.context.getTeamPropLineScored() || !StateMachineOpMode.context.getTeamPropBackdropScored()) {
            if (!StateMachineOpMode.context.getTeamPropLineScored()) {
                driveToTeamPropLine();
            } else {
                driveToBackdrop();
            }
        } else if (StateMachineOpMode.elapsedTime.time() > 25) {
            driveToAutonPark();
        } else if (StateMachineOpMode.context.getPixelsInControl() < 2) {
            /* Sets state to scoring. */
            driveToIntakePixels();
        } else if (StateMachineOpMode.context.getPixelsInControl() == 2) {
            /* Sets state to scoring. */
            driveToBackdrop();
        } else {
            /* Not throwing RuntimeException to avoid crashing robot. This code should
            theoretically never be run, but if it is, it does not necessarily indicate a serious
            error, just something that we should be aware of to avoid silent logic errors.  */
            telemetry.addLine();
            telemetry.addLine("***");
            telemetry.addLine("RULE BROKEN: POSSESSION OF MORE THAN 2 PIXELS.");
            telemetry.addLine("***");
            telemetry.addLine();
            telemetry.addData("Time", StateMachineOpMode.elapsedTime);
            telemetry.addData("TeamPropLineScored", StateMachineOpMode.context.getTeamPropLineScored());
            telemetry.addData("TeamPropBackdropScored", StateMachineOpMode.context.getTeamPropBackdropScored());
            telemetry.addData("controlledPixels", StateMachineOpMode.context.getTeamPropLineScored());
        }
    }

    public void driveToBackdrop() {
        telemetry.addLine("Driving to Backdrop.");

        // TODO: Make this method actually make the robot drive to the backdrop.
        StateMachineOpMode.context.setInScoringPosition(true);
    }

    public void driveToTeamPropLine() {
        telemetry.addLine("Driving to Team Prop Line.");

        // TODO: Make the robot drive to the Team Prop line.
        /* "Scoring" State generalized to handle scoring on the Team Prop Line and on the backdrop. */
        StateMachineOpMode.context.setInScoringPosition(true);
    }

    public void driveToIntakePixels() {
        telemetry.addLine("Driving to pixels.");

        // TODO: Make the robot figure out which pixels to go to, and then go to them.
        StateMachineOpMode.context.setInIntakePosition(true);
    }

    public void driveToAutonPark() {
        telemetry.addLine("Parking.");

        // TODO: Make robot Auton Park.
        StateMachineOpMode.context.setRobotParked(true);
    }

}