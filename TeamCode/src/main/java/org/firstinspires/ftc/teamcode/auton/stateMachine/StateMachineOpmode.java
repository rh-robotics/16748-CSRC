package org.firstinspires.ftc.teamcode.auton.stateMachine;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Edge;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.State;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.StateMachine;

import java.util.ArrayList;

@Autonomous(name = "Autonomous State Machine")
public class StateMachineOpmode extends OpMode {
    public static Gamepad gamepad;
    public static Telemetry pTelemetry;
    static ElapsedTime elapsedTime;

    StateMachine stateMachine;

    static boolean teamPropLineScored;
    static boolean teamPropBackdropScored;
    static boolean robotParked;
    static boolean inScoringPosition;
    static boolean inIntakePosition;
    static byte pixelsInControl;

    @Override
    public void init() {
        gamepad = gamepad1;
        pTelemetry = telemetry;
        elapsedTime = new ElapsedTime();

        teamPropLineScored = false;
        teamPropBackdropScored = false;
        inScoringPosition = false;
        inIntakePosition = false;
        robotParked = true; /* Changed to false on start(). */

        pixelsInControl = 0;

        stateMachine = new StateMachine();
        stateMachine.addState(new DrivingState());
        stateMachine.addState(new ScoringState());

        /* Initial state. */
        stateMachine.currentState = stateMachine.states.get(DrivingState.class);
    }

    @Override
    public void start() {
        robotParked = false;
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        if (pixelsInControl > 2) {
            // TODO: Embed this error/rule-breaking check into stateMachine.loop().
            /* Figure out if there's any other rules we need to be consistently being careful
            not to break. */
            telemetry.addLine("RULE BROKEN: POSSESSION OF MORE THAN 2 PIXELS.");
        }
        /* Passing telemetry around. */
        stateMachine.loop(telemetry);
    }
}

/**
 * "Default" state that most things point back to. This state handles everything to do with
 * getting from point A to point B. Control flow of Auton plan handled within this state.
 */
class DrivingState implements State {
    ArrayList<Edge> edges = new ArrayList();

    public ArrayList<Edge> getEdges() {
        /* Although theoretically mutually exclusive within the control flow, a message should
         * occur with Telemetry (no RuntimeException to avoid robot crashing) to limit silent
         * logic errors if multiple edges are true.
         */
        edges.add(new Edge(IntakeState.class, () -> StateMachineOpmode.inIntakePosition));
        edges.add(new Edge(ScoringState.class, () -> StateMachineOpmode.inScoringPosition));
        edges.add(new Edge(IdleState.class, () -> StateMachineOpmode.robotParked));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Driving State");

        /* Edges are always checked before loop is run. */
        StateMachineOpmode.inScoringPosition = false;
        StateMachineOpmode.inIntakePosition = false;
        StateMachineOpmode.robotParked = false;

        /* Main autonomous state machine control flow here. */

        /* For the sake of clarity within the code, we are grouping these together in an
        if-statement as the initial "Team Prop" scoring opportunities. */
        if (!StateMachineOpmode.teamPropLineScored || !StateMachineOpmode.teamPropBackdropScored) {
            if (!StateMachineOpmode.teamPropLineScored) {
                driveToTeamPropLine();
            } else {
                driveToBackdrop();
            }
        } else if (StateMachineOpmode.elapsedTime.time() > 25) {
            driveToAutonPark();
        } else if (StateMachineOpmode.pixelsInControl < 2) {
            /* Sets state to scoring. */
            driveToIntakePixels();
        } else if (StateMachineOpmode.pixelsInControl == 2) {
            /* Sets state to scoring. */
            driveToBackdrop();
        } else {
            /* Not throwing RuntimeException to avoid crashing robot. This code should
            theoretically never be run, but if it is, it does not necessarily indicate a serious
            error, just something that we should be aware of to avoid silent logic errors.  */
            telemetry.addLine("Current status unaccounted for in Drive State.");
            telemetry.addData("time", StateMachineOpmode.elapsedTime);
            telemetry.addData("teamPropLineScored", StateMachineOpmode.teamPropLineScored);
            telemetry.addData("TeamPropBackdropScored", StateMachineOpmode.teamPropBackdropScored);
            telemetry.addData("controlledPixels", StateMachineOpmode.teamPropLineScored);
        }
    }

    public void driveToBackdrop() {
        telemetry.addLine("Driving to Backdrop.");

        // TODO: Make this method actually make the robot drive to the backdrop...
        StateMachineOpmode.inScoringPosition = true;
    }

    public void driveToTeamPropLine() {
        telemetry.addLine("Driving to Team Prop Line.");

        // TODO: Make the robot drive to the Team Prop line
        /* "Scoring" State generalized to handle scoring on the Team Prop Line and on the backdrop. */
        StateMachineOpmode.inScoringPosition = true;
    }

    public void driveToIntakePixels() {
        telemetry.addLine("Driving to pixels.");

        // TODO: Make the robot figure out which pixels to go to, and then go to them
        StateMachineOpmode.inIntakePosition = true;
    }

    public void driveToAutonPark() {
        telemetry.addLine("Parking.");

        // TODO: Make robot Auton Park
        StateMachineOpmode.robotParked = true;
    }

}

class ScoringState implements State {
    ArrayList<Edge> edges = new ArrayList();

    public ArrayList<Edge> getEdges() {
        edges.add(new Edge(IdleState.class, () -> StateMachineOpmode.robotParked));
        edges.add(new Edge(DrivingState.class, () -> StateMachineOpmode.pixelsInControl == 0));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Driving State");

        if (!StateMachineOpmode.teamPropLineScored) {
            teamPropLineScore();
            StateMachineOpmode.teamPropLineScored = true;
        } else if (!StateMachineOpmode.teamPropBackdropScored) {
            teamPropBackdropScore();
            StateMachineOpmode.teamPropBackdropScored = true;
        } else {
            scoreOnBackdrop();
        }
    }

    public void teamPropLineScore() {
        telemetry.addLine("Scoring Pixel on Line according to Team Prop.");

        // TODO: Robot should detect which line the pixel is on and score accordingly. */

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpmode.pixelsInControl -= 1;
    }

    public void teamPropBackdropScore() {
        telemetry.addLine("Scoring Pixel on Backdrop according to Team Prop.");

        // TODO: Robot should score on backdrop accordingly to previously-stored information

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpmode.pixelsInControl -= 1;
    }

    public void scoreOnBackdrop() {
        telemetry.addLine("Scoring Pixel on Backdrop.");

        // TODO: Robot should analyze board for where to place a pixel and then score on backboard. */
        /* Consider keeping a list (that resets before we switch to scoring state) of "best moves"
         *
         * Could potentially cause problems if allied robot is scoring simultaneously with us to
         * have "predetermined" moves from when we started scoring, but also would most likely be
         * the most reasonable and efficient method of scoring. */

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpmode.pixelsInControl -= 1;
    }
}

class IntakeState implements State {
    ArrayList<Edge> edges = new ArrayList();

    public ArrayList<Edge> getEdges() {
        edges.add(new Edge(IdleState.class, () -> StateMachineOpmode.robotParked));
        edges.add(new Edge(DrivingState.class, () -> StateMachineOpmode.pixelsInControl == 2));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Driving State");

        intakePixel();
    }

    public void intakePixel() {
        telemetry.addLine("Intaking Pixel");

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpmode.pixelsInControl += 1;
    }
}

class IdleState implements State {
    ArrayList<Edge> edges = new ArrayList();

    public ArrayList<Edge> getEdges() {
        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Idle State");
    }
}