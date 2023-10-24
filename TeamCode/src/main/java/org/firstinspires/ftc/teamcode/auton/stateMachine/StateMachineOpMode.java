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
public class StateMachineOpMode extends OpMode {
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
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        /* Although theoretically mutually exclusive within the control flow, a message should
         * occur with Telemetry (no RuntimeException to avoid robot crashing) to limit silent
         * logic errors if multiple edges are true.
         */
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.robotParked));
        edges.add(new Edge(IntakeState.class, () -> StateMachineOpMode.inIntakePosition));
        edges.add(new Edge(ScoringState.class, () -> StateMachineOpMode.inScoringPosition));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Driving");

        /* Edges are always checked before loop is run. */
        StateMachineOpMode.inScoringPosition = false;
        StateMachineOpMode.inIntakePosition = false;
        StateMachineOpMode.robotParked = false;

        /* Main autonomous state machine control flow here. */

        /* For the sake of clarity within the code, we are grouping these together in an
        if-statement as the initial "Team Prop" scoring opportunities. */
        if (!StateMachineOpMode.teamPropLineScored || !StateMachineOpMode.teamPropBackdropScored) {
            if (!StateMachineOpMode.teamPropLineScored) {
                driveToTeamPropLine();
            } else {
                driveToBackdrop();
            }
        } else if (StateMachineOpMode.elapsedTime.time() > 25) {
            driveToAutonPark();
        } else if (StateMachineOpMode.pixelsInControl < 2) {
            /* Sets state to scoring. */
            driveToIntakePixels();
        } else if (StateMachineOpMode.pixelsInControl == 2) {
            /* Sets state to scoring. */
            driveToBackdrop();
        } else if (StateMachineOpMode.pixelsInControl > 2) {
            // TODO: Handle this situation.
            /*  Theoretically, should never happen, but something weird with hardware
             *  might make this possible. */

            telemetry.addLine("RULE BROKEN: POSSESSION OF MORE THAN 2 PIXELS.");
        } else {
            /* Keep this else despite the last else if currently always be true
             * to catch any potential errors if we complicate the control flow. */

            /* Not throwing RuntimeException to avoid crashing robot. This code should
            theoretically never be run, but if it is, it does not necessarily indicate a serious
            error, just something that we should be aware of to avoid silent logic errors.  */
            telemetry.addLine("Current status unaccounted for in Drive State.");
            telemetry.addData("time", StateMachineOpMode.elapsedTime);
            telemetry.addData("teamPropLineScored", StateMachineOpMode.teamPropLineScored);
            telemetry.addData("TeamPropBackdropScored", StateMachineOpMode.teamPropBackdropScored);
            telemetry.addData("controlledPixels", StateMachineOpMode.teamPropLineScored);
        }
    }

    public void driveToBackdrop() {
        telemetry.addLine("Driving to Backdrop.");

        // TODO: Make this method actually make the robot drive to the backdrop...
        StateMachineOpMode.inScoringPosition = true;
    }

    public void driveToTeamPropLine() {
        telemetry.addLine("Driving to Team Prop Line.");

        // TODO: Make the robot drive to the Team Prop line
        /* "Scoring" State generalized to handle scoring on the Team Prop Line and on the backdrop. */
        StateMachineOpMode.inScoringPosition = true;
    }

    public void driveToIntakePixels() {
        telemetry.addLine("Driving to pixels.");

        // TODO: Make the robot figure out which pixels to go to, and then go to them
        StateMachineOpMode.inIntakePosition = true;
    }

    public void driveToAutonPark() {
        telemetry.addLine("Parking.");

        // TODO: Make robot Auton Park
        StateMachineOpMode.robotParked = true;
    }

}

class ScoringState implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.pixelsInControl <= 0));

        edges.add(new Edge(teamPropLineScoring.class, () -> !StateMachineOpMode.teamPropLineScored));
        edges.add(new Edge(teamPropBackdropScoring.class, () -> StateMachineOpMode.teamPropLineScored
                && !StateMachineOpMode.teamPropBackdropScored));
        edges.add(new Edge(scoringOnBackdrop.class, () -> StateMachineOpMode.teamPropLineScored
                && StateMachineOpMode.teamPropBackdropScored));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Scoring");

        /* State should never run loop. Should always redirect on an edge before looping. */
        telemetry.addLine("Scoring state failing to redirect.");
    }
}

class teamPropLineScoring implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.pixelsInControl <= 0));

        return edges;
    }

    public void loop() {
        telemetry.addLine("Scoring Pixel on Line according to Team Prop.");

        // TODO: Robot should detect which line the pixel is on and score accordingly. */

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpMode.pixelsInControl -= 1;
    }
}

class teamPropBackdropScoring implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.pixelsInControl <= 0));

        return edges;
    }

    public void loop() {
        telemetry.addLine("Scoring Pixel on Backdrop according to Team Prop.");

        // TODO: Robot should score on backdrop accordingly to previously-stored information.

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpMode.pixelsInControl -= 1;
    }
}

class scoringOnBackdrop implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.pixelsInControl <= 0));

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
        StateMachineOpMode.pixelsInControl -= 1;
    }
}

class IntakeState implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.pixelsInControl == 2));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Intake");

        intakePixel();
    }

    public void intakePixel() {
        telemetry.addLine("Intaking Pixel");

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpMode.pixelsInControl += 1;
    }
}

class IdleState implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList();
        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Idle");

        if (StateMachineOpMode.elapsedTime.time() >= 30) {
            telemetry.addLine("Switch to Driver Control TeleOp.");
            /* TODO: Make this redirect this to TeleOp. */
        }
    }
}