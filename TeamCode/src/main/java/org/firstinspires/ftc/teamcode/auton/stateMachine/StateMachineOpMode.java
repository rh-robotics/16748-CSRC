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
    public static Context context;
    @Override
    public void init() {
        gamepad = gamepad1;
        pTelemetry = telemetry;
        elapsedTime = new ElapsedTime();

        context = new Context();
        context.robotParked = true; /* Changed to false on start(). */

        context.pixelsInControl = 0;

        stateMachine = new StateMachine();
        stateMachine.addState(new DrivingState());
        stateMachine.addState(new ScoringState());
        // TODO:

        /* Initial state. */
        stateMachine.currentState = stateMachine.states.get(DrivingState.class);
    }

    @Override
    public void start() {
        context.robotParked = false;
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        if (context.pixelsInControl > 2) {
            // TODO: Embed this error/rule-breaking check into stateMachine.loop().
            /* TODO: Figure out if there's any other rules we need to be consistently being careful
                not to break. */
            telemetry.addLine("RULE BROKEN: POSSESSION OF MORE THAN 2 PIXELS.");
        }

        /* Passing telemetry around. */
        stateMachine.loop(telemetry);
    }
}

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

        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.context.robotParked));
        edges.add(new Edge(IntakeState.class, () -> StateMachineOpMode.context.inIntakePosition));
        edges.add(new Edge(ScoringState.class, () -> StateMachineOpMode.context.inScoringPosition));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Driving");

        /* Edges are always checked before loop is run. */
        StateMachineOpMode.context.inScoringPosition = false;
        StateMachineOpMode.context.inIntakePosition = false;
        StateMachineOpMode.context.robotParked = false;

        /* Main autonomous state machine control flow here. */

        /* For the sake of clarity within the code, we are grouping these together in an
        if-statement as the initial "Team Prop" scoring opportunities. */
        if (!StateMachineOpMode.context.teamPropLineScored || !StateMachineOpMode.context.teamPropBackdropScored) {
            if (!StateMachineOpMode.context.teamPropLineScored) {
                driveToTeamPropLine();
            } else {
                driveToBackdrop();
            }
        } else if (StateMachineOpMode.elapsedTime.time() > 25) {
            driveToAutonPark();
        } else if (StateMachineOpMode.context.pixelsInControl < 2) {
            /* Sets state to scoring. */
            driveToIntakePixels();
        } else if (StateMachineOpMode.context.pixelsInControl == 2) {
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
            telemetry.addData("TeamPropLineScored", StateMachineOpMode.context.teamPropLineScored);
            telemetry.addData("TeamPropBackdropScored", StateMachineOpMode.context.teamPropBackdropScored);
            telemetry.addData("controlledPixels", StateMachineOpMode.context.teamPropLineScored);
        }
    }

    public void driveToBackdrop() {
        telemetry.addLine("Driving to Backdrop.");

        // TODO: Make this method actually make the robot drive to the backdrop.
        StateMachineOpMode.context.inScoringPosition = true;
    }

    public void driveToTeamPropLine() {
        telemetry.addLine("Driving to Team Prop Line.");

        // TODO: Make the robot drive to the Team Prop line.
        /* "Scoring" State generalized to handle scoring on the Team Prop Line and on the backdrop. */
        StateMachineOpMode.context.inScoringPosition = true;
    }

    public void driveToIntakePixels() {
        telemetry.addLine("Driving to pixels.");

        // TODO: Make the robot figure out which pixels to go to, and then go to them.
        StateMachineOpMode.context.inIntakePosition = true;
    }

    public void driveToAutonPark() {
        telemetry.addLine("Parking.");

        // TODO: Make robot Auton Park.
        StateMachineOpMode.context.robotParked = true;
    }

}

class ScoringState implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.context.pixelsInControl <= 0));

        edges.add(new Edge(TeamPropLineScoring.class, () -> !StateMachineOpMode.context.teamPropLineScored));
        edges.add(new Edge(TeamPropBackdropScoring.class, () -> StateMachineOpMode.context.teamPropLineScored && !StateMachineOpMode.context.teamPropBackdropScored));
        edges.add(new Edge(ScoringOnBackdrop.class, () -> StateMachineOpMode.context.teamPropLineScored && StateMachineOpMode.context.teamPropBackdropScored));

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
        edges = new ArrayList();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.context.pixelsInControl <= 0));

        return edges;
    }

    public void loop() {
        telemetry.addLine("Scoring Pixel on Line according to Team Prop.");

        // TODO: Robot should detect which line the pixel is on and score accordingly. */

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpMode.context.pixelsInControl -= 1;
    }
}

class TeamPropBackdropScoring implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.context.pixelsInControl <= 0));

        return edges;
    }

    public void loop() {
        telemetry.addLine("Scoring Pixel on Backdrop according to Team Prop.");

        // TODO: Robot should score on backdrop accordingly to previously-stored information.

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system.
        StateMachineOpMode.context.pixelsInControl -= 1;
    }
}

class ScoringOnBackdrop implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.context.pixelsInControl <= 0));

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
        StateMachineOpMode.context.pixelsInControl -= 1;
    }
}

class IntakeState implements State {
    ArrayList<Edge> edges;

    public ArrayList<Edge> getEdges() {
        edges = new ArrayList<Edge>();

        // Idle state redirects to TeleOp after elapsedTime >= 30.
        edges.add(new Edge(IdleState.class, () -> StateMachineOpMode.elapsedTime.time() >= 30));

        edges.add(new Edge(DrivingState.class, () -> StateMachineOpMode.context.pixelsInControl == 2));

        return edges;
    }

    public void loop() {
        telemetry.addData("State", "Intake");

        intakePixel();
    }

    public void intakePixel() {
        telemetry.addLine("Intaking Pixel");

        // TODO: Consider changing how we keep track of pixel possession to a sensor-managed system. */
        StateMachineOpMode.context.pixelsInControl += 1;
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

        /* TODO: Consider if we actually want it to cut out after 30 sec. Maybe make it a manual
         *  transfer instead to prevent the robot cutting out if something went wrong.
         */

        if (StateMachineOpMode.elapsedTime.time() >= 30) {
            telemetry.addLine("Switch to Driver Control TeleOp.");
        }
    }
}

class Context {
    boolean teamPropLineScored = false;
    boolean teamPropBackdropScored = false;
    boolean robotParked = false;
    boolean inScoringPosition = false;
    boolean inIntakePosition = false;
    byte pixelsInControl = 0;
}