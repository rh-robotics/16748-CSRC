package org.firstinspires.ftc.teamcode.auton.stateMachine;

import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous State Machine")
public class StateMachineOpMode extends OpMode {
    protected static ElapsedTime elapsedTime;

    StateMachine stateMachine;
    protected static Context context;

    double[][] startingLocation = new double[][] {new double[] {0, 0, 0}, new double[] {0, 0, 0},
            new double[] {0, 0, 0}, new double[] {0, 0, 0}};
    int startingLocationIndex = 0;

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();

        context = new Context();
        context.setRobotParked(true); /* Changed to false on start(). */

        context.setPixelsInControl((byte) 0);

        stateMachine = new StateMachine();

        stateMachine.addState(new DrivingState());
        stateMachine.addState(new ScoringState());
        stateMachine.addState(new IdleState());
        stateMachine.addState(new IntakeState());

        /* Initialize state. */
        stateMachine.currentState = stateMachine.states.get(DrivingState.class);

        if (gamepad1.a) {
            startingLocationIndex = (startingLocationIndex + 1) % startingLocation.length;
            telemetry.addData("Starting Location", startingLocation[startingLocationIndex]);
        }
    }


    @Override
    public void start() {
        context.setLocation(startingLocation[startingLocationIndex][0],
                startingLocation[startingLocationIndex][1], startingLocation[startingLocationIndex][2]);
        context.setRobotParked(false);
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        if (context.getPixelsInControl() > 2) {
            // TODO: Embed this error/rule-breaking check into stateMachine.loop().
            /* TODO: Figure out if there's any other rules we need to be consistently being careful
                not to break. */
            telemetry.addLine("RULE BROKEN: POSSESSION OF MORE THAN 2 PIXELS.");
        } else if (context.getPixelsInControl() < 0) {
            telemetry.addLine("LESS THAN 0 PIXELS LOGGED IN POSSESSION.");
            context.setPixelsInControl((byte) 0);
            telemetry.addLine("PIXELS IN POSSESSION SET TO ZERO.");
        }

        /* Passing telemetry around. */
        stateMachine.loop(telemetry);
    }
}