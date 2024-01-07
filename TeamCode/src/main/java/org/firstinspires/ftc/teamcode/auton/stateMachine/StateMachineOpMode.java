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

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();

        context = new Context();
        context.robotParked = true; /* Changed to false on start(). */

        context.pixelsInControl = 0;

        stateMachine = new StateMachine();

        stateMachine.addState(new DrivingState());
        stateMachine.addState(new ScoringState());
        stateMachine.addState(new IdleState());
        stateMachine.addState(new IntakeState());

        /* Initialize state. */
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
        } else if (context.pixelsInControl < 0) {
            telemetry.addLine("LESS THAN 0 PIXELS LOGGED IN POSSESSION.");
            context.setPixelsInControl((byte) 0);
            telemetry.addLine("PIXELS IN POSSESSION SET TO ZERO.");
        }

        /* Passing telemetry around. */
        stateMachine.loop(telemetry);
    }
}