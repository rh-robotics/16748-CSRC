package org.firstinspires.ftc.teamcode.auton.stateMachine;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.*;

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
        // TODO: add more states

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