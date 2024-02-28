package org.firstinspires.ftc.teamcode.auton.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Context;

@Autonomous(name = "AA Blue Front Auton Test")
public class autonTest extends OpMode {
    Gamepad previousActionGamepad1  = new Gamepad();
    Gamepad previousActionGamepad2  = new Gamepad();
    Gamepad currentGamepad1  = new Gamepad();
    Gamepad currentGamepad2  = new Gamepad();

    Hardware robot;
    Context context;
    double motorRunPower = 0.5;

    private boolean modeToggle = false;
    RobotMethods robotMethods;

    final double strafeAdjustment = 6/4.5;
    double targetPos = 100;

    double adjustment1 = 1;
    double adjustment2 = 1;
    AprilTagAlignment aprilTagAlignment;
    @Override
    public void init() {
        robotMethods = new RobotMethods();
        aprilTagAlignment = new AprilTagAlignment();

        // Blue Back, Blue Front, Right Back, Right Front
        context = new Context(1);

        robot = robotMethods.init(telemetry, hardwareMap);
    }

    @Override
    public void start() {
        robotMethods.start(robot);

        // Left front blue
        robotMethods.outerIntakeUp(robot);
        context = robotMethods.moveX(robot, context,130);
        robotMethods.rest(0.25);

        robotMethods.turnTo(robot, context, 0);

        robotMethods.rest(0.25);
        context = robotMethods.moveY(robot, context,85);
        robotMethods.outerIntakeDown(robot);
        robotMethods.flushPixel(robot);
        robotMethods.outerIntakeUp(robot);
        context = robotMethods.moveY(robot, context,85);
        // Middle front blue

//        // Right front blue
//        robotMethods.outerIntakeUp(robot);
//        context = robotMethods.moveX(robot, context,130);
//        robotMethods.rest(3);
//
//        robotMethods.turnTo(robot, context, 0);
//        robotMethods.rest(3);
//
//        context = robotMethods.moveX(robot, context,80);
//        robotMethods.rest(3);
//
//        robotMethods.outerIntakeDown(robot);
//        robotMethods.flushPixel(robot);
//        robotMethods.outerIntakeUp(robot);
//
//        context = robotMethods.moveY(robot, context,120);
//        robotMethods.rest(3);

        // Left front red
        robotMethods.outerIntakeUp(robot);
        context = robotMethods.moveX(robot, context,130);
        robotMethods.rest(3);

        robotMethods.turnTo(robot, context, 0);
        robotMethods.rest(3);

        context = robotMethods.moveX(robot, context,80);
        robotMethods.rest(3);

        robotMethods.outerIntakeDown(robot);
        robotMethods.flushPixel(robot);
        robotMethods.outerIntakeUp(robot);

        context = robotMethods.moveY(robot, context,120);
        robotMethods.rest(3);

//        context = robotMethods.moveY(robot, context,85);
//        // Middle front red

        // Right front red


        //context = robotMethods.moveY(robot, context, 120);
       // robotMethods.scoringPosition(robot);
}

    @Override
    public void loop() {
        gamepadUpdate();
        telemetry.addData("Direction", context.getDirection());
    }

    @Override
    public void stop(){

    }

    public void gamepadUpdate() {
        previousActionGamepad1.copy(currentGamepad1);
        previousActionGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }
}
