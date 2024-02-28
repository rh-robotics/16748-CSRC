package org.firstinspires.ftc.teamcode.auton.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

//import org.firstinspires.ftc.teamcode.auton.vision.TeamPropDetection;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Context;

@Autonomous(name = "Blue Front")
public class blueFront extends OpMode {
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
//    TeamPropDetection teamPropDetection;

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

//        int startPos = 0;
//        telemetry.addData("prop", teamPropDetection.getPropPlacement(hardwareMap, startPos));

        // Left front blue
        context = robotMethods.moveX(robot, context,130);
        robotMethods.rest(0.25);

        context.setDirection(robotMethods.turnTo(robot, context, 200));

        robotMethods.rest(0.25);
        robotMethods.outerIntakeDown(robot);
        robotMethods.flushPixel(robot);

        robotMethods.outerIntakeUp(robot);
        robotMethods.rest(0.25);
//
//        context.setDirection(robotMethods.turnTo(robot, context, 20));
        robotMethods.runWheelsToTarget(robot, -20);
//
        // Middle front blue
//        context = robotMethods.moveX(robot, context,130);
//        robotMethods.rest(0.25);
//        context.setDirection(robotMethods.turnTo(robot, context, 100));
//
//        robotMethods.rest(0.25);
//        robotMethods.outerIntakeDown(robot);
//        robotMethods.flushPixel(robot);
//
//        robotMethods.outerIntakeUp(robot);
//        robotMethods.rest(0.25);
//
//        context.setDirection(robotMethods.turnTo(robot, context, 0));

//        robotMethods.runWheelsToTarget(robot, 150);

        // Right front blue
//        context = robotMethods.moveX(robot, context,130);
//        robotMethods.rest(0.25);
//        context.setDirection(robotMethods.turnTo(robot, context, 0));
//
//        robotMethods.rest(0.25);
//        robotMethods.outerIntakeDown(robot);
//        robotMethods.flushPixel(robot);
//
//        robotMethods.outerIntakeUp(robot);
//        robotMethods.rest(0.25);
//
//        context.setDirection(robotMethods.turnTo(robot, context, 0));

//        robotMethods.runWheelsToTarget(robot, 150);

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
