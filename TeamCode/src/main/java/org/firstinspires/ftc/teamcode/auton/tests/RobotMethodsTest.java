package org.firstinspires.ftc.teamcode.auton.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Context;

@Autonomous(name = "Robot Methods Test")
public class RobotMethodsTest extends OpMode {
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
    double targetPos = 90;

    double leftAdjustment = 1;
    double rightAdjustment = 1;

    @Override
    public void init() {
        robotMethods = new RobotMethods();

        robot = robotMethods.init(telemetry, hardwareMap);
    }

    @Override
    public void start() {
        robotMethods.start(robot);
    }

    @Override
    public void loop() {
        gamepadUpdate();

        if (currentGamepad1.left_trigger > 0.5 && previousActionGamepad1.left_trigger < 0.5) {
            leftAdjustment += 0.05;
        } else if (currentGamepad1.right_trigger > 0.5 && previousActionGamepad1.right_trigger < 0.5) {
            leftAdjustment -= 0.05;
        }

        if (currentGamepad1.left_bumper && previousActionGamepad1.left_bumper) {
            rightAdjustment += 0.05;
        } else if (currentGamepad1.right_bumper && previousActionGamepad1.right_bumper) {
            rightAdjustment -= 0.05;
        }

        telemetry.addData("Left Adjustment", leftAdjustment);
        telemetry.addData("Right Adjustment", rightAdjustment);

        if (currentGamepad1.y) {
            robotMethods.runWheelsToTarget(robot, targetPos);
        } else if (currentGamepad1.a) {
            robotMethods.runWheelsToTarget(robot, -targetPos);
        } else if (currentGamepad1.x) {
            robotMethods.runWheelsToTarget(robot, strafeAdjustment * targetPos, -targetPos * strafeAdjustment,
                    -targetPos * strafeAdjustment, targetPos * strafeAdjustment);
        } else if (currentGamepad1.b) {
            robotMethods.runWheelsToTarget(robot, -targetPos, targetPos, targetPos, -targetPos);
        }

        if (currentGamepad1.dpad_up) {
            robotMethods.scoringPosition(robot);
        } else if (currentGamepad1.dpad_down) {
            robotMethods.intakePosition(robot);
        }

        if (currentGamepad1.dpad_left) {
            robotMethods.turn(robot, -90 * leftAdjustment);
        } else if (currentGamepad1.dpad_right) {
            robotMethods.turn(robot, 90 * rightAdjustment);
        }
        

        if (currentGamepad1.a && currentGamepad1.back && currentGamepad1.b) {
            telemetry.addLine("Ending Opmode...");
            robotMethods.endResetMotors(robot);
            requestOpModeStop();
        }
    }

    public void gamepadUpdate() {
        previousActionGamepad1.copy(currentGamepad1);
        previousActionGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }
}