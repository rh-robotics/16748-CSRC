package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Context;

@Autonomous(name = "Robot Methods Test")
public class RobotMethodsTest extends OpMode {
    Hardware robot;
    Context context;
    double motorRunPower = 0.5;
    double tolerance = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);
        context = new Context();

        // Init DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "testMotor"));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        RobotMethods.setTargetPosition(robot.<DcMotor>get("testMotor"), 300);
    }

    @Override
    public void loop() {
        RobotMethods.updateMotor(telemetry, robot.<DcMotor>get("testMotor"), motorRunPower);

        if (gamepad1.a) {
            robot.<DcMotor>get("testMotor").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
