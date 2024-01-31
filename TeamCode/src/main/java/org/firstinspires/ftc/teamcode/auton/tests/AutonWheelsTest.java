package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Context;

@Autonomous(name = "Auton Wheels Test")
public class AutonWheelsTest extends OpMode {
    Hardware robot;
    Context context;
    double motorRunPower = 0.5;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);
        context = new Context();

        // Init DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront",
                "setDirection:REVERSE"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear",
                "setDirection:REVERSE"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear"));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        RobotMethods.setTargetPosition(robot.<DcMotor>get("leftFront"), 300);
        RobotMethods.setTargetPosition(robot.<DcMotor>get("leftRear"), 300);
        RobotMethods.setTargetPosition(robot.<DcMotor>get("rightFront"), 300);
        RobotMethods.setTargetPosition(robot.<DcMotor>get("rightRear"), 300);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.<DcMotor>get("leftFront").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.<DcMotor>get("leftRear").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.<DcMotor>get("rightFront").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.<DcMotor>get("rightRear").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            RobotMethods.updateMotor(telemetry, robot.<DcMotor>get("leftFront"), motorRunPower);
            RobotMethods.updateMotor(telemetry, robot.<DcMotor>get("leftRear"), motorRunPower);
            RobotMethods.updateMotor(telemetry, robot.<DcMotor>get("rightFront"), motorRunPower);
            RobotMethods.updateMotor(telemetry, robot.<DcMotor>get("rightRear"), motorRunPower);
        }
    }
}
