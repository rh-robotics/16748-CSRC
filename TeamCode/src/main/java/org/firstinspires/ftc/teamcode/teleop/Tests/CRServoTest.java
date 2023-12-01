package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@TeleOp(name = "CRServo Test", group = "Tests")
public class CRServoTest extends OpMode {

    private Hardware robot;

    public void init() {
        telemetry.addLine("Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "testCRServo"));

        telemetry.addLine("Initialized");
    }

    public void loop() {
        if (gamepad1.a) {
            robot.<CRServo>get("testCRServo").setPower(0.5);
        } else {
            robot.<CRServo>get("testCRServo").setPower(0);
        }
        telemetry.addData("CRServo", "Running");
    }
}

