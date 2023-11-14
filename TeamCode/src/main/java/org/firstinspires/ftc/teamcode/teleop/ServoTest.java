package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends OpMode {
    Servo servoTest;
    public void init() {
        telemetry.addLine("Initializing");
        servoTest = hardwareMap.servo.get("clawServo");
        telemetry.addLine("Initialized");
    }

    public void loop() {
        if (gamepad1.a) {
            servoTest.setPosition(0);
        } else if (gamepad1.b) {
            servoTest.setPosition(0.25);
        }

        telemetry.addData("Servo Port Number", servoTest.getPortNumber());
        telemetry.addData("Servo Position", servoTest.getPosition());
    }
}

