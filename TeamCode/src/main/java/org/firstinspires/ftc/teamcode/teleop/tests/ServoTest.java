package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Tests")
public class ServoTest extends OpMode {

    Servo testServo;
    public void init() {
        telemetry.addLine("Initializing");

        testServo = hardwareMap.servo.get("testServo");

        telemetry.addLine("Initialized");
    }

    public void loop() {
        if (gamepad1.a)  {
            testServo.setPosition(testServo.getPosition() - 0.25);
        } else if (gamepad1.b) {
            testServo.setPosition(testServo.getPosition() + 0.25);
        }

        telemetry.addData("Servo Position", testServo.getPosition());

    }
}
