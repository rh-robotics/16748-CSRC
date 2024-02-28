package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
            testServo.setPosition(testServo.getPosition() - 0.05);
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            while(elapsedTime.seconds() < 0.5) {}
        } else if (gamepad1.b) {
            testServo.setPosition(testServo.getPosition() + 0.05);
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();
            while(elapsedTime.seconds() < 0.5) {}
        }

        telemetry.addData("Servo Position", testServo.getPosition());

    }
}
