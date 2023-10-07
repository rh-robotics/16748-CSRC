package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * TeleOp OpMode for testing motors.
 */
// "BasicMotorTest" is the name of OpMode in Driver Station
@TeleOp(name = "BasicMotorTest", group = "Iterative OpMode")

public class BasicMotorTest extends OpMode {
    private final ElapsedTime time = new ElapsedTime();
    HWC robot; // Declares roboBt as an instance of a hardware class.

    // Runs once when "INIT" button is pressed.
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        // Telemetry.addData stores data in a hashmap-like data set.
        robot = new HWC(hardwareMap, telemetry); // assigns robot value
        telemetry.addData("Status", "Initialized"); // Updates "Status" key's value.
    }

    // Runs repetitively between "INIT" and "START" button presses,
    // FRC SDK default version does nothing.
    @Override
    public void init_loop() {
    }

    // Runs once after "START" button pressed.
    @Override
    public void start() {
        time.reset();
    }

    // Runs forever after "START" button pressed
    // (unless given an exception or "STOP" button pressed).
    @Override
    public void loop() {
        if (gamepad1.a) { // press button 'A'
            robot.leftFront.setPower(1.0); // set motor power
            robot.leftRear.setPower(1.0);
            robot.rightFront.setPower(1.0);
            robot.rightRear.setPower(1.0);
        } else if (gamepad1.b) { // press button 'B'
            robot.leftFront.setPower(0.5);
            robot.leftRear.setPower(0.5);
            robot.rightFront.setPower(0.5);
            robot.rightRear.setPower(0.5);
        } else {
            robot.leftFront.setPower(0.1);
            robot.leftRear.setPower(0.1);
            robot.rightFront.setPower(0.1);
            robot.rightRear.setPower(0.1);
        }
    }
}
