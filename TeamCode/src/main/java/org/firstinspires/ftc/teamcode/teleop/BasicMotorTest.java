package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Hardware.*;

// "BasicMotorTest" is the name of OpMode in Driver Station.
/**
 * TeleOp OpMode for testing motors.
 */
@TeleOp(name = "BasicMotorTest", group = "Iterative OpMode")
public class BasicMotorTest extends OpMode {
    private final ElapsedTime time = new ElapsedTime();
    // Declares robot as an instance of a hardware class.
    private Hardware robot;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private DcMotor intakeMotor;
    // Runs once when "INIT" button is pressed.
    @Override
    public void init() {
        // Telemetry.addData stores data in a hashmap-like data set.
        telemetry.addData("Status", "Initializing");

        robot = new Hardware();
        robot.introduce(new HardwareElement(hardwareMap, DcMotorEx.class, "leftFront");
        robot.introduce(new HardwareElement(hardwareMap, DcMotorEx.class, "leftRear", "setDirection:REVERSE"");
        robot.introduce(new HardwareElement(hardwareMap, DcMotorEx.class, "rightFront");
        robot.introduce(new HardwareElement(hardwareMap, DcMotorEx.class, "rightRear");
        robot.introduce(new HardwareElement(hardwareMap, DcMotor.class, "intakeMotor");

        leftFront = (DcMotorEx) robot.HardwareElements.get("leftFront");
        leftRear = (DcMotorEx) robot.HardwareElements.get("leftRear");
        rightFront = (DcMotorEx) robot.HardwareElements.get("rightFront");
        rightRear = (DcMotorEx) robot.HardwareElements.get("rightRear");
        intakeMotor = (DcMotor) robot.HardwareElements.get("intakeMotor");

        telemetry.addData("Status", "Initialized"); // Updates "Status" key's value.
    }

    /* Method init_loop() runs repetitively between "INIT" and "START" button presses,
     * FTC SDK default version does nothing. */

    // Runs once after "START" button pressed.
    @Override
    public void start() {
        time.reset();
    }

    /* Runs forever after "START" button pressed
     * (unless given an exception or "STOP" button pressed). */
    @Override
    public void loop() {
        if (gamepad1.a) {
            leftFront.setPower(1.0); // Sets motor power.
            leftRear.setPower(1.0);
            rightFront.setPower(1.0);
            rightRear.setPower(1.0);
        } else if (gamepad1.b) {
            leftFront.setPower(0.5);
            leftRear.setPower(0.5);
            rightFront.setPower(0.5);
            rightRear.setPower(0.5);
        } else {
            leftFront.setPower(0.1);
            leftRear.setPower(0.1);
            rightFront.setPower(0.1);
            rightRear.setPower(0.1);
        }
    }
}
