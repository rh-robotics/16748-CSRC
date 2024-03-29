package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

/**
 * TeleOp OpMode for testing motors.
 * <br/>
 * "BasicMotorTest" is the name of OpMode in Driver Station.
 */
@TeleOp(name = "BasicMotorTest", group = "Tests")
public class BasicMotorTest extends OpMode {
    private final ElapsedTime time = new ElapsedTime();
    private Hardware robot;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        /** DcMotorEx is a child class of DcMotor, so for now we just introduce them as DcMotors. */
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear"));

        robot.<DcMotor>get("leftRear").setDirection(DcMotorSimple.Direction.REVERSE);
        robot.<DcMotor>get("rightRear").setDirection(DcMotorSimple.Direction.REVERSE);
        robot.<DcMotor>get("leftFront").setDirection(DcMotorSimple.Direction.REVERSE);
        robot.<DcMotor>get("rightFront").setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Initialized"); // Updates "Status" key's value.
    }

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void loop() {
        double powerValue = gamepad1.a ? 1.0 : (gamepad1.b ? 0.5 : 0.1);

        robot.<DcMotor>get("leftFront").setPower(powerValue);
        robot.<DcMotor>get("leftRear").setPower(powerValue);
        robot.<DcMotor>get("rightFront").setPower(powerValue);
        robot.<DcMotor>get("rightRear").setPower(powerValue);
    }
}
