package org.firstinspires.ftc.teamcode.auton.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@Autonomous(name="Auton Driving Test")
public class AutonDrivingTest extends OpMode {
    private Hardware robot;
    private double CPR = 537.7;
    private double targetPos = 3;
    private double motorTolerance = 50; // ticks
    private double inchesPerRevolution = 38/3;

    @Override
    public void init() {
        telemetry.addLine("Initializing");

        robot = new Hardware(hardwareMap, telemetry);
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear", "setDirection:REVERSE"));

        telemetry.addLine("Initialized");
    }

    /**
     * Reset motors to initial position here before we run anything else.
     * This step is necessary because we are using relative encoders, which give encoder information
     * based on where the motor is started/reset.
     **/
    @Override
    public void start() {
        resetEncoders();

        robot.<DcMotor>get("leftRear").setTargetPosition(0);
        robot.<DcMotor>get("leftFront").setTargetPosition(0);
        robot.<DcMotor>get("rightRear").setTargetPosition(0);
        robot.<DcMotor>get("rightFront").setTargetPosition(0);
    }

    public void resetEncoders() {
        robot.<DcMotor>get("leftRear").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("leftRear").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.<DcMotor>get("leftFront").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("leftFront").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.<DcMotor>get("rightRear").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("rightRear").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.<DcMotor>get("rightFront").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("rightFront").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.b) {
            robot.<DcMotor>get("leftRear").setPower(0);
            robot.<DcMotor>get("leftFront").setPower(0);
            robot.<DcMotor>get("rightRear").setPower(0);
            robot.<DcMotor>get("rightFront").setPower(0);

            resetEncoders();
            robot.<DcMotor>get("leftRear").setTargetPosition(0);
            robot.<DcMotor>get("leftFront").setTargetPosition(0);
            robot.<DcMotor>get("rightRear").setTargetPosition(0);
            robot.<DcMotor>get("rightFront").setTargetPosition(0);
        } else if (gamepad1.a) {
            runToTarget(targetPos);

            robot.<DcMotor>get("leftRear").setPower(0);
            robot.<DcMotor>get("leftFront").setPower(0);
            robot.<DcMotor>get("rightRear").setPower(0);
            robot.<DcMotor>get("rightFront").setPower(0);

            resetEncoders();
            robot.<DcMotor>get("leftRear").setTargetPosition(0);
            robot.<DcMotor>get("leftFront").setTargetPosition(0);
            robot.<DcMotor>get("rightRear").setTargetPosition(0);
            robot.<DcMotor>get("rightFront").setTargetPosition(0);
        } else if (gamepad1.x) {
            runToTarget(targetPos, -targetPos, -targetPos, targetPos);

            robot.<DcMotor>get("leftRear").setPower(0);
            robot.<DcMotor>get("leftFront").setPower(0);
            robot.<DcMotor>get("rightRear").setPower(0);
            robot.<DcMotor>get("rightFront").setPower(0);

            resetEncoders();
            robot.<DcMotor>get("leftRear").setTargetPosition(0);
            robot.<DcMotor>get("leftFront").setTargetPosition(0);
            robot.<DcMotor>get("rightRear").setTargetPosition(0);
            robot.<DcMotor>get("rightFront").setTargetPosition(0);
        } else if (gamepad1.y) {
            runToTarget(-(targetPos));

            robot.<DcMotor>get("leftRear").setPower(0);
            robot.<DcMotor>get("leftFront").setPower(0);
            robot.<DcMotor>get("rightRear").setPower(0);
            robot.<DcMotor>get("rightFront").setPower(0);

            resetEncoders();
            robot.<DcMotor>get("leftRear").setTargetPosition(0);
            robot.<DcMotor>get("leftFront").setTargetPosition(0);
            robot.<DcMotor>get("rightRear").setTargetPosition(0);
            robot.<DcMotor>get("rightFront").setTargetPosition(0);
        }

        printTelemetry();

        if (gamepad1.a && gamepad1.b) {
            stopOpMode();
        }
    }

    public void runToTarget(double target) {
        robot.<DcMotor>get("leftRear").setTargetPosition(inchesToTicks(target));
        robot.<DcMotor>get("leftFront").setTargetPosition(inchesToTicks(target));
        robot.<DcMotor>get("rightRear").setTargetPosition(inchesToTicks(target));
        robot.<DcMotor>get("rightFront").setTargetPosition(-inchesToTicks(target));

        telemetry.addLine("1");
        runMotorsToTargets();
    }

    public void runMotorsToTargets() {
        while (Math.abs(robot.<DcMotor>get("leftRear").getCurrentPosition() - robot.<DcMotor>get("leftRear").getTargetPosition()) > motorTolerance
                || Math.abs(robot.<DcMotor>get("leftFront").getCurrentPosition() - robot.<DcMotor>get("leftFront").getTargetPosition()) > motorTolerance
                || Math.abs(robot.<DcMotor>get("rightRear").getCurrentPosition() - robot.<DcMotor>get("rightRear").getTargetPosition()) > motorTolerance) {
            setPower(robot.<DcMotor>get("leftRear"));
            setPower(robot.<DcMotor>get("leftFront"));
            setPower(robot.<DcMotor>get("rightRear"));
            setPower(robot.<DcMotor>get("rightFront"));

            printTelemetry();
        }
        robot.<DcMotor>get("rightFront").setPower(0);
        robot.<DcMotor>get("rightRear").setPower(0);
        robot.<DcMotor>get("leftFront").setPower(0);
        robot.<DcMotor>get("leftRear").setPower(0);
    }

    public void setPower(DcMotor motor) {
        if (motor.getTargetPosition() < motor.getCurrentPosition()) {
            motor.setPower(-0.5);
        } else if (motor.getTargetPosition() > motor.getCurrentPosition()) {
            motor.setPower(0.5);
        } else {
            motor.setPower(0);
        }
    }

    public void runToTarget(double lrTarget, double lfTarget, double rrTarget, double rfTarget) {
        robot.<DcMotor>get("leftRear").setTargetPosition(inchesToTicks(lrTarget));
        robot.<DcMotor>get("leftFront").setTargetPosition(inchesToTicks(lfTarget));
        robot.<DcMotor>get("rightRear").setTargetPosition(inchesToTicks(rrTarget));
        robot.<DcMotor>get("rightFront").setTargetPosition(-inchesToTicks(rfTarget));

        runMotorsToTargets();
    }

    public void printTelemetry() {
        telemetry.addData("leftRear Target", robot.<DcMotor>get("leftRear").getTargetPosition());
        telemetry.addData("leftRear Current", robot.<DcMotor>get("leftRear").getCurrentPosition());
        telemetry.addData("leftRear Power", robot.<DcMotor>get("leftRear").getPower());

        telemetry.addLine();
        telemetry.addData("leftFront Target", robot.<DcMotor>get("leftFront").getTargetPosition());
        telemetry.addData("leftFront Current", robot.<DcMotor>get("leftFront").getCurrentPosition());
        telemetry.addData("leftFront Power", robot.<DcMotor>get("leftFront").getPower());

        telemetry.addLine();
        telemetry.addData("rightRear Target", robot.<DcMotor>get("rightRear").getTargetPosition());
        telemetry.addData("rightRear Current", robot.<DcMotor>get("rightRear").getCurrentPosition());
        telemetry.addData("rightRear Power", robot.<DcMotor>get("rightRear").getPower());

        telemetry.addLine();
        telemetry.addData("rightFront Target", robot.<DcMotor>get("rightFront").getTargetPosition());
        telemetry.addData("rightFront Current", robot.<DcMotor>get("rightFront").getCurrentPosition());
        telemetry.addData("rightFront Power", robot.<DcMotor>get("rightFront").getPower());
    }

    /** Reset motors to initial position here before ending the program. This helps for testing
     * when we want to run auton multiple times in a row, helps with redundancy for making sure
     * our encoders are accurate, and also is just convenient to have reset at the end of auton period. **/
    public void stopOpMode() {
        requestOpModeStop();
    }

    public int inchesToTicks(double revolutions) {
        return (int) Math.round((revolutions * CPR) /inchesPerRevolution);
    }
}
