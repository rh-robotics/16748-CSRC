package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

/**
 * TeleOp OpMode for simply driving with strafing wheels
 */
@TeleOp(name = "StrafeDriveBasic", group = "Iterative OpMode")
public class StrafeDrive extends OpMode {
    private final ElapsedTime time = new ElapsedTime();
    private Hardware robot;

    // init() Runs ONCE after the driver hits initialize.
    @Override
    public void init() {
        // Tell the driver the Op is initializing
        telemetry.addData("Status", "Initializing");

        // Do all init stuff.
        robot = new Hardware(hardwareMap, telemetry);

        /** DcMotorEx is a child class of DcMotor, so for now we just introduce them as DcMotors.*/
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightBack"));

        telemetry.addData("Status", "Initialized");
    }

    // init_loop() - Runs continuously until the driver hits play.
    @Override
    public void init_loop() {
    }

    // Start() - Runs ONCE when the driver presses play.
    @Override
    public void start() {
        time.reset();
    }

    // loop() - Runs continuously while the OpMode is active.
    @Override
    public void loop() {
        double leftFPower;
        double rightFPower;
        double leftBPower;
        double rightBPower;
        double drive = -gamepad1.left_stick_y * 0.8;
        double turn = gamepad1.left_stick_x * 0.6;
        double strafe = -gamepad1.right_stick_x * 0.8;

        // Calculate drive power.
        if (drive != 0 || turn != 0) {
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
            // Strafing.
            leftFPower = -strafe;
            rightFPower = strafe;
            leftBPower = strafe;
            rightBPower = -strafe;
        } else {
            leftFPower = 0;
            rightFPower = 0;
            leftBPower = 0;
            rightBPower = 0;
        }

        // Set power to values calculated above.
        robot.<DcMotor>get("leftFront").setPower(leftFPower);
        robot.<DcMotor>get("leftRear").setPower(leftBPower);
        robot.<DcMotor>get("rightFront").setPower(rightFPower);
        robot.<DcMotor>get("rightRear").setPower(rightBPower);

        telemetry.addData("leftFPower", leftFPower);
        telemetry.addData("leftBPower", leftBPower);
        telemetry.addData("rightFPower", rightFPower);
        telemetry.addData("rightBPower", rightBPower);
    }
}