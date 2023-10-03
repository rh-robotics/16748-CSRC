package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.HWC;

/**
 * TeleOp OpMode for testing motors.
 */
@TeleOp(name = "BasicMotorTest", group = "Iterative OpMode")
public class BasicMotorTest extends OpMode {
    private final ElapsedTime time = new ElapsedTime();
    HWC robot;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        robot = new HWC(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.leftFront.setPower(1.0);
            robot.leftRear.setPower(1.0);
            robot.rightFront.setPower(1.0);
            robot.rightRear.setPower(1.0);
        } else if (gamepad1.b) {
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
