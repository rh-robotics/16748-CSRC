package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
@TeleOp(name = "Intake Running Code", group = "Iterative OpMode")
public class IntakeRunningCode extends OpMode{
    private final ElapsedTime time = new ElapsedTime();
    private Hardware robot;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        /**Introduced Servos*/
       // robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "geekoWheelServo", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "tubeServo", "setDirection:FORWARD"));
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
        double intakePower = 0.5;

        if (gamepad1.dpad_left) {
            /* Setting power of intake motors and servos. */
            //robot.<CRServo>get("geekoWheelServo").setPower(intakePower);
            robot.<CRServo>get("tubeServo").setPower(intakePower);
            telemetry.addData("Intake", "Running");
        } else if (gamepad1.dpad_right) {

        }
    }
}