package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        /** DcMotorEx is a child class of DcMotor, so for now we just introduce them as DcMotors.*/
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "intakeMotor"));
        /**Introduced Servos*/
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "geekoWheelServo"));
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "entraptionServo1"));
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "entraptionServo2"));
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

        if (gamepad1.a) {
            /* Setting power of intake motors and servos. */
            robot.<DcMotor>get("intakeMotor").setPower(intakePower);
            robot.<CRServo>get("geekoWheelServo").setPower(intakePower);
            robot.<CRServo>get("entraptionServo1").setPower(intakePower);
            robot.<CRServo>get("entraptionServo2").setPower(intakePower);
            telemetry.addData("Intake", "Running");
        } else {
            robot.<DcMotor>get("intakeMotor").setPower(0);
            robot.<CRServo>get("geekoWheelServo").setPower(0);
            robot.<CRServo>get("entraptionServo1").setPower(0);
            robot.<CRServo>get("entraptionServo2").setPower(0);
            telemetry.addData("Intake", "At rest");
        }
    }
}