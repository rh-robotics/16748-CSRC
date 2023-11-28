package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

@Autonomous(name = "Scoring Algorithm", group = "auton")
public class ScoringAlgorithm extends OpMode {

    private Hardware robot;
    // TODO: Get values for variables.
    boolean pixelsInClaw = false;
    double armStartPos = 0.0;
    double armTargetPos = 0.0;
    double viperSlideStartPos = 0.0;
    double viperSlideTargetPos = 0.0;
    double jointStartPos = 0.0;
    double jointTargetPos = 0.0;

    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);
        // Introducing servos.
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawServo"));
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "jointServo"));
        // Introducing motors and encoders.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armMotor"));
        robot.<DcMotor>get("armMotor").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("armMotor").setDirection(DcMotor.Direction.FORWARD);
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlideMotor"));
        robot.<DcMotor>get("rightViperSlideMotor").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("leftViperSlideMotor").setDirection(DcMotorSimple.Direction.FORWARD);
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlideMotor"));
        robot.<DcMotor>get("leftViperSlideMotor").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("rightViperSlideMotor").setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");

    }

    private void scoring() {
        telemetry.addData("Scoring Status", "Started");

        setViperSlidePos();

        setArmPos();

        setJointPos();

        while (pixelsInClaw) {
            robot.<Servo>get("clawServo").setPosition(robot.<Servo>get("clawServo").getPosition() - 0.125);
        }

        resetPos();

        telemetry.addData("Scoring Status", "Scored");
    }

    public void setArmPos() {
        telemetry.addData("Scoring Status", "Setting Arm Pos");
        // Setting ArmPos based on Target.
        while (robot.<DcMotor>get("armMotor").getCurrentPosition() != armTargetPos) {
            robot.<DcMotor>get("armMotor").setPower(0.6);
            telemetry.addData("armMotor Pos", robot.<DcMotor>get("armMotor").getCurrentPosition());
        }
        robot.<DcMotor>get("armMotor").setPower(0);

        telemetry.addData("Scoring Status", "Arm Pos Set");
    }

    public void setViperSlidePos() {
        telemetry.addData("Scoring Status", "Setting VS Pos");
        // Setting ViperSlidePoses based on Targets.
        while (robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition() != viperSlideTargetPos ) {

            if (robot.<DcMotor>get("rightViperSlideMotor").getCurrentPosition() != viperSlideTargetPos) {
                robot.<DcMotor>get("rightViperSlideMotor").setPower(0.6);
            } else if (robot.<DcMotor>get("rightViperSlideMotor").getCurrentPosition() == viperSlideTargetPos) {
                robot.<DcMotor>get("rightViperSlideMotor").setPower(0);
            } else if (robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition() != viperSlideTargetPos) {
                robot.<DcMotor>get("leftViperSlideMotor").setPower(0.6);
            } else if (robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition() == viperSlideTargetPos) {
                robot.<DcMotor>get("leftViperSlideMotor").setPower(0);
            }
            telemetry.addData("VS right Pos", robot.<DcMotor>get("rightViperSlideMotor").getCurrentPosition());
            telemetry.addData("VS left Pos", robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition());
        }

        telemetry.addData("Scoring Status", "VS Pos Set");
    }

    public void setJointPos() {
        telemetry.addData("Scoring Status", "Setting Joint Pos");
        // Setting JointPos based on Target.
        robot.<Servo>get("jointServo").setPosition(jointTargetPos);

        telemetry.addData("Scoring Status", "Joint Pos Set");
    }

    public void resetPos() {
        telemetry.addData("Scoring Status1` ", "Resetting Pos");

        robot.<Servo>get("jointServo").setPosition(jointStartPos);
        robot.<DcMotor>get("armMotor").setDirection(DcMotor.Direction.REVERSE);
        robot.<DcMotor>get("leftViperSlideMotor").setDirection(DcMotorSimple.Direction.REVERSE);
        robot.<DcMotor>get("rightViperSlideMotor").setDirection(DcMotorSimple.Direction.REVERSE);

        while (robot.<DcMotor>get("armMotor").getCurrentPosition() != armStartPos) {

            if (robot.<DcMotor>get("rightViperSlideMotor").getCurrentPosition() != viperSlideStartPos) {
                robot.<DcMotor>get("rightViperSlideMotor").setPower(0.6);
            } else if (robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition() == viperSlideStartPos) {
                robot.<DcMotor>get("leftViperSlideMotor").setPower(0);
            } else if (robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition() != viperSlideStartPos) {
                robot.<DcMotor>get("leftViperSlideMotor").setPower(0.6);
            } else if (robot.<DcMotor>get("leftViperSlideMotor").getCurrentPosition() == viperSlideStartPos) {
                robot.<DcMotor>get("leftViperSlideMotor").setPower(0);
            }

        }
        robot.<DcMotor>get("armMotor").setPower(0);

        telemetry.addData("Scoring Status", "Pos Reset");
    }

    public void loop() {}
}
