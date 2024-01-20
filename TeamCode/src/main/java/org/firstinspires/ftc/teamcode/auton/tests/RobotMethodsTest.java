package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Context;

@Autonomous(name = "Robot Methods Test")
public class RobotMethodsTest extends OpMode {
    Hardware robot;
    Context context;
    double motorRunPower = 0.5;
    double tolerance = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);
        context = new Context();

        // Init Servos.
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawLock"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "clawJoint"));

        // Init CR Servos.
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeGeckoWheels"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeTube"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeJoint1"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeJoint2"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeTube"));

        // Init DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear", "setDirection:FORWARD"));

        // Init arm and Viper Slide DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armJoint", "setMode:RUN_USING_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlide", "setMode:RUN_USING_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlide", "setMode:RUN_USING_ENCODER"));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        robot.<DcMotor>get("leftFront").setTargetPosition(robot.<DcMotor>get("leftFront").getCurrentPosition() + 1000);
        robot.<DcMotor>get("leftRear").setTargetPosition(robot.<DcMotor>get("leftRear").getCurrentPosition() + 1000);
        robot.<DcMotor>get("rightFront").setTargetPosition(robot.<DcMotor>get("rightFront").getCurrentPosition() + 1000);
        robot.<DcMotor>get("rightRear").setTargetPosition(robot.<DcMotor>get("rightRear").getCurrentPosition() + 1000);
    }

    @Override
    public void loop() {
        double power = 0.25;
        if (robot.<DcMotor>get("leftFront").getCurrentPosition() < robot.<DcMotor>get("leftFront").getTargetPosition()
                && power > 0) {
            robot.<DcMotor>get("leftFront").setPower(0.25);
        }
        if (robot.<DcMotor>get("leftRear").getCurrentPosition() < robot.<DcMotor>get("leftFront").getTargetPosition()
                && power > 0) {
            robot.<DcMotor>get("leftRear").setPower(0.25);
        }
        if (robot.<DcMotor>get("rightFront").getCurrentPosition() < robot.<DcMotor>get("leftFront").getTargetPosition()
                && power > 0) {
            robot.<DcMotor>get("rightFront").setPower(0.25);
        }
        if (robot.<DcMotor>get("rightRear").getCurrentPosition() < robot.<DcMotor>get("leftFront").getTargetPosition()
                && power > 0) {
            robot.<DcMotor>get("rightRear").setPower(0.25);
        }

        telemetry.addData("leftFront current pos", robot.<DcMotor>get("leftFront").getCurrentPosition());
        telemetry.addData("leftFront target", robot.<DcMotor>get("leftFront").getTargetPosition());
        telemetry.addLine("***");
        telemetry.addData("leftRear current pos", robot.<DcMotor>get("leftFront").getCurrentPosition());
        telemetry.addData("leftRear target", robot.<DcMotor>get("leftFront").getTargetPosition());
        telemetry.addLine("***");
        telemetry.addData("rightFront current pos", robot.<DcMotor>get("leftFront").getCurrentPosition());
        telemetry.addData("rightFront target", robot.<DcMotor>get("leftFront").getTargetPosition());
        telemetry.addLine("***");
        telemetry.addData("rightRear current pos", robot.<DcMotor>get("leftFront").getCurrentPosition());
        telemetry.addData("rightRear target", robot.<DcMotor>get("leftFront").getTargetPosition());
    }
}
