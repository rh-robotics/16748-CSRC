package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;

// TODO: Outer intake

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "OpMode")
public class TeleOp extends OpMode {
    private Hardware robot;
    double leftFPower;
    double rightFPower;
    double leftBPower;
    double rightBPower;
    double drive;
    double turn;
    double strafe;
    double intakePower = 0.5;
    private final double armStartPos = 0.0;
    private final double armTargetPos = 0.0;
    private final double armTolerance = 2.5;
    private final double viperSlideStartPos = 0.0;
    private final double viperSlideTargetPos = 0.0;
    private final double jointStartPos = 0.0;
    private final double jointTargetPos = 0.0;
    boolean scoringATM;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new Hardware(hardwareMap, telemetry);

        // Init Servos.
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawLock"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "clawJoint"));

        // Init CR Servos.
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeGeckoWheels"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeTube"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeJoint1"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeJoint2"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeTube1"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeTube2"));
      
        // Init DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront", "setDirection:FORWARD"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightBack", "setDirection:FORWARD"));

        // Init arm and Viper Slide DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armJoint", "setMode(DcMotor.RunMode.RUN_USING_ENCODER)"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlide", "setMode(DcMotor.RunMode.RUN_USING_ENCODER)"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlide", "setMode(DcMotor.RunMode.RUN_USING_ENCODER)"));

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Controls:\n" +
                "         *** \n" +
                "         * Gamepad 1\n" +
                "         * Left Stick y = Drive\n" +
                "         * Left Stick x = Strafe\n" +
                "         * Right Stick x = Turn\n" +
                "         * \n" +
                "         *** \n" +
                "         * Gamepad 2\n" +
                "         * Right Bumper = Claw Joint +\n" +
                "         * Right Trigger = Claw Joint -\n" +
                "         * X = Automatic Scoring\n" +
                "         * Y = VS, Arm and Claw reset (Automatic Scoring Reset)\n" +
                "         * A = Activate Intake (Hold)\n" +
                "         * Left Bumper = Claw Lock + (Manual)\n" +
                "         * Left Trigger = Claw Lock - (Manual)\n" +
                "         * D Pad Up = VS Up (Manual)\n" +
                "         * D Pad Down = VS Down (Manual)");
    }

    @Override
    public void loop() {

        // Values for drive.
        drive = gamepad1.left_stick_y * 0.8;
        turn = -gamepad1.right_stick_x * 0.6;
        strafe = gamepad1.left_stick_x * 0.8;

        // Set power to values calculated above.
        robot.<DcMotor>get("leftFront").setPower(leftFPower);
        robot.<DcMotor>get("leftRear").setPower(leftBPower);
        robot.<DcMotor>get("rightFront").setPower(rightFPower);
        robot.<DcMotor>get("rightRear").setPower(rightBPower);

        telemetry.addData("leftFPower: ", leftFPower);
        telemetry.addData("leftBPower: ", leftBPower);
        telemetry.addData("rightFPower: ", rightFPower);
        telemetry.addData("rightBPower: ", rightBPower);

        // Calling scoring() via left bumper and resetting via left trigger
        if (gamepad2.x) {
            scoring();
        } else if (gamepad2.y && !scoringATM) {
            resetPos();
            telemetry.addData("Scoring Pos", "Reset");
        }

        // Move claw via x and y.
        if (gamepad2.left_bumper) {
            robot.<Servo>get("clawLock").setPosition(robot.<Servo>get("clawLock").getPosition() + -0.125);
        } else if (gamepad2.left_trigger > 0.5) {
            robot.<Servo>get("clawLock").setPosition(robot.<Servo>get("clawLock").getPosition() + 0.125);
        }

        // Moving VS manually via dpad up and down.
        if (gamepad2.dpad_up) {
            robot.<DcMotor>get("leftViperSlide").setPower(0.5);
            robot.<DcMotor>get("rightViperSlide").setPower(0.5);
        } else if (gamepad2.dpad_down) {
            robot.<DcMotor>get("leftViperSlide").setPower(-0.5);
            robot.<DcMotor>get("rightViperSlide").setPower(-0.5);
        } else {
            robot.<DcMotor>get("leftViperSlide").setPower(0);
            robot.<DcMotor>get("rightViperSlide").setPower(0);
        }

        // Controls clawJoint using right bumper and right trigger.
        if (gamepad2.right_bumper) {
            robot.<CRServo>get("clawJoint").setPower(0.15);
        } else if (gamepad2.right_trigger > 0.5) {
            robot.<CRServo>get("clawJoint").setPower(-0.15);
        } else {
            robot.<CRServo>get("clawJoint").setPower(0);
        }

        // Activating Intake via gamepad a.
        if (gamepad1.a) {
            robot.<CRServo>get("intakeTube").setPower(intakePower);
            robot.<CRServo>get("intakeGeckoWheels").setPower(intakePower);
            robot.<CRServo>get("outerIntakeTube1").setPower(-0.5);
            robot.<CRServo>get("outerIntakeTube2").setPower(-0.5);
            telemetry.addData("Intake", "Running");
        } else {
            robot.<CRServo>get("intakeTube").setPower(0);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0);
            robot.<CRServo>get("outerIntakeTube1").setPower(0);
            robot.<CRServo>get("outerIntakeTube2").setPower(0);
            telemetry.addData("Intake", "Stopped");
        }

        if (gamepad1.right_bumper) {
            robot.<CRServo>get("outerIntakeJoint1").setPower(-0.25);
            robot.<CRServo>get("outerIntakeJoint2").setPower(0.25);
        } else if (gamepad1.right_trigger > 0.5) {
            robot.<CRServo>get("outerIntakeJoint1").setPower(0.25);
            robot.<CRServo>get("outerIntakeJoint2").setPower(-0.25);
        }
    }

    // Scoring method, all together like a bowl of chili.
    public void scoring() {
        telemetry.addData("Scoring Status", "Started");

        scoringATM = true;

        setViperSlidePos();
        setArmPos();
        setJointPos();

        scoringATM = false;

        telemetry.addData("Scoring Status", "Ended");
    }

    // Setting VS POS.
    public void setViperSlidePos() {
        // If this is not true, the rest of the code will not function.
        assert viperSlideTargetPos >= viperSlideStartPos;

        telemetry.addData("Scoring Status", "Setting VS Pos");

        // Setting ViperSlidePoses based on Targets.
        while (robot.<DcMotor>get("leftViperSlide").getCurrentPosition() < viperSlideTargetPos || robot.<DcMotor>get("rightViperSlide").getCurrentPosition() < viperSlideTargetPos) {
            runViperSlidesToPosition(viperSlideTargetPos, 0.4);
            viperSlideTelemetryUpdate();
        }

        telemetry.addData("Scoring Status", "VS Pos Set");
    }

    // Setting Arm Pos.
    public void setArmPos() {
        // If this is not true, the rest of the code will not function.
        assert armTargetPos >= armStartPos;

        telemetry.addData("Scoring Status", "Setting Arm Pos");

        // Setting ArmPos based on Target.
        while (robot.<DcMotor>get("armJoint").getCurrentPosition() < armTargetPos) {
            robot.<DcMotor>get("armJoint").setPower(0.6);
            telemetry.addData("armJoint Pos", robot.<DcMotor>get("armJoint").getCurrentPosition());
        }

        robot.<DcMotor>get("armJoint").setPower(0);
        telemetry.addData("Scoring Status", "Arm Pos Set");
    }

    //Setting Joint Pos.
    public void setJointPos() {
        telemetry.addData("Scoring Status", "Setting Joint Pos");
        robot.<Servo>get("clawJoint").setPosition(jointTargetPos); // Setting JointPos based on Target.
        telemetry.addData("Scoring Status", "Joint Pos Set");
    }

    // Resetting Scoring POS.
    public void resetPos() {
        // If this is not true, the rest of the code will not function.
        assert viperSlideTargetPos >= viperSlideStartPos;

        telemetry.addData("Scoring Status", "Resetting Pos");
        robot.<Servo>get("clawJoint").setPosition(jointStartPos);

        while (robot.<DcMotor>get("leftViperSlide").getCurrentPosition() > viperSlideStartPos || robot.<DcMotor>get("rightViperSlide").getCurrentPosition() > viperSlideStartPos) {
            runViperSlidesToPosition(viperSlideStartPos, -0.4);
            viperSlideTelemetryUpdate();
        }

        robot.<DcMotor>get("leftViperSlide").setPower(0);
        robot.<DcMotor>get("rightViperSlide").setPower(0);
        telemetry.addData("Scoring Status", "Pos Reset");
    }

    // VS Telemetry Updates.
    public void viperSlideTelemetryUpdate() {
        telemetry.addData("VS Left Pos", robot.<DcMotor>get("leftViperSlide").getCurrentPosition());
        telemetry.addData("VS Left Power", robot.<DcMotor>get("leftViperSlide").getPower());

        telemetry.addData("VS Right Pos", robot.<DcMotor>get("rightViperSlide").getCurrentPosition());
        telemetry.addData("VS Right Power", robot.<DcMotor>get("rightViperSlide").getPower());
    }

    // VS run Code to Target.
    public void runViperSlidesToPosition(double position, double power) {
        robot.<DcMotor>get("leftViperSlide").setPower(0);
        robot.<DcMotor>get("rightViperSlide").setPower(0);

        if (robot.<DcMotor>get("leftViperSlide").getCurrentPosition() > position) {
            robot.<DcMotor>get("leftViperSlide").setPower(power);
            telemetry.addLine("Running VS Left");
        }

        if (robot.<DcMotor>get("rightViperSlide").getCurrentPosition() > position) {
            robot.<DcMotor>get("rightViperSlide").setPower(power);
        }
    }

    // Strafe Drive using sticks on Gamepad 1
    public void driving() {
        // Calculate drive power.
        if (drive != 0 || turn != 0) {
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
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
    }
}
