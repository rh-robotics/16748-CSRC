package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.PID;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "OpMode")
public class TeleOp extends OpMode {
    private Hardware robot;
    private RobotMethods rMethods = new RobotMethods();

    // PID
    PID armPID = new PID(robot.<DcMotorEx>get("armJoint"), 537.7, 0.1, 0.1, 0.1, 0.1);
    PID leftVSPID = new PID(robot.<DcMotorEx>get("leftViperSlide"), 537.7,0.1,0.1,0.1, 0.1);
    PID rightVSPID = new PID(robot.<DcMotorEx>get("rightViperSlide"), 537.7, 0.1, 0.1, 0.1, 0.1);

    // Drive Variables
    double leftFPower;
    double rightFPower;
    double leftBPower;
    double rightBPower;
    double drive;
    double turn;
    double strafe;

    // Intake Variables
    double intakePower = 0.5;
    boolean intakeToggle = false;

    // Scoring Algorithm Variables
    private final double armStartPos = 0.0;
    private final double armTargetPos = 0.0;
    private final double armTolerance = 2.5;
    private final double viperSlideStartPos = 0.0;
    private final double viperSlideTargetPos = 0.0;
    private final double jointStartPos = 0.0;
    private final double jointTargetPos = 0.0;
    boolean scoringATM;
    byte lockPositionIndex = 0;
    byte slidePositionIndex = 0;
    byte armPositionIndex = 0;
    double PROPORTIONAL = 0.1;
    double INTEGRAL = 0.1;
    double DERIVATIVE = 0.1;

    double[] lockPositions = new double[]{0.0, 0.4, 0.8};
    double[] slidePositions = new double[]{0.0, 1.0};
    double[] armPositions = new double[]{0.0, 1.0};

    ElapsedTime clawPositionChange = new ElapsedTime();
    ElapsedTime viperSlidePositionChange = new ElapsedTime();
    ElapsedTime armPositionChange = new ElapsedTime();

    // Gamepads
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousActionGamepad1 = new Gamepad();
    Gamepad previousActionGamepad2 = new Gamepad();

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
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeJoint"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeTube"));

        // Init DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront", "setDirection:FORWARD,setMode:RUN_USING_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear", "setDirection:FORWARD,setMode:RUN_USING_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront", "setDirection:FORWARD,setMode:RUN_USING_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear", "setDirection:REVERSE,setMode:RUN_USING_ENCODER"));

        // Init arm and Viper Slide DcMotors.
        robot.introduce(new HardwareElement<>(DcMotorEx.class, hardwareMap, "armJoint", "setMode:RUN_USING_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotorEx.class, hardwareMap, "leftViperSlide", "setMode:RUN_USING_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotorEx.class, hardwareMap, "rightViperSlide", "setMode:RUN_USING_ENCODER"));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Ready For Play!");
    }

    public void start() {
        clawPositionChange.reset();
        viperSlidePositionChange.reset();
        armPositionChange.reset();
    }

    @Override
    public void loop() {
        telemetryUpdate();
        gamepadUpdate();

        driving();
        outerIntakeJoints();
        clawLock();
        viperSlides();
        clawJoint();
        intake();
        arm();
        // automaticScoring();
    }

    // Strafe Drive using sticks on Gamepad 1.
    public void driving() {
        double drive = currentGamepad1.left_stick_y * 0.8;
        double turn = -currentGamepad1.left_stick_x * 0.6;
        double strafe = currentGamepad1.right_stick_x * 0.8;

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

    public void clawLock() {
        // Move claw via x and y.
        if (clawPositionChange.time() > 0.25) {
            if (gamepad1.left_bumper) {
                lockPositionIndex++;
                if (lockPositionIndex > 2) {
                    lockPositionIndex = 2;
                }
                robot.<Servo>get("clawLock").setPosition(lockPositions[lockPositionIndex]);
                clawPositionChange.reset();
            } else if (gamepad1.left_trigger > 0.5) {
                lockPositionIndex--;
                if (lockPositionIndex < 0) {
                    lockPositionIndex = 0;
                }
                robot.<Servo>get("clawLock").setPosition(lockPositions[lockPositionIndex]);
                clawPositionChange.reset();
            }
        }
        telemetry.addData("claw lock position", robot.<Servo>get("clawLock").getPosition());
        telemetry.addData("claw lock index", lockPositionIndex);
    }

    public void viperSlides() {
        // Moving VS to pre-set positions.
        if (viperSlidePositionChange.time() > 0.5) {
            if (false) { // gamepad1.b
                if (lockPositionIndex == 1) {
                    lockPositionIndex = 0;
                } else {
                    lockPositionIndex = 1;
                }

                robot.<DcMotor>get("leftViperSlide").setTargetPosition((int) slidePositions[slidePositionIndex]);
                robot.<DcMotor>get("rightViperSlide").setTargetPosition((int) slidePositions[slidePositionIndex]);
                viperSlidePositionChange.reset();
            }
        }

        // Moving VS manually via dpad up and down.
        if (gamepad1.dpad_up) {
            robot.<DcMotor>get("leftViperSlide").setPower(0.25);
            robot.<DcMotor>get("rightViperSlide").setPower(-0.25);
        } else if (gamepad1.dpad_down) {
            robot.<DcMotor>get("leftViperSlide").setPower(-0.25);
            robot.<DcMotor>get("rightViperSlide").setPower(0.25);
        } else {
            robot.<DcMotor>get("leftViperSlide").setPower(0);
            robot.<DcMotor>get("rightViperSlide").setPower(0);
        }
    }

    public void clawJoint() {
        // Controls clawJoint using right bumper and right trigger.
        if (gamepad1.right_bumper) {
            robot.<CRServo>get("clawJoint").setPower(0.15);
        } else if (gamepad1.right_trigger > 0.5) {
            robot.<CRServo>get("clawJoint").setPower(-0.15);
        } else {
            robot.<CRServo>get("clawJoint").setPower(0);
        }
    }

    public void intake() {
        // Activating Intake via gamepad a.
        if (currentGamepad1.a && !previousActionGamepad1.a) {
            intakeToggle = !intakeToggle;
        }

        if (intakeToggle) {
            robot.<CRServo>get("intakeTube").setPower(intakePower);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0.2);
            robot.<CRServo>get("outerIntakeTube").setPower(0.5);
            telemetry.addData("Intake", "Running");
        } else if (gamepad1.x) {
            robot.<CRServo>get("intakeTube").setPower(-intakePower);
            robot.<CRServo>get("intakeGeckoWheels").setPower(-0.2);
            robot.<CRServo>get("outerIntakeTube").setPower(-0.5);
            telemetry.addData("Intake", "Running");
        } else {
            robot.<CRServo>get("intakeTube").setPower(0);
            robot.<CRServo>get("intakeGeckoWheels").setPower(0);
            robot.<CRServo>get("outerIntakeTube").setPower(0);
            telemetry.addData("Intake", "Stopped");
        }

        if (gamepad1.y) {
            robot.<CRServo>get("intakeGeckoWheels").setPower(-0.2);
        }
    }

    public void arm() {
        boolean armToggle = false;

        if (currentGamepad1.dpad_left && !previousActionGamepad1.dpad_left) {
            armToggle = !armToggle;
        }

        if (armToggle && rMethods.motorCloseEnoughPosition(robot.<DcMotorEx>get("armJoint"), armTolerance, armStartPos)) {
            armPID.setTarget(armTargetPos);
            armPID.moveUsingPID();
        } else if (!armToggle && rMethods.motorCloseEnoughPosition(robot.<DcMotorEx>get("armJoint"), armTolerance, armTargetPos)) {
            armPID.setTarget(armStartPos);
            armPID.moveUsingPID();
        }

        // Moving arm manually.
        if (currentGamepad2.dpad_left) {
            armPID.incrementTarget(0.5);
            armPID.moveUsingPID();
        } else if (currentGamepad2.dpad_right) {
            armPID.incrementTarget(-0.5);
            armPID.moveUsingPID();
        }
    }

    public void outerIntakeJoints() {
        if (gamepad1.dpad_up) {
           robot.<CRServo>get("outerIntakeJoint").setPower(-0.25);
        } else if (gamepad1.dpad_down) {
           robot.<CRServo>get("outerIntakeJoint").setPower(0.25);
       }
    }

    // VS Telemetry Updates.
    public void viperSlideTelemetryUpdate() {
        telemetry.addData("VS Left Pos", robot.<DcMotor>get("leftViperSlide").getCurrentPosition());
        telemetry.addData("VS Left Power", robot.<DcMotor>get("leftViperSlide").getPower());

        telemetry.addData("VS Right Pos", robot.<DcMotor>get("rightViperSlide").getCurrentPosition());
        telemetry.addData("VS Right Power", robot.<DcMotor>get("rightViperSlide").getPower());
    }

    public void automaticScoring() {
        // Calling scoring() via left bumper and resetting via left trigger
        if (gamepad1.dpad_left) {
            scoring();
        } else if (gamepad1.y && !scoringATM) {
            resetPos();
            telemetry.addData("Scoring Pos", "Reset");
        }
    }

    // Scoring method, all together like a bowl of chili.
    public void scoring() {
        telemetry.addData("Scoring Status", "Started");
        scoringATM = true;

        setViperSlidePos(viperSlideTargetPos);
        setArmPos(armTargetPos);
        setJointPos(jointTargetPos);

        scoringATM = false;
        telemetry.addData("Scoring Status", "Ended");
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

    // Setting VS POS.
    public void setViperSlidePos(double targetPos) {
        boolean increasing = targetPos >= viperSlideStartPos;
        telemetry.addData("Scoring Status", "Setting VS Pos");

        if (increasing) {
            while (increasing && (robot.<DcMotor>get("leftViperSlide").getCurrentPosition() < targetPos
                    || robot.<DcMotor>get("rightViperSlide").getCurrentPosition() < viperSlideTargetPos )) {
                runViperSlidesToPosition(viperSlideTargetPos, 0.4);
                viperSlideTelemetryUpdate();
            }
        } else {
            while (robot.<DcMotor>get("leftViperSlide").getCurrentPosition() > viperSlideTargetPos
                    || robot.<DcMotor>get("rightViperSlide").getCurrentPosition() > viperSlideTargetPos ) {
                runViperSlidesToPosition(viperSlideTargetPos, -0.4);
                viperSlideTelemetryUpdate();
            }
        }

        // telemetry.addData("Scoring Status", "VS Pos Set");
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

    // Setting Arm Pos.
    public void setArmPos(double targetPos) {
        // If this is not true, the rest of the code will not function.
        assert armTargetPos >= armStartPos;

        telemetry.addData("Scoring Status", "Setting Arm Pos");

        armPID.setTarget(targetPos);
        armPID.moveUsingPID();

        telemetry.addData("Scoring Status", "Arm Pos Set");
    }

    // Setting Joint Pos.
    public void setJointPos(double targetPos) {
        telemetry.addData("Scoring Status", "Setting Joint Pos");
        robot.<Servo>get("clawJoint").setPosition(targetPos); // Setting JointPos based on Target.
        telemetry.addData("Scoring Status", "Joint Pos Set");
    }

    public void telemetryUpdate() {
        telemetry.addLine("Controls:\n" +
                "         *** \n" +
                "         * Gamepad 1\n" +
                "         * Left Stick y = Drive\n" +
                "         * Left Stick x = Strafe\n" +
                "         * Right Stick x = Turn\n" +
                "         * Left Trigger = Claw Lock -\n" +
                "         * Left Bumper = Claw Lock +\n" +
                "         * Right Trigger = Claw Joint -\n" +
                "         * Right Bumper = Claw Joint +\n" +
                "         * A = Run Intake (Toggle)\n" +
                "         * B = VS +\n" +
                "         * X = VS -\n" +
                "         * Y = Arm Position Change\n" +
                "         * D Pad Left = Arm -\n" +
                "         * D Pad Right = Arm +\n" +
                "         * D Pad Up = Outer Intake Down\n" +
                "         * D Pad Down = Outer Intake Up");
        telemetry.addData("Scoring Status", scoringATM);
    }

    public void gamepadUpdate() {
        previousActionGamepad1.copy(currentGamepad1);
        previousActionGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }
}