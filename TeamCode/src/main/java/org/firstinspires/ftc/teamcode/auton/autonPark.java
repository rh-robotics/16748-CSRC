package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Context;

import org.firstinspires.ftc.teamcode.subsystems.robotMethods.*;

public class autonPark extends OpMode {
    byte startingLocationIndex = 0;
    double[][] startingLocation = new double[][] {new double[] {0, 0, 0}, new double[] {0, 0, 0},
            new double[] {0, 0, 0}, new double[] {0, 0, 0}};
    byte parkingPositionIndex = 0;
    double[][] parkingPosition = new double[][] {new double[] {0, 0}, new double[] {0, 0},
            new double[] {0, 0}, new double[] {0, 0}};

    double motorRunPower = 0.5;
    double tolerance = 1;

    Hardware robot;
    Context context;
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
    public void init_loop() {
        telemetry.addLine("Y: Backboard Blue \n A: Wall Blue \n X: Backboard Red \n" +
                "B: Wall Red");
        telemetry.addLine("***");
        telemetry.addLine("Right Bumper: Corner Parking \n Left Bumper: Middle Parking");

        updateStartingPosition();
        updateParkingPosition();
    }

    private void updateStartingPosition() {
        if (gamepad1.y || startingLocationIndex == 0) {
            startingLocationIndex = 0;
            telemetry.addData("Starting Position", "Backboard Blue");
        }

        if (gamepad1.a || startingLocationIndex == 1) {
            startingLocationIndex = 1;
            telemetry.addData("Starting Position", "Wall Blue");
        }

        if (gamepad1.x || startingLocationIndex == 2) {
            startingLocationIndex = 2;
            telemetry.addData("Starting Position", "Backboard Red");
        }

        if (gamepad1.b || startingLocationIndex == 3) {
            startingLocationIndex = 3;
            telemetry.addData("Starting Position", "Wall Red");
        }
    }

    private void updateParkingPosition() {
        if (gamepad1.right_bumper && startingLocationIndex <= 1 || parkingPositionIndex == 0) {
            parkingPositionIndex = 0;
            telemetry.addData("Parking Position", "Blue Corner");
        }

        if (gamepad1.right_bumper || parkingPositionIndex == 0){
            parkingPositionIndex = 1;
            telemetry.addData("Parking Position", "Red Corner");
        }

        if (gamepad1.left_bumper && startingLocationIndex <= 1 || parkingPositionIndex == 2) {
            parkingPositionIndex = 2;
            telemetry.addData("Parking Position", "Blue Middle");
        }

        if (gamepad1.left_bumper || parkingPositionIndex == 3) {
            parkingPositionIndex = 3;
            telemetry.addData("Parking Position", "Red Middle");
        }
    }

    @Override
    public void start() {
        handleStartPosition();
        RobotMethods.driveTo(robot, context, parkingPosition[parkingPositionIndex][0],
                parkingPosition[parkingPositionIndex][1], motorRunPower, tolerance, telemetry);
    }

    @Override
    public void loop() {
        telemetry.addLine("Uh oh");
    }

    public void handleStartPosition() {
        context.setLocation(startingLocation[startingLocationIndex][0],
                startingLocation[startingLocationIndex][1], startingLocation[startingLocationIndex][2]);
    }
}
