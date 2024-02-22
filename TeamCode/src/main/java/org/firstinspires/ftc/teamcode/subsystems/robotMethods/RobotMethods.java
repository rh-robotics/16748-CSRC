package org.firstinspires.ftc.teamcode.subsystems.robotMethods;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Context;

public class RobotMethods {
    private double wheelsCPR = 537.7;
    private double targetPos = 3;
    private double motorTolerance = 50; // ticks
    private double inchesPerRevolution = 38/3;
    private double armCPR = 1993.6;

    private final double armIntakePos = 0; // In revolutions
    private final double armScoringPos = -0.35;
    private final double VSIntakePos = 0;
    private final double VSScoringPos = 0.5;
    private final double strafeAdjustment = 6/4.5;
    private final double degreesToCoordinates = 0.8;

    public void move(Hardware robot, Context context, double targetY, double targetX) {
        if (context.getDirection() != 0 && context.getDirection() != 180) {
            if (Math.abs(180-context.getDirection()) < Math.abs(0-context.getDirection())) {
                turnTo(robot, context, 180);
                context.setDirection(180);
            } else {
                turnTo(robot, context, 0);
                context.setDirection(0);
            }
        }

        moveX(robot, context, (context.getX() - targetX));
        context = updateContext(context, context.getX() - targetX);

        moveY(robot, context, (context.getX() - targetY));
        context = updateContext(context, 0, context.getY() - targetY);
    }

    public Context updateContext(Context context, double xDifference) {
        context.setLocation(xDifference+context.getX(), context.getY());
        return context;
    }

    public Context updateContext(Context context, double xDifference, double yDifference) {
        context.setLocation(xDifference + context.getX(), yDifference + context.getY());
        return context;
    }

    public Context updateContext(Context context, double xDifference, double yDifference,
                                 double directionDifference) {
        context.setLocation(xDifference + context.getX(), yDifference + context.getY(),
                directionDifference + context.getDirection());
        return context;
    }

    public void moveX(Hardware robot, Context context, double x) {
        if(context.getDirection() == 0) {
            runWheelsToTarget(robot, strafeAdjustment * -x, x * strafeAdjustment,
                    x * strafeAdjustment, -x * strafeAdjustment);
        } else if (context.getDirection() == 180) {
            x = -x;
            runWheelsToTarget(robot, strafeAdjustment * -x, x * strafeAdjustment,
                    x * strafeAdjustment, -x * strafeAdjustment);
        } else if (Math.abs(180-context.getDirection()) < Math.abs(0-context.getDirection())) {
            turnTo(robot, context, 180);
            x = -x;
            runWheelsToTarget(robot, strafeAdjustment * -x, x * strafeAdjustment,
                    x * strafeAdjustment, -x * strafeAdjustment);
        } else {
            turnTo(robot, context, 0);
            runWheelsToTarget(robot, strafeAdjustment * -x, x * strafeAdjustment,
                    x * strafeAdjustment, -x * strafeAdjustment);
        }
    }

    public void moveY(Hardware robot, Context context, double y) {
        if(context.getDirection() == 0) {
            runWheelsToTarget(robot, y);
        } else if (context.getDirection() == 180) {
            y = -y;
            runWheelsToTarget(robot, -y);
        } else if (Math.abs(180-context.getDirection()) < Math.abs(0-context.getDirection())) {
            turnTo(robot, context, 180);
            y = -y;
            runWheelsToTarget(robot, y);
        } else {
            turnTo(robot, context, 0);
            runWheelsToTarget(robot, y);
        }
    }

    public void turnTo(Hardware robot, Context context, double direction) {
        // Clockwise distance < counter-clockwise distance
        if ((direction - context.getDirection()) % 360 < (context.getDirection() - direction) % 360) {
            turn(robot, context.getDirection() - direction);
        } else {
            turn(robot, direction - context.getDirection());
        }
    }

    public void turn(Hardware robot, double degrees) { // clockwise
        runWheelsToTarget(robot, -degreesToCoordinates * degrees, -degreesToCoordinates * degrees,
                degreesToCoordinates * degrees, degreesToCoordinates * degrees);
    }

    public Hardware init(Telemetry telemetry, HardwareMap hardwareMap) {
        telemetry.addLine("Initializing");

        Hardware robot;
        robot = new Hardware(hardwareMap, telemetry);

        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear", "setDirection:REVERSE"));

        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armJoint"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlide"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlide", "setDirection:REVERSE"));

        telemetry.addLine("Initialized");

        return robot;
    }

    private void resetWheelEncoders(Hardware robot) {
        robot.<DcMotor>get("leftRear").setPower(0);
        robot.<DcMotor>get("leftFront").setPower(0);
        robot.<DcMotor>get("rightRear").setPower(0);
        robot.<DcMotor>get("rightFront").setPower(0);

        robot.<DcMotor>get("leftRear").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("leftRear").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.<DcMotor>get("leftFront").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("leftFront").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.<DcMotor>get("rightRear").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("rightRear").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.<DcMotor>get("rightFront").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("rightFront").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.<DcMotor>get("leftRear").setTargetPosition(0);
        robot.<DcMotor>get("leftFront").setTargetPosition(0);
        robot.<DcMotor>get("rightRear").setTargetPosition(0);
        robot.<DcMotor>get("rightFront").setTargetPosition(0);
    }

    private void runMotorsToTargets(Hardware robot) {
        while (Math.abs(robot.<DcMotor>get("leftRear").getCurrentPosition() - robot.<DcMotor>get("leftRear").getTargetPosition()) > motorTolerance) {
            setPower(robot.<DcMotor>get("leftRear"));
            setPower(robot.<DcMotor>get("leftFront"));
            setPower(robot.<DcMotor>get("rightRear"));
            setPower(robot.<DcMotor>get("rightFront"));
        }
        robot.<DcMotor>get("rightFront").setPower(0);
        robot.<DcMotor>get("rightRear").setPower(0);
        robot.<DcMotor>get("leftFront").setPower(0);
        robot.<DcMotor>get("leftRear").setPower(0);

        resetWheelEncoders(robot);
    }

    private void setPower(DcMotor motor) {
        if (motor.getTargetPosition() < motor.getCurrentPosition()) {
            motor.setPower(-0.5);
        } else if (motor.getTargetPosition() > motor.getCurrentPosition()) {
            motor.setPower(0.5);
        } else {
            motor.setPower(0);
        }
    }

    public void runWheelsToTarget(Hardware robot, double target) {
        robot.<DcMotor>get("leftRear").setTargetPosition(wheelsCoordinatesToTicks(target));
        robot.<DcMotor>get("leftFront").setTargetPosition(wheelsCoordinatesToTicks(target));
        robot.<DcMotor>get("rightRear").setTargetPosition(wheelsCoordinatesToTicks(target));
        robot.<DcMotor>get("rightFront").setTargetPosition(-wheelsCoordinatesToTicks(target));

        runMotorsToTargets(robot);
    }

    public void runWheelsToTarget(Hardware robot, double lrTarget, double lfTarget, double rrTarget, double rfTarget) {
        robot.<DcMotor>get("leftRear").setTargetPosition(wheelsCoordinatesToTicks(lrTarget));
        robot.<DcMotor>get("leftFront").setTargetPosition(wheelsCoordinatesToTicks(lfTarget));
        robot.<DcMotor>get("rightRear").setTargetPosition(wheelsCoordinatesToTicks(rrTarget));
        robot.<DcMotor>get("rightFront").setTargetPosition(-wheelsCoordinatesToTicks(rfTarget));

        runMotorsToTargets(robot);
    }

    public void printWheelsTelemetry(Hardware robot, Telemetry telemetry) {
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

    private int wheelsCoordinatesToTicks(double revolutions) {
        return (int) Math.round((revolutions * wheelsCPR) / inchesPerRevolution * ((double) 144 /600));
    }

    /** Reset motors to initial position here before we run anything else.
     * This step is necessary because we are using relative encoders, which give encoder information
     * based on where the motor is started/reset.
     **/
    public void start(Hardware robot) {
        resetWheelEncoders(robot);

        robot.<DcMotor>get("armJoint").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("armJoint").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.<DcMotor>get("armJoint").setTargetPosition(0);

        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.<DcMotor>get("leftViperSlide").setTargetPosition(0);

        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.<DcMotor>get("rightViperSlide").setTargetPosition(0);
    }

    private void printScoringTelemetry(Telemetry telemetry, Hardware robot) {
        telemetry.addData("Target Position", robot.<DcMotor>get("armJoint").getTargetPosition());
        telemetry.addData("Current Position", robot.<DcMotor>get("armJoint").getCurrentPosition());
        telemetry.addData("Arm Motor Power", robot.<DcMotor>get("armJoint").getPower());

        telemetry.addLine();
        telemetry.addData("VS Left Target Position", robot.<DcMotor>get("leftViperSlide").getTargetPosition());
        telemetry.addData("VS Left Current Position", robot.<DcMotor>get("leftViperSlide").getCurrentPosition());
        telemetry.addData("VS Left Power", robot.<DcMotor>get("leftViperSlide").getPower());

        telemetry.addLine();
        telemetry.addData("VS Right Target Position", robot.<DcMotor>get("rightViperSlide").getTargetPosition());
        telemetry.addData("VS Right Current Position", robot.<DcMotor>get("rightViperSlide").getCurrentPosition());
        telemetry.addData("VS Right Power", robot.<DcMotor>get("rightViperSlide").getPower());
    }

    public void scoringPosition(Hardware robot) {
        robot.<DcMotor>get("leftViperSlide").setTargetPosition(revolutionsToTicks(VSScoringPos));
        robot.<DcMotor>get("rightViperSlide").setTargetPosition(revolutionsToTicks(VSScoringPos));

        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.<DcMotor>get("leftViperSlide").setPower(0.5);
        robot.<DcMotor>get("rightViperSlide").setPower(0.5);

        while (Math.abs(robot.<DcMotor>get("leftViperSlide").getTargetPosition() - robot.<DcMotor>get("leftViperSlide").getCurrentPosition())
                > Math.abs(VSScoringPos - VSIntakePos) / 2) {}

        robot.<DcMotor>get("armJoint").setTargetPosition(revolutionsToTicks(armScoringPos));

        robot.<DcMotor>get("armJoint").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("armJoint").setPower(0.5);
    }

    public void intakePosition(Hardware robot) {
        robot.<DcMotor>get("armJoint").setTargetPosition(revolutionsToTicks(armIntakePos));

        robot.<DcMotor>get("armJoint").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("armJoint").setPower(0.5);


        robot.<DcMotor>get("leftViperSlide").setTargetPosition(revolutionsToTicks(VSIntakePos));
        robot.<DcMotor>get("rightViperSlide").setTargetPosition(revolutionsToTicks(VSIntakePos));

        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.<DcMotor>get("leftViperSlide").setPower(0.5);
        robot.<DcMotor>get("rightViperSlide").setPower(0.5);
    }

    /** Reset motors to initial position here before ending the program. This helps for testing
     * when we want to run auton multiple times in a row, helps with redundancy for making sure
     * our encoders are accurate, and also is just convenient to have reset at the end of auton period. **/
    public void endResetMotors(Hardware robot) {
        intakePosition(robot);

        while (Math.abs(robot.<DcMotor>get("leftViperSlide").getCurrentPosition() - robot.<DcMotor>get("leftViperSlide").getTargetPosition()) < 5
                && Math.abs(robot.<DcMotor>get("armJoint").getCurrentPosition() - robot.<DcMotor>get("armJoint").getTargetPosition()) < 5) {}
    }

    private int revolutionsToTicks(double revolutions) {
        return (int) Math.round(revolutions * armCPR);
    }
}
