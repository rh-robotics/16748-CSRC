package org.firstinspires.ftc.teamcode.subsystems.robotMethods;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Context;

public class RobotMethods {
    static float wheelEncoderPPR = 537.7f; // PPR
    static int wheelDiameter = 96; // mm
    static double mmPerEncoderTick = (360/wheelEncoderPPR)/360*(wheelDiameter*Math.PI); // 0.56089435511 mm
    static double distanceBetweenWheels = 264; // mm

    public static void driveTo(Hardware robot, Context context, double targetX, double targetY,
                               double motorRunPower, double tolerance, Telemetry telemetry) {
        double[] passingLocation = new double[] {300,200};
        if (context.getY() < 250 && targetY > 250) {
            goToPosition(robot, context, 300, 200, motorRunPower, tolerance, telemetry);
            goToPosition(robot, context, 300, 350, motorRunPower, tolerance, telemetry);
        } else if (context.getY() > 250 && targetY < 250) {
            goToPosition(robot, context, 300, 350, motorRunPower, tolerance, telemetry);
            goToPosition(robot, context, 300, 200, motorRunPower, tolerance, telemetry);
        }

        goToPosition(robot, context, targetX, targetY, motorRunPower, tolerance, telemetry);
    }

    public static void goToPosition(Hardware robot, Context context, double targetX, double targetY,
                                    double motorRunPower, double tolerance, Telemetry telemetry) {
        double moveX = targetX - context.getX();
        double moveY = targetY - context.getY();

        move(robot, context, moveX, moveY, motorRunPower, tolerance, telemetry);
    }

    public static void turnTo(Hardware robot, Context context, double directionTo, double motorRunPower,
                              double tolerance, Telemetry telemetry) {
        double currentDirection = context.getDirection();
        double degreesToTravel = directionTo - currentDirection;

        turn(robot, degreesToTravel, motorRunPower, tolerance, telemetry);
    }

    private static void move(Hardware robot, Context context, double moveX, double moveY,
                             double motorRunPower, double tolerance, Telemetry telemetry) {
        double angleToTarget = 0;

        // TODO: Kinda bs'd this math, ask Sadie to look over it.
        if(moveY == 0) {
            if (moveX > 0) {
                angleToTarget = 90;
            } else if (moveX < 0) {
                angleToTarget = 270;
            }
        } else if (moveY < 0){ // Negative moveY.
            angleToTarget = (Math.tan(moveX/moveY)*180/Math.PI) + 180;
        } else {
            angleToTarget = (Math.tan(moveX/moveY)*180/Math.PI) % 360;
        }

        turnTo(robot, context, angleToTarget, motorRunPower, tolerance, telemetry);

        double distanceToTarget = Math.sqrt(Math.pow(moveX, 2) + Math.pow(moveY, 2));
        runWheelsToPosition(robot, distanceToTarget, motorRunPower, tolerance, telemetry);
    }

    private static void turn(Hardware robot, double degreesToTravel, double motorRunPower, double tolerance,
                             Telemetry telemetry) {
        // Clockwise turn.
        double circumference = Math.PI * distanceBetweenWheels;
        double travelDistance = circumference*(degreesToTravel/360);

        runWheelsToPosition(robot, travelDistance, motorRunPower, tolerance, telemetry);
    }

    private static void runWheelsToPosition(Hardware robot, double targetPosition, double motorRunPower,
                                            double tolerance, Telemetry telemetry) {
        setWheelTargets(robot, targetPosition);

        while(!wheelsInPosition(robot, tolerance)) {
            updateWheels(telemetry, robot, motorRunPower);
        }

        stopAndResetWheels(robot);
    }

    private static void updateWheels(Telemetry telemetry, Hardware robot, double motorRunPower) {
        updateMotor(telemetry, robot.<DcMotor>get("leftFront"), motorRunPower);
        updateMotor(telemetry, robot.<DcMotor>get("leftRear"), motorRunPower);
        updateMotor(telemetry, robot.<DcMotor>get("rightFront"), motorRunPower);
        updateMotor(telemetry, robot.<DcMotor>get("rightRear"), motorRunPower);
    }

    private static void setWheelTargets(Hardware robot, double mm) {
        setTargetPosition(robot.<DcMotor>get("leftFront"), mm);
        setTargetPosition(robot.<DcMotor>get("leftRear"), mm);
        setTargetPosition(robot.<DcMotor>get("rightFront"), mm);
        setTargetPosition(robot.<DcMotor>get("rightRear"), mm);
    }

    private static void setTargetPosition(DcMotor motor, double mm) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition((int) Math.round(mm/mmPerEncoderTick));
    }

    private static void updateMotor(Telemetry telemetry, DcMotor motor, double motorRunPower) {
        showMotorStatus(motor, telemetry);
        updateMotorPower(motor, motorRunPower);
    }

    private static void updateMotorPower(DcMotor motor, double motorRunPower) {
        if (motor.getTargetPosition() == motor.getCurrentPosition()) {
            motor.setPower(0);
        } else {
            motor.setPower(motorRunPower);
        }
    }

    private static void stopAndResetWheels(Hardware robot) {
        DcMotor motor1 = robot.<DcMotor>get("leftFront");
        DcMotor motor2 = robot.<DcMotor>get("leftRear");
        DcMotor motor3 = robot.<DcMotor>get("rightFront");
        DcMotor motor4 = robot.<DcMotor>get("rightRear");

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private static void showMotorStatus(DcMotor motor, Telemetry telemetry) {
        if (motor.getTargetPosition() == motor.getCurrentPosition()) {
            telemetry.addLine(motor.getDeviceName() + " in Position");
        } else {
            telemetry.addLine("Running " + motor.getDeviceName() + " Motor");
            telemetry.addData(motor.getDeviceName() + " Target Position",
                    motor.getTargetPosition());
            telemetry.addData(motor.getDeviceName() + " Current Position",
                    motor.getTargetPosition());
        }
    }

    private static boolean wheelsInPosition(Hardware robot, double tolerance) {
        DcMotor motor1 = robot.<DcMotor>get("leftFront");
        DcMotor motor2 = robot.<DcMotor>get("leftRear");
        DcMotor motor3 = robot.<DcMotor>get("rightFront");
        DcMotor motor4 = robot.<DcMotor>get("rightRear");

        return Math.abs(motor1.getTargetPosition()-motor1.getCurrentPosition()) < tolerance &&
                Math.abs(motor2.getTargetPosition()-motor2.getCurrentPosition()) < tolerance &&
                Math.abs(motor3.getTargetPosition()-motor3.getCurrentPosition()) < tolerance &&
                Math.abs(motor4.getTargetPosition()-motor4.getCurrentPosition()) < tolerance;
    }

    private double inchesToMM(double inches) {
        return inches*25.4;
    }
}