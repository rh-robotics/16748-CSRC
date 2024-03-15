package org.firstinspires.ftc.teamcode.subsystems.robotMethods;

import static android.text.TextUtils.indexOf;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.stateMachineController.Context;

public class RobotMethods {
    private double wheelsCPR = 537.7;
    private double targetPos = 3;
    private double motorTolerance = 50; // ticks
    private double inchesPerRevolution = 38 / 3;
    private double armCPR = 1993.6;
    private final double strafeAdjustment = 6 / 4.5;
    private final double degreesToCoordinates = 0.9;

    private final int armIntakePos = 0; // In revolutions
    private final int armScoringPos = -989;
    private final int VSIntakePos = 0;
    private final int VSScoringPos = 402;
    private final double clawIntakePos = 0.8;
    private final double clawScoringPos = 0.325;

    private final double forwardWheelsAdjustment = 0.9315;
    private final double backwardWheelsAdjustment = 0.885;
    private final double rightStrafeAdjustment = 1.2199999;

    private double[] lockPositions = new double[]{0.0, 0.4, 1.0};

    public void moveTo(Hardware robot, Context context, double targetY, double targetX) {
        if (context.getY() >= 300 && context.getY() >= 400) {
            context.setDirection(turnToClosestDirection(robot, context, false, true, false, true));
            moveY(robot, context, (context.getX() - 200));
            context = updateContext(context, 0, context.getY() - 200);
        }

        if (targetY < context.getY() && targetY < 300 && context.getY() > 300
                || targetY > context.getY() && targetY > 400 && context.getY() < 400) {
            context.setDirection(turnToClosestDirection(robot, context, true, false, true, false));
            moveX(robot, context, (context.getX() - context.getCrossingPos()[0]));
            context = updateContext(context, (context.getX() - context.getCrossingPos()[0]), 0);

            context.setDirection(turnToClosestDirection(robot, context, false, true, false, true));
            moveY(robot, context, (context.getY() - targetY));
            context = updateContext(context, 0, context.getY() - targetY);

            context.setDirection(turnToClosestDirection(robot, context, true, false, true, false));
            moveX(robot, context, (context.getX() - targetX));
        } else {
            context.setDirection(turnToClosestDirection(robot, context, true, false, true, false));
            moveX(robot, context, (context.getX() - targetX));
            context = updateContext(context, context.getX() - targetX, 0);

            context.setDirection(turnToClosestDirection(robot, context, false, true, false, true));
            moveY(robot, context, context.getY() - targetY);
        }
    }

    public double turnToClosestDirection(Hardware robot, Context context, boolean d270, boolean d180,
                                         boolean d90, boolean d0) {
        double distance270;
        double distance180;
        double distance90;
        double distance0;

        if (d270) {
            distance270 = shortestRoute(context, 270);
        } else {
            distance270 = 1000;
        }

        if (d180) {
            distance180 = shortestRoute(context, 180);
        } else {
            distance180 = 1000;
        }

        if (d90) {
            distance90 = shortestRoute(context, 90);
        } else {
            distance90 = 1000;
        }

        if (d0) {
            distance0 = shortestRoute(context, 0);
        } else {
            distance0 = 1000;
        }

        double minDistance = Math.min(Math.min(Math.abs(distance0), Math.abs(distance180)),
                Math.min(Math.abs(distance90), Math.abs(distance270)));

        if (minDistance == Math.abs(distance0)) {
            return turnTo(robot, context, 0);
        } else if (minDistance == Math.abs(distance180)) {
            return turnTo(robot, context, 180);
        } else if (minDistance == Math.abs(distance270)) {
            return turnTo(robot, context, 270);
        } else {
            return turnTo(robot, context, 0);
        }

    }

    public double shortestRoute(Context context, double direction) {
        if (Math.abs(direction - context.getDirection()) % 360 < (direction - context.getDirection()) % 360) {
            return (-(Math.abs(direction - context.getDirection()) % 360));
        } else {
            double degrees = (direction - context.getDirection()) % 360;
            while (degrees < 0) {
                degrees += 360;
            }
            return (degrees);
        }
    }

    public Context updateContext(Context context, double xDifference, double yDifference) {
        context.setLocation(xDifference + context.getX(), yDifference + context.getY());
        return context;
    }

    public Context moveX(Hardware robot, Context context, double x) {
        context.setDirection(turnToClosestDirection(robot, context, true, false, true, false));

        if (context.getDirection() == 90) {
            runWheelsToTarget(robot, x);
        } else { // context.getDirection == 270
            runWheelsToTarget(robot, -x);
        }
        context = updateContext(context, x, 0);

        return context;
    }

    public Context moveY(Hardware robot, Context context, double y) {
        context.setDirection(turnToClosestDirection(robot, context, false, true, false, true));

        if (context.getDirection() == 0) {
            runWheelsToTarget(robot, y);
        } else if (context.getDirection() == 180) {
            y = -y;
            runWheelsToTarget(robot, y);
        }
        context = updateContext(context, 0, y);

        return context;
    }

    public double turnTo(Hardware robot, Context context, double direction) {
        if (mod360(Math.abs(direction - context.getDirection())) < mod360(direction - context.getDirection())) {
            turn(robot, -1 * mod360(Math.abs(direction - context.getDirection())));
        } else {
            turn(robot, mod360(direction - context.getDirection()));
        }

        return direction;
    }

    private double mod360(double number) {
        while (number < 0) {
            number += 360;
        }
        return number % 360;
    }

    public void turn(Hardware robot, double degrees) { // clockwise
        runWheelsToTarget(robot, -degreesToCoordinates * degrees, -degreesToCoordinates * degrees,
                degreesToCoordinates * degrees, degreesToCoordinates * degrees);
    }

    public Hardware init(Telemetry telemetry, HardwareMap hardwareMap) {
        telemetry.addLine("Initializing");

        Hardware robot;
        robot = new Hardware(hardwareMap, telemetry);

        // Init Servos.
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawLock"));
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "clawJoint"));
        robot.introduce(new HardwareElement<>(Servo.class, hardwareMap, "outerIntakeJoint"));

        // Init CR Servos.
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeGeckoWheels"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "intakeTube"));
        robot.introduce(new HardwareElement<>(CRServo.class, hardwareMap, "outerIntakeTube"));

        // Init DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront", "setDirection:FORWARD, setMode:RUN_WITHOUT_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear", "setDirection:FORWARD, setMode:RUN_WITHOUT_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront", "setDirection:FORWARD, setMode:RUN_WITHOUT_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear", "setDirection:REVERSE, setMode:RUN_WITHOUT_ENCODER"));

        // Init arm and Viper Slide DcMotors.
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "armJoint", "setMode:RUN_WITHOUT_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftViperSlide", "setMode:RUN_WITHOUT_ENCODER"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightViperSlide", "setDirection:REVERSE,setMode:RUN_WITHOUT_ENCODER"));

        // Init Sensors
        robot.introduce(new HardwareElement<>(DistanceSensor.class, hardwareMap, "clawDistanceSensor"));
        robot.introduce(new HardwareElement<>(DistanceSensor.class, hardwareMap, "rampColorSensor"));
        robot.introduce(new HardwareElement<>(DistanceSensor.class, hardwareMap, "intakeColorSensor"));

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
        if (target > 0) {
            target *= forwardWheelsAdjustment;
        } else {
            target *= backwardWheelsAdjustment;
        }

        robot.<DcMotor>get("leftRear").setTargetPosition(wheelsCoordinatesToTicks(target));
        robot.<DcMotor>get("leftFront").setTargetPosition(wheelsCoordinatesToTicks(target));
        robot.<DcMotor>get("rightRear").setTargetPosition(wheelsCoordinatesToTicks(target));
        robot.<DcMotor>get("rightFront").setTargetPosition(-wheelsCoordinatesToTicks(target));

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
        return (int) Math.round((revolutions * wheelsCPR) / inchesPerRevolution * ((double) 144 / 600));
    }

    /**
     * Reset motors to initial position here before we run anything else.
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

    public void runWheelsToTarget(Hardware robot, double lrTarget, double lfTarget, double rrTarget, double rfTarget) {
        robot.<DcMotor>get("leftRear").setTargetPosition(wheelsCoordinatesToTicks(lrTarget));
        robot.<DcMotor>get("leftFront").setTargetPosition(wheelsCoordinatesToTicks(lfTarget));
        robot.<DcMotor>get("rightRear").setTargetPosition(wheelsCoordinatesToTicks(rrTarget));
        robot.<DcMotor>get("rightFront").setTargetPosition(-wheelsCoordinatesToTicks(rfTarget));

        runMotorsToTargets(robot);
    }

    public void scoringPosition(Hardware robot) {
        robot.<DcMotor>get("leftViperSlide").setTargetPosition(VSScoringPos);
        robot.<DcMotor>get("rightViperSlide").setTargetPosition(VSScoringPos);

        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.<DcMotor>get("leftViperSlide").setPower(0.5);
        robot.<DcMotor>get("rightViperSlide").setPower(0.5);

        while (Math.abs(robot.<DcMotor>get("leftViperSlide").getTargetPosition() - robot.<DcMotor>get("leftViperSlide").getCurrentPosition())
                > motorTolerance) {
        }

        robot.<DcMotor>get("armJoint").setTargetPosition(armScoringPos);

        robot.<DcMotor>get("armJoint").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("armJoint").setPower(0.5);

        robot.<Servo>get("clawJoint").setPosition(clawScoringPos);
    }

    public void intakePosition(Hardware robot) {
        robot.<DcMotor>get("leftViperSlide").setTargetPosition(VSScoringPos + 300);
        robot.<DcMotor>get("rightViperSlide").setTargetPosition(VSScoringPos + 300);

        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.<Servo>get("clawJoint").setPosition(clawIntakePos);

        robot.<DcMotor>get("armJoint").setTargetPosition(armIntakePos);

        robot.<DcMotor>get("armJoint").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("armJoint").setPower(0.5);


        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < 0.625) {
        }

        robot.<DcMotor>get("leftViperSlide").setTargetPosition(VSIntakePos);
        robot.<DcMotor>get("rightViperSlide").setTargetPosition(VSIntakePos);

        robot.<DcMotor>get("leftViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.<DcMotor>get("rightViperSlide").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.<DcMotor>get("leftViperSlide").setPower(0.5);
        robot.<DcMotor>get("rightViperSlide").setPower(0.5);
    }

    /**
     * Reset motors to initial position here before ending the program. This helps for testing
     * when we want to run auton multiple times in a row, helps with redundancy for making sure
     * our encoders are accurate, and also is just convenient to have reset at the end of auton period.
     **/
    public void endResetMotors(Hardware robot) {
        intakePosition(robot);

        while (Math.abs(robot.<DcMotor>get("leftViperSlide").getCurrentPosition() - robot.<DcMotor>get("leftViperSlide").getTargetPosition()) < 5
                && Math.abs(robot.<DcMotor>get("armJoint").getCurrentPosition() - robot.<DcMotor>get("armJoint").getTargetPosition()) < 5) {
        }
    }

    private int revolutionsToTicks(double revolutions) {
        return (int) Math.round(revolutions * armCPR);
    }

    public void intakePixel(Hardware robot) {
        int clawLock = getClawLock(robot);
        if (clawLock == 0) {
            while (robot.<DistanceSensor>get("clawDistanceSensor").getDistance(DistanceUnit.CM) > 4) {
                runIntake(robot);
            }
            stopIntake(robot);
            setClawLock(robot, clawLock + 1);
        } else if (clawLock == 1) {
            while (robot.<ColorSensor>get("rampColorSensor").green() < 1000 ||
                    robot.<ColorSensor>get("rampColorSensor").red() < 1000 ||
                    robot.<ColorSensor>get("rampColorSensor").blue() < 1000) {
                runIntake(robot);
            }
            while (robot.<ColorSensor>get("rampColorSensor").green() > 1000 ||
                    robot.<ColorSensor>get("rampColorSensor").red() > 1000 ||
                    robot.<ColorSensor>get("rampColorSensor").blue() > 1000) {
                runIntake(robot);
            }
            rest(1);
            stopIntake(robot);
            setClawLock(robot, clawLock + 1);
        } else if (clawLock == 2) {
            stopIntake(robot);
        }
    }

    public void flushPixel(Hardware robot) {
        if (robot.<ColorSensor>get("rampColorSensor").green() > 1000 ||
                robot.<ColorSensor>get("rampColorSensor").red() > 1000 ||
                robot.<ColorSensor>get("rampColorSensor").blue() > 1000) {
            while (robot.<ColorSensor>get("rampColorSensor").green() > 1000 ||
                    robot.<ColorSensor>get("rampColorSensor").red() > 1000 ||
                    robot.<ColorSensor>get("rampColorSensor").blue() > 1000) {
                reverseIntake(robot);
            }
            rest(0.5);
            stopIntake(robot);
        } else {
            reverseIntake(robot);
            rest(5);
            stopIntake(robot);
        }
    }

    public void setClawLock(Hardware robot, int clawLock) {
        robot.<Servo>get("clawLock").setPosition(lockPositions[clawLock]);
    }

    public void rest(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < seconds) {
        }
    }

    public void reverseIntake(Hardware robot) {
        robot.<CRServo>get("intakeTube").setPower(-1);
        robot.<CRServo>get("intakeGeckoWheels").setPower(0);
        robot.<CRServo>get("outerIntakeTube").setPower(0);
    }

    public void runIntake(Hardware robot) {
        robot.<CRServo>get("intakeTube").setPower(1);
        robot.<CRServo>get("intakeGeckoWheels").setPower(1);
        robot.<CRServo>get("outerIntakeTube").setPower(1);
    }

    public void stopIntake(Hardware robot) {
        robot.<CRServo>get("intakeTube").setPower(0);
        robot.<CRServo>get("intakeGeckoWheels").setPower(0);
        robot.<CRServo>get("outerIntakeTube").setPower(0);
    }

    private int getClawLock(Hardware robot) {
        if (robot.<Servo>get("clawLock").getPosition() == lockPositions[0]) {
            return 0;
        } else if (robot.<Servo>get("clawLock").getPosition() == lockPositions[1]) {
            return 1;
        } else {
            return 2;
        }
    }

    public void outerIntakeDown(Hardware robot) {
        robot.<Servo>get("outerIntakeJoint").setPosition(1);
    }

    public void outerIntakeUp(Hardware robot) {
        robot.<Servo>get("outerIntakeJoint").setPosition(0);
    }
}