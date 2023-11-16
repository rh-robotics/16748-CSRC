package org.firstinspires.ftc.teamcode.auton.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

/* If an AprilTag is being used to mark the location of something that must be approached
 * squarely from the front, then it’s important to consider the Tag Yaw value. This is a direct
 * indication of how far off (in degrees) the camera is from the tag image’s centerline. This is
 * related to, but not the same as the Tag Bearing. So, all three parameters
 * (Range, Bearing & Yaw) must be used to approach the target and end up directly in front of it.
 *
 * Reaching a certain distance directly in front of the target can be easily performed by a
 * robot with a holonomic (Omnidirectional) drive, because strafing can be used for direct
 * sideways motion. A three-pronged approach can be used. 1) The Target bearing can be used to
 * turn the robot towards the target (as described above). 2) The Target Yaw can be used to
 * strafe sideways, thereby rotating around the target to get directly in front of it. 3) The
 * target range can be used to drive forward or backward to obtain the correct standoff distance.
 *
 * Each of the three axis motions could be controlled by a simple proportional control loop,
 * where turning towards the tag is given the highest gain (priority), followed by strafing
 * sideways, followed by approaching the tag.
 *
 * Yaw: rotation of the tag around the Z axis. A Counter-Clockwise rotation is considered positive.
 * Bearing: how many degrees the camera must turn to point directly at the target
 * Range: the direct distance to the center of the target
 *
 * X: The green X axis value represents the sideways offset to the tag. Note that this value
 * is negative (to the left of the camera center).
 *
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html
 */

public class AprilTagAlignment extends OpMode {
    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    private Hardware robot;
    OpenCvCamera camera;
    WebcamName cameraHardwareElement;

    List<AprilTagDetection> currentDetections;
    AprilTagDetection targetTag;

    float wheelEncoderPPR = 537.7f; // PPR
    int wheelDiameter = 96; // mm
    double mmPerEncoderTick = (360/wheelEncoderPPR)/360*(wheelDiameter*Math.PI); // 0.56089435511 mm
    // TODO: Measure and record turning radius. Should be the same as the distance between wheels.
    float turningRadius = 228.6f; // mm
    int targetTagID = 2;

    float yawTolerance = 1.0f; // mm
    float bearingTolerance = 1.0f; // mm
    float rangeTolerance = 1.0f; // mm
    float motorRunPower = 0.4f;
    float leftFTarget;
    float rightFTarget;
    float leftBTarget;
    float rightBTarget;

    float targetYaw = 0; // mm
    float targetBearing = 0; // mm
    float targetRange = 600; // mm

    @Override
    public void init() {
        robot = new Hardware(hardwareMap, telemetry);

        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "leftRear", "setDirection:REVERSE"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightFront"));
        robot.introduce(new HardwareElement<>(DcMotor.class, hardwareMap, "rightRear"));

        // TODO: Implement camera (type: WebcamName) in hardware class.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cameraHardwareElement = hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(cameraHardwareElement, cameraMonitorViewId);
    }

    @Override
    public void loop() {
            currentDetections = aprilTagProcessor.getDetections();

            if (targetTagDetected(currentDetections)) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == targetTagID) {
                        targetTag = detection;
                    }
                }

                telemetry.addData("AprilTag Detected", true);
                telemetry.addData("ID", targetTag.id);
                telemetry.addData("Yaw", targetTag.ftcPose.yaw);
                telemetry.addData("Bearing", targetTag.ftcPose.bearing);
                telemetry.addData("Range", targetTag.ftcPose.range);

                stopAndResetMotors();

                if (Math.abs(targetYaw - degreesToMM(targetTag.ftcPose.yaw)) > yawTolerance) {
                    moveYaw();
                } else if (targetBearing - Math.abs(degreesToMM(targetTag.ftcPose.bearing)) > bearingTolerance) {
                    moveBearing();
                } else if (targetRange - Math.abs(inchesToMM(targetTag.ftcPose.range)) > rangeTolerance) {
                    moveRange();
                } else {
                    telemetry.addLine("Robot in Position");
                }
            } else {
                telemetry.addData("AprilTag Detected", false);
            }

    }

    public void stopAndResetMotors() {
        robot.<DcMotor>get("rightFront").setPower(0);
        robot.<DcMotor>get("leftFront").setPower(0);
        robot.<DcMotor>get("rightRear").setPower(0);
        robot.<DcMotor>get("leftRear").setPower(0);

        robot.<DcMotor>get("rightFront").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("leftFront").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("rightRear").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.<DcMotor>get("leftRear").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveYaw() {
        if (targetTag.ftcPose.yaw > targetYaw) { // Left side down, moving rightFront.
            setTargetPosition(robot.<DcMotor>get("rightFront"), degreesToMM(Math.abs(targetTag.ftcPose.yaw)));
            robot.<DcMotor>get("rightFront").setPower(motorRunPower);

            while (robot.<DcMotor>get("rightFront").getTargetPosition() !=
                    robot.<DcMotor>get("rightFront").getCurrentPosition()) {
                showMotorStatus(robot.<DcMotor>get("rightFront"));
            }

            robot.<DcMotor>get("rightFront").setPower(0);

        } else { // Right side down, moving leftFront.
            setTargetPosition(robot.<DcMotor>get("leftFront"), degreesToMM(Math.abs(targetTag.ftcPose.yaw)));
            robot.<DcMotor>get("leftFront").setPower(motorRunPower);

            while (robot.<DcMotor>get("leftFront").getTargetPosition() !=
                    robot.<DcMotor>get("leftFront").getCurrentPosition()) {
                showMotorStatus(robot.<DcMotor>get("leftFront"));
            }

            robot.<DcMotor>get("leftFront").setPower(0);
        }
    }

    public void moveBearing() {
        float xDistance = (float) inchesToMM(targetTag.ftcPose.x);

        // Tag is left of where we want it in relation to the robot (targetBearing). Move robot left.
        if(xDistance < targetBearing) {
            leftFTarget = xDistance;
            rightFTarget = -xDistance;
            leftBTarget = -xDistance;
            rightBTarget = xDistance;
        } else { // Move robot right.
            leftFTarget = -xDistance;
            rightFTarget = xDistance;
            leftBTarget = xDistance;
            rightBTarget = -xDistance;
        }

        setTargetPosition(robot.<DcMotor>get("leftFront"), leftFTarget);
        setTargetPosition(robot.<DcMotor>get("rightFront"), rightFTarget);
        setTargetPosition(robot.<DcMotor>get("leftRear"), leftBTarget);
        setTargetPosition(robot.<DcMotor>get("rightRear"), rightBTarget);

        while (robot.<DcMotor>get("leftFront").getTargetPosition() !=
                robot.<DcMotor>get("leftFront").getCurrentPosition() &&
                robot.<DcMotor>get("rightFront").getTargetPosition() !=
                    robot.<DcMotor>get("rightFront").getCurrentPosition() &&
                robot.<DcMotor>get("leftRear").getTargetPosition() !=
                    robot.<DcMotor>get("leftRear").getCurrentPosition() &&
                robot.<DcMotor>get("rightRear").getTargetPosition() !=
                    robot.<DcMotor>get("rightRear").getCurrentPosition()) {

            showMotorStatus(robot.<DcMotor>get("leftFront")); // Also turns on/off motor power.
            showMotorStatus(robot.<DcMotor>get("rightFront"));
            showMotorStatus(robot.<DcMotor>get("leftRear"));
            showMotorStatus(robot.<DcMotor>get("rightRear"));
        }
    }

    public void moveRange() {
        double distance = inchesToMM(targetTag.ftcPose.range) - targetRange;

        setTargetPosition(robot.<DcMotor>get("leftFront"), distance);
        setTargetPosition(robot.<DcMotor>get("rightFront"), distance);
        setTargetPosition(robot.<DcMotor>get("leftRear"), distance);
        setTargetPosition(robot.<DcMotor>get("rightRear"), distance);

        while (robot.<DcMotor>get("leftFront").getTargetPosition() !=
                robot.<DcMotor>get("leftFront").getCurrentPosition() &&
                robot.<DcMotor>get("rightFront").getTargetPosition() !=
                        robot.<DcMotor>get("rightFront").getCurrentPosition() &&
                robot.<DcMotor>get("leftRear").getTargetPosition() !=
                        robot.<DcMotor>get("leftRear").getCurrentPosition() &&
                robot.<DcMotor>get("rightRear").getTargetPosition() !=
                        robot.<DcMotor>get("rightRear").getCurrentPosition()) {

            showMotorStatus(robot.<DcMotor>get("leftFront")); // Also turns on/off motor power.
            showMotorStatus(robot.<DcMotor>get("rightFront"));
            showMotorStatus(robot.<DcMotor>get("leftRear"));
            showMotorStatus(robot.<DcMotor>get("rightRear"));
        }
    }

    public void showMotorStatus(DcMotor motor) {
        if (motor.getTargetPosition() ==
                motor.getCurrentPosition()) {
            motor.setPower(0);
            telemetry.addLine(motor.getDeviceName() + " in Position");
        } else {
            motor.setPower(motorRunPower);
            telemetry.addLine("Running " + motor.getDeviceName() + " Motor");
            telemetry.addData(motor.getDeviceName() + " Target Position",
                    motor.getTargetPosition());
            telemetry.addData(motor.getDeviceName() + " Current Position",
                    motor.getTargetPosition());
        }
    }

    public boolean targetTagDetected(List<AprilTagDetection> currentDetections) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetTagID) {
                return true;
            }
        }
        return false;
    }

    public void initAprilTag() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(false)
            .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(cameraHardwareElement)
                .addProcessor(aprilTagProcessor)
                // TODO: Figure out if these are necessary/would be useful. Look through the rest
                //  of the VisionPortal Builder features.
//                .setCameraResolution(new Size(640, 480));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
//                .enableCameraMonitoring(true);      // Enable LiveView (RC preview).
//                .setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.

                .build();
    }

    public double degreesToMM(double degrees) {
        return (degrees / 360) * turningRadius;
    }

    public double inchesToMM(double inches) {
        return inches*25.4;
    }

    public void setTargetPosition(DcMotor motor, double mm) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition((int) Math.round(mm/mmPerEncoderTick));
    }
}