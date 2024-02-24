package org.firstinspires.ftc.teamcode.auton.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Autonomous OpMode to test scanning April Tags
 */
@Autonomous(name = "AAApril Tag Test")
public class AprilTagTest2 extends OpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // Variable to store our instance of the AprilTag processor.
    private AprilTagProcessor aprilTag;

    // Variable to store our instance of the vision portal.
    private VisionPortal visionPortal;

    private int targetID = 2;

    RobotMethods robotMethods;
    Hardware robot;

    @Override
    public void init() {
        robotMethods = new RobotMethods();

        robot = robotMethods.init(telemetry, hardwareMap);

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }

    @Override
    public void loop(){
        List<AprilTagDetection> detections = telemetryAprilTag();

        for (AprilTagDetection detection : detections) {
            if (detection.id == targetID) {
//                alignToAprilTag(detection);
                moveYaw(detection);
                telemetry.addData("target april tag", true);
            }
        }
    }

    private void alignToAprilTag(AprilTagDetection detection) {
        boolean robotInPosition = false;
        double yawTolerance = 5;
        double bearingTolerance = 6;

        while (!robotInPosition) {
            telemetry.addData("AprilTag Detected", true);
            telemetry.addData("ID", detection.id);
            telemetry.addData("Yaw", detection.ftcPose.yaw);
            telemetry.addData("Bearing", detection.ftcPose.bearing);
            telemetry.addData("Range", detection.ftcPose.range);

            if (Math.abs(detection.ftcPose.yaw) > yawTolerance) {
                moveYaw(detection);
                waitToFinish();
            } else if (Math.abs(detection.ftcPose.bearing) > bearingTolerance) {
                moveBearing(detection);
                waitToFinish();
            } else {
                moveRange(0, detection);
                waitToFinish();
                robotInPosition = true;
            }
        }

        telemetry.addData("Robot in position", true);
    }

    public void waitToFinish(){
        while(robot.<DcMotor>get("leftFront").getCurrentPosition() - robot.<DcMotor>get("leftFront").getTargetPosition() < 5){}
    }

    public void stop() {
        visionPortal.close();
    }

    public void moveYaw(AprilTagDetection detection) {
        robotMethods.turn(robot, -detection.ftcPose.yaw);
    }

    public void moveBearing(AprilTagDetection detection) {
        double xDistance = detection.ftcPose.bearing;

        double leftFTarget;
        double rightFTarget;
        double rightBTarget;
        double leftBTarget;

        // Tag is left of where we want it in relation to the robot (targetBearing). Move robot left.
        if(xDistance < 0) {
            leftFTarget = -xDistance;
            rightFTarget = xDistance;
            leftBTarget = xDistance;
            rightBTarget = -xDistance;
        } else { // Move robot right.
            leftFTarget = xDistance;
            rightFTarget = -xDistance;
            leftBTarget = -xDistance;
            rightBTarget = xDistance;
        }

        robotMethods.runWheelsToTarget(robot, leftBTarget, leftFTarget, rightBTarget, rightFTarget);
    }

    public void moveRange(double targetRange, AprilTagDetection detection) {
        double distance = detection.ftcPose.range - targetRange;

        robotMethods.runWheelsToTarget(robot, -distance);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam1"));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }


    /**
     * Add telemetry about AprilTag detections.
     */
    private List<AprilTagDetection> telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        return currentDetections;
    }

}