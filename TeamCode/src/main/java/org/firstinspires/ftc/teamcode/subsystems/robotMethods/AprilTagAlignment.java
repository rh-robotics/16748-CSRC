package org.firstinspires.ftc.teamcode.subsystems.robotMethods;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.hardware.HardwareElement;
import org.firstinspires.ftc.teamcode.subsystems.robotMethods.RobotMethods;
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

public class AprilTagAlignment{
    Gamepad previousActionGamepad1  = new Gamepad();
    Gamepad previousActionGamepad2  = new Gamepad();
    Gamepad currentGamepad1  = new Gamepad();
    Gamepad currentGamepad2  = new Gamepad();

    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    private Hardware robot;
    OpenCvCamera camera;
    WebcamName cameraHardwareElement;

    List<AprilTagDetection> currentDetections;
    AprilTagDetection targetTag;

    int targetTagID = 2;

    double yawTolerance = 6; // mm
    double bearingTolerance = 7; // mm
    double rangeTolerance = 5; // mm
    double motorRunPower = 0.4;
    double leftFTarget;
    double rightFTarget;
    double leftBTarget;
    double rightBTarget;

    double targetYaw = 0; // mm
    double targetBearing = 0; // mm

    double adjustment1 = 1;
    double adjustment2 = 1;

    RobotMethods robotMethods;
    boolean robotInPosition = false;

    public void alignToAprilTag(HardwareMap hardwareMap, Telemetry telemetry, double targetRange) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam1"));

        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();

        currentDetections = aprilTagProcessor.getDetections();

        while (!robotInPosition) {
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

                if (Math.abs(targetTag.ftcPose.yaw) > yawTolerance) {
                    moveYaw();
                    waitToFinish();
                } else if (Math.abs(targetTag.ftcPose.bearing) > bearingTolerance) {
                    moveBearing();
                    waitToFinish();
                } else if (Math.abs(targetTag.ftcPose.range) > rangeTolerance) {
                    moveRange(targetTag.ftcPose.range - targetRange);
                    waitToFinish();
                    robotInPosition = true;
                }

            } else {
                telemetry.addData("AprilTag Detected", false);
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

    public void moveYaw() {
        robotMethods.turn(robot, -targetTag.ftcPose.yaw);
    }

    public void moveBearing() {
        double xDistance = targetTag.ftcPose.bearing;

        // Tag is left of where we want it in relation to the robot (targetBearing). Move robot left.
        if(xDistance < targetBearing) {
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

    public void moveRange(double targetRange) {
        double distance = targetTag.ftcPose.range - targetRange;

        robotMethods.runWheelsToTarget(robot, -distance);
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
}