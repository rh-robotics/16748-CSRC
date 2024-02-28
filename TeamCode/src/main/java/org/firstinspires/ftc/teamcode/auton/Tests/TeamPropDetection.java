//package org.firstinspires.ftc.teamcode.auton.vision;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//@Autonomous(name = "Prop Detection Test")
//public class TeamPropDetection {
//    public OpenCvCamera webcam1 = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
//    String webcamName = "webcam1";
//
//    propPlaces propPlacement;
//    robotStartPlaces robotStartPosition;
//    enum propPlaces {
//        RIGHT,
//        CENTER,
//        LEFT
//    }
//    enum robotStartPlaces {
//        RED_BACKDROP,
//        RED_HUMANPLAYER,
//        BLUE_BACKDROP,
//        BLUE_HUMANPLAYER
//    }
//
//    private final Scalar
//            RED = new Scalar(255, 0, 0),
//            BLUE = new Scalar(0, 0, 255);
//
//    public void init(HardwareMap hardwareMap) {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
//        webcam1.setPipeline(new ColorDetection());
//
//        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//    }
//
//    public int getPropPlacement(HardwareMap hardwareMap, int startPosition) {
//        init(hardwareMap);
//
//        if (startPosition == 0) {
//            robotStartPosition = robotStartPlaces.RED_BACKDROP;
//        } else if (startPosition == 1) {
//            robotStartPosition = robotStartPlaces.RED_HUMANPLAYER;
//        } else if (startPosition == 2) {
//            robotStartPosition = robotStartPlaces.BLUE_BACKDROP;
//        } else if (startPosition == 3) {
//            robotStartPosition = robotStartPlaces.BLUE_HUMANPLAYER;
//        }
//
//        if (propPlacement == propPlaces.LEFT) {
//            return 0;
//        } else if (propPlacement == propPlaces.RIGHT) {
//            return 1;
//        }
//
//        return 2;
//   }
//
//    public class ColorDetection extends OpenCvPipeline {
//        @Override
//        public Mat processFrame(Mat input) {
//            Mat YCbCr = new Mat();
//            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
//
//            // Define sectors left, center, and right sectors
//            Rect leftRect = new Rect(1, 1, 104, 239);
//            Rect centerRect = new Rect(105, 1, 109, 239);
//            Rect rightRect = new Rect(215, 1, 104, 239);
//
//            Mat leftCrop = YCbCr.submat(leftRect);
//            Mat centerCrop = YCbCr.submat(centerRect);
//            Mat rightCrop = YCbCr.submat(rightRect);
//
//            if (robotStartPosition == robotStartPlaces.RED_HUMANPLAYER || robotStartPosition == robotStartPlaces.RED_BACKDROP) {
//                Imgproc.rectangle(input, leftRect, RED, 2);
//                Imgproc.rectangle(input, centerRect, RED, 2);
//                Imgproc.rectangle(input, rightRect, RED, 2);
//
//                Core.extractChannel(leftCrop, leftCrop,2);
//                Core.extractChannel(centerCrop, centerCrop,2);
//                Core.extractChannel(rightCrop, rightCrop,2);
//
//                Scalar leftAvg = Core.mean(leftCrop);
//                Scalar centerAvg = Core.mean(centerCrop);
//                Scalar rightAvg = Core.mean(rightCrop);
//
//                double leftAvgRed = leftAvg.val[0];
//                double centerAvgRed = centerAvg.val[0];
//                double rightAvgRed = rightAvg.val[0];
//
//                double maxPercentRed = Math.max(Math.max(leftAvgRed, centerAvgRed), rightAvgRed);
//
//                if (maxPercentRed == leftAvgRed) {
//                    propPlacement = propPlaces.LEFT;
//                } else if (maxPercentRed == centerAvgRed) {
//                    propPlacement = propPlaces.CENTER;
//                } else if (maxPercentRed == rightAvgRed) {
//                    propPlacement = propPlaces.RIGHT;
//                }
//            } else if (robotStartPosition == robotStartPlaces.BLUE_HUMANPLAYER || robotStartPosition == robotStartPlaces.BLUE_BACKDROP){
//
//                Core.extractChannel(leftCrop, leftCrop,0);
//                Core.extractChannel(centerCrop, centerCrop,0);
//                Core.extractChannel(rightCrop, rightCrop,0);
//
//                Scalar leftAvg = Core.mean(leftCrop);
//                Scalar centerAvg = Core.mean(centerCrop);
//                Scalar rightAvg = Core.mean(rightCrop);
//
//                double leftAvgBlue = leftAvg.val[0];
//                double centerAvgBlue = centerAvg.val[0];
//                double rightAvgBlue = rightAvg.val[0];
//
//                double maxPercentBlue = Math.max(Math.max(leftAvgBlue, centerAvgBlue), rightAvgBlue);
//
//                if (maxPercentBlue == leftAvgBlue) {
//                    propPlacement = propPlaces.LEFT;
//                } else if (maxPercentBlue == centerAvgBlue) {
//                    propPlacement = propPlaces.CENTER;
//                } else if (maxPercentBlue == rightAvgBlue) {
//                    propPlacement = propPlaces.RIGHT;
//                }
//            }
//
//
//
//            Imgproc.rectangle(input, leftRect, new Scalar(255, 0, 0), 2);
//            Imgproc.rectangle(input, centerRect, new Scalar(0, 255, 0), 2);
//            Imgproc.rectangle(input, rightRect, new Scalar(0, 0, 255), 2);
//
//            return input;
//        }
//    }
//
//}
//
//
