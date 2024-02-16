package org.firstinspires.ftc.teamcode.auton.colorSensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Team Prop Detection")

public class PropDetection extends OpMode {
    static int X = 145; //145
    static int Y = 168;
    static int W = 30;//30
    static int H = 50;//50

    public PropDetection(int boundX, int boundY, int width, int height) {
        X = boundX;
        Y = boundY;
        W = width;
        H = height;
    }

    @Override
    public void init() {
        telemetry.addData("Camera", "initializing");
    }

    @Override
    public void loop() {

    }

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }


    // TOPLEFT anchor point for the bounding box
    private static Point PROP_TOPLEFT_ANCHOR_POINT = new Point(X, Y);

    // Width and height for the bounding box
    public static int REGION_WIDTH = W;
    public static int REGION_HEIGHT = H;

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_blue_bounds = new Scalar(200, 200, 200, 255),
            upper_blue_bounds = new Scalar(255, 255, 255, 255),
            lower_red_bounds = new Scalar(0, 0, 0, 0),
            upper_red_bounds = new Scalar(100, 100, 100, 255);

    // Color definitions
    private final Scalar
            RED = new Scalar(255, 0, 0),
            BLUE = new Scalar(0, 0, 255);

    // Percent and mat definitions
    private double redPercent, bluePercent;
    private Mat redMat = new Mat(), blueMat = new Mat(), blurredMat = new Mat();

    // Anchor point definitions
    Point prop_pointA = new Point(
            PROP_TOPLEFT_ANCHOR_POINT.x,
            PROP_TOPLEFT_ANCHOR_POINT.y);
    Point prop_pointB = new Point(
            PROP_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            PROP_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    int position = 0;

    //@Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(prop_pointA, prop_pointB));

        // Apply Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_red_bounds, upper_red_bounds, redMat);
        Core.inRange(blurredMat, lower_blue_bounds, upper_blue_bounds, blueMat);

        // Gets color specific values
        redPercent = Core.countNonZero(redMat);
        bluePercent = Core.countNonZero(blueMat);


        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(redPercent, bluePercent);

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected

        if (maxPercent == redPercent) {
            position = 2;
            Imgproc.rectangle(
                    input,
                    prop_pointA,
                    prop_pointB,
                    RED,
                    2
            );
            telemetry.addData("Color", "red");
        } else if (maxPercent == bluePercent) {
            position = 1;
            Imgproc.rectangle(
                    input,
                    prop_pointA,
                    prop_pointB,
                    BLUE,
                    2
            );

            telemetry.addData("Color", "blue");
        }
        return input;
    }
    // Memory cleanup
        /*blurredMat.release();
        redMat.release();
        blueMat.release();*/

    //return input;
}


// Returns an enum being the current position where the robot will park
//public int getPosition() {
//   return position;
//}
//}
