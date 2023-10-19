package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Random;

/* All image processing happens here. */
public class IntakeDetectionPipeline extends OpenCvPipeline {
    public Mat srcGray;
    private Mat cannyOutput;
    private Mat hierarchy;
    private Mat contoursPoly;
    private ArrayList<MatOfPoint> contours;
    public int threshold;
    @Override
    public Mat processFrame(Mat src) {
        int threshold = 255;
        Random random = new Random();

        /* Create Grayscale version of Mat. */
        srcGray = new Mat();
        Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);
        /* Reduces noise using normalized box filter, which sets each section
         * value to the average of all the pixels under the kernel area. */
        Imgproc.blur(srcGray, srcGray, new Size(3, 3));

        /* Outlines of objects. */
        Mat cannyOutput = new Mat();
        Imgproc.Canny(srcGray, cannyOutput, threshold/3f, threshold);

        /* Creates ArrayList and assigns value to hold contours. */
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        /* Creates list of Ellipses with minimum area to contain contours. */
        RotatedRect[] minEllipse = new RotatedRect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            if (contours.get(i).rows() > 5) {
                /* fitEllipse returns the rotated rectangle that best fits 2 points
                * (least total area) in which the ellipse is inscribed. */
                minEllipse[i] = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(i).toArray()));
            }
        }
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(random.nextInt(256), random.nextInt(256), random.nextInt(256));
            // contour
            Imgproc.drawContours(src, contours, i, color);
            // ellipse
            Imgproc.ellipse(src, minEllipse[i], color, 2);
        }

        return src;
    }
}