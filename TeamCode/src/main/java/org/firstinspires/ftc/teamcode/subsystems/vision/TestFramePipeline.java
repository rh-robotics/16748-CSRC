package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestFramePipeline extends OpenCvPipeline {
    /* All image processing happens here. */
    @Override
    public Mat processFrame(Mat input) {
//        Imgproc.rectangle(
//                input,
//                new Point(
//                        input.cols()/4,
//                        input.rows()/4),
//                new Point(
//                        input.cols()*(3f/4f),
//                        input.rows()*(3f/4f)),
//                new Scalar(0, 255, 0), 4);

        return input;
    }
}