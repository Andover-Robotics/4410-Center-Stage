package org.firstinspires.ftc.teamcode.auto.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;

public class ColorDetectionPipeline2 extends OpenCvPipeline {
    // VARIABLES
    Telemetry telemetry;

    // Processing frames
    private final Mat matYCrCb = new Mat();

    // Cb and Cr predetermined Mats
    private final Mat matCbCenter = new Mat();
    private final Mat matCbLeft = new Mat();
    private final Mat matCrCenter = new Mat();
    private final Mat matCrLeft = new Mat();

    // Average Cb and Cr values
    public double avgCenter = 0, avgLeft = 0;
    public static double minimumAvg = 50;

    // Configurations
    enum SpikeMark{
        LEFT, MIDDLE, RIGHT, NONE
    }
    public SpikeMark spikeMark = SpikeMark.NONE;
    public static int alliance = 0; // Alliance: 0 - NONE, 1 - RED, 2 - BLUE
    public void setAlliance(int alliance) {
        ColorDetectionPipeline2.alliance = alliance;
    }

    // CONSTRUCTOR
    public ColorDetectionPipeline2(Telemetry telemetry){
        spikeMark = SpikeMark.NONE;
        this.telemetry = telemetry;
    }

    // Execute logic
    public int getSpikeMark() {
        if (getAvgLeft() < minimumAvg && getAvgCenter() < minimumAvg) { // Right, both avgs is too small
            return 3;
        } else if (getAvgCenter() < getAvgLeft()) { // Left, left avg greater than center avg
            return 1;
        } else if (getAvgLeft() < getAvgCenter()) { // Center, center avg greater than left avg
            return 2;
        }
        return 3;
    }

    // PROCESSING FRAMES
    public Mat processFrame(Mat input) {
        // Convert from RGB to YCrCb
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Draw rectangles of left and center positions
        Rect rectCenter = new Rect(600, 50, 680, 240);
        Rect rectLeft = new Rect(100, 0, 325, 400);
        // Display rectangles
        Imgproc.rectangle(input, rectCenter, new Scalar(0, 255, 0), 5);
        Imgproc.rectangle(input, rectLeft, new Scalar(0, 255, 0), 5);

        // Create mats for each rectangle
        Mat spikeCenter = matYCrCb.submat(rectCenter);
        Mat spikeLeft = matYCrCb.submat(rectLeft);

        // Extract color based off of alliance
        if (alliance == 1) { // Blue
            Core.extractChannel(spikeCenter, matCbCenter, 1);
            Core.extractChannel(spikeLeft, matCbLeft, 1);
            // Calculate average
            Scalar meanCenter = Core.mean(matCbCenter);
            Scalar meanLeft = Core.mean(matCbLeft);
            avgCenter = meanCenter.val[0];
            avgLeft = meanLeft.val[0];
        } else if (alliance == 2) { // Red
            Core.extractChannel(spikeCenter, matCrCenter, 2);
            Core.extractChannel(spikeLeft, matCrLeft, 2);
            // Calculate average
            Scalar meanCenter = Core.mean(matCrCenter);
            Scalar meanLeft = Core.mean(matCrLeft);
            avgCenter = meanCenter.val[0];
            avgLeft = meanLeft.val[0];
        }

        // Return img frame
        return input;
    }

    public double getAvgCenter() { return avgCenter; }
    public double getAvgLeft() { return avgLeft; }

    public void setMinAvg(double minimumAvg) { ColorDetectionPipeline2.minimumAvg = minimumAvg; }
    public double getMinAvg() { return minimumAvg; }
}
