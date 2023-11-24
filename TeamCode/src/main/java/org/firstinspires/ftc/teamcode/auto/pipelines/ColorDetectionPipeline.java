package org.firstinspires.ftc.teamcode.auto.pipelines;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.firstinspires.ftc.teamcode.auto.MainAutonomous;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;

/*
TODO: FIX WHATEVER MESS OF CODE THIS IS SOME OTHER TIME AFTER SCRIMMAGE - ZACHERY
 */

public class ColorDetectionPipeline extends OpenCvPipeline{

    Telemetry telemetry;
    Mat HSV = new Mat();
    MatOfPoint biggest;

    public static int minheight = 100;
    public static int minwidth = 100;
    public static int width = 0;
    public static int height = 0;
    public static int camwidth = 1280;
    public static int camheight = 720;

    public static double midpointrect;

    public enum SpikeMark{
        LEFT,
        MIDDLE,
        RIGHT,
        NOTDETECTED
    }
    public static SpikeMark spikeMark = SpikeMark.NOTDETECTED;

    // Alliance: 0 - NONE, 1 - RED, 2 - BLUE
    int alliance = 0;

    // Red HSV Values
    public static double redLH = 0, redLS = 50, redLV = 50, redHH = 5, redHS = 255, redHV = 255, redLH2 = 165, redHH2 = 180;
    public static Scalar redLowHSV= new Scalar(redLH,redLS,redLV);
    public static Scalar redHighHSV = new Scalar(redHH,redHS,redHV);
    public static Scalar redLowHSV2= new Scalar(redLH2,redLS,redLV);
    public static Scalar redHighHSV2 = new Scalar(redHH2,redHS,redHV);

    // Blue HSV Values
    public static double blueLH = 90, blueLS = 50, blueLV = 50, blueHH = 117, blueHS = 255, blueHV = 255;
    public static Scalar blueLowHSV= new Scalar(blueLH,blueLS,blueLV);
    public static Scalar blueHighHSV = new Scalar(blueHH,blueHS,blueHV);

    public ColorDetectionPipeline(Telemetry telemetry){ // CONSTRUCTOR :D
        spikeMark = SpikeMark.NOTDETECTED;
        this.telemetry = telemetry;
    }

    public void setAlliance(int alliance) {
        this.alliance = alliance;
    }

    public int getSpikeMark() {
        switch (spikeMark) {
            case LEFT: return 1;
            case MIDDLE: return 2;
            case RIGHT: return 3;
        }
        return 0; // None detected
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV); //converting RGB colors to HSV

        Rect rightrect = new Rect(600, 50, 680, 240);
        Rect leftrect = new Rect(100, 0, 325, 400); // rectangle sizes

        Imgproc.rectangle(input, leftrect, new Scalar(0, 255, 0), 5); //displays rectangles with red color
        Imgproc.rectangle(input, rightrect, new Scalar(0, 255, 0), 5);

        // filters HSV mat into image with black being the lowest red/blue HSV and white being the highest red/blue HSV
        if (alliance == 1) {
            Core.inRange(HSV, redLowHSV, redHighHSV, HSV);
            //Core.inRange(HSV, redLowHSV2, redHighHSV2, HSV);
        } else {
            Core.inRange(HSV, blueLowHSV, blueHighHSV, HSV);
        }

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(HSV, contours, new Mat(), 0,1); // finds contours in HSV mat

        if (!contours.isEmpty()) { // checks if no contours are found
            contours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width))); //orders contours in array from big to small(by width)

            // Make rectangle
            int iterations = 0;
            biggest = contours.get(0); //contour with the largest width(first in the array)
            Rect rect = Imgproc.boundingRect(biggest); // turns biggest contour into a rectangle
            midpointrect = rect.tl().x + rect.width/2.0; // gets midpoint x of the rectangle
            width = rect.width;
            height = rect.height;
            while (height < 50) {
                if (iterations < contours.size()) {
                    biggest = contours.get(iterations);
                    rect = Imgproc.boundingRect(biggest); // turns biggest contour into a rectangle
                    midpointrect = rect.tl().x + rect.width / 2.0; // gets midpoint x of the rectangle
                    width = rect.width;
                    height = rect.height;
                    iterations++;
                } else {
                    iterations = 0;
                    spikeMark = SpikeMark.RIGHT;
                    break;
                }
            }

            // Draw rectangle on screen
            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 6); // puts border around contours with a green shade

            // Check which zone it is in (left or right)
            if (midpointrect > leftrect.tl().x && midpointrect < leftrect.br().x) { // LEFT SPIKE
                if (height < 200) { // make sure it is not left tape
                    spikeMark = SpikeMark.RIGHT;
                } else {
                    spikeMark = SpikeMark.LEFT;
                }
            } else if (midpointrect > rightrect.tl().x && midpointrect < leftrect.br().x) {
                if (width < 50 || height < 50) {
                    spikeMark = SpikeMark.RIGHT;
                } else {
                    spikeMark = SpikeMark.MIDDLE;
                }

            } else {
                if (width < 50 || height < 50) {
                    spikeMark = SpikeMark.RIGHT;
                } else {
                    spikeMark = SpikeMark.MIDDLE;
                }
            }

            //telemetry.addLine("Midpoint of Bounding Box :"+ midpointrect);
        } else { // NOT IN FRAME, RIGHT SPIKE
            spikeMark = SpikeMark.RIGHT;
        }
        //telemetry.addData("contours: ", contours.size());
        // telemetry.addData("Spikemark status: ",spikeMark);
        // Releasing all our mats for the next iteration
        HSV.release();
        return input; // return end frame with rectangles drawn
    }
}