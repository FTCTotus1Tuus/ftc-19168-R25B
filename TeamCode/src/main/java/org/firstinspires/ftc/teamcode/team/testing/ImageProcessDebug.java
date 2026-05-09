package org.firstinspires.ftc.teamcode.team.testing;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class ImageProcessDebug extends OpenCvPipeline {
    private Mat workingMat1 = new Mat(), workingMat2 = new Mat(), workingMatPurple = new Mat(), workingMatGreen = new Mat();
    public static int minHueP = 140, minSaturationP = 80, minValueP = 80, maxHueP = 170, maxSaturationP = 255, maxValueP = 255,
            minHueG = 40, minSaturationG = 15, minValueG = 20, maxHueG = 90, maxSaturationG = 255, maxValueG = 255, frameWidth, frameHeight;
    private double purpleCount, greenCount;
    private boolean lastResult;
    public static boolean showPurple = true;

    public boolean getLastResults() {
        return lastResult;
    }


    public void setColor(boolean isRed) {
        showPurple = isRed;
    }

    @Override
    public Mat processFrame(Mat frame) {
        workingMat1.release();
        workingMat2.release();
        workingMatPurple.release();
        workingMatGreen.release();


        workingMat1 = frame.clone();

        Imgproc.cvtColor(workingMat1, workingMat2, Imgproc.COLOR_RGB2HSV);
        Core.inRange(workingMat2, new Scalar(minHueP, minSaturationP, minValueP), new Scalar(maxHueP, maxSaturationP, maxValueP), workingMatPurple);
        Core.inRange(workingMat2, new Scalar(minHueG, minSaturationG, minValueG), new Scalar(maxHueG, maxSaturationG, maxValueG), workingMatGreen);

        purpleCount = Core.countNonZero(workingMatPurple);
        greenCount = Core.countNonZero(workingMatGreen);

        lastResult = purpleCount > greenCount;


        if (showPurple) {
            return workingMatPurple;
        } else {
            return workingMatGreen;
        }
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}