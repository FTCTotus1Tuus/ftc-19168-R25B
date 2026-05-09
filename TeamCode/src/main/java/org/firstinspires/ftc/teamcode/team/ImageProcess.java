package org.firstinspires.ftc.teamcode.team;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class ImageProcess implements VisionProcessor {
    private Mat workingMat1 = new Mat(), workingMat2 = new Mat(), workingMatPurple = new Mat(), workingMatGreen = new Mat();
    public static int minHueP = 140, minSaturationP = 80, minValueP = 80, maxHueP = 170, maxSaturationP = 255, maxValueP = 255,
            minHueG = 40, minSaturationG = 15, minValueG = 20, maxHueG = 90, maxSaturationG = 255, maxValueG = 255, frameWidth, frameHeight;
    private double PurpleCount, GreenCount;
    private boolean lastResult;

    public boolean getLastResult() {
        return lastResult;
    }


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        workingMat1 = frame.clone();

        Imgproc.cvtColor(workingMat1, workingMat2, Imgproc.COLOR_RGB2HSV);
        Core.inRange(workingMat2, new Scalar(minHueP, minSaturationP, minValueP), new Scalar(maxHueP, maxSaturationP, maxValueP), workingMatPurple);
        Core.inRange(workingMat2, new Scalar(minHueG, minSaturationG, minValueG), new Scalar(maxHueG, maxSaturationG, maxValueG), workingMatGreen);

        PurpleCount = Core.countNonZero(workingMatPurple);
        GreenCount = Core.countNonZero(workingMatGreen);

        lastResult = PurpleCount > GreenCount;

        workingMat1.release();
        workingMat2.release();
        workingMatPurple.release();
        workingMatGreen.release();

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}