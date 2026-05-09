package org.firstinspires.ftc.teamcode.team.testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.team.ImageProcess;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class cameraDebugTest extends LinearOpMode {


    ImageProcessDebug imageProcessDebug;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        imageProcessDebug = new ImageProcessDebug();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(imageProcessDebug);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("error openCv", errorCode);
            }
        });


        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("cound", getRuntime());

                telemetry.addData("Is Red:", imageProcessDebug.getLastResults());
                telemetry.update();

                if (gamepad1.a) {
                    imageProcessDebug.setColor(true);
                } else if (gamepad1.b) {
                    imageProcessDebug.setColor(false);
                }
            }
        }
    }
}