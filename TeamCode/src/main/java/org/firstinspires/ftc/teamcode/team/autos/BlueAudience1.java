package org.firstinspires.ftc.teamcode.team.autos;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "Blue Audience 1", group = "Blues", preselectTeleOp = "Teleop")
@Config
public class BlueAudience1 extends DarienOpModeAuto {
    ArrayList<AprilTagDetection> Motif;
    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        waitForStart();
        if (isStopRequested()) return;

        //displayTrayTelemetry();

        //move to the desired position
        moveXY(70, 0, AUTO_MOVE_POWER);
        Elevator.setPosition(ELEVATOR_POS_DOWN);
        Feeder.setPosition(FEEDER_POS_DOWN);
        servoIncremental(TrayServo,TRAY_POS_2_SCORE,currentTrayPosition, 1,4);
        //TrayServo.setPosition(TRAY_POS_2_SCORE);
        currentTrayPosition = TRAY_POS_2_SCORE;
        sleep(500);

        //read obelisk
        Motif = readAprilTagSequence();
        waitForMotors(true);
        encoderRotate(Math.toRadians(-50), AUTO_ROTATATE_POWER, true);
        waitForMotors(true);
        moveXY(10, 0, AUTO_MOVE_POWER);
        waitForMotors(true);

        // ASSUMES THAT GREEN IS PRELOADED IN POSITION 2
        shootApriltagSequence(Motif);

        //move to pickup mid
        moveXY(-10, 0, AUTO_MOVE_POWER);
        waitForMotors(true);
        encoderRotate(Math.toRadians(55), AUTO_ROTATATE_POWER, true);
        waitForMotors(true);
        moveXY(-40, 0, AUTO_MOVE_POWER);
        waitForMotors(true);
        encoderRotate(Math.toRadians(90), AUTO_ROTATATE_POWER, true);
        waitForMotors(true);
    }

    private void displayTrayTelemetry() {
        tp.put("currentTrayPosition",currentTrayPosition);
        tp.put("currentTime",getRuntime());
        dash.sendTelemetryPacket(tp);
    }

}
