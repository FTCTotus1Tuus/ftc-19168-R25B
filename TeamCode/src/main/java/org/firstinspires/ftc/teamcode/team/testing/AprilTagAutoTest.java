package org.firstinspires.ftc.teamcode.team.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.fsm.AprilTagStorageFSM;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;


@Autonomous(name = "AprilTagAutoTest", preselectTeleOp = "TeleopFSM")
@Configurable
public class AprilTagAutoTest extends DarienOpModeFSM {
    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    private int pathState;                      // State machine state
    private Timer pathTimer, opmodeTimer;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- ROBOT + HARDWARE INIT (from DarienOpModeFSM) ---
        initControls(); // sets up TrayServo, Elevator, Feeder, motors, AprilTag, etc.

        // --- PEDRO + TIMERS INIT ---
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Starting pose – same as your OpMode version
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.addLine("BlueGoalSidePedro: READY");
        telemetry.update();

        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();
        setPathState(0);

        // Set the initial tray position immediately.
        setTrayPosition(TRAY_POS_1_SCORE);

        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            // Drive the state machine
            pathState = autonomousPathUpdate();


            /*
            // Update tray servo FSM if running
            if (trayServoFSM.isRunning()) {
                trayServoFSM.update(getRuntime());
                if (!trayServoFSM.isRunning()) {
                    // Update current tray position when done
                    currentTrayPosition = targetTrayPosition;
                }
            }

             */

            // Panels/driver telemetry
            panelsTelemetry.addData("Tray Curr", currentTrayPosition);
            //panelsTelemetry.addData("Tray Targ", targetTrayPosition);
            panelsTelemetry.addData("Path State", pathState);
            panelsTelemetry.addData("X", follower.getPose().getX());
            panelsTelemetry.addData("Y", follower.getPose().getY());
            panelsTelemetry.addData("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);

            telemetry.update();
        }
    }



//67

    public int autonomousPathUpdate() {
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                //start reading april tags
                telemetry.addLine("Case " + pathState + ": Wait for Camera");
                AprilTagStorageFSM.reset();
                tagFSM.start(getRuntime());
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    setPathState(pathState + 1);
                }
                break;

            case 1:
                //once april tags done reading, move to shooting position 1
                telemetry.addLine("Case " + pathState + ":");

                tagFSM.update(getRuntime(), true, telemetry);

                if ((tagFSM.isDone()) || pathTimer.getElapsedTimeSeconds() > 4) {
                    aprilTagDetections = tagFSM.getDetections();
                    aprilTagDetections.removeIf(tag -> tag.id == 20 || tag.id == 24);

                    if (aprilTagDetections != null && !aprilTagDetections.isEmpty()) {
                        AprilTagStorageFSM.saveMotifTagId(aprilTagDetections.get(0).id);
                    } else {
                        // No tag found; keep storage cleared.
                        AprilTagStorageFSM.reset();
                    }

                    setPathState(pathState + 1);
                }
                break;

            case 2:
                //move to shooting position 1
                telemetry.addLine("Case " + pathState + "Read April Tag");

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2) {

                    setPathState(-1);
                }
                break;


            default:
                // -1 or any undefined state: do nothing, stay idle
                telemetry.addLine("Idle state (pathState = " + pathState + ")");
                break;
        }

        return pathState;
    }

    /**
     * Sets the path state and resets its timer.
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}