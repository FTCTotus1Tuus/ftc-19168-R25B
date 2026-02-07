package org.firstinspires.ftc.teamcode.team.autosPedroPathing;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import android.content.SharedPreferences;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Pedro Pathing auto using LinearOpMode via DarienOpModeFSM.
 */

@Autonomous(name = "Blue Goal 6", group = "Pedro:Blues", preselectTeleOp = "TeleopFSM")
@Config
@Configurable
public class BlueGoalSide1 extends DarienOpModeFSM {

    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;

    public static double PATH_POWER_STANDARD = 0.9;
    public static double PATH_POWER_SLOW = 0.3;
    //public static double SHOT_GUN_POWER_UP = 0.6*.9;

    public static double INTAKE_RUBBER_BANDS_DELAY = 0.2;
    public static double BALL_INTAKE_DELAY = 1.5;
    public static double SHOTGUN_SPINUP_DELAY = 1.0;
    public static double STANDARD_PATH_TIMEOUT = 2.0;
    public static double SHOOT_TRIPLE_TIMEOUT = 5.0;
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
        follower.setStartingPose(new Pose(20.286, 124.378, Math.toRadians(54)));

        // Build all the paths once
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.addLine("BlueGoalSidePedro: READY");
        telemetry.update();

        // Save alliance color to shared preferences for TeleOp
        SharedPreferences prefs = AppUtil.getInstance().getActivity().getSharedPreferences("ftc_prefs", android.content.Context.MODE_PRIVATE);
        prefs.edit().putString("auto_alliance", "BLUE").apply();

        telemetry.addLine("Alliance Color: BLUE (Saved to Preferences)");


        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();
        setPathState(0);

        targetGoalId = APRILTAG_ID_GOAL_BLUE;
        // Set the initial tray position immediately.
        TrayServo.setPosition(TRAY_POS_1_SCORE);
        //shotgunFSM.toPowerUp(SHOT_GUN_POWER_UP_RPM);

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
            panelsTelemetry.addData("Alliance Color", "BLUE");
            panelsTelemetry.update(telemetry);
            telemetry.addData("Alliance Color Saved", "BLUE");

            telemetry.update();
        }
    }

    /**
     * Inner class defining all the Pedro paths.
     */
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        // 67
        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(20.286, 124.378),
                                    new Pose(47.224, 96.443))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(70))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.224, 96.443, Math.toRadians(70)), new Pose(47.234, 96.443, Math.toRadians(145)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(145))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.234, 96.443), new Pose(43.5, 84.305))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.5, 84.305), new Pose(36, 84.305)) //intake 1
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36, 84.305), new Pose(31, 84.305)) //intake 2
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(31, 84.305), new Pose(26, 84.305)) //intake 3
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(26, 84.305),
                                    new Pose(48.222, 97.613),
                                    new Pose(59.235, 116.63499420625725)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.224, 96.443), new Pose(47.557, 122.383))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    /**
     * State machine for autonomous sequence.
     * Returns current pathState for logging.
     */
    public int autonomousPathUpdate() {

        // Helpful debug info every loop
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                telemetry.addLine("Case " + pathState + ": Start Path1");

                // Set the initial tray position
                TrayServo.setPosition(TRAY_POS_1_SCORE);
                follower.setMaxPower(PATH_POWER_STANDARD);
                shootArtifactFSM.shotGun(SHOT_GUN_POWER_UP);
                follower.followPath(paths.Path1);
                setPathState(pathState + 1);
                break;

            case 1:
                telemetry.addLine("Case " + pathState + ": Wait for Path1 and camera, then start read AprilTag");

                if ((!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT)) {

                    telemetry.addLine("Case " + pathState + ": exiting");
                    // Start AprilTag reading after path1 is done
                    tagFSM.start(getRuntime());

                    setPathState(pathState + 1);
                }
                break;

            case 2:
                telemetry.addLine("Case " + pathState + ": Read AprilTag then start Path2");

                tagFSM.update(getRuntime(), true, telemetry);

                if ((tagFSM.isDone()) || pathTimer.getElapsedTimeSeconds() > TIMEOUT_APRILTAG_DETECTION) {
                    aprilTagDetections = tagFSM.getDetections();

                    telemetry.addLine("Case " + pathState + ": exiting");
                    follower.followPath(paths.Path2);
                    setPathState(pathState + 1);
                }
                break;

            case 3:
                telemetry.addLine("Case " + pathState + ": Wait for Path2, then shoot artifact");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    setPathState(pathState + 1);
                }
                break;

            case 4:
                telemetry.addLine("Case " + pathState + ": start shooting");

                shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP);

                if (pathTimer.getElapsedTimeSeconds() > SHOTGUN_SPINUP_DELAY) {
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                telemetry.addLine("Case " + pathState + ": updateShooting...");
                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > SHOOT_TRIPLE_TIMEOUT) {

                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                    topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
                    TrayServo.setPosition(TRAY_POS_2_INTAKE);

                    // now continue with next path
                    follower.followPath(paths.Path3, true);
                    setPathState(pathState + 1);
                }
                break;

            case 6:
                telemetry.addLine("Case " + pathState + ": Wait for Path3, then start Path4");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 1p");

                    follower.setMaxPower(PATH_POWER_SLOW); //slow down for pickup

                    follower.followPath(paths.Path4, true);
                    setPathState(pathState + 1);
                }
                break;

            case 7:
                telemetry.addLine("Case " + pathState + ": Wait for Path4");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > BALL_INTAKE_DELAY) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 2p");

                    /*
                    rubberBands.setPower(0);
                    topIntake.setPower(0);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);

                     */
                    TrayServo.setPosition(TRAY_POS_1_INTAKE);

                    setPathState(pathState + 1);
                }
                break;

            case 8:
                //wait for tray to rotate before next pickup
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_RUBBER_BANDS_DELAY) {
                    /*
                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                    topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);

                     */
                    follower.followPath(paths.Path5, true);
                    setPathState(pathState + 1);
                }
                break;

            case 9:
                telemetry.addLine("Case " + pathState + ": Wait for Path5");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > BALL_INTAKE_DELAY) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 3g");
                    /*
                    rubberBands.setPower(0);
                    topIntake.setPower(0);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);

                     */
                    TrayServo.setPosition(TRAY_POS_3_INTAKE);
                    setPathState(pathState + 1);
                }
                break;

            case 10:
                //wait for tray to rotate before next pickup
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_RUBBER_BANDS_DELAY) {
                    /*
                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                    topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);

                     */
                    follower.followPath(paths.Path6, true);
                    setPathState(pathState + 1);
                }
                break;

            case 11:
                telemetry.addLine("Case " + pathState + ": Wait for Path6 to pick up artifact, then start Path7");

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > BALL_INTAKE_DELAY) {
                    telemetry.addLine("Case " + pathState + ": Moving to shooting position");
                    follower.setMaxPower(PATH_POWER_STANDARD);// resume normal speed

                    follower.followPath(paths.Path7, true);
                    TrayServo.setPosition(TRAY_POS_2_SCORE);
                    rubberBands.setPower(0);
                    topIntake.setPower(0);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    shootArtifactFSM.shotGun(SHOT_GUN_POWER_UP);
                    setPathState(pathState + 1);
                }
                break;

            case 12:
                telemetry.addLine("Case " + pathState + ": Wait for Path7 to get into position, then start Path8");

                //shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP); // 31/32 power
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > SHOTGUN_SPINUP_DELAY) {
                    telemetry.addLine("Case " + pathState + ": Shoot the pattern");

                    shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP);
                    setPathState(pathState + 1);
                }
                break;

            case 13:
                telemetry.addLine("Case " + pathState + ": Wait for Path6 to finish, then stop");
                shootPatternFSM.updateShootPattern(getRuntime());
                if (shootPatternFSM.isShootPatternDone()) {
                    telemetry.addLine("Case " + pathState + ": Done, setting state -1");
                    //rubberBands.setPower(0);
                    setPathState(-1); // done
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