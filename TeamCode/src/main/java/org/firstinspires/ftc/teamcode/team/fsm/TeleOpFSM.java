package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import android.content.SharedPreferences;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@TeleOp(name = "TeleopFSM", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM extends DarienOpModeFSM {

    // INSTANCES
    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    private GoBildaPinpointDriver odo;          // Pinpoint odometry driver for position reset

    // TUNING CONSTANTS
    public static double INTAKE_TIME = 1;
    public static double SHOT_TIMEOUT = 2.0; // seconds
    public static double ROTATION_SCALE = 0.5;

    // VARIABLES
    private double shotStartTime;
    private boolean shotStarted = false;
    private boolean isReadingAprilTag = false;
    private double tripleShotStartTime;
    private boolean tripleShotStarted = false;

    // Track previous bumper state for edge detection
    private boolean prevRightBumper1 = false;
    //private boolean prevRightBumper2 = false;
    private boolean prevBackButton = false;
    private ShotgunPowerLevel shotgunPowerLatch = ShotgunPowerLevel.LOW;

    private enum TurretStates {MANUAL, CAMERA, ODOMETRY}
    private TurretStates turretState = TurretStates.MANUAL;

    private enum ShootingPowerModes {MANUAL, ODOMETRY}

    private ShootingPowerModes shootingPowerMode = ShootingPowerModes.MANUAL;

    // Turret fallback tracking
    private double lastCameraDetectionTime = 0;  // Timestamp of last successful camera detection

    // AUTOMATIC TURRET CONTROLS BASED ON CAMERA APRILTAG DETECTION
    AprilTagDetection detection;
    double yaw, range; // Stores detection.ftcPose.yaw
    double currentHeadingDeg;
    double relativeHeadingDeg; // Camera-relative bearing to AprilTag (degrees)
    double targetServoPos = Double.NaN; // Convert heading → servo position
    double rawBearingDeg; // Stores detection.ftcPose.bearing;

    double robotX, robotY, robotHeadingRadians;

    // cameraOffsetX < 0 if camera is mounted on the LEFT
   // public static double cameraOffsetX = 0.105; // in centimeter, positive is right, negative is left
    double correctedBearingRad;
    double correctedBearingDeg;
    boolean isCalculatingTurretTargetPosition = false;

    int targetGoalTagId;
    double turretOffset;
    private String autoAlliance = "UNKNOWN";

    // PIDF Tuning values for ejection motor
    /*
    public static double NEW_P = 0.1;
    public static double NEW_I = 0.1;
    public static double NEW_D = 0;
    public static double NEW_F = 0;

     */

    //private clamp test
    private static double clampT(double v, double min, double max) {
        if (Double.isNaN(v)) return min;               // defensive: treat NaN as min
        if (min > max) {                               // tolerate inverted bounds
            double t = min; min = max; max = t;
        }
        return Math.max(min, Math.min(max, v));
    }

    @Override
    public void initControls() {
        super.initControls();
        follower = Constants.createFollower(hardwareMap);

        // Initialize GoBildaPinpointDriver for odometry position reset
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Initialize rubber bands to off
        rubberBands.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        float gain = 2;
        initControls();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        SharedPreferences prefs = AppUtil.getInstance().getActivity().getSharedPreferences("ftc_prefs", android.content.Context.MODE_PRIVATE);
        autoAlliance = prefs.getString("auto_alliance", "UNKNOWN");

        // Set align color based on saved color from auto
        if ("BLUE".equals(autoAlliance)) {
            targetGoalTagId = APRILTAG_ID_GOAL_BLUE;
            turretOffset = TURRET_OFFSET_BLUE;
        } else if ("RED".equals(autoAlliance)) {
            targetGoalTagId = APRILTAG_ID_GOAL_RED;
            turretOffset = TURRET_OFFSET_RED;
        }

        waitForStart();
        if (isStopRequested()) return;
        //Start
        follower.startTeleopDrive(true);
        follower.update();

        //PIDFCoefficients pidfOrig = ejectionMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        //PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        //ejectionMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        // Re-read coefficients and verify change.
        //PIDFCoefficients pidfModified = ejectionMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);


        while (this.opModeIsActive() && !isStopRequested()) {

            // -----------------
            // ALWAYS RUN
            // -----------------
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * ROTATION_SCALE, true);
            follower.update();
            leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
            rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);

            if (turretState == TurretStates.CAMERA) {
                if (!isReadingAprilTag) {
                    startReadingGoalId();
                } else {
                    updateReadingGoalId();
                }
            }

            // -----------------
            // GAMEPAD1 CONTROLS
            // -----------------

            //RubberBands + topIntake CONTROLS:
            if (gamepad1.y) {
                // Intake mode
                rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                //leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                //rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
            } else if (gamepad1.a) {
                // Eject mode
                rubberBands.setPower(-INTAKE_RUBBER_BANDS_POWER);
                topIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
                //leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                //rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
            } else if (gamepad1.x) {
                rubberBands.setPower(0);
                topIntake.setPower(0);
                //leftIntake.setPower(0);
                //rightIntake.setPower(0);
            }

            // Toggle auto-intake on right bumper press (edge triggered)
            if (gamepad1.right_bumper && !prevRightBumper1) {
                // toggle the TrayFSM instance (from DarienOpModeFSM)
                trayFSM.toggleAutoIntake();
            }
            prevRightBumper1 = gamepad1.right_bumper;

            // Show current auto-intake status on telemetry
            telemetry.addData("AutoIntakeRunning", trayFSM != null && trayFSM.isAutoIntakeRunning());

            // Update trayFSM state machine each loop so it can run when toggled on
            trayFSM.update();
            if (!trayFSM.isAutoIntakeRunning() && shotgunPowerLatch != ShotgunPowerLevel.OFF) {
                setLedGreen();
            }
            // ODOMETRY RESET BUTTON - Reset to human player starting position
            if (gamepad1.backWasPressed()) {
                // Determine which human player position based on alliance color
                double resetX = 0, resetY = 0, resetHdeg = 0;
                if ("RED".equals(autoAlliance)) {
                    resetX = HUMAN_PLAYER_RED_X + ROBOT_CENTER_OFFSET_X;
                    resetY = HUMAN_PLAYER_RED_Y + ROBOT_CENTER_OFFSET_Y;
                    resetHdeg = 180; // Front-first into red corner: robot drives straight into red wall (-X direction)
                    telemetry.addLine("ODOMETRY RESET: Red Human Player Position (0, 0)");
                } else if ("BLUE".equals(autoAlliance)) {
                    resetX = HUMAN_PLAYER_BLUE_X - ROBOT_CENTER_OFFSET_X;
                    resetY = HUMAN_PLAYER_BLUE_Y + ROBOT_CENTER_OFFSET_Y;
                    resetHdeg = 0; // Front-first into blue corner: robot drives straight into blue wall (+X direction)
                    telemetry.addLine("ODOMETRY RESET: Blue Human Player Position (144, 0)");
                } /*else {
                    // Default to (0,0) if alliance unknown
                    resetX = 0;
                    resetY = 0;
                    telemetry.addLine("ODOMETRY RESET: Default Position (0, 0)");
                } */

                // Reset the pinpoint odometry position
                odo.setPosition(new Pose2D(
                        DistanceUnit.INCH,
                        resetX,
                        resetY,
                        AngleUnit.DEGREES,
                        resetHdeg
                ));

                // Update the follower's pose to match
                follower.setPose(new Pose(resetX, resetY, Math.toRadians(resetHdeg)));

                telemetry.addData("New Odometry Position", String.format("(%.1f, %.1f, %.1f)", resetX, resetY, resetHdeg));
            }


            // -----------------
            // GAMEPAD2 CONTROLS
            // -----------------

            //SET ALLIANCE COLOR CONTROL
            if (gamepad2.b && !isReadingAprilTag) {
                // ALIGN TO RED GOAL
                autoAlliance = "RED";
                targetGoalTagId = APRILTAG_ID_GOAL_RED;
                telemetry.addLine("ALLIANCE SET TO RED!");
            } else if (gamepad2.x && !isReadingAprilTag) {
                // ALIGN TO BLUE GOAL
                autoAlliance = "BLUE";
                targetGoalTagId = APRILTAG_ID_GOAL_BLUE;
                telemetry.addLine("ALLIANCE SET TO BLUE!");
            }

            if (!shotStarted && !tripleShotStarted) {
                // -----------------
                // MANUAL CONTROLS: only allowed when not running macros
                // -----------------

                //CONTROL: ELEVATOR
                if (gamepad2.left_bumper) {
                    Elevator.setPosition(ELEVATOR_POS_UP);
                } else {
                    Elevator.setPosition(ELEVATOR_POS_DOWN);
                }
                // CONTROL: ROTATING TRAY USING FSM
                if (gamepad2.dpad_left) {
                    setTrayPosition(TRAY_POS_1_SCORE);
                } else if (gamepad2.dpad_up) {
                    setTrayPosition(TRAY_POS_2_SCORE);
                } else if (gamepad2.dpad_right) {
                    setTrayPosition(TRAY_POS_3_SCORE);
                }

                /*
                // DRIVER 1 MANUAL CONTROLS FOR INTAKE TRAY POSITIONS
                if (gamepad1.dpad_left) {
                    setTrayPosition(TRAY_POS_1_INTAKE);
                } else if (gamepad1.dpad_up) {
                    setTrayPosition(TRAY_POS_2_INTAKE);
                } else if (gamepad1.dpad_right) {
                    setTrayPosition(TRAY_POS_3_INTAKE);
                }
                 */

                // -----------------
                // IMPORTANT: ALWAYS PUT MACRO CONTROLS AFTER MANUAL CONTROLS
                // -----------------

                //CONTROL: START SHOTGUN MACRO USING FSM
                if (gamepad2.dpad_down && gamepad2.right_stick_y < -0.05) {
                    shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP_FAR);
                    shotStartTime = getRuntime();
                    shotStarted = true;
                } else if (gamepad2.dpad_down) {
                    shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP);
                    shotStartTime = getRuntime();
                    shotStarted = true;
                }

                // CONTROL: START TRIPLE SHOT MACRO USING FSM
                // Toggle auto-intake on right bumper press (edge triggered)
                /*
                if (gamepad1.right_bumper && !prevRightBumper1) {
                    // toggle the TrayFSM instance (from DarienOpModeFSM)
                    ShootTripleFSM.toggle();
                }
                prevRightBumper2 = gamepad2.right_bumper;

                 */

                // Edge-triggered start: press right bumper to start triple shoot, Triple shot
                if (gamepad2.right_bumper && gamepad2.right_stick_y < -0.05) {
                    //stop auto intake if running
                    if (trayFSM.isAutoIntakeRunning()) {
                        trayFSM.toggleAutoIntake();
                    }
                    shootTripleFSM.startShootTriple(getRuntime(), SHOT_GUN_POWER_UP_FAR);
                    tripleShotStartTime = getRuntime();
                    tripleShotStarted = true;
                } else if (gamepad2.right_bumper) {
                    //stop auto intake if running
                    if (trayFSM.isAutoIntakeRunning()) {
                        trayFSM.toggleAutoIntake();
                    }
                    shootTripleFSM.startShootTriple(getRuntime(), SHOT_GUN_POWER_UP);
                    tripleShotStartTime = getRuntime();
                    tripleShotStarted = true;
                }

                //TURRET STATE CHANGE CONTROLS
                if (gamepad2.left_trigger > 0.1) {
                    turretState = TurretStates.ODOMETRY;
                    shootingPowerMode = ShootingPowerModes.ODOMETRY;
                }

                if (gamepad2.right_trigger > 0.1) {
                    turretState = TurretStates.CAMERA;
                    startReadingGoalId();
                }

                telemetry.addData("shotStarted", shotStarted);
                telemetry.addData("tripleShotStarted", tripleShotStarted);


            } //manual controls
            else {
                // -----------------
                // MACRO CONTROLS
                // -----------------

                //CONTROL: SHOTGUN MACRO
                if (shotStarted)  {
                    // ONLY UPDATE IF IN MACRO CONTROL MODE
                    shootArtifactFSM.updateShooting();
                    if (shootArtifactFSM.shootingDone() || getRuntime() - shotStartTime >= SHOT_TIMEOUT) {
                        shootArtifactFSM.resetShooting();
                        shotStarted = false;
                    }
                }

                // CONTROL: TRIPLE SHOT MACRO
                else if (tripleShotStarted) {
                    // ONLY UPDATE IF IN MACRO CONTROL MODE
                    shootTripleFSM.updateShootTriple(getRuntime());
                    if (shootTripleFSM.isDone() || getRuntime() - tripleShotStartTime >= 10) {
                        setLedOff();
                        TrayServo.setPosition(DarienOpModeFSM.TRAY_POS_3_INTAKE);
                        tripleShotStarted = false;
                    }
                }

                /*
                telemetry.addData("shotStarted", shotStarted);
                telemetry.addData("shootingStage", shootArtifactFSM.getShootingStage());
                telemetry.addData("shootingDone", shootArtifactFSM.shootingDone());
                telemetry.addData("Elevator Pos", Elevator.getPosition());
                telemetry.addData("Ejection L", ejectionMotorLeft.getPower());
                telemetry.addData("Ejection R", ejectionMotorRight.getPower());

                 */

            } //macro controls

            // Get current robot pose from follower
            robotX = follower.getPose().getX();
            robotY = follower.getPose().getY();
            robotHeadingRadians = follower.getPose().getHeading();

            //turret rotation
            if (gamepad2.left_stick_x <=-0.05) {    //turn turret clockwise
                //updating the current turret position to be in range of the min and max
                currentTurretPosition = clampT(currentTurretPosition + TURRET_ROTATION_INCREMENT, TURRET_ROTATION_MAX_LEFT, TURRET_ROTATION_MAX_RIGHT);
                //sets turret position
                turretServo.setPosition(currentTurretPosition);
                turretState = TurretStates.MANUAL;
            }
            else if (gamepad2.left_stick_x >= 0.05) {   //turn turret counterclockwise
                //updating the current turret position to be in range of the min and max
                currentTurretPosition = clampT(currentTurretPosition - TURRET_ROTATION_INCREMENT, TURRET_ROTATION_MAX_LEFT, TURRET_ROTATION_MAX_RIGHT);
                //sets turret position
                turretServo.setPosition(currentTurretPosition);
                turretState = TurretStates.MANUAL;
            }
            // Odometry-based turret aiming (when not in manual control)
            else if (turretState == TurretStates.ODOMETRY) {

                // Determine target goal based on alliance color
                double targetGoalX, targetGoalY;
                if ("RED".equals(autoAlliance)) {
                    targetGoalX = DarienOpModeFSM.GOAL_RED_X;
                    targetGoalY = DarienOpModeFSM.GOAL_RED_Y;
                } else {
                    targetGoalX = DarienOpModeFSM.GOAL_BLUE_X;
                    targetGoalY = DarienOpModeFSM.GOAL_BLUE_Y;
                }

                // Calculate raw angle, then clamp via servo conversion
                double targetTurretAngleDeg = turretFSM.calculateTurretAngleFromOdometry(
                        targetGoalX, targetGoalY, robotX, robotY, robotHeadingRadians);
                double targetServoPos = turretFSM.calculateServoPositionFromAngle(targetTurretAngleDeg);

                currentTurretPosition = targetServoPos;
                turretServo.setPosition(targetServoPos);

                telemetry.addData("Turret ODO Angle (deg)", String.format("%.1f", targetTurretAngleDeg));
                telemetry.addData("Turret ODO Servo", String.format("%.3f", targetServoPos));
            }

            //CONTROL: EJECTION MOTORS
            if (!trayFSM.isAutoIntakeRunning()) {
                //ODOMETRY BASED SHOOT POWER
                if (shootingPowerMode == ShootingPowerModes.ODOMETRY) {
                    if (robotY <= 48) {
                        shotgunPowerLatch = ShotgunPowerLevel.HIGH;
                    } else {
                        shotgunPowerLatch = ShotgunPowerLevel.LOW;
                    }
                }

                //Latch control
                if (gamepad2.right_stick_y < -.05) {
                    shotgunPowerLatch = ShotgunPowerLevel.HIGH;
                    shootingPowerMode = ShootingPowerModes.MANUAL;
                } else if (gamepad2.right_stick_y > 0.05) {
                    shotgunPowerLatch = ShotgunPowerLevel.LOW;
                    shootingPowerMode = ShootingPowerModes.MANUAL;
                } else if (gamepad2.rightStickButtonWasPressed()) {
                    shotgunPowerLatch = ShotgunPowerLevel.OFF;
                    shootingPowerMode = ShootingPowerModes.MANUAL;
                }
                switch (shotgunPowerLatch) {
                    case OFF:
                        shotgunFSM.toOff();
                        telemetry.addData("Requested ShotGun RPM", 0);
                        break;
                    case HIGH:
                        shotgunFSM.toPowerUpFar(SHOT_GUN_POWER_UP_FAR_RPM_TELEOP);
                        telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_FAR_RPM_TELEOP);
                        break;
                    default:
                    case LOW:
                        shotgunFSM.toPowerUp(SHOT_GUN_POWER_UP_RPM);
                        telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_RPM);
                        break;
                }

            } else {
                shotgunFSM.toOff();
            }
            telemetry.addData("Actual ShotGun RPM", ejectionMotor.getVelocity() * 60 / TICKS_PER_ROTATION); // convert from ticks per second to RPM
            telemetry.addData("ejectionMotor power", ejectionMotor.getPower());
            telemetry.addData("Actual ShotGun TPS", ejectionMotor.getVelocity()); // convert from ticks per second to RPM

            // Turret Aiming Mode Telemetry
            telemetry.addData("Turret State", turretState);

            double targetGoalX, targetGoalY;
            if ("RED".equals(autoAlliance)) {
                targetGoalX = DarienOpModeFSM.GOAL_RED_X;
                targetGoalY = DarienOpModeFSM.GOAL_RED_Y;
            } else {
                targetGoalX = DarienOpModeFSM.GOAL_BLUE_X;
                targetGoalY = DarienOpModeFSM.GOAL_BLUE_Y;
            }

            /*
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);

             */

            // Display alliance color from SharedPreferences
            telemetry.addData("Alliance Color from Auto", autoAlliance);
            telemetry.addData("Target AprilTag ID", targetGoalTagId);
            // telemetry.addData("Time Since Last Camera Detection (ms)",
            //       (getRuntime() - lastCameraDetectionTime) * 1000);
            telemetry.addData("Odometry Pos (X,Y)", String.format("%.1f, %.1f", robotX, robotY));
            telemetry.addData("Odometry Bearing (deg)", String.format("%.1f", Math.toDegrees(robotHeadingRadians)));
            displayTurretTelemetry(yaw, rawBearingDeg, currentHeadingDeg, currentTurretPosition, targetServoPos, range);
            telemetry.update();
        } //while opModeIsActive
    } //runOpMode

    private void startReadingGoalId() {
        tagFSM.start(getRuntime());
        isReadingAprilTag = true;
    }

    private void updateReadingGoalId() {
        tagFSM.update(getRuntime(), true, telemetry);
        telemetry.addLine("Goal Detection: Reading...");

        if (tagFSM.isDone()) {
            telemetry.addLine("Goal Detection: DONE reading!");
            isReadingAprilTag = false;
            aprilTagDetections = tagFSM.getDetections();
            //aprilTagDetections.removeIf(tag -> tag.id != 24);
            if (targetGoalTagId == APRILTAG_ID_GOAL_RED) {
                aprilTagDetections.removeIf(tag -> tag.id == 20 || tag.id == 21 || tag.id == 22 || tag.id == 23);
                turretOffset = TURRET_OFFSET_RED;
            } else if (targetGoalTagId == APRILTAG_ID_GOAL_BLUE) {
                aprilTagDetections.removeIf(tag -> tag.id == 24 || tag.id == 21 || tag.id == 22 || tag.id == 23);
                turretOffset = TURRET_OFFSET_BLUE;
            }
            if (!aprilTagDetections.isEmpty()) {
                telemetry.addLine("Goal Detection: FOUND APRILTAG!");
                // Rotate the turret only if an apriltag is detected and it's the target goal apriltag id
                detection = aprilTagDetections.get(0);
                if (detection.id == targetGoalTagId && detection.ftcPose != null) {
                    telemetry.addLine("Goal Detection: ALIGNING TURRET TO GOAL " + targetGoalTagId);
                    //yaw = detection.ftcPose.yaw; //  REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY
                    //range = detection.ftcPose.range; //  REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY

                    // Current turret heading (degrees)
                    //currentHeadingDeg = turretFSM.getTurretHeading(); //  REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY

                    // Camera-relative bearing to AprilTag (degrees)
                    rawBearingDeg = detection.ftcPose.bearing;

                    if (!isCalculatingTurretTargetPosition) {
                        isCalculatingTurretTargetPosition = true;
                        targetServoPos = TURRET_POSITION_CENTER + turretOffset + RATIO_BETWEEN_TURRET_GEARS * rawBearingDeg / FIVE_ROTATION_SERVO_SPAN_DEG;
                        if (!Double.isNaN(targetServoPos)) {
                            currentTurretPosition = targetServoPos;
                            turretServo.setPosition(targetServoPos);
                            // lastCameraDetectionTime = getRuntime();  // Update last detection time
                        }
                    }
                } else if (detection.ftcPose == null) {
                    telemetry.addLine("Goal Detection: WARNING - Pose estimation failed!");
                } // end detection.id == 20 or 24
            } // end detection is empty
            //THIS IS FALLBACK FOR ODOMETRY MODE WHEN CAMERA DOES NOT WORK (ai gen untested)
        /*} else {
            // No detection found - check if we should fall back to odometry
            if (turretState == TurretStates.CAMERA &&
                    getRuntime() - lastCameraDetectionTime >= (DarienOpModeFSM.CAMERA_FALLBACK_TIMEOUT_MS / 1000.0)) {
                telemetry.addLine("Goal Detection: Camera timeout - FALLING BACK TO ODOMETRY!");
                turretState = TurretStates.ODOMETRY;
            }
             */

        } // end tagFSM is done
        isCalculatingTurretTargetPosition = false;
    }

    private void displayTurretTelemetry(double yaw, double rawBearingDeg, double currentHeadingDeg, double currentTurretPosition, double targetServoPos, double distanceFromGoal) {
        //telemetry.addData("TURRET: yaw", yaw);
        telemetry.addData("TURRET: Raw Bearing Deg (alpha)", rawBearingDeg);
        //telemetry.addData("TURRET: currentHeadingDeg (C0)", currentHeadingDeg);
        telemetry.addData("TURRET: Current Turret Pos", currentTurretPosition);
        telemetry.addData("TURRET: Target Turret Pos", targetServoPos);
        //telemetry.addData("TURRET: range", distanceFromGoal);
        // DO NOT ADD telemetry.update() HERE
    }

} //TeleOpFSM class