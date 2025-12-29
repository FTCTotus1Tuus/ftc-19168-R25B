package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

@TeleOp(name = "TeleopFSM", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM extends DarienOpModeFSM {

    // INSTANCES
    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance

    // TUNING CONSTANTS
    public static double INTAKE_TIME = 1;
    public static double SHOT_TIMEOUT = 2.0; // seconds
    public static double TURRET_ROTATION_MIN = 0.63;//todo: VALUES GOOD - CHANGE NAMES BECAUSE INVERTED
    public static double TURRET_ROTATION_MAX = 0.3;
    public static double TURRET_ROTATION_INCREMENT = 0.001;

    // VARIABLES
    private double shotStartTime;
    private boolean shotStarted = false;

    // Track previous bumper state for edge detection
    private boolean prevRightBumper = false;

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

        // Initialize rubber bands to off
        rubberBands.setPower(0);
        //TrayServo.setPosition(currentTrayPosition); // Prevent movement at init
    }

    @Override
    public void runOpMode() throws InterruptedException {
        float gain = 2;
        initControls();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();


        waitForStart();
        if (isStopRequested()) return;
        //Start
        follower.startTeleopDrive(true);
        follower.update();

        while (this.opModeIsActive() && !isStopRequested()) {

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            // -----------------
            // GAMEPAD1 CONTROLS
            // -----------------

            //RubberBands + IntakeRoller CONTROLS:
            if (gamepad1.y) {
                intakeRoller.setPower(INTAKE_INTAKE_ROLLER_POWER);
                rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
            } else if (gamepad1.a) {
                intakeRoller.setPower(-OUTPUT_INTAKE_ROLLER_POWER);
                rubberBands.setPower(-OUTPUT_RUBBER_BANDS_POWER);
            } else if (gamepad1.x) {
                intakeRoller.setPower(0);
                rubberBands.setPower(0);
            }

            // Toggle auto-intake on right bumper press (edge triggered)
            if (gamepad1.right_bumper && !prevRightBumper) {
                // toggle the TrayFSM instance (from DarienOpModeFSM)
                if (trayFSM != null) trayFSM.toggleAutoIntake();
            }
            prevRightBumper = gamepad1.right_bumper;

            // Show current auto-intake status on telemetry
            telemetry.addData("AutoIntakeRunning", trayFSM != null && trayFSM.isAutoIntakeRunning());

            // Update trayFSM state machine each loop so it can run when toggled on
            if (trayFSM != null) {
                trayFSM.update();
            }


            // -----------------
            // GAMEPAD2 CONTROLS
            // -----------------

            if (!shotStarted) {
                // -----------------
                // MANUAL CONTROLS: only allowed when not in shooting macro
                // -----------------

                //CONTROL: EJECTION MOTORS
                if (gamepad2.right_trigger > 0.05 && gamepad2.right_stick_y >= -0.05) {
                    ejectionMotor.setPower(getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP));
                } else if (gamepad2.right_trigger > 0.05 && gamepad2.right_stick_y < -0.05) {
                    ejectionMotor.setPower(getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP_FAR));
                } else if (gamepad2.left_trigger > 0.05) {
                    ejectionMotor.setPower(-getVoltageAdjustedMotorPower(SHOT_GUN_POWER_DOWN));
                } else {
                    ejectionMotor.setPower(0);
                }

                //CONTROL: ELEVATOR
                if (gamepad2.left_bumper) {
                    Elevator.setPosition(ELEVATOR_POS_UP);
                } else {
                    Elevator.setPosition(ELEVATOR_POS_DOWN);
                }

                // CONTROL: ROTATING TRAY
                if (gamepad2.dpad_left) {
                    setTrayPosition(TRAY_POS_1_INTAKE);
                    //servoIncremental(TrayServo, TRAY_POS_1_INTAKE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_1_INTAKE;
                } else if (gamepad2.x) {
                    setTrayPosition(TRAY_POS_1_SCORE);
                    //servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_1_SCORE;
                } else if (gamepad2.dpad_up) {
                    setTrayPosition(TRAY_POS_2_INTAKE);
                    //servoIncremental(TrayServo, TRAY_POS_2_INTAKE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_2_INTAKE;
                } else if (gamepad2.y) {
                    setTrayPosition(TRAY_POS_2_SCORE);
                    //servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_2_SCORE;
                } else if (gamepad2.dpad_right) {
                    setTrayPosition(TRAY_POS_3_INTAKE);
                    //servoIncremental(TrayServo, TRAY_POS_3_INTAKE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_3_INTAKE;
                } else if (gamepad2.b && !gamepad2.start) {
                    setTrayPosition(TRAY_POS_3_SCORE);
                    //servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_3_SCORE;
                }

                //turret rotation
                if (gamepad2.left_stick_x <=-0.05) {    //turn turret clockwise
                    //updating the current turret position to be in range of the min and max
                    currentTurretPosition = clampT(currentTurretPosition + TURRET_ROTATION_INCREMENT, TURRET_ROTATION_MIN,TURRET_ROTATION_MAX);
                    //sets turret position
                    turretServo.setPosition(currentTurretPosition);
                }
                else if (gamepad2.left_stick_x >= 0.05) {   //turn turret counterclockwise
                    //updating the current turret position to be in range of the min and max
                    currentTurretPosition = clampT(currentTurretPosition - TURRET_ROTATION_INCREMENT,TURRET_ROTATION_MIN,TURRET_ROTATION_MAX);
                    //sets turret position
                    turretServo.setPosition(currentTurretPosition);
                }
            /*    telemetry.addData("TurretPos", "%.3f", currentTurretPosition);
                telemetry.addData("Turret Min/Max/Inc", "%.3f / %.3f / %.3f", TURRET_ROTATION_MIN, TURRET_ROTATION_MAX, TURRET_ROTATION_INCREMENT); */

                /*
                // CONTROL: ROTATING TRAY USING FSM
                if (gamepad2.dpad_left && gamepad2.back) {
                    setTrayPosition(TRAY_POS_1_INTAKE);
                } else if (gamepad2.x && gamepad2.back) {
                    setTrayPosition(TRAY_POS_1_SCORE);
                } else if (gamepad2.dpad_up && gamepad2.back) {
                    setTrayPosition(TRAY_POS_2_INTAKE);
                } else if (gamepad2.y && gamepad2.back) {
                    setTrayPosition(TRAY_POS_2_SCORE);
                } else if (gamepad2.dpad_right && gamepad2.back) {
                    setTrayPosition(TRAY_POS_3_INTAKE);
                } else if (gamepad2.b && gamepad2.back) {
                    setTrayPosition(TRAY_POS_3_SCORE);
                }

                 */

                // -----------------
                // IMPORTANT: ALWAYS PUT MACRO CONTROLS AFTER MANUAL CONTROLS
                // -----------------

                //CONTROL: START SHOTGUN MACRO USING FSM
                if (gamepad2.dpad_down && gamepad2.right_trigger > 0.05) {
                    // TODO: pre-spin up the shotgun before starting the shooting FSM
                    if (gamepad2.right_stick_y < -0.05) {
                        shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP_FAR);
                    } else {
                        shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP);
                    }
                    shotStartTime = getRuntime();
                    shotStarted = true;
                }
                telemetry.addData("shotStarted", shotStarted);

            } //manual controls
            else {
                // -----------------
                // MACRO CONTROLS
                // -----------------

                //CONTROL: SHOTGUN MACRO
                shootArtifactFSM.updateShooting();
                if (shootArtifactFSM.shootingDone() || getRuntime() - shotStartTime >= SHOT_TIMEOUT) {
                    shootArtifactFSM.resetShooting();
                    shotStarted = false;
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
            telemetry.update();

        } //while opModeIsActive
    } //runOpMode
} //TeleOpFSM class