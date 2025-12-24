package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

/**
 * Base OpMode for Pedro pathing and state machine logic.
 * Extend this class for autonomous OpModes using Pedro pathing.
 */
@Config
@Configurable
public abstract class DarienOpModeFSM extends LinearOpMode {

    // Pedro pathing/state machine FSMs (declare as needed)
    // public PathFollowerFSM pathFollowerFSM;
    public AprilTagDetectionFSM tagFSM;
    public ShootPatternFSM shootPatternFSM;
    public ShootArtifactFSM shootArtifactFSM;

    // AprilTag
    public ArrayList<AprilTagDetection> aprilTagDetections;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal = null;

    // telemetry
    public TelemetryPacket tp;
    public FtcDashboard dash;

    // HARDWARE DEVICES
    public Servo TrayServo, Elevator;
    public CRServo rubberBands, intakeRoller;
    public DcMotor ejectionMotor;

    // HARDWARE FIXED CONSTANTS
    public static final double encoderResolution = 537.7; //no change unless we change motors
    public static final double wheelDiameter = 3.75; // inches
    public static final double constMult = (wheelDiameter * (Math.PI));
    public static final double inchesToEncoder = encoderResolution / constMult;
    public static final double PI = 3.1416;

    // HARDWARE TUNING CONSTANTS
    public static double TRAY_SERVO_DURATION_ROTATE = 1.5; // seconds
    public static double TRAY_POS_1_INTAKE = 0.275;
    public static double TRAY_POS_2_INTAKE = 0.205;
    public static double TRAY_POS_3_INTAKE = 0.350;
    public static double TRAY_POS_1_SCORE = 0.385;
    public static double TRAY_POS_2_SCORE = 0.310;
    public static double TRAY_POS_3_SCORE = 0.240;
    public static final double ELEVATOR_POS_UP = 0.85;
    public static final double ELEVATOR_POS_DOWN = 0.45;
    public static double SHOT_GUN_POWER_UP = 0.60;
    public static double SHOT_GUN_POWER_UP_FAR = 0.80;
    public static double SHOT_GUN_POWER_DOWN = 0.2; // tuned to 6000 rpm motor
    public static final double TIMEOUT_APRILTAG_DETECTION = 3;
    public static double INTAKE_RUBBER_BANDS_POWER = 1;
    public static double OUTPUT_RUBBER_BANDS_POWER = 0.2;
    public static double INTAKE_INTAKE_ROLLER_POWER = 1;
    public static double OUTPUT_INTAKE_ROLLER_POWER = 0.2;

    public double currentTrayPosition;

    // Abstract method for child classes to implement
    @Override
    public abstract void runOpMode() throws InterruptedException;

    public ServoIncrementalFSM intakeServoFSM;
    //public ServoIncrementalFSM trayServoFSM;

    public void initControls() {

        // INITIALIZE SENSORS

        // Initialize 2 Deadwheel odometry
        //configure2DeadWheel();

        //TELEMETRY
        // TODO: Put a flag to turn on/off ftc dashboard. We don't want that to run during matches.
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        // INITIALIZE SERVOS
        TrayServo = hardwareMap.get(Servo.class, "Tray");
        Elevator = hardwareMap.get(Servo.class, "Elevator");
        rubberBands = hardwareMap.get(CRServo.class, "rubberBands");
        intakeRoller = hardwareMap.get(CRServo.class, "intakeRoller");

        // INITIALIZE SENSORS

        // INITIALIZE MOTORS
        ejectionMotor = hardwareMap.get(DcMotor.class, "ejectionMotor");
        ejectionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //initAprilTag();

        // INSTANTIATE THE STATE MACHINES
        tagFSM = new AprilTagDetectionFSM(aprilTag, TIMEOUT_APRILTAG_DETECTION);
        shootArtifactFSM = new ShootArtifactFSM(this);
        shootPatternFSM = new ShootPatternFSM(this);

        //trayServoFSM = new ServoIncrementalFSM(TrayServo);
        //currentTrayPosition = TRAY_POS_1_SCORE; // set a default tray position


        telemetry.addLine("FTC 19168 Robot Initialization Done!");
        telemetry.update();
    }

    /**
     * Set the tray position and update the currentTrayPosition variable.
     *
     * @param position The desired position for the tray servo.
     * @param duration The duration over which to move the tray servo to the desired position.
     */
    public void setTrayPosition(double position, double duration) {
        TrayServo.setPosition(position);
        //servoIncremental(TrayServo, position, currentTrayPosition, duration, 4);
        currentTrayPosition = position;
        /*
        if (!trayServoFSM.isRunning()) {
            targetTrayPosition = position;
            trayServoFSM.start(position, currentTrayPosition, duration, getRuntime());
        }
         */
    }

    /**
     * Set the tray position with a default duration in seconds.
     *
     * @param position The desired position for the tray servo.
     */
    public void setTrayPosition(double position) {
        setTrayPosition(position, TRAY_SERVO_DURATION_ROTATE);
    }

    public DcMotor initializeMotor(String name) {
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void servoIncremental(Servo servo, double endPos, double startPos, double endDuration, double divisor) {
        //calculate how many increments it will take to reach to position in the target time
        double currentPos;
        double startTime = getRuntime();
        double currentTime = startTime;

        while (currentTime - startTime < endDuration) {
            if (endPos > startPos) {
                // rotate tray clockwise
                currentPos = ((endPos - startPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + startPos;
            } else {
                // rotate tray counterclockwise
                currentPos = ((startPos - endPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + endPos;
            }
            servo.setPosition(currentPos / divisor);

            //telemetry.addData("currentPos:", currentPos);
            //telemetry.addData("currentTime:", currentTime);
            //telemetry.update();
            //tp.put("currentServo", currentPos);
            //tp.put("currentTime", currentTime);

            //dash.sendTelemetryPacket(tp);

            if (currentPos >= endPos) {
                return;
            }
            currentTime = getRuntime();
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

    }

    public double getVoltageAdjustedMotorPower(double power) {
        double nominalVoltage = 13.0; // Typical full battery voltage for FTC
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double scale = nominalVoltage / currentVoltage;
        return power * scale;
    }
}
