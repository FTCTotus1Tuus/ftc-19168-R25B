package org.firstinspires.ftc.teamcode.team.fsm;

public class TurretFSM {
    private final DarienOpModeFSM opMode;

    /**
     * Constructor for TurretFSM
     * @param opMode The main op mode instance
     */
    public TurretFSM(DarienOpModeFSM opMode) {
        this.opMode = opMode;
    }

    /**
     * Get the current turret heading in degrees based on the servo position.
     * Zero heading is straight ahead (forward motion of the robot)
     * Positive degrees indicate left, negative degrees indicate right.
     * @return The turret heading in degrees.
     */
    public double getTurretHeading(){
        return -900 + DarienOpModeFSM.FIVE_ROTATION_SERVO_SPAN_DEG * opMode.currentTurretPosition;
    }
//todo: work on this math, The turret has a specific range of motion from 0 currently around +-30 degrees from center, it is PROBABLY doing the calculations based on +-180 degrees as if it were those 30 degrees

    /**
     * Calculate the turret aiming angle based on robot odometry and goal position.
     * The angle is relative to the robot's forward direction (0 degrees = straight ahead).
     *
     * @param goalX               Goal X position in inches (field coordinates)
     * @param goalY               Goal Y position in inches (field coordinates)
     * @param robotX              Robot X position in inches (field coordinates)
     * @param robotY              Robot Y position in inches (field coordinates)
     * @param robotHeadingRadians Robot heading in radians (0 = forward along X axis)
     * @return Turret angle in degrees relative to robot forward (positive = left, negative = right)
     */
    public double calculateTurretAngleFromOdometry(double goalX, double goalY,
                                                   double robotX, double robotY,
                                                   double robotHeadingRadians) {
        // Calculate vector from robot to goal
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        // Calculate bearing angle to goal in radians (0 = along X axis, counterclockwise positive)
        double bearingToGoalRadians = Math.atan2(deltaY, deltaX);

        // Calculate the angle relative to robot heading
        // Positive = counterclockwise from robot forward = left
        double relativeAngleRadians = bearingToGoalRadians - robotHeadingRadians;

        // Normalize to [-π, π] range
        while (relativeAngleRadians > Math.PI) {
            relativeAngleRadians -= 2 * Math.PI;
        }
        while (relativeAngleRadians < -Math.PI) {
            relativeAngleRadians += 2 * Math.PI;
        }

        // Convert to degrees
        return Math.toDegrees(relativeAngleRadians);
    }

    /**
     * Convert a turret angle (in degrees) to a servo position, clamped to safe limits.
     * Zero angle = straight ahead (center position).
     * Positive angles = left, Negative angles = right.
     *
     * @param angleDegrees Turret angle in degrees
     * @return Servo position (0.0 to 1.0), clamped to [TURRET_ROTATION_MAX_RIGHT, TURRET_ROTATION_MAX_LEFT]
     */
    public double calculateServoPositionFromAngle(double angleDegrees) {
        // Convert angle to servo position using the turret's servo math
        // The turret spans from -900 to +900 degrees across its full servo range
        // Center position (0 degrees) = TURRET_POSITION_CENTER (0.5)

        // Calculate raw servo position based on angle offset from center
        double servoPosition = DarienOpModeFSM.TURRET_POSITION_CENTER +
                (angleDegrees / DarienOpModeFSM.FIVE_ROTATION_SERVO_SPAN_DEG);

        // Clamp to servo physical limits to prevent damage
        servoPosition = Math.max(DarienOpModeFSM.TURRET_ROTATION_MAX_RIGHT,
                Math.min(DarienOpModeFSM.TURRET_ROTATION_MAX_LEFT, servoPosition));

        return servoPosition;
    }

    /**
     * Calculate turret servo position directly from robot and goal positions.
     * Convenience method that combines odometry calculation with servo positioning.
     *
     * @param goalX               Goal X position in inches
     * @param goalY               Goal Y position in inches
     * @param robotX              Robot X position in inches
     * @param robotY              Robot Y position in inches
     * @param robotHeadingRadians Robot heading in radians
     * @return Servo position (0.0 to 1.0), clamped to safe limits
     */
    public double calculateServoPositionFromOdometry(double goalX, double goalY,
                                                     double robotX, double robotY,
                                                     double robotHeadingRadians) {
        // Calculate the target turret angle based on (a) the robot position and heading and (b) the target goal
        double angleDegrees = calculateTurretAngleFromOdometry(goalX, goalY, robotX, robotY, robotHeadingRadians);
        // Convert the desired angle to a servo position
        return calculateServoPositionFromAngle(angleDegrees);
    }

}


