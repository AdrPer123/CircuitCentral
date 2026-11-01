package org.firstinspires.ftc.teamcode.TunedTeleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
/*
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumPID {
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    // ======================== PID CONSTANTS ========================
    // Tune these values by editing them here, then rebuild and deploy
    // Start with these defaults and adjust based on your robot's behavior

    private double kP = 0.03;   // Proportional: How much to correct based on current error
    // Higher = stronger correction, but too high causes oscillation
    // Try: 0.02, 0.04, 0.05, 0.06 if robot drifts

    private double kI = 0.0;    // Integral: Corrects persistent steady-state error over time
    // Usually keep at 0 for heading control
    // Only add if persistent drift: try 0.001, 0.002

    private double kD = 0.005;  // Derivative: Dampens oscillation by predicting future error
    // Helps smooth out the correction
    // Try: 0.003, 0.007, 0.01 if robot oscillates

    // ======================== TUNABLE PARAMETERS ========================
    private double ROTATE_DEADZONE_DISABLE = 0.03;  // Deadzone to disable heading lock
    private double ROTATE_DEADZONE_ENABLE = 0.03;   // Deadzone to enable heading lock
    private double MAX_CORRECTION_POWER = 0.3;      // Maximum heading correction power (0.0 to 1.0)

    // ======================== STATE VARIABLES ========================
    private double targetHeading = 0;        // The heading (angle) we want the robot to maintain (in radians)
    private double lastError = 0;            // Error from the previous loop - used to calculate derivative
    private double integralSum = 0;          // Accumulated error over time - used for integral term
    private boolean headingLockEnabled = true; // Whether heading lock is currently active

    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "left_front_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "left_back_motor");
        frontRightMotor = hwMap.get(DcMotor.class, "right_front_motor");
        backRightMotor = hwMap.get(DcMotor.class, "right_back_motor");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(revOrientation));
    }

    /**
     * Basic robot-centric drive (no field orientation)
     * Used internally by other drive methods
     */
/*
    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(frontLeftPower / maxPower);
        backLeftMotor.setPower(backLeftPower / maxPower);
        frontRightMotor.setPower(frontRightPower / maxPower);
        backRightMotor.setPower(backRightPower / maxPower);
    }

    /**
     * Tank drive style (left/right power)
     */
/*
    public void moveXY(double leftPower, double rightPower) {
        frontLeftMotor.setPower(leftPower);
        backLeftMotor.setPower(leftPower);
        frontRightMotor.setPower(rightPower);
        backRightMotor.setPower(rightPower);
    }

    /**
     * Basic field-oriented drive WITHOUT heading lock
     * Kept for backwards compatibility
     */
/*
    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double robotAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Use the standard 2D vector rotation formula to transform the joystick inputs
        // from field-centric to robot-centric.
        double newStrafe  = strafe * Math.cos(robotAngle) - forward * Math.sin(robotAngle);
        double newForward = strafe * Math.sin(robotAngle) + forward * Math.cos(robotAngle);

        this.drive(newForward, newStrafe, rotate);
    }

    /**
     * Field-oriented drive WITH automatic heading correction (PID heading lock)
     * This is the MAIN method you should use in teleop for best control
     *
     * When the driver isn't rotating, the robot maintains its current heading using PID.
     *
     * @param forward  Forward/backward joystick input (-1.0 to 1.0)
     * @param strafe   Left/right joystick input (-1.0 to 1.0)
     * @param rotate   Rotation joystick input (-1.0 to 1.0)
     */
/*
    public void driveFieldRelativeWithHeadingLock(double forward, double strafe, double rotate) {
        // Get the robot's current angle from the IMU (negative because of coordinate system)
        double robotAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // ======================== FIELD-ORIENTED TRANSFORMATION ========================
        // Convert joystick inputs from field frame to robot frame
        // This allows the robot to move in the direction the driver pushes,
        // regardless of which way the robot is facing

        // Standard 2D rotation matrix formulas:
        // newX = x*cos(θ) - y*sin(θ)
        // newY = x*sin(θ) + y*cos(θ)
        double newStrafe = strafe * Math.cos(robotAngle) - forward * Math.sin(robotAngle);
        double newForward = strafe * Math.sin(robotAngle) + forward * Math.cos(robotAngle);

        // ======================== HEADING LOCK LOGIC ========================
        // Start by assuming we'll use the driver's rotation input directly
        double rotateCommand = rotate;

        // Check if driver is actively trying to rotate the robot
        if (Math.abs(rotate) > ROTATE_DEADZONE_DISABLE) {
            // Deadzone: prevents tiny joystick drift from triggering this

            // Driver IS rotating - disable heading lock and track the new heading
            targetHeading = robotAngle;      // Remember where we're going
            headingLockEnabled = false;       // Turn off auto-correction
            integralSum = 0;                  // Clear accumulated error (prevents integral windup)

        } else if (!headingLockEnabled && Math.abs(rotate) < ROTATE_DEADZONE_ENABLE) {
            // Driver just released the rotation stick (within smaller deadzone)
            // Now we want to LOCK onto whatever heading we ended up at

            headingLockEnabled = true;        // Turn on auto-correction
            targetHeading = robotAngle;       // Lock to current heading
        }

        // ======================== PID CORRECTION ========================
        // If heading lock is enabled, calculate the correction needed
        if (headingLockEnabled) {

            // Calculate how far off we are from target heading
            // normalizeAngle ensures we take the shortest path (-180° to +180°)
            // Example: If target is 350° and current is 10°, error = -20° (not +340°)
            double error = normalizeAngle(targetHeading - robotAngle);

            // INTEGRAL: Add current error to running sum
            // This accumulates over time to eliminate steady-state error
            // (Usually not needed for heading control, which is why kI = 0)
            integralSum += error;

            // DERIVATIVE: Calculate rate of change of error
            // If error is decreasing fast, this term reduces correction to prevent overshoot
            // If error is increasing, this adds more correction
            double derivative = error - lastError;

            // Calculate final correction using PID formula:
            // output = (P × error) + (I × sum of errors) + (D × rate of change)
            rotateCommand = kP * error + kI * integralSum + kD * derivative;

            // Limit the correction power to prevent jerky movements
            // Max correction is defined by MAX_CORRECTION_POWER
            // This keeps auto-correction smooth and not too aggressive
            rotateCommand = Math.max(-MAX_CORRECTION_POWER, Math.min(MAX_CORRECTION_POWER, rotateCommand));

            // Save current error for next loop's derivative calculation
            lastError = error;
        }

        // ======================== APPLY MOVEMENT ========================
        // Send the calculated powers (including rotation correction) to motors
        this.drive(newForward, newStrafe, rotateCommand);
    }

    /**
     * Normalize an angle to the range -π to π (-180° to +180°).
     * This ensures we always take the shortest rotational path.
     *
     * Example: 350° becomes -10° (both represent the same angle, but -10° is closer to 0°)
     *
     * @param angle  The angle to normalize (in radians)
     * @return       The normalized angle between -π and π
     */
/*
    private double normalizeAngle(double angle) {
        // If angle is greater than 180°, subtract 360° until it's in range
        while (angle > Math.PI) angle -= 2 * Math.PI;

        // If angle is less than -180°, add 360° until it's in range
        while (angle < -Math.PI) angle += 2 * Math.PI;

        return angle;
    }

    /**
     * Reset the target heading to the robot's current heading.
     * Call this when you want to set a new "locked" heading.
     * Typically called at the start of teleop.
     */
/*
    public void resetHeading() {
        targetHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastError = 0;
        integralSum = 0;
        headingLockEnabled = true;
    }

    /**
     * Get the IMU object (for resetting yaw in OpMode)
     */
/*
    public IMU getImu() {
        return this.imu;
    }

    /**
     * Get current heading error for telemetry/debugging
     * @return Error in radians
     */
    /*
    public double getHeadingError() {
        double robotAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return normalizeAngle(targetHeading - robotAngle);
    }
*/
    /**
     * Check if heading lock is currently active
     * @return true if heading lock is maintaining heading
     */
    /*
    public boolean isHeadingLockActive() {
        return headingLockEnabled;
    }
*/
    /**
     * Get current PID constants (for telemetry)
     */
    /*
    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }

    /**
     * Set PID constants manually (for tuning without Dashboard)
     * Call these in your OpMode's init() to override defaults
     */
    /*
    public void setKP(double kP) { this.kP = kP; }
    public void setKI(double kI) { this.kI = kI; }
    public void setKD(double kD) { this.kD = kD; }
    public void setMaxCorrectionPower(double power) { this.MAX_CORRECTION_POWER = power; }
}
*/