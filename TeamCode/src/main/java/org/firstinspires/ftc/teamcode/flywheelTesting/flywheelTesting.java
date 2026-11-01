package org.firstinspires.ftc.teamcode.flywheelTesting;
/* FAILED AI CODE CLAUDE IS BAD
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;
@Disabled
@TeleOp(name = "Blue Basket Shot Tuner (v3.0)", group = "Tuning")
public class flywheelTesting extends LinearOpMode {
/* FAILED AI GENERATED CODE CLAUDE IS BAD
    // ==================== HARDWARE ====================
    private Follower follower;
    private DcMotorEx flywheel;
    private Servo hood;

    // ==================== CONFIGURATION ====================
    // Blue Alliance High Basket coordinates (cm)
    private final BezierPoint BLUE_GOAL = new BezierPoint(0.0, 365.0);
    private final double FIXED_HEADING = Math.toRadians(145); // Robot angle locked to goal

    // Motor Configuration
    private final double TICKS_PER_REV = 28.0;
    private final double MAX_RPM = 3000.0;     // Safety limit

    // Tuning Adjustment Rates
    private final double RPM_ADJUSTMENT_RATE = 15.0;
    private final double DISTANCE_INCREMENT = 5.0; // cm
    private final double HOOD_ADJUSTMENT_RATE = 0.005;
    private final double MIN_DISTANCE = 10.0; // cm

    // RPM Tolerance for "ready to shoot" validation
    private final double RPM_TOLERANCE = 50.0; // RPM

    // Telemetry Update Throttling
    private final double TELEMETRY_UPDATE_MS = 100.0;

    // ==================== STATE VARIABLES ====================
    private double currentDistance = 60.0; // Initial distance from goal (cm)
    private double targetRPM = 1200.0;
    private double targetHoodPos = 0.5;

    // Button debouncing
    private boolean aPressed, bPressed, xPressed, yPressed, dpadUpPressed, dpadDownPressed;

    // File I/O
    private FileWriter writer;
    private int successCount = 0;
    private int failCount = 0;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    private double lastTelemetryUpdate = 0.0;

    @Override
    public void runOpMode() {
        initializeHardware();
        setupLogging();

        // Calculate starting position
        double startX = BLUE_GOAL.getFirstControlPoint().getX() - (Math.cos(FIXED_HEADING) * currentDistance);
        double startY = BLUE_GOAL.getFirstControlPoint().getY() - (Math.sin(FIXED_HEADING) * currentDistance);
        follower.setStartingPose(new Pose(startX, startY, FIXED_HEADING));

        telemetry.addLine("✓ Shot Tuner Initialized");
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update(); // CRITICAL: Must be called every loop

            handleMovementControls();
            handleShooterControls();
            handleDataLogging();
            handlePresets();
            updateTelemetry();
        }

        closeLogging();
    }

    // ==================== INITIALIZATION ====================

    private void initializeHardware() {
        // Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);

        // Optimize performance: Enable Auto Bulk Caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize Shooter Hardware
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        hood = hardwareMap.get(Servo.class, "hood");

        // Configure flywheel for velocity control
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Tune these PIDF values for your specific flywheel motor
        // Use FTC Dashboard or a separate tuning OpMode to find optimal values
        // flywheel.setVelocityPIDFCoefficients(YOUR_P, YOUR_I, YOUR_D, YOUR_F);

        // Recommended starting values (adjust as needed):
        // P = 10-30, I = 0-5, D = 0-10, F = 12-15
        flywheel.setVelocityPIDFCoefficients(20, 0, 5, 12.5);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // ==================== MOVEMENT CONTROLS ====================

    private void handleMovementControls() {
        // Y Button: Move backward (increase distance)
        if (gamepad1.y && !yPressed) {
            currentDistance += DISTANCE_INCREMENT;
            gamepad1.rumble(50); // Quick haptic feedback
            yPressed = true;
        } else if (!gamepad1.y) {
            yPressed = false;
        }

        // X Button: Move forward (decrease distance)
        if (gamepad1.x && !xPressed) {
            currentDistance = Math.max(MIN_DISTANCE, currentDistance - DISTANCE_INCREMENT);
            gamepad1.rumble(50);
            xPressed = true;
        } else if (!gamepad1.x) {
            xPressed = false;
        }

        // Calculate target position based on distance from goal
        double targetX = BLUE_GOAL.getFirstControlPoint().getX() - (Math.cos(FIXED_HEADING) * currentDistance);
        double targetY = BLUE_GOAL.getFirstControlPoint().getY() - (Math.sin(FIXED_HEADING) * currentDistance);

        // Hold robot at target position with locked heading
        follower.holdPoint(new Pose(targetX, targetY, FIXED_HEADING));
    }

    // ==================== SHOOTER CONTROLS ====================

    private void handleShooterControls() {
        // Left Stick Y: Adjust RPM
        if (Math.abs(gamepad1.left_stick_y) > 0.05) { // Deadzone
            targetRPM += (-gamepad1.left_stick_y * RPM_ADJUSTMENT_RATE);
            targetRPM = Math.max(0, Math.min(MAX_RPM, targetRPM));
        }

        // Convert RPM to ticks per second for motor controller
        double ticksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;
        flywheel.setVelocity(ticksPerSec);

        // Right Stick Y: Adjust Hood Position
        if (Math.abs(gamepad1.right_stick_y) > 0.05) { // Deadzone
            targetHoodPos += (-gamepad1.right_stick_y * HOOD_ADJUSTMENT_RATE);
            targetHoodPos = Math.max(0.0, Math.min(1.0, targetHoodPos));
        }

        hood.setPosition(targetHoodPos);
    }

    // ==================== DATA LOGGING ====================

    private void handleDataLogging() {
        // Get current robot position
        Pose currentPose = follower.getPose();

        // Calculate actual distance to goal using odometry
        double actualDist = Math.hypot(
                currentPose.getX() - BLUE_GOAL.getFirstControlPoint().getX(),
                currentPose.getY() - BLUE_GOAL.getFirstControlPoint().getY()
        );

        // Get actual flywheel RPM
        double actualRPM = (flywheel.getVelocity() / TICKS_PER_REV) * 60.0;
        double rpmError = Math.abs(targetRPM - actualRPM);
        boolean flywheelReady = rpmError < RPM_TOLERANCE;

        // A Button: Log SUCCESS
        if (gamepad1.a && !aPressed) {
            if (!flywheelReady && targetRPM > 100) { // Only check if flywheel is actually spinning
                telemetry.addLine("⚠️ FLYWHEEL NOT READY! Wait for RPM to stabilize");
                telemetry.addData("RPM Error", "%.0f RPM", rpmError);
                gamepad1.rumble(100); // Short error rumble
            } else {
                saveData(currentDistance, actualDist, targetRPM, actualRPM, targetHoodPos, "SUCCESS");
                successCount++;
                gamepad1.rumble(500); // Long success rumble
            }
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }

        // B Button: Log FAIL
        if (gamepad1.b && !bPressed) {
            if (!flywheelReady && targetRPM > 100) {
                telemetry.addLine("⚠️ FLYWHEEL NOT READY! Wait for RPM to stabilize");
                telemetry.addData("RPM Error", "%.0f RPM", rpmError);
                gamepad1.rumble(100);
            } else {
                saveData(currentDistance, actualDist, targetRPM, actualRPM, targetHoodPos, "FAIL");
                failCount++;
                gamepad1.rumble(150); // Medium fail rumble
            }
            bPressed = true;
        } else if (!gamepad1.b) {
            bPressed = false;
        }
    }

    // ==================== PRESETS ====================

    private void handlePresets() {
        // D-Pad Up: Close Range Preset
        if (gamepad1.dpad_up && !dpadUpPressed) {
            targetRPM = 1200;
            targetHoodPos = 0.4;
            currentDistance = 60.0;
            gamepad1.rumble(200);
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        // D-Pad Down: Long Range Preset
        if (gamepad1.dpad_down && !dpadDownPressed) {
            targetRPM = 2000;
            targetHoodPos = 0.7;
            currentDistance = 120.0;
            gamepad1.rumble(200);
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }
    }

    // ==================== TELEMETRY ====================

    private void updateTelemetry() {
        // Throttle telemetry updates to improve performance
        if (runtime.milliseconds() - lastTelemetryUpdate < TELEMETRY_UPDATE_MS) {
            return;
        }

        Pose currentPose = follower.getPose();
        double actualDist = Math.hypot(
                currentPose.getX() - BLUE_GOAL.getFirstControlPoint().getX(),
                currentPose.getY() - BLUE_GOAL.getFirstControlPoint().getY()
        );

        double actualRPM = (flywheel.getVelocity() / TICKS_PER_REV) * 60.0;
        double rpmError = targetRPM - actualRPM;
        boolean flywheelReady = Math.abs(rpmError) < RPM_TOLERANCE;

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("POSITION");
        telemetry.addData("  Target Distance", "%.1f cm", currentDistance);
        telemetry.addData("  Actual Distance", "%.1f cm", actualDist);
        telemetry.addData("  Position Error", "%.1f cm", Math.abs(currentDistance - actualDist));
        telemetry.addData("  Robot X/Y", "%.1f, %.1f", currentPose.getX(), currentPose.getY());

        telemetry.addLine();
        telemetry.addLine("SHOOTER");
        telemetry.addData("  Target RPM", "%.0f", targetRPM);
        telemetry.addData("  Actual RPM", "%.0f", actualRPM);
        telemetry.addData("  RPM Error", "%.0f", rpmError);
        telemetry.addData("  Hood Position", "%.3f", targetHoodPos);
        telemetry.addData("  Ready to Shoot", flywheelReady ? "✓ YES" : "✗ NO");

        telemetry.addLine();
        telemetry.addLine("DATA LOG");
        telemetry.addData("  Success Shots", successCount);
        telemetry.addData("  Failed Shots", failCount);
        telemetry.addData("  Total Logged", successCount + failCount);

        telemetry.addLine();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("🎮 CONTROLS");
        telemetry.addLine("  Left Stick Y  = Adjust RPM");
        telemetry.addLine("  Right Stick Y = Adjust Hood");
        telemetry.addLine("  Y Button      = Move Back (+5cm)");
        telemetry.addLine("  X Button      = Move Forward (-5cm)");
        telemetry.addLine("  A Button      = Log SUCCESS");
        telemetry.addLine("  B Button      = Log FAIL");
        telemetry.addLine("  D-Pad Up      = Close Range Preset");
        telemetry.addLine("  D-Pad Down    = Long Range Preset");

        telemetry.update();
        lastTelemetryUpdate = runtime.milliseconds();
    }

    // ==================== FILE I/O ====================

    private void setupLogging() {
        try {
            File path = new File("/sdcard/FIRST/");
            if (!path.exists()) {
                path.mkdirs();
            }

            // Create timestamped filename to avoid overwriting
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File file = new File(path, "shot_log_" + timestamp + ".csv");

            writer = new FileWriter(file, true);

            // Write header if file is new
            if (file.length() == 0) {
                writer.write("Target_Dist_CM,Actual_Dist_CM,Target_RPM,Actual_RPM,Hood_Pos,Result\n");
            }

            writer.flush();
            telemetry.addData("Log File", file.getName());
        } catch (IOException e) {
            telemetry.addLine("ERROR: Log Initialization Failed!");
            telemetry.addData("Error", e.getMessage());
        }
    }

    private void saveData(double targetDist, double actualDist, double targetRPM, double actualRPM, double hood, String result) {
        try {
            if (writer != null) {
                String line = String.format(Locale.US, "%.2f,%.2f,%.0f,%.0f,%.3f,%s\n",
                        targetDist, actualDist, targetRPM, actualRPM, hood, result);
                writer.write(line);
                writer.flush();

                telemetry.addLine("Data Saved: " + result);
            } else {
                telemetry.addLine("ERROR: Logger not initialized!");
            }
        } catch (IOException e) {
            telemetry.addLine("ERROR: Failed to save data!");
            telemetry.addData("Error", e.getMessage());
        }
    }

    private void closeLogging() {
        try {
            if (writer != null) {
                writer.close();
                telemetry.addLine("✓ Log file closed successfully");
                telemetry.addData("Total Entries", successCount + failCount);
                telemetry.update();
            }
        } catch (IOException e) {
            telemetry.addLine("ERROR: Failed to close log file!");
        }
    }
}
 */