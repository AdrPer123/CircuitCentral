package org.firstinspires.ftc.teamcode.flywheelTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
/**
 * FLYWHEEL PIDF TUNER
 *
 * Use this OpMode to tune your flywheel's PIDF values for precise velocity control.
 *
 * TUNING PROCESS:
 * 1. Start with kF only (set kP, kI, kD to 0)
 * 2. Tune kF until flywheel reaches approximately the target velocity
 * 3. Add kP to reduce steady-state error
 * 4. Add kI to eliminate remaining error (use sparingly)
 * 5. Add kD if oscillation occurs (rarely needed for flywheels)
 *
 * CONTROLS:
 * === TARGET VELOCITY ===
 * - D-Pad UP: Increase target by 100 RPM
 * - D-Pad DOWN: Decrease target by 100 RPM
 * - X Button: Set target to 0 (stop)
 * - Y Button: Set target to max (4000 RPM)
 *
 * === PIDF TUNING ===
 * - Right Bumper: Next tuning parameter (cycles through kF → kP → kI → kD)
 * - Left Stick Y: Increase/decrease selected parameter
 * - A Button: Reset integral sum (if flywheel gets stuck)
 *
 * === PRESETS ===
 * - B Button: Toggle auto-spin mode (cycles through common velocities)
 *
 * The telemetry will show:
 * - Current and target velocity
 * - Error (difference between target and actual)
 * - Current PIDF values
 * - Which parameter you're adjusting
 * - Performance metrics

@TeleOp(name = "Flywheel PIDF Tuner", group = "Tuning")
public class flywheeltuner extends OpMode {

    Flywheelsubsystem flywheel = new Flywheelsubsystem();

    // Current target velocity
    private double targetVelocity = 0;

    // Which PIDF parameter we're currently tuning
    private enum TuningParameter {
        KF, KP, KI, KD
    }
    private TuningParameter currentParameter = TuningParameter.KF;

    // Button debouncing
    private boolean rbLast = false;
    private boolean xLast = false;
    private boolean yLast = false;
    private boolean aLast = false;
    private boolean bLast = false;
    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;

    // Auto-spin mode
    private boolean autoSpinMode = false;
    private int autoSpinIndex = 0;
    private double[] autoSpinVelocities = {1000, 2000, 3000, 4000};
    private ElapsedTime autoSpinTimer = new ElapsedTime();
    private static final double AUTO_SPIN_DURATION = 3.0;  // Seconds per velocity

    // Performance tracking
    private ElapsedTime performanceTimer = new ElapsedTime();
    private double minError = Double.MAX_VALUE;
    private double maxError = Double.MIN_VALUE;
    private double avgError = 0;
    private int errorSamples = 0;

    @Override
    public void init() {
        flywheel.init(hardwareMap);

        telemetry.addLine("================================");
        telemetry.addLine("  FLYWHEEL PIDF TUNER");
        telemetry.addLine("================================");
        telemetry.addLine();
        telemetry.addLine("STEP 1: Tune kF first!");
        telemetry.addLine("  - Set target velocity");
        telemetry.addLine("  - Adjust kF until close");
        telemetry.addLine();
        telemetry.addLine("STEP 2: Tune kP");
        telemetry.addLine("  - Reduce remaining error");
        telemetry.addLine();
        telemetry.addLine("STEP 3: Add kI (optional)");
        telemetry.addLine("  - Eliminate tiny errors");
        telemetry.addLine();
        telemetry.addLine("Ready to tune!");
        telemetry.addLine("================================");
        telemetry.update();

        performanceTimer.reset();
    }

    @Override
    public void start() {
        autoSpinTimer.reset();
    }

    @Override
    public void loop() {
        // ======================== TARGET VELOCITY CONTROL ========================

        // D-Pad: Adjust target velocity
        if (gamepad1.dpad_up && !dpadUpLast) {
            targetVelocity += 100;
            targetVelocity = Math.min(6000, targetVelocity);  // Cap at 6000 RPM
            resetPerformanceTracking();
        }
        dpadUpLast = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownLast) {
            targetVelocity -= 100;
            targetVelocity = Math.max(0, targetVelocity);
            resetPerformanceTracking();
        }
        dpadDownLast = gamepad1.dpad_down;

        // X: Stop flywheel
        if (gamepad1.x && !xLast) {
            targetVelocity = 0;
            autoSpinMode = false;
            resetPerformanceTracking();
        }
        xLast = gamepad1.x;

        // Y: Max velocity
        if (gamepad1.y && !yLast) {
            targetVelocity = 4000;
            autoSpinMode = false;
            resetPerformanceTracking();
        }
        yLast = gamepad1.y;

        // B: Toggle auto-spin mode
        if (gamepad1.b && !bLast) {
            autoSpinMode = !autoSpinMode;
            if (autoSpinMode) {
                autoSpinIndex = 0;
                autoSpinTimer.reset();
            }
        }
        bLast = gamepad1.b;

        // Auto-spin mode: Cycle through velocities
        if (autoSpinMode && autoSpinTimer.seconds() > AUTO_SPIN_DURATION) {
            autoSpinIndex = (autoSpinIndex + 1) % autoSpinVelocities.length;
            targetVelocity = autoSpinVelocities[autoSpinIndex];
            autoSpinTimer.reset();
            resetPerformanceTracking();
        }

        // ======================== PIDF TUNING CONTROLS ========================

        // Right Bumper: Cycle through parameters
        if (gamepad1.right_bumper && !rbLast) {
            switch (currentParameter) {
                case KF: currentParameter = TuningParameter.KP; break;
                case KP: currentParameter = TuningParameter.KI; break;
                case KI: currentParameter = TuningParameter.KD; break;
                case KD: currentParameter = TuningParameter.KF; break;
            }
        }
        rbLast = gamepad1.right_bumper;

        // Left Stick Y: Adjust current parameter
        double adjustAmount = -gamepad1.left_stick_y * 0.02;  // Negative because Y is inverted

        if (Math.abs(adjustAmount) > 0.05) {  // Deadzone
            switch (currentParameter) {
                case KF:
                    double newKF = flywheel.getKF() + adjustAmount * 0.00001;  // kF adjustment
                    flywheel.setKF(Math.max(0, newKF));
                    break;
                case KP:
                    double newKP = flywheel.getKP() + adjustAmount * 0.00001;  // kP adjustment
                    flywheel.setKP(Math.max(0, newKP));
                    break;
                case KI:
                    double newKI = flywheel.getKI() + adjustAmount * 0.000001;  // kI adjustment
                    flywheel.setKI(Math.max(0, newKI));
                    break;
                case KD:
                    double newKD = flywheel.getKD() + adjustAmount * 0.000001;  // kD adjustment
                    flywheel.setKD(Math.max(0, newKD));
                    break;
            }
        }

        // A: Reset integral
        if (gamepad1.a && !aLast) {
            flywheel.resetIntegral();
        }
        aLast = gamepad1.a;

        // ======================== UPDATE FLYWHEEL ========================

        flywheel.setTargetVelocity(targetVelocity);
        flywheel.update();

        // ======================== PERFORMANCE TRACKING ========================

        double currentError = Math.abs(flywheel.getError());
        if (targetVelocity > 0 && performanceTimer.seconds() > 1.0) {  // Skip first second
            minError = Math.min(minError, currentError);
            maxError = Math.max(maxError, currentError);
            avgError = (avgError * errorSamples + currentError) / (errorSamples + 1);
            errorSamples++;
        }

        // ======================== TELEMETRY ========================

        telemetry.addLine("======== FLYWHEEL PIDF TUNER ========");
        telemetry.addLine();

        // Current velocities
        telemetry.addLine("--- VELOCITY ---");
        telemetry.addData("Target", "%.0f RPM", targetVelocity);
        telemetry.addData("Actual (Avg)", "%.0f RPM", flywheel.getAverageVelocityRPM());
        telemetry.addData("Motor 1", "%.0f RPM", flywheel.getMotor1VelocityRPM());
        telemetry.addData("Motor 2", "%.0f RPM", flywheel.getMotor2VelocityRPM());
        telemetry.addData("Error", "%.0f RPM", flywheel.getError());
        telemetry.addData("Power", "%.3f", flywheel.getMotorPower());
        telemetry.addLine();

        // Current PIDF values
        telemetry.addLine("--- CURRENT PIDF VALUES ---");
        String kfIndicator = (currentParameter == TuningParameter.KF) ? " <--" : "";
        String kpIndicator = (currentParameter == TuningParameter.KP) ? " <--" : "";
        String kiIndicator = (currentParameter == TuningParameter.KI) ? " <--" : "";
        String kdIndicator = (currentParameter == TuningParameter.KD) ? " <--" : "";

        telemetry.addData("kF", "%.6f%s", flywheel.getKF(), kfIndicator);
        telemetry.addData("kP", "%.6f%s", flywheel.getKP(), kpIndicator);
        telemetry.addData("kI", "%.6f%s", flywheel.getKI(), kiIndicator);
        telemetry.addData("kD", "%.6f%s", flywheel.getKD(), kdIndicator);
        telemetry.addData("Integral Sum", "%.1f", flywheel.getIntegralSum());
        telemetry.addLine();

        // Performance metrics
        if (errorSamples > 0) {
            telemetry.addLine("--- PERFORMANCE ---");
            telemetry.addData("Min Error", "%.1f RPM", minError);
            telemetry.addData("Max Error", "%.1f RPM", maxError);
            telemetry.addData("Avg Error", "%.1f RPM", avgError);
            telemetry.addData("Samples", "%d", errorSamples);
            telemetry.addLine();
        }

        // Status
        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Tuning", currentParameter.toString());
        if (autoSpinMode) {
            telemetry.addData("Auto-Spin", "ON (%.1fs)", AUTO_SPIN_DURATION - autoSpinTimer.seconds());
        } else {
            telemetry.addData("Auto-Spin", "OFF");
        }
        telemetry.addData("At Target?", flywheel.isAtTarget(50) ? "YES ✓" : "NO");
        telemetry.addLine();

        // Controls
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("D-Pad U/D: Target ±100 RPM");
        telemetry.addLine("X: Stop | Y: Max Speed");
        telemetry.addLine("Right Bumper: Next parameter");
        telemetry.addLine("Left Stick Y: Adjust value");
        telemetry.addLine("A: Reset integral");
        telemetry.addLine("B: Toggle auto-spin");
        telemetry.addLine();

        // Tuning advice
        telemetry.addLine("--- TUNING TIPS ---");
        if (targetVelocity == 0) {
            telemetry.addLine("Set a target velocity to start");
        } else {
            double errorPercent = Math.abs(flywheel.getError()) / targetVelocity * 100;
            if (errorPercent > 10) {
                telemetry.addLine("⚠ Large error - adjust kF");
            } else if (errorPercent > 2) {
                telemetry.addLine("→ Moderate error - adjust kP");
            } else if (errorPercent > 0.5) {
                telemetry.addLine("→ Small error - try kI");
            } else {
                telemetry.addLine("✓ Excellent! Write down values");
            }
        }

        telemetry.addLine("====================================");
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
    }

    /**
     * Reset performance tracking when target changes
     */
/*
    private void resetPerformanceTracking() {
        minError = Double.MAX_VALUE;
        maxError = Double.MIN_VALUE;
        avgError = 0;
        errorSamples = 0;
        performanceTimer.reset();
    }
}

*/