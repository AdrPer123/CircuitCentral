package org.firstinspires.ftc.teamcode.TunedTeleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.TunedTeleop.MecanumPID;

/**
 * SIMPLE PID TUNING OPMODE (No Dashboard Required)
 *
 * This OpMode lets you tune PID values using gamepad buttons.
 * No need for FTC Dashboard - all tuning is done on the driver station!
 *
 * CONTROLS:
 * - Left Stick: Drive/Strafe
 * - Right Stick X: Rotate
 * - A Button: Reset heading
 *
 * TUNING CONTROLS (D-Pad):
 * - D-Pad UP: Increase kP by 0.005
 * - D-Pad DOWN: Decrease kP by 0.005
 * - D-Pad RIGHT: Increase kD by 0.001
 * - D-Pad LEFT: Decrease kD by 0.001
 * - Left Trigger: Decrease kI by 0.0001
 * - Right Trigger: Increase kI by 0.0001
 *
 * HOW TO TUNE:
 * 1. Run this OpMode
 * 2. Drive and strafe - watch if robot holds heading
 * 3. Use D-Pad to adjust kP, kI, kD in real-time
 * 4. Once tuned, write down the values and copy to MecanumDrive.java
 */
/*
@TeleOp(name = "PID Tuner (No Dashboard)", group = "Tuning")
public class SimplePIDTuner extends OpMode {
    MecanumPID drive = new MecanumPID();

    // Button debouncing
    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;

    @Override
    public void init() {
        drive.init(hardwareMap);

        telemetry.addLine("================================");
        telemetry.addLine("  SIMPLE PID TUNER");
        telemetry.addLine("================================");
        telemetry.addLine();
        telemetry.addLine("Use D-Pad to tune:");
        telemetry.addLine("  UP/DOWN: Adjust kP");
        telemetry.addLine("  LEFT/RIGHT: Adjust kD");
        telemetry.addLine("  Triggers: Adjust kI");
        telemetry.addLine();
        telemetry.addLine("Drive and strafe to test!");
        telemetry.addLine("================================");
        telemetry.update();
    }

    @Override
    public void start() {
        drive.getImu().resetYaw();
        drive.resetHeading();
    }

    @Override
    public void loop() {
        // ======================== DRIVE CONTROLS ========================
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.driveFieldRelativeWithHeadingLock(forward, strafe, rotate);

        // Reset heading with A button
        if (gamepad1.a) {
            drive.resetHeading();
        }

        // ======================== PID TUNING CONTROLS ========================
        // D-Pad UP: Increase kP
        if (gamepad1.dpad_up && !dpadUpLast) {
            drive.setKP(drive.getKP() + 0.005);
        }
        dpadUpLast = gamepad1.dpad_up;

        // D-Pad DOWN: Decrease kP
        if (gamepad1.dpad_down && !dpadDownLast) {
            drive.setKP(Math.max(0, drive.getKP() - 0.005));  // Don't go negative
        }
        dpadDownLast = gamepad1.dpad_down;

        // D-Pad RIGHT: Increase kD
        if (gamepad1.dpad_right && !dpadRightLast) {
            drive.setKD(drive.getKD() + 0.001);
        }
        dpadRightLast = gamepad1.dpad_right;

        // D-Pad LEFT: Decrease kD
        if (gamepad1.dpad_left && !dpadLeftLast) {
            drive.setKD(Math.max(0, drive.getKD() - 0.001));  // Don't go negative
        }
        dpadLeftLast = gamepad1.dpad_left;

        // Right Trigger: Increase kI
        if (gamepad1.right_trigger > 0.5) {
            drive.setKI(drive.getKI() + 0.0001);
        }

        // Left Trigger: Decrease kI
        if (gamepad1.left_trigger > 0.5) {
            drive.setKI(Math.max(0, drive.getKI() - 0.0001));  // Don't go negative
        }

        // ======================== TELEMETRY ========================
        telemetry.addLine("======== PID TUNER ========");
        telemetry.addLine();

        // Current PID values (WRITE THESE DOWN when done!)
        telemetry.addLine("--- CURRENT VALUES ---");
        telemetry.addData("kP", "%.4f", drive.getKP());
        telemetry.addData("kI", "%.6f", drive.getKI());
        telemetry.addData("kD", "%.4f", drive.getKD());
        telemetry.addLine();

        // Heading status
        telemetry.addLine("--- HEADING STATUS ---");
        telemetry.addData("Lock Active", drive.isHeadingLockActive() ? "✓ YES" : "✗ NO");
        telemetry.addData("Error", "%.2f°", Math.toDegrees(drive.getHeadingError()));
        telemetry.addLine();

        // Controls reminder
        telemetry.addLine("--- TUNING CONTROLS ---");
        telemetry.addLine("D-Pad UP/DOWN: kP ±0.005");
        telemetry.addLine("D-Pad L/R: kD ±0.001");
        telemetry.addLine("Triggers: kI ±0.0001");
        telemetry.addLine("A Button: Reset Heading");
        telemetry.addLine();

        // Tuning advice
        telemetry.addLine("--- TUNING GUIDE ---");
        double errorDeg = Math.abs(Math.toDegrees(drive.getHeadingError()));
        if (drive.isHeadingLockActive()) {
            if (errorDeg > 5.0) {
                telemetry.addLine("⚠ Drifting - INCREASE kP");
            } else if (errorDeg < 0.5) {
                telemetry.addLine("✓ Good! Write down values");
            } else {
                telemetry.addLine("→ Small drift - adjust kP/kD");
            }
        } else {
            telemetry.addLine("Rotate to test lock");
        }

        telemetry.addLine("===========================");
        telemetry.update();
    }
}
*/