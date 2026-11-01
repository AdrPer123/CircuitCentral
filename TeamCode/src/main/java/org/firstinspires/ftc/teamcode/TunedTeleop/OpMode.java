package org.firstinspires.ftc.teamcode.TunedTeleop;

/*
@TeleOp (name = "NewFieldOpMode", group = "Movement")
public class NewFieldOpModePID extends OpMode {
    MecanumPID drive = new MecanumPID();
    OuttakeSetup outtake = new OuttakeSetup();

    private boolean isFlywheelRunning = false;
    private boolean lbWasPressed = false;
    private boolean rbWasPressed = false;
    private boolean yButtonWasPressed = false;
    private boolean isIntakeRunning = false;

    private final double[] outtakePowers = {0.25, 0.5, 0.75, 1};
    private int powerIndex = 0;
    private double currentTargetPower;

    @Override
    public void init() {
        drive.init(hardwareMap);
        outtake.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Mode", "Field-Oriented with Heading Lock");
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset the IMU heading to 0 when teleop starts
        drive.getImu().resetYaw();

        // Initialize the PID heading lock to current position
        drive.resetHeading();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ======================== DRIVE CONTROLS ========================
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Use heading lock for better control
        drive.driveFieldRelativeWithHeadingLock(forward, strafe, rotate);

        // ======================== INTAKE CONTROLS ========================
        if (gamepad1.right_bumper && !rbWasPressed) {
            isIntakeRunning = !isIntakeRunning;
        }
        rbWasPressed = gamepad1.right_bumper;

        // ======================== POWER SELECTION ========================
        if (gamepad1.y && !yButtonWasPressed) {
            powerIndex = (powerIndex + 1) % outtakePowers.length;
        }
        yButtonWasPressed = gamepad1.y;

        // ======================== FLYWHEEL CONTROLS ========================
        if (gamepad1.left_bumper && !lbWasPressed) {
            isFlywheelRunning = !isFlywheelRunning;
            currentTargetPower = outtakePowers[powerIndex];
        }
        lbWasPressed = gamepad1.left_bumper;

        // ======================== APPLY POWERS ========================
        if (isIntakeRunning) {
            outtake.setIntakePow(0.7);
            outtake.setOuttakePow(currentTargetPower);
        } else {
            outtake.setIntakePow(0);
            outtake.setOuttakePow(0);
        }

        // ======================== TELEMETRY ========================
        telemetry.addData("=== DRIVE ===", "");
        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Rotate Input", "%.2f", rotate);

        telemetry.addData("=== HEADING LOCK ===", "");
        telemetry.addData("Lock Active", drive.isHeadingLockActive() ? "YES" : "NO");
        telemetry.addData("Error", "%.1f°", Math.toDegrees(drive.getHeadingError()));

        telemetry.addData("=== PID VALUES ===", "");
        telemetry.addData("kP", "%.4f", drive.getKP());
        telemetry.addData("kI", "%.4f", drive.getKI());
        telemetry.addData("kD", "%.4f", drive.getKD());

        telemetry.addData("=== INTAKE/OUTTAKE ===", "");
        telemetry.addData("Intake", isIntakeRunning ? "ON" : "OFF");
        telemetry.addData("Power", "%.0f%% (%d/4)", outtakePowers[powerIndex] * 100, powerIndex + 1);

        telemetry.update();
    }
}

 */