package org.firstinspires.ftc.teamcode.BIOBUZZ.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class limelightAlignmentTurretAdvanced extends OpMode {
    private Limelight3A limelight;

    private final TurretMechanismAdvanced turret = new TurretMechanismAdvanced();

    private static final double MAX_STALENESS_MS = 100;

    double[] stepSizes = {0.1, 0.001, 0.0001, 0.00001};

    int stepIndex = 2;

    @Override
    public void init() {
        turret.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        limelight.start();

        turret.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        boolean hasTarget = result != null
                && result.isValid()
                && result.getStaleness() < MAX_STALENESS_MS;

        double tx = hasTarget ? result.getTx() : 0;

        turret.update(hasTarget, tx);

        if (gamepad1.yWasPressed()) {
            turret.resetTurret();
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }
        if (gamepad1.dpadRightWasPressed()) {
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadUpWasPressed()) {
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
        }

        if (hasTarget) {
            telemetry.addData("Target", "Found");
            telemetry.addData("tx (deg)", "%.2f", tx);
            telemetry.addData("Result age (ms)", result.getStaleness());
        } else {
            telemetry.addLine("No Target (none seen, or data stale)");
        }

        telemetry.addLine("-----------------------------");

        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", turret.getkP());
        telemetry.addData("Tuning D", "%.5f (D-Pad U/D)", turret.getkD());
        telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);

        telemetry.addLine("-----------------------------");

        telemetry.addData("Turret State", turret.getState());

        telemetry.addData("Turret Encoder Pos", turret.getCurrentPosition());
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}