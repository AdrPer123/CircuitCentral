package org.firstinspires.ftc.teamcode.BIOBUZZ.LimeLight;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanismAdvanced {
    public enum State {
        IDLE,
        TRACKING,
        AT_TARGET,
        LIMIT
    }

    public static DcMotorEx turret;

    private double kP = 0.014;
    private double kD = 0.000001;
    private double kV = 0.005;

    private final double goalX = 0;
    private final double angleTolerance = 0.2;
    private final double MAX_POWER = 0.6;

    private static final boolean REVERSE_MOTOR = false;

    private final boolean limitsEnabled = false;
    private final int MIN_TICKS = -1000;
    private final int MAX_TICKS = 1000;

    private double lastError = 0;
    private boolean hasLastError = false;
    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void start() {
        timer.reset();
        hasLastError = false;
    }

    public void update(boolean hasTarget, double tx) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (!hasTarget) {
            turret.setPower(0);
            hasLastError = false;
            state = State.IDLE;
            return;
        }

        double error = goalX - tx;

        double pTerm = error * kP;

        double dTerm = 0;
        if (hasLastError && deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }
        hasLastError = true;

        double kVTerm = Math.signum(error) * kV;

        double power;
        if (Math.abs(error) < angleTolerance) {
            power = 0;
            state = State.AT_TARGET;
        } else {
            power = Range.clip(pTerm + dTerm + kVTerm, -MAX_POWER, MAX_POWER);
            state = State.TRACKING;
        }

        if (limitsEnabled) {
            int pos = turret.getCurrentPosition();
            if ((pos >= MAX_TICKS && power > 0) || (pos <= MIN_TICKS && power < 0)) {
                power = 0;
                state = State.LIMIT;
            }
        }

        turret.setPower(power);
        lastError = error;
    }

    public void resetTurret() {
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hasLastError = false;
    }

    public State getState() {
        return state;
    }

    public int getCurrentPosition() {
        return turret.getCurrentPosition();
    }

    public double getkP() { return kP; }
    public void setkP(double newKP) { kP = Math.max(0, newKP); }

    public double getkD() { return kD; }
    public void setkD(double newKD) { kD = Math.max(0, newKD); }

    public double getkV() { return kV; }
    public void setkV(double newKV) { kV = Math.max(0, newKV); }




}