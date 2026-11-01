package org.firstinspires.ftc.teamcode.flywheelTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp
public class FlywheelTunerTutorial extends OpMode {

    public DcMotorEx outtakeMotorLeft, outtakeMotorRight;
    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 14.2600;
    double P = 4.1100;
    double I = 0.0;
    double D = 0.0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 0;
//F:14.2600, P: 4.1100

    @Override
    public void init() {
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outtake_motor_left");
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outtake_motor_right");
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeMotorRight.setDirection(DcMotorEx.Direction.FORWARD);
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        outtakeMotorLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        outtakeMotorRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        telemetry.addLine("Init complete");
    }
// 4.91, 16.26

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }


        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.leftBumperWasPressed()) {
            I += stepSizes[stepIndex];
        }
        if (gamepad1.left_trigger_pressed) {
            I -= stepSizes[stepIndex];
        }

        if (gamepad1.rightBumperWasPressed()) {
            D += stepSizes[stepIndex];
        }
        if (gamepad1.right_trigger_pressed) {
            D -= stepSizes[stepIndex];
        }
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        outtakeMotorLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        outtakeMotorRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        outtakeMotorLeft.setVelocity(curTargetVelocity);
        outtakeMotorRight.setVelocity(curTargetVelocity);
        double curVelocity = (outtakeMotorLeft.getVelocity() + outtakeMotorRight.getVelocity()) / 2;
        double error = curVelocity - curTargetVelocity;
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Left Velocity", outtakeMotorLeft.getVelocity());
        telemetry.addData("Right Velocity", outtakeMotorRight.getVelocity());
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-pad L/R)", F);
        telemetry.addData("Tuning I", "%.4f (D-pad U/D)", I);
        telemetry.addData("Tuning D", "%.4f (D-pad L/R)", D);


    }
}