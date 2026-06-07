package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeSetup {
    public DcMotorEx outtakeMotorLeft, outtakeMotorRight, intakeMotor;
    private Servo release_servo, hood_servo;

    public double hood_position = 0.3;

    public void init(HardwareMap hwMap) {
        // Initialize motors
        release_servo = hwMap.get(Servo.class, "release_servo");
        hood_servo = hwMap.get(Servo.class, "hood_servo");
        release_servo.setDirection(Servo.Direction.REVERSE);
        outtakeMotorLeft = hwMap.get(DcMotorEx.class, "outtake_motor_left");
        outtakeMotorRight = hwMap.get(DcMotorEx.class, "outtake_motor_right");
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeMotorRight.setDirection(DcMotorEx.Direction.FORWARD);
        PIDFCoefficients pidf = new PIDFCoefficients(75.11, 0, 0, 13.4600);
        outtakeMotorLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        outtakeMotorRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        intakeMotor = hwMap.get(DcMotorEx.class, "intake_motor");

        // Set directions
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior - FLOAT for flywheels!
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setOuttakePow(double power) {
        outtakeMotorLeft.setPower(power);
        outtakeMotorRight.setPower(power);
    }
    public void setIntakePow(double power) {
        intakeMotor.setPower(power);
    }

    public void setOuttakeVelocity(double ticksPerSecond) {
        outtakeMotorLeft.setVelocity(ticksPerSecond);
        outtakeMotorRight.setVelocity(ticksPerSecond);
    }
    public void Servo_release() {
        release_servo.setPosition(0.87);
    }

    public void hood_servo_max() {
        hood_servo.setPosition(0.0);
    }
    public void hood_servo_min() {
        hood_servo.setPosition(0.6);
    }
    public void hood_servo_adjust(double position_change) {
        hood_position = hood_position + position_change;
        hood_servo.setPosition(hood_position);
    }


    public void Servo_reset(){
        release_servo.setPosition(0.67);
    }


    public double getOuttakeVelocityRight() {
        return outtakeMotorRight.getVelocity();
    }

    public double getOuttakeVelocityLeft() {
        return outtakeMotorLeft.getVelocity();
    }

}