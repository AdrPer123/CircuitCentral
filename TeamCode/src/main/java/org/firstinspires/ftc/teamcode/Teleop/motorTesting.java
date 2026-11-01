package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class motorTesting extends OpMode {
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    OuttakeSetup outtake = new OuttakeSetup();
    @Override
    public void init () {
        outtake.init(hardwareMap);

        frontLeftMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "left_back_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "right_back_motor");
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            frontLeftMotor.setPower(0.5);
        } else {
            frontLeftMotor.setPower(0);
        }

        if (gamepad1.dpad_down) {
            backLeftMotor.setPower(0.5);
        } else {
            backLeftMotor.setPower(0);
        }

        if (gamepad1.y) {
            frontRightMotor.setPower(0.5);
        } else {
            frontRightMotor.setPower(0);
        }

        if (gamepad1.a) {
            backRightMotor.setPower(0.5);
        } else {
            backRightMotor.setPower(0);
        }
    }
}
