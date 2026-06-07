package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Motor Direction Test", group = "Test")
public class MotorTest extends OpMode {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private int testState = 0; // 0=idle, 1=FL, 2=BL, 3=FR, 4=BR
    private double testPower = 0.3;
    private boolean aButtonPressed = false;
    private boolean bButtonPressed = false;
    private boolean xButtonPressed = false;
    private boolean yButtonPressed = false;

    @Override
    public void init() {
        // Initialize all four motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "left_back_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "right_back_motor");

        // Set to RUN_WITHOUT_ENCODER for direct testing
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A=Front Left, B=Back Left, X=Front Right, Y=Back Right");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Stop all motors by default
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        // Test individual motors
        if (gamepad1.a && !aButtonPressed) {
            testState = (testState == 1) ? 0 : 1;
            aButtonPressed = true;
        }
        if (!gamepad1.a) {
            aButtonPressed = false;
        }

        if (gamepad1.b && !bButtonPressed) {
            testState = (testState == 2) ? 0 : 2;
            bButtonPressed = true;
        }
        if (!gamepad1.b) {
            bButtonPressed = false;
        }

        if (gamepad1.x && !xButtonPressed) {
            testState = (testState == 3) ? 0 : 3;
            xButtonPressed = true;
        }
        if (!gamepad1.x) {
            xButtonPressed = false;
        }

        if (gamepad1.y && !yButtonPressed) {
            testState = (testState == 4) ? 0 : 4;
            yButtonPressed = true;
        }
        if (!gamepad1.y) {
            yButtonPressed = false;
        }

        // Adjust test power with bumpers
        if (gamepad1.right_bumper) {
            testPower = Math.min(testPower + 0.01, 1.0);
        }
        if (gamepad1.left_bumper) {
            testPower = Math.max(testPower - 0.01, 0.0);
        }

        // Run selected motor
        switch (testState) {
            case 1:
                frontLeftMotor.setPower(testPower);
                break;
            case 2:
                backLeftMotor.setPower(testPower);
                break;
            case 3:
                frontRightMotor.setPower(testPower);
                break;
            case 4:
                backRightMotor.setPower(testPower);
                break;
        }

        // Telemetry
        telemetry.addData("Test Power", testPower);
        telemetry.addData("Active Motor", getMotorName(testState));
        telemetry.addData("", "");
        telemetry.addData("CONTROLS:", "");
        telemetry.addData("A - Front Left Motor", testState == 1 ? "ON" : "OFF");
        telemetry.addData("B - Back Left Motor", testState == 2 ? "ON" : "OFF");
        telemetry.addData("X - Front Right Motor", testState == 3 ? "ON" : "OFF");
        telemetry.addData("Y - Back Right Motor", testState == 4 ? "ON" : "OFF");
        telemetry.addData("LB/RB - Decrease/Increase Power", "");
        telemetry.update();
    }

    private String getMotorName(int state) {
        switch (state) {
            case 1: return "Front Left";
            case 2: return "Back Left";
            case 3: return "Front Right";
            case 4: return "Back Right";
            default: return "None";
        }
    }

    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}