package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple; // Added for direction
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Outtake Test")
public class OuttakeTest extends OpMode {
    private DcMotor outtakeMotorLeft, outtakeMotorRight, intakeMotor;
    private Servo launcherServo;
    MecanumDrive drive = new MecanumDrive();
    @Override
    public void init() {
        // Use the built-in hardwareMap, don't pass it as a parameter
        outtakeMotorLeft = hardwareMap.get(DcMotor.class, "outtake_motor_left");
        outtakeMotorRight = hardwareMap.get(DcMotor.class, "outtake_motor_right");
        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        // launcherServo = hardwareMap.get(Servo.class, "launcher_servo");
            // Optional: Set a starting position so it doesn't "jump" when you press Start
        //launcherServo.setPosition(0.0);


        // Reset encoders
        // Scaled for the REV Hub's 16-bit controller
// P = 19.66, I = 0.0, D = 0.0, F = 11.7

        //outtakeMotorLeft.setVelocityPIDFCoefficients(0.0002, 0.0, 0.0, 0.000139);
        //outtakeMotorRight.setVelocityPIDFCoefficients(0.0002, 0.0, 0.0, 0.000139);
        // One motor usually needs to be reversed for a dual-flywheel setup
        outtakeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        outtakeMotorRight.setPower(0.7);
        outtakeMotorLeft.setPower(0.7);
        drive.driveFieldRelative(forward, strafe, rotate);
        intakeMotor.setPower(1.0);
    }
}
