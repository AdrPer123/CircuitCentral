package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ControlledOpMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    OuttakeSetup outtake = new OuttakeSetup();

    private boolean isFlywheelRunning = true;
    private boolean lbWasPressed = false;
    private boolean rbWasPressed = false;
    private boolean yButtonWasPressed = false;
    private boolean isIntakeOn = false;
    private boolean isIntakeRunning = true;

    private final double[] outtakePowers = {0.0, 1100.0, 1250.0, 1500.0};
    private int powerIndex = 0;
    private double currentTargetPower;
    private Timer releaseTime;
    private Boolean timerState = false;


    public void init() {
        drive.init(hardwareMap);
        outtake.init(hardwareMap);
        releaseTime = new Timer();
        releaseTime.resetTimer();
    }


    public void start() {
        drive.getImu().resetYaw();
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            outtake.Servo_release();
        }
        else {
            outtake.Servo_reset();
        }
        outtake.setOuttakeVelocity(currentTargetPower);
        double forward = drive.squareWithSign(-gamepad1.left_stick_y);
        double strafe = drive.squareWithSign(gamepad1.left_stick_x);
        double rotate = drive.squareWithSign(gamepad1.right_stick_x);
        drive.drive(forward, strafe, rotate);
        telemetry.addData("target power: ", outtakePowers[powerIndex]);
        currentTargetPower = outtakePowers[powerIndex];
        outtake.setOuttakeVelocity(currentTargetPower);
        if (gamepad1.rightBumperWasPressed()) {
            isIntakeOn = !isIntakeOn;
            }
        if (isIntakeOn) {
            outtake.setIntakePow(1.0);
        }
        else {
            outtake.setIntakePow(0.0);
        }
        if (gamepad1.yWasPressed()) {
            powerIndex = (powerIndex + 1) % outtakePowers.length;
        }

        if (gamepad1.dpadUpWasPressed()) {
            outtake.hood_servo_adjust(-0.1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            outtake.hood_servo_adjust(0.1);
        }
        /*
        if (gamepad1.a){
            outtake.Servo_release();
        }
        if (gamepad1.b){
            outtake.Servo_reset();
        }
         */
        // Update the button state for the next loop
        telemetry.addData("Flywheel", "Off");
        telemetry.addData("Target TPS: ", currentTargetPower);
        telemetry.addData("Left Velocity", outtake.getOuttakeVelocityLeft());
        telemetry.addData("Right Velocity", outtake.getOuttakeVelocityRight());
        telemetry.update();
    }
}




