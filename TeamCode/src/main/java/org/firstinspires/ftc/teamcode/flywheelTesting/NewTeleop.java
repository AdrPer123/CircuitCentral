package org.firstinspires.ftc.teamcode.flywheelTesting;
/*
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Teleop.MecanumDrive;
@TeleOp
public class NewTeleop extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Flywheelsubsystem flywheel = new Flywheelsubsystem();

    private boolean isFlywheelRunning = false;
    private boolean lbWasPressed = false;
    private boolean rbWasPressed = false;
    private boolean yButtonWasPressed = false;
    private boolean isIntakeRunning = false;
    private final double[] outtakePowers = {1000.0, 2000.0, 3000.0, 4000.0};
    private int powerIndex = 0;
    private double currentTargetPower;


    public void init() {
        drive.init(hardwareMap);
        flywheel.init(hardwareMap);
        flywheel.setKF(0.000139);
        flywheel.setKP(0.000544);
        flywheel.setKI(0.0);
        flywheel.setKD(0.000003);
    }


    public void start() {
        drive.getImu().resetYaw();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        telemetry.addData("outtake left velocity: ", flywheel.getMotor1VelocityRPM());
        telemetry.addData("outtake right velocity: ", flywheel.getMotor2VelocityRPM());
        telemetry.addData("target power: ", outtakePowers[powerIndex]);

        drive.driveFieldRelative(forward, strafe, rotate);
        /*
        if (gamepad1.right_bumper && !rbWasPressed) {
            isIntakeRunning = !isIntakeRunning;
        }
        // Update the button state for the next loop
        rbWasPressed = gamepad1.right_bumper;

         */
/*
        if (gamepad1.y && !yButtonWasPressed) {
            powerIndex = (powerIndex + 1) % outtakePowers.length;
        }
        yButtonWasPressed = gamepad1.y;

        if (gamepad1.right_bumper && !lbWasPressed) {

            isFlywheelRunning = !isFlywheelRunning;
            currentTargetPower = outtakePowers[powerIndex];
        }

        lbWasPressed = gamepad1.left_bumper;

        if (isIntakeRunning) {
        } else {
        }

        if (isFlywheelRunning) {
            flywheel.setTargetVelocity(currentTargetPower);
        }
        else {
            telemetry.addData("Flywheel", "Off");
        }
        flywheel.update();
    }
}
*/

