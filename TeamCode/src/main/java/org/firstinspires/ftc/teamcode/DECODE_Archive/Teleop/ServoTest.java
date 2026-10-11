package org.firstinspires.ftc.teamcode.DECODE_Archive.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {
    private Servo release_servo, hood_servo;
    private double release_servo_pos, hood_servo_pos;

    private boolean dpadup, dpaddown;


    @Override
    public void init() {
        release_servo = hardwareMap.get(Servo.class, "release_servo");
        hood_servo = hardwareMap.get(Servo.class, "hood_servo");
        release_servo.setDirection(Servo.Direction.REVERSE);
        hood_servo_pos = 0.5;
        //scaleRange(double min, double max)
    }
    @Override
    public void start() {
//        hood_servo.setPosition(hood_servo_pos);
    }
    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            release_servo.setPosition(0.9);
        }
        if (gamepad1.aWasPressed()) {
            release_servo.setPosition(0.57);
        }

        if (gamepad1.dpadLeftWasPressed()) {
            release_servo_pos += 0.05;
            release_servo.setPosition(release_servo_pos);
        }
        if (gamepad1.dpadRightWasPressed()){
            release_servo_pos -= 0.05;
            release_servo.setPosition(release_servo_pos);
        }

        if(gamepad1.dpadUpWasPressed()){
            hood_servo_pos += 0.05;
            hood_servo.setPosition(hood_servo_pos);

        }
        if(gamepad1.dpadDownWasPressed()) {
            hood_servo_pos -= 0.05;
            hood_servo.setPosition(hood_servo_pos);

        }
        //perfect position is 0.15 for hood extended, 0.8 for hood retracted
        if (gamepad1.xWasPressed()) {
            hood_servo.setPosition(0.8);
        }
        if (gamepad1.yWasPressed()) {
            hood_servo.setPosition(0.15);
        }

        /*
        if (gamepad1.xWasPressed()) {
            hood_servo.setPosition(1);
        }
        if (gamepad1.yWasPressed()) {
            hood_servo.setPosition(0);
        }

         */

        telemetry.addData("release_servo position", release_servo.getPosition());
        telemetry.addData("hood_servo position", hood_servo.getPosition());
        telemetry.update();

    }
}