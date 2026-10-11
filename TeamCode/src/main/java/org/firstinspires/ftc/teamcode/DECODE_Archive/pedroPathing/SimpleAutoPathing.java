package org.firstinspires.ftc.teamcode.DECODE_Archive.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class SimpleAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, servoTimer;
    private DcMotorEx flywheel;
    private Servo launch_Trigger_Servo;
    private Double flywheelRPM;
    private boolean timerReset = false;
    private int ticksPerRevolution;
    private double flywheelVelocity;
    private double targetRPM;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        START_INTAKE1_SHOOT,
        SHOOT_PRELOAD,
        INTAKE_2,
        INTAKE_3,
        Park
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.40134228187919, 123.77986577181207, Math.toRadians(143));
    private final Pose intakePose1 = new Pose(19.081879194630872, 84.12885906040268, Math.toRadians(180));
    private final Pose shootPos = new Pose(57.34131736526946, 84.12885906040268, Math.toRadians(134));
    private final Pose intakePose2 = new Pose(19.081879194630872, 59.84161073825504, Math.toRadians(180));
    private final Pose intakePose3 = new Pose(19.081879194630872, 35.49261744966442, Math.toRadians(180));
    private PathChain startIntake1Shoot1;
    private final Pose startIntake1ControlPoint = new Pose(91.42550335570469, 79.05503355704695, Math.toRadians(0));
    private PathChain intake2Shoot2;
    private final Pose shoot1Intake2ControlPoint = new Pose(67.13978619941327, 55.59328859060401, Math.toRadians(0));
    private PathChain intake3Shoot3;
    private final Pose shoot2Intake3ControlPoint = new Pose(59.820994253104516, 34.33422818791943, Math.toRadians(0));
    private int shootingSequence = 1;
    private double ticksPerSecond;

    public void buildPaths() {
        startIntake1Shoot1 = follower.pathBuilder()
                //Move from Start to Shoot
                .addPath(new BezierCurve(startPose, startIntake1ControlPoint, intakePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), intakePose1.getHeading())
                .addPath(new BezierLine(intakePose1, shootPos))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), shootPos.getHeading())
                .build();
        intake2Shoot2 = follower.pathBuilder()
                //Drive from shooting spot to intake 2
                .addPath(new BezierCurve(shootPos,shoot1Intake2ControlPoint,intakePose2))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakePose2.getHeading())
                //Drive from intake 2 to shooting spot
                .addPath(new BezierLine(intakePose2, shootPos))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), shootPos.getHeading())
                .build();
        intake3Shoot3 = follower.pathBuilder()
                //Drive from shooting spot to intake 3
                .addPath(new BezierCurve(shootPos,shoot2Intake3ControlPoint, shoot2Intake3ControlPoint,intakePose3))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakePose3.getHeading())
                //Drive from intake 3 to shooting spot
                .addPath(new BezierLine(intakePose3, shootPos))
                .setLinearHeadingInterpolation(intakePose3.getHeading(), shootPos.getHeading())
                .build();

    }


    public void statePathUpdate() {
        switch (pathState) {
            case START_INTAKE1_SHOOT:
                targetRPM = 1500.0;
                setFlywheelRPM(targetRPM);
                if (flywheelIsCorrectSpeed()) {
                    launch_balls();
                    if (servoTimer.getElapsedTimeSeconds() > 1 ) {
                        //moves on to intake balls
                        follower.followPath(startIntake1Shoot1, true);
                        setPathState(PathState.SHOOT_PRELOAD);
                        resetServo();
                        timerReset = false;
                    }
                }
                break;
            case SHOOT_PRELOAD:
                //speeds up flywheel while intaking
                targetRPM = 3000.0;
                setFlywheelRPM(targetRPM);
                if (!follower.isBusy()) {
                    // 3. Move to the next state once shot is fired
                    if (shootingSequence == 1 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        // power servos to launch balls
                        if (servoTimer.getElapsedTimeSeconds() > 1) {
                            setPathState(PathState.INTAKE_2); // Transition to next move
                            resetServo();
                        }
                    }
                    else if (shootingSequence == 2 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        // power servos to launch balls
                        if (servoTimer.getElapsedTimeSeconds() > 1) {
                            setPathState(PathState.INTAKE_3); // Transition to next move
                            resetServo();
                        }
                    }
                    else if (shootingSequence == 3 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        // power servos to launch balls
                        if (servoTimer.getElapsedTimeSeconds() > 1) {
                            setPathState(PathState.Park); // Transition to next move
                            resetServo();
                        }
                    }
                }
                telemetry.addLine("Done Path 1");
                break;
            case INTAKE_2:
                follower.followPath(intake2Shoot2, true);
                shootingSequence = 2;
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case INTAKE_3:
                follower.followPath(intake3Shoot3, true);
                shootingSequence = 3;
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case Park:
                telemetry.addLine("Completed Autonomous");
                flywheel.setPower(0);

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }
    public void setFlywheelRPM(double targetRPM) {
        ticksPerRevolution = 28;
        ticksPerSecond = (targetRPM / 60.0) * ticksPerRevolution;
        flywheel.setVelocity(ticksPerSecond);
    }
    public boolean flywheelIsCorrectSpeed() {
        flywheelVelocity = flywheel.getVelocity();
        flywheelRPM = (flywheelVelocity / ticksPerRevolution) * 60;
        telemetry.addLine("Flywheel current RPM: " + flywheelRPM);
        telemetry.addLine("Flywheel target RPM: " + targetRPM);
        return (flywheel.getVelocity() <= (ticksPerSecond + ticksPerRevolution*2) && flywheel.getVelocity() >= (ticksPerSecond - ticksPerRevolution*2));
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void launch_balls() {
        if (!timerReset && flywheelIsCorrectSpeed()) {
            launch_Trigger_Servo.setPosition(1);
            servoTimer.resetTimer();
            timerReset = true;
        }
    }

    public void resetServo() {
        timerReset = false;
        launch_Trigger_Servo.setPosition(0);
    }

    @Override
    public void init() {
        pathState = PathState.START_INTAKE1_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        servoTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Best for high RPM stability
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Let it spin down naturally
        launch_Trigger_Servo = hardwareMap.get(Servo.class, "launch_Trigger_Servo");
        // TODO
        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        statePathUpdate();
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }
}