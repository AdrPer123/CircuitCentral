package org.firstinspires.ftc.teamcode.DECODE_Archive.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class SimpleAutoPathing2 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, servoTimer, telemetryTimer;
    private DcMotorEx flywheel;
    private DcMotor intake_motor;
    private Servo launch_Trigger_Servo;
    private Double flywheelRPM;
    private boolean timerReset = false;
    private final int ticksPerRevolution = 28; // Set once at declaration
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

    private final Pose startPose = new Pose(22.174496644295296, 124.9395973154362, Math.toRadians(144));
    private final Pose intakePose1 = new Pose(32.61208053691274, 84.12885906040268, Math.toRadians(180));
    private final Pose shootPos = new Pose(55.204026845637586, 78.055033557047, Math.toRadians(128));
    private final Pose intakePose2 = new Pose(33.38523489932886, 59.918120805369135, Math.toRadians(180));
    private final Pose intakePose3 = new Pose(35.265771812080544, 35.472483221476516, Math.toRadians(180));
    private PathChain startIntake1Shoot1;
    private final Pose startIntake1ControlPoint1 = new Pose(59.92065553701722, 86.59664429530206, Math.toRadians(0));
    private final Pose intake1Shoot1ControlPoint1 = new Pose(12.108053691275149, 78.055033557047, Math.toRadians(0));
    private final Pose intake1Shoot1ControlPoint2 = new Pose(12.563758389261743, 72.28993288590604, Math.toRadians(0));
    private PathChain intake2Shoot2;

    private final Pose shoot1Intake2ControlPoint1 = new Pose(48.197986577181204, 59.35167785234897, Math.toRadians(0));
    private final Pose intake2Shoot2ControlPoint1 = new Pose(2.10536912751678, 56.37181208053691, Math.toRadians(0));
    private final Pose intake2Shoot2ControlPoint2 = new Pose(26.686577181208055, 74.34765100671142, Math.toRadians(0));
    private PathChain intake3Shoot3;
    private final Pose shoot2Intake3ControlPoint1 = new Pose(50.69932885906039, 38.08590604026848, Math.toRadians(0));
    private final Pose intake3Shoot3ControlPoint1 = new Pose(9.061744966442962, 30.803355704698028, Math.toRadians(0));
    private final Pose intake3Shoot3ControlPoint2 = new Pose(6.76510067114094, 40.97718120805369, Math.toRadians(0));
    private int shootingSequence = 1;
    private double ticksPerSecond;

    public void buildPaths() {
        startIntake1Shoot1 = follower.pathBuilder()
                //Move from Start to Shoot
                .addPath(new BezierCurve(startPose, startIntake1ControlPoint1, intakePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), intakePose1.getHeading())
                .addPath(new BezierCurve(intakePose1,intake1Shoot1ControlPoint1, intake1Shoot1ControlPoint2, shootPos))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), shootPos.getHeading())
                .build();
        intake2Shoot2 = follower.pathBuilder()
                //Drive from shooting spot to intake 2
                .addPath(new BezierCurve(shootPos,shoot1Intake2ControlPoint1,intakePose2))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakePose2.getHeading())
                //Drive from intake 2 to shooting spot
                .addPath(new BezierCurve(intakePose2, intake2Shoot2ControlPoint1, intake2Shoot2ControlPoint2, shootPos))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), shootPos.getHeading())
                .build();
        intake3Shoot3 = follower.pathBuilder()
                //Drive from shooting spot to intake 3
                .addPath(new BezierCurve(shootPos,shoot2Intake3ControlPoint1, intakePose3))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakePose3.getHeading())
                //Drive from intake 3 to shooting spot
                .addPath(new BezierCurve(intakePose3, intake3Shoot3ControlPoint1, intake3Shoot3ControlPoint2, shootPos))
                .setLinearHeadingInterpolation(intakePose3.getHeading(), shootPos.getHeading())
                .build();

    }


    public void statePathUpdate() {
        switch (pathState) {
            case START_INTAKE1_SHOOT:
                setFlywheelRPM(1500.0);
                intake_motor.setPower(1);
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
                        if (servoTimer.getElapsedTimeSeconds() > 1 && launch_Trigger_Servo.getPosition() <= Servo.MIN_POSITION+0.05) {
                            setPathState(PathState.INTAKE_2); // Transition to next move
                            resetServo();
                        }
                    }
                    else if (shootingSequence == 2 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        if (servoTimer.getElapsedTimeSeconds() > 1 && launch_Trigger_Servo.getPosition() <= Servo.MIN_POSITION+0.05) {
                            setPathState(PathState.INTAKE_3); // Transition to next move
                            resetServo();
                        }
                    }
                    else if (shootingSequence == 3 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        // power servos to launch balls
                        if (servoTimer.getElapsedTimeSeconds() > 1 && launch_Trigger_Servo.getPosition() <= Servo.MIN_POSITION+0.05) {
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
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }
    // Pre-calculate tolerance once
    private double velocityToleranceHigh;
    private double velocityToleranceLow;

    public void setFlywheelRPM(double targetRPM) {
        ticksPerSecond = (targetRPM / 60.0) * ticksPerRevolution;
        velocityToleranceHigh = ticksPerSecond + (ticksPerRevolution * 3);
        velocityToleranceLow = ticksPerSecond - (ticksPerRevolution * 2);
        flywheel.setVelocity(ticksPerSecond);
    }

    public boolean flywheelIsCorrectSpeed() {
        double velocity = flywheel.getVelocity();
        return (velocity <= velocityToleranceHigh && velocity >= velocityToleranceLow);
    }

    // Move telemetry to updateTelemetry() method
    private void updateTelemetry() {
        flywheelVelocity = flywheel.getVelocity();
        flywheelRPM = (flywheelVelocity / ticksPerRevolution) * 60;
        telemetry.addData("Flywheel RPM", "%.0f / %.0f", flywheelRPM, targetRPM);
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
        // ... other telemetry
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void launch_balls() {
        if (!timerReset) {
            // releases ball
            launch_Trigger_Servo.setPosition(Servo.MIN_POSITION);
            servoTimer.resetTimer();
            timerReset = true;
        }
    }

    public void resetServo() {
        timerReset = false;
        // blocks balls
        launch_Trigger_Servo.setPosition(Servo.MAX_POSITION);
    }

    @Override
    public void init() {
        pathState = PathState.START_INTAKE1_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        servoTimer = new Timer();
        telemetryTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Best for high RPM stability
        flywheel.setVelocityPIDFCoefficients(20.0, 0.0, 5.0, 12.5); // TODO TUNE PIDF VALUES
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Let it spin down naturally
        launch_Trigger_Servo = hardwareMap.get(Servo.class, "launch_Trigger_Servo");
        launch_Trigger_Servo.scaleRange(0.0, 1.0);
        intake_motor = hardwareMap.get(DcMotor.class, "intake");
        // TODO add in any other init mechanicms
        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        if (telemetryTimer.getElapsedTimeSeconds() > 0.1) {
            updateTelemetry();
            telemetryTimer.resetTimer();
        }
    }
}