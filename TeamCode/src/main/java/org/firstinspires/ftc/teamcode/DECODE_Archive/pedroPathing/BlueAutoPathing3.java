package org.firstinspires.ftc.teamcode.DECODE_Archive.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DECODE_Archive.Teleop.OuttakeSetup;
// ~/Library/Android/sdk/platform-tools/adb connect 192.168.43.1:5555
@Autonomous
public class BlueAutoPathing3 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, servoTimer, telemetryTimer;
    private boolean timerReset = false;
    private double targetTPS;
    private Servo hood_servo;

    private OuttakeSetup outtake = new OuttakeSetup();

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        START_EARLY_SHOOT,
        INTAKE1,
        SHOOT_PRELOAD,
        INTAKE_2,
        INTAKE_3,
        Park
    }

    PathState pathState = PathState.START_EARLY_SHOOT;

    private final Pose startPose = new Pose(21.208053691275158, 124.35973154362415, Math.toRadians(144));
    private final Pose earlyShoot = new Pose(58.73422818791945, 98.99060402684565, Math.toRadians(141));
    private final Pose intakePose1 = new Pose(39, 84.12885906040268, Math.toRadians(180));
    private final Pose shootPos = new Pose(55, 81, Math.toRadians(135));
    private final Pose intakePose2 = new Pose(39, 59.918120805369135, Math.toRadians(180));
    private final Pose intakePose3 = new Pose(41, 35.472483221476516, Math.toRadians(180));
    private final Pose park = new Pose(38.138255033557044, 64.944966442953, Math.toRadians(90));

    private PathChain shoot3park, startEarlyshoot;
    private PathChain startIntake1Shoot1;
    private final Pose earlyShootIntake1ControlPoint1 = new Pose(61.466964261849434, 81.58480865812676, Math.toRadians(0));
    private final Pose intake1Shoot1ControlPoint1 = new Pose(10.755033557046964, 86.59664429530206, Math.toRadians(0));
    private final Pose intake1Shoot1ControlPoint2 = new Pose(0, 74.99597315436242, Math.toRadians(0));
    private PathChain intake2Shoot2;

    private final Pose shoot1Intake2ControlPoint1 = new Pose(62.30805369127517, 59.35167785234897, Math.toRadians(0));
    private final Pose intake2Shoot2ControlPoint1 = new Pose(3.479194630872483, 53.85906040268453, Math.toRadians(0));
    private final Pose intake2Shoot2ControlPoint2 = new Pose(6.571812080536913, 77.50872483221478, Math.toRadians(0));
    private final Pose intake2Shoot2ControlPoint3 = new Pose(33.94429530201343, 83.21610738255033, Math.toRadians(0));
    private PathChain intake3Shoot3;
    private final Pose shoot2Intake3ControlPoint1 = new Pose(54.69932885906039, 38.08590604026848, Math.toRadians(0));
    private final Pose intake3Shoot3ControlPoint1 = new Pose(0.7503355704698078, 31.76979865771816, Math.toRadians(0));
    private final Pose intake3Shoot3ControlPoint2 = new Pose(0.19328859060402698, 34.79194630872482, Math.toRadians(0));
    private int shootingSequence = 1;
    private double currentVelocity;

    public void buildPaths() {
        startEarlyshoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,earlyShoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), earlyShoot.getHeading())
                .build();
        startIntake1Shoot1 = follower.pathBuilder()
                //Move from Start to Shoot
                .addPath(new BezierCurve(earlyShoot, earlyShootIntake1ControlPoint1, intakePose1))
                .setLinearHeadingInterpolation(earlyShoot.getHeading(), intakePose1.getHeading())
                .addPath(new BezierCurve(intakePose1,intake1Shoot1ControlPoint1, intake1Shoot1ControlPoint2, shootPos))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), shootPos.getHeading())
                .build();
        intake2Shoot2 = follower.pathBuilder()
                //Drive from shooting spot to intake 2
                .addPath(new BezierCurve(shootPos,shoot1Intake2ControlPoint1,intakePose2))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakePose2.getHeading())
                //Drive from intake 2 to shooting spot
                .addPath(new BezierCurve(intakePose2, intake2Shoot2ControlPoint1, intake2Shoot2ControlPoint2, intake2Shoot2ControlPoint3, shootPos))
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
        shoot3park = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, park))
                .setLinearHeadingInterpolation(shootPos.getHeading(), park.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch (pathState) {
            case START_EARLY_SHOOT:
                hood_servo.setPosition(0.2);
                outtake.setIntakePow(1.0);
                targetTPS = 1100.0;
                setFlywheelTPS(targetTPS);
                follower.followPath(startEarlyshoot, true);
                setPathState(PathState.INTAKE1);
                break;
            case INTAKE1:
                if (flywheelIsCorrectSpeed() && !follower.isBusy()) {
                    launch_balls();
                    if (servoTimer.getElapsedTimeSeconds() > 2.0 && timerReset) {
                        //moves on to intake balls
                        follower.followPath(startIntake1Shoot1, true);
                        setPathState(PathState.SHOOT_PRELOAD);
                        resetServo();
                    }
                }
                break;
            case SHOOT_PRELOAD:
                //speeds up flywheel while intaking
                targetTPS = 1175.0;
                setFlywheelTPS(targetTPS);
                if (!follower.isBusy()) {
                    // 3. Move to the next state once shot is fired
                    if (shootingSequence == 1 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        if (servoTimer.getElapsedTimeSeconds() > 2.5 &&timerReset) {
                            setPathState(PathState.INTAKE_2); // Transition to next move
                            resetServo();
                        }
                    }
                    else if (shootingSequence == 2 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        if (servoTimer.getElapsedTimeSeconds() > 2.5 && timerReset) {
                            setPathState(PathState.INTAKE_3); // Transition to next move
                            resetServo();
                        }
                    }
                    else if (shootingSequence == 3 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        // power servos to launch balls
                        if (servoTimer.getElapsedTimeSeconds() > 2.5 && timerReset) {
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
                follower.followPath(shoot3park, true);
                telemetry.addLine("Completed Autonomous");
                outtake.setOuttakeVelocity(0);
                break;

            default:
                telemetry.addLine("No State Commanded") ;
                break;
        }
    }
    // Pre-calculate tolerance once
    private double velocityToleranceHigh;
    private double velocityToleranceLow;

    public void setFlywheelTPS(double targetTPS) {
        velocityToleranceHigh = targetTPS * 1.03;
        velocityToleranceLow = targetTPS * 0.97;
        outtake.setOuttakeVelocity(targetTPS);
    }

    public boolean flywheelIsCorrectSpeed() {
        currentVelocity = (outtake.getOuttakeVelocityLeft()+ outtake.getOuttakeVelocityRight())/2;
        return (currentVelocity <= velocityToleranceHigh && currentVelocity >= velocityToleranceLow);
    }

    // Move telemetry to updateTelemetry() method
    private void updateTelemetry() {
        telemetry.addData("Flywheel TPS", "%.0f / %.0f", currentVelocity, targetTPS);
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
            outtake.Servo_release();
            servoTimer.resetTimer();
            timerReset = true;
            outtake.setIntakePow(0.0);
        }
        if (timerReset && servoTimer.getElapsedTimeSeconds() > 0.2){
            outtake.setIntakePow(1.0);
        }
    }
    public void resetServo() {
        timerReset = false;
        // blocks balls
        outtake.Servo_reset();
    }

    @Override
    public void init() {
        pathState = PathState.START_EARLY_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        servoTimer = new Timer();
        telemetryTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        outtake.init(hardwareMap);
        hood_servo = hardwareMap.get(Servo.class, "hood_servo");
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        if (telemetryTimer.getElapsedTimeSeconds() > 0.2) {
            updateTelemetry();
            telemetryTimer.resetTimer();
        }
    }
}