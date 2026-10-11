package org.firstinspires.ftc.teamcode.DECODE_Archive.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.DECODE_Archive.Teleop.OuttakeSetup;

@Autonomous
public class RedAutoPathing3 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, servoTimer, telemetryTimer;
    private boolean timerReset = false;
    private double targetTPS;

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

    private final Pose startPose = new Pose(120.79194630872485, 124.35973154362415, Math.toRadians(36));
    private final Pose earlyShoot = new Pose(81.26577181208054, 98.99060402684565, Math.toRadians(41));
    private final Pose intakePose1 = new Pose(101, 84.12885906040268, Math.toRadians(0));
    private final Pose shootPos = new Pose(85, 81, Math.toRadians(48));
    private final Pose intakePose2 = new Pose(101, 59.918120805369135, Math.toRadians(0));
    private final Pose intakePose3 = new Pose(99, 35.472483221476516, Math.toRadians(0));
    private final Pose park = new Pose(103.86174496644296, 64.944966442953, Math.toRadians(90));

    private PathChain shoot3park, startEarlyshoot;
    private PathChain startIntake1Shoot1;
    private final Pose earlyShootIntake1ControlPoint1 = new Pose(76.53303573815057, 81.58480865812676, Math.toRadians(0));
    private final Pose intake1Shoot1ControlPoint1 = new Pose(114.24496644295303, 86.59664429530206, Math.toRadians(0));
    private final Pose intake1Shoot1ControlPoint2 = new Pose(128, 74.99597315436242, Math.toRadians(0));
    private PathChain intake2Shoot2;

    private final Pose shoot1Intake2ControlPoint1 = new Pose(75.69194630872482, 59.35167785234897, Math.toRadians(0));
    private final Pose intake2Shoot2ControlPoint1 = new Pose(117.52080536912752, 56.85906040268453, Math.toRadians(0));
    private final Pose intake2Shoot2ControlPoint2 = new Pose(122.42818791946308, 80.50872483221478, Math.toRadians(0));
    private final Pose intake2Shoot2ControlPoint3 = new Pose(99.05570469798657, 82.21610738255033, Math.toRadians(0));
    private PathChain intake3Shoot3;
    private final Pose shoot2Intake3ControlPoint1 = new Pose(82.30067114093961, 38.08590604026848, Math.toRadians(0));
    private final Pose intake3Shoot3ControlPoint1 = new Pose(124.24966442953018, 31.76979865771816, Math.toRadians(0));
    private final Pose intake3Shoot3ControlPoint2 = new Pose(124.80671140939597, 34.79194630872482, Math.toRadians(0));
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
                outtake.setIntakePow(1.0);
                targetTPS = 1150.0;
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
                targetTPS = 1225.0;
                setFlywheelTPS(targetTPS);
                if (!follower.isBusy()) {
                    // 3. Move to the next state once shot is fired
                    if (shootingSequence == 1 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        if (servoTimer.getElapsedTimeSeconds() > 2.0 &&timerReset) {
                            setPathState(PathState.INTAKE_2); // Transition to next move
                            resetServo();
                        }
                    }
                    else if (shootingSequence == 2 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        if (servoTimer.getElapsedTimeSeconds() > 2.0 && timerReset) {
                            setPathState(PathState.INTAKE_3); // Transition to next move
                            resetServo();
                        }
                    }
                    else if (shootingSequence == 3 && flywheelIsCorrectSpeed()) {
                        launch_balls();
                        // power servos to launch balls
                        if (servoTimer.getElapsedTimeSeconds() > 2.0 && timerReset) {
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