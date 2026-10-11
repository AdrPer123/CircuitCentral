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
public class blueAutoNoShoot extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, servoTimer, telemetryTimer;
    private Double flywheelRPM;
    private boolean timerReset = false;
    private final int ticksPerRevolution = 28; // Set once at declaration
    private double flywheelVelocity;
    private double targetTPS;

    private OuttakeSetup outtake = new OuttakeSetup();

    private enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        START_INTAKE1_SHOOT,
        SHOOT_PRELOAD,
        INTAKE_2,
        INTAKE_3,
        Park
    }

    private PathState pathState = PathState.START_INTAKE1_SHOOT;

    private final Pose startPose = new Pose(22.174496644295296, 124.9395973154362, Math.toRadians(144));
    private final Pose intakePose1 = new Pose(32.61208053691274, 84.12885906040268, Math.toRadians(180));
    private final Pose shootPos = new Pose(55.204026845637586, 78.055033557047, Math.toRadians(128));
    private final Pose intakePose2 = new Pose(36.38523489932886, 59.918120805369135, Math.toRadians(180));
    private final Pose intakePose3 = new Pose(36.265771812080544, 35.472483221476516, Math.toRadians(180));
    private final Pose park = new Pose(55.147651006711406, 62.04563758389262, Math.toRadians(0));

    private PathChain shoot3park;
    private PathChain startIntake1Shoot1;
    private final Pose startIntake1ControlPoint1 = new Pose(59.92065553701722, 81.19823147691874, Math.toRadians(0));
    private final Pose intake1Shoot1ControlPoint1 = new Pose(12.108053691275149, 86.59664429530206, Math.toRadians(0));
    private final Pose intake1Shoot1ControlPoint2 = new Pose(12.563758389261743, 72.28993288590604, Math.toRadians(0));
    private PathChain intake2Shoot2;

    private final Pose shoot1Intake2ControlPoint1 = new Pose(48.197986577181204, 59.35167785234897, Math.toRadians(0));
    private final Pose intake2Shoot2ControlPoint1 = new Pose(0, 56.75838926174494, Math.toRadians(0));
    private final Pose intake2Shoot2ControlPoint2 = new Pose(14.122818791946312, 79.9530201342282, Math.toRadians(0));
    private PathChain intake3Shoot3;
    private final Pose shoot2Intake3ControlPoint1 = new Pose(50.69932885906039, 38.08590604026848, Math.toRadians(0));
    private final Pose intake3Shoot3ControlPoint1 = new Pose(9.061744966442962, 30.803355704698028, Math.toRadians(0));
    private final Pose intake3Shoot3ControlPoint2 = new Pose(6.76510067114094, 40.97718120805369, Math.toRadians(0));
    private int shootingSequence = 1;
    private double ticksPerSecond, currentVelocity;

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
        shoot3park = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, park))
                .setLinearHeadingInterpolation(shootPos.getHeading(), park.getHeading())
                .build();

    }


    public void statePathUpdate() {
        switch (pathState) {
            case START_INTAKE1_SHOOT:
                follower.followPath(startIntake1Shoot1, true);
                shootingSequence = 1;
                break;
            case SHOOT_PRELOAD:
                //speeds up flywheel while intaking
                if (!follower.isBusy()) {
                    // 3. Move to the next state once shot is fired
                    if (shootingSequence == 1) {
                        pathState = PathState.INTAKE_2;
                    }
                    else if (shootingSequence == 2) {
                        pathState = PathState.INTAKE_3;
                    }
                    else if (shootingSequence == 3 ) {
                        pathState = PathState.Park;
                    }
                }
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
        velocityToleranceHigh = targetTPS * 1.1;
        velocityToleranceLow = targetTPS * 0.95;
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
        pathState = PathState.START_INTAKE1_SHOOT;
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
    }
}
