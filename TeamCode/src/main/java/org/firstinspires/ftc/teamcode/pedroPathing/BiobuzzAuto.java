package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Teleop.OuttakeSetup;

/**
 * BIOBUZZ autonomous: leave the wall, launch into the upward-facing CELL to TIP the
 * HIVE, then park.
 *
 * Scoring this targets: AUTO LEAVE 3, HIVE TIP 20, AUTO PARK 5.
 *
 * Aiming uses the AprilTag cluster on the underside of the CELL. The cluster origin
 * is at the centre of the CELL opening, so pointing at the tag pose is pointing at
 * the hole. Tags move with the HIVE, so there is no field localisation here.
 *
 * Everything in the TUNE block below is a starting guess and must be set on a real
 * field before this scores.
 */
@Autonomous(name = "BIOBUZZ Auto - Tip Hive", group = "Auto")
public class BiobuzzAuto extends OpMode {

    // ===================== TUNE =====================
    private static final int LIMELIGHT_PIPELINE = 8;

    /** Flywheel speed for a CELL shot, in encoder ticks per second. */
    private static final double SHOT_VELOCITY_TPS = 1500.0;
    private static final double VELOCITY_TOLERANCE_TPS = 60.0;

    /** How many ARTIFACTS to launch before giving up on the TIP and parking. */
    private static final int SHOTS_TO_ATTEMPT = 3;
    private static final double RELEASE_HOLD_SEC = 0.35;
    private static final double RECOVER_SEC = 0.45;

    /** Hood position at two known ranges; anything between is interpolated. */
    private static final double NEAR_RANGE_IN = 24.0;
    private static final double NEAR_HOOD = 0.15;
    private static final double FAR_RANGE_IN = 96.0;
    private static final double FAR_HOOD = 0.55;

    /** Give up on a vision lock after this long and shoot on the path heading. */
    private static final double AIM_TIMEOUT_SEC = 2.0;
    private static final double AIM_TOLERANCE_DEG = 2.0;

    // Field poses. Replace with values measured on your field.
    private final Pose startPose = new Pose(9.0, 60.0, Math.toRadians(0));
    private final Pose shootPose = new Pose(36.0, 60.0, Math.toRadians(0));
    private final Pose parkPose = new Pose(60.0, 36.0, Math.toRadians(0));
    // ================== END OF TUNE ==================

    private enum State { DRIVE_TO_SHOOT, AIM, SHOOT, RECOVER, DRIVE_TO_PARK, DONE }

    private Follower follower;
    private OuttakeSetup outtake;
    private Limelight3A limelight;
    private Timer stateTimer, opModeTimer;

    private PathChain toShoot, toPark;
    private State state;
    private int shotsFired;
    private double lastRangeIn = -1;
    private double lastTxDeg = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        outtake = new OuttakeSetup();
        outtake.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        stateTimer = new Timer();
        opModeTimer = new Timer();

        buildPaths();
        telemetry.addLine("BIOBUZZ auto ready");
    }

    private void buildPaths() {
        toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        limelight.start();
        follower.followPath(toShoot, true);
        setState(State.DRIVE_TO_SHOOT);
    }

    @Override
    public void loop() {
        // Pedro only moves the drivetrain when this is called, every single loop.
        follower.update();
        runStateMachine();
        report();
    }

    private void runStateMachine() {
        switch (state) {
            case DRIVE_TO_SHOOT:
                // Spin up while still driving so the flywheel is ready on arrival.
                outtake.setOuttakeVelocity(SHOT_VELOCITY_TPS);
                if (!follower.isBusy()) {
                    setState(State.AIM);
                }
                break;

            case AIM:
                outtake.setOuttakeVelocity(SHOT_VELOCITY_TPS);
                boolean haveTarget = updateAimFromLimelight();
                boolean aimed = haveTarget && Math.abs(lastTxDeg) <= AIM_TOLERANCE_DEG;
                boolean timedOut = stateTimer.getElapsedTimeSeconds() > AIM_TIMEOUT_SEC;

                if (haveTarget && !aimed && !follower.isTurning()) {
                    follower.turnToDegrees(
                            Math.toDegrees(follower.getPose().getHeading()) + lastTxDeg);
                }
                if ((aimed || timedOut) && atShotSpeed() && !follower.isTurning()) {
                    outtake.Servo_release();
                    setState(State.SHOOT);
                }
                break;

            case SHOOT:
                if (stateTimer.getElapsedTimeSeconds() > RELEASE_HOLD_SEC) {
                    outtake.Servo_reset();
                    shotsFired++;
                    setState(State.RECOVER);
                }
                break;

            case RECOVER:
                outtake.setOuttakeVelocity(SHOT_VELOCITY_TPS);
                if (stateTimer.getElapsedTimeSeconds() > RECOVER_SEC) {
                    if (shotsFired >= SHOTS_TO_ATTEMPT) {
                        follower.followPath(toPark, true);
                        setState(State.DRIVE_TO_PARK);
                    } else {
                        setState(State.AIM);
                    }
                }
                break;

            case DRIVE_TO_PARK:
                outtake.setOuttakeVelocity(0);
                outtake.setIntakePow(0);
                if (!follower.isBusy()) {
                    setState(State.DONE);
                }
                break;

            case DONE:
            default:
                outtake.setOuttakeVelocity(0);
                outtake.setIntakePow(0);
                break;
        }
    }

    /** Points the hood at the CELL. Returns false when no tag is visible. */
    private boolean updateAimFromLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            return false;
        }

        LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
        // Camera space is x = right, y = down, z = forward.
        Position p = tag.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
        lastRangeIn = Math.hypot(p.z, p.x);
        lastTxDeg = result.getTx();

        outtake.setHoodPosition(hoodForRange(lastRangeIn));
        return true;
    }

    private double hoodForRange(double rangeIn) {
        double t = (rangeIn - NEAR_RANGE_IN) / (FAR_RANGE_IN - NEAR_RANGE_IN);
        t = Math.max(0.0, Math.min(1.0, t));
        return NEAR_HOOD + t * (FAR_HOOD - NEAR_HOOD);
    }

    private boolean atShotSpeed() {
        double left = outtake.getOuttakeVelocityLeft();
        double right = outtake.getOuttakeVelocityRight();
        return Math.abs(left - SHOT_VELOCITY_TPS) <= VELOCITY_TOLERANCE_TPS
                && Math.abs(right - SHOT_VELOCITY_TPS) <= VELOCITY_TOLERANCE_TPS;
    }

    private void setState(State next) {
        state = next;
        stateTimer.resetTimer();
    }

    private void report() {
        telemetry.addData("state", state);
        telemetry.addData("auto time", "%.1f s", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("shots", "%d / %d", shotsFired, SHOTS_TO_ATTEMPT);
        telemetry.addData("pose", "x %.1f y %.1f h %.1f",
                follower.getPose().getX(), follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("flywheel", "L %.0f R %.0f (target %.0f)",
                outtake.getOuttakeVelocityLeft(), outtake.getOuttakeVelocityRight(),
                SHOT_VELOCITY_TPS);
        if (lastRangeIn >= 0) {
            telemetry.addData("target", "range %.1f in | tx %.1f deg", lastRangeIn, lastTxDeg);
        } else {
            telemetry.addLine("target: no tag seen yet");
        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
