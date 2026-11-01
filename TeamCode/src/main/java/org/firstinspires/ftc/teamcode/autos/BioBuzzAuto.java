package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * BIOBUZZ auto for a TURRET robot (Pedro Pathing 2.x).
 * Same route as BioBuzz_Red_Auto.turt / BioBuzz_Blue_Auto.turt.
 *
 * The chassis NEVER rotates: it holds one heading the whole match (intake facing your side wall).
 * The turret aims at the HIVE every loop from the robot's pose.
 *
 * Coordinates are Pedro (inches, 0-144, origin bottom-left) written for RED.
 * BLUE is the same route rotated 180 degrees around the field center (see p()).
 *
 * Route: launch preload -> sweep GARDEN pollen -> launch -> left FLOWER (intake + launch in place)
 *        -> top FLOWER -> launch -> PARK in LOADING ZONE (LEAVE 3 + PARK 5).
 */
public abstract class BioBuzzAuto extends OpMode {

    /** true = Blue alliance (mirrors every pose). */
    protected abstract boolean isBlue();

    // ---- Tune these for your robot -----------------------------------------
    static final double LAUNCH_TIME_S        = 1.2;  // time spent firing 4 POLLEN
    static final double FLOWER_INTAKE_S      = 0.8;  // time at a FLOWER intaking
    static final double SWEEP_POWER          = 0.4;  // slow drive while sweeping the GARDEN
    static final double LAUNCHER_VELOCITY    = 1800; // ticks/sec for the launcher motor
    static final double TURRET_TICKS_PER_DEG = 4.0;  // turret encoder ticks per degree of turret turn
    static final double TURRET_LIMIT_DEG     = 170;  // don't spin the turret past this (cable safety)
    static final double TURRET_POWER         = 0.8;
    // Turret direction at INIT, relative to the robot front (0 = same way as intake).
    // 180 = turret points out the BACK of the robot, toward the HIVE, when you set up.
    static final double TURRET_HOME_DEG      = 180;
    // Where the turret aims (RED coords). Middle of the red side of the HIVE.
    // Better: use the HIVE AprilTags to pick whichever CELL is facing up after a tip.
    static final double AIM_X = 55, AIM_Y = 72;
    // Hardware names. Anything missing from your config is just skipped.
    static final String INTAKE_NAME   = "intake";
    static final String LAUNCHER_NAME = "launcher";
    static final String FEEDER_NAME   = "feeder";
    static final String TURRET_NAME   = "turret";
    // ------------------------------------------------------------------------

    /** Chassis heading for the whole auto (RED): intake faces the red side wall. */
    static final double CHASSIS_DEG = 180;

    private Follower follower;
    private final Timer stateTimer = new Timer();
    private int state = 0;
    private DcMotorEx intake, launcher, feeder, turret;
    private double aimX, aimY;

    private PathChain scorePreload, toGarden, sweepGarden, scoreGarden,
            toLeftFlower, toTopFlower, scoreTopFlower, park;

    /** Red point -> alliance pose, always at the fixed chassis heading. */
    private Pose p(double x, double y) {
        if (isBlue()) return new Pose(144 - x, 144 - y, Math.toRadians(CHASSIS_DEG + 180));
        return new Pose(x, y, Math.toRadians(CHASSIS_DEG));
    }

    private Pose START, LAUNCH_START, GARDEN_APPROACH, GARDEN_END, LAUNCH_SOUTH,
            LEFT_FLOWER, TOP_FLOWER, LAUNCH_NORTH, PARK;

    private void buildPoses() {
        START           = p(9,   72);
        LAUNCH_START    = p(20,  72);
        GARDEN_APPROACH = p(30,  11);
        GARDEN_END      = p(13,  11);
        LAUNCH_SOUTH    = p(22,  30);
        LEFT_FLOWER     = p(17,  49);   // intake AND launch here
        TOP_FLOWER      = p(61, 134);   // intake reaches the FLOWER from its side
        LAUNCH_NORTH    = p(26, 112);
        PARK            = p(13, 104);
        aimX = isBlue() ? 144 - AIM_X : AIM_X;
        aimY = isBlue() ? 144 - AIM_Y : AIM_Y;
    }

    private PathChain path(BezierLine line, Pose end) {
        return follower.pathBuilder()
                .addPath(line)
                .setConstantHeadingInterpolation(end.getHeading())
                .build();
    }

    private PathChain path(BezierCurve curve, Pose end) {
        return follower.pathBuilder()
                .addPath(curve)
                .setConstantHeadingInterpolation(end.getHeading())
                .build();
    }

    private void buildPaths() {
        scorePreload   = path(new BezierLine(START, LAUNCH_START), LAUNCH_START);
        toGarden       = path(new BezierCurve(LAUNCH_START, p(14, 40), GARDEN_APPROACH), GARDEN_APPROACH);
        sweepGarden    = path(new BezierLine(GARDEN_APPROACH, GARDEN_END), GARDEN_END);
        scoreGarden    = path(new BezierLine(GARDEN_END, LAUNCH_SOUTH), LAUNCH_SOUTH);
        toLeftFlower   = path(new BezierLine(LAUNCH_SOUTH, LEFT_FLOWER), LEFT_FLOWER);
        toTopFlower    = path(new BezierCurve(LEFT_FLOWER, p(16, 100), p(30, 134), TOP_FLOWER), TOP_FLOWER);
        scoreTopFlower = path(new BezierLine(TOP_FLOWER, LAUNCH_NORTH), LAUNCH_NORTH);
        park           = path(new BezierLine(LAUNCH_NORTH, PARK), PARK);
    }

    // ---- Mechanisms (replace with your own subsystem calls) -----------------
    private void intakeOn()       { if (intake != null) intake.setPower(1); }
    private void intakeOff()      { if (intake != null) intake.setPower(0); }
    private void launcherSpinUp() { if (launcher != null) launcher.setVelocity(LAUNCHER_VELOCITY); }
    private void launcherOff()    { if (launcher != null) launcher.setVelocity(0); }
    private void feedOn()         { if (feeder != null) feeder.setPower(1); }
    private void feedOff()        { if (feeder != null) feeder.setPower(0); }

    /** Turret angle (degrees from its INIT direction) that points at the HIVE. */
    private double turretAngleDeg(Pose pose) {
        double fieldAngle = Math.atan2(aimY - pose.getY(), aimX - pose.getX());
        double rel = Math.toDegrees(fieldAngle - pose.getHeading()) - TURRET_HOME_DEG;
        rel = ((rel + 540) % 360) - 180; // wrap to [-180, 180)
        return Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, rel));
    }

    /** Called every loop, so the turret tracks the HIVE while driving. */
    private void aimTurret(Pose pose) {
        if (turret == null) return;
        turret.setTargetPosition((int) Math.round(turretAngleDeg(pose) * TURRET_TICKS_PER_DEG));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(TURRET_POWER);
    }
    // ------------------------------------------------------------------------

    private void go(PathChain path, int next) {
        follower.followPath(path, true);
        setState(next);
    }

    private void setState(int s) {
        state = s;
        stateTimer.resetTimer();
    }

    private boolean pathDone() { return !follower.isBusy(); }
    private boolean waited(double s) { return stateTimer.getElapsedTimeSeconds() >= s; }

    private void autoUpdate() {
        switch (state) {
            case 0:  launcherSpinUp(); go(scorePreload, 1); break;
            case 1:  if (pathDone()) { feedOn(); setState(2); } break;                   // LaunchPreload
            case 2:  if (waited(LAUNCH_TIME_S)) { feedOff(); intakeOn(); go(toGarden, 3); } break;
            case 3:  if (pathDone()) { follower.followPath(sweepGarden, SWEEP_POWER, true); setState(4); } break;
            case 4:  if (pathDone()) setState(5); break;
            case 5:  if (waited(0.3)) { go(scoreGarden, 6); } break;                       // finish intake
            case 6:  if (pathDone()) { intakeOff(); feedOn(); setState(7); } break;     // LaunchGarden
            case 7:  if (waited(LAUNCH_TIME_S)) { feedOff(); intakeOn(); go(toLeftFlower, 8); } break;
            case 8:  if (pathDone()) setState(9); break;
            case 9:  if (waited(FLOWER_INTAKE_S)) { intakeOff(); feedOn(); setState(10); } break; // intake, then launch in place
            case 10: if (waited(LAUNCH_TIME_S)) { feedOff(); intakeOn(); go(toTopFlower, 11); } break;
            case 11: if (pathDone()) setState(12); break;
            case 12: if (waited(FLOWER_INTAKE_S)) { go(scoreTopFlower, 13); } break;     // IntakeTopFlower
            case 13: if (pathDone()) { intakeOff(); feedOn(); setState(14); } break;    // LaunchTopFlower
            case 14: if (waited(LAUNCH_TIME_S)) { feedOff(); launcherOff(); go(park, 15); } break;
            case 15: if (pathDone()) setState(-1); break;                                // Parked
            default: break;
        }
    }

    @Override
    public void init() {
        intake   = hardwareMap.tryGet(DcMotorEx.class, INTAKE_NAME);
        launcher = hardwareMap.tryGet(DcMotorEx.class, LAUNCHER_NAME);
        feeder   = hardwareMap.tryGet(DcMotorEx.class, FEEDER_NAME);
        turret   = hardwareMap.tryGet(DcMotorEx.class, TURRET_NAME);
        if (turret != null) {
            // At INIT the turret must point at the HIVE (out the back of the robot). That is 0 ticks.
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        follower = Constants.createFollower(hardwareMap);
        buildPoses();
        buildPaths();
        follower.setStartingPose(START);

        telemetry.addData("Alliance", isBlue() ? "BLUE" : "RED");
        telemetry.addLine("Robot centered on your alliance wall, INTAKE FACING THE WALL, turret pointing at the HIVE.");
        telemetry.update();
    }

    @Override
    public void start() {
        launcherSpinUp();
        setState(0);
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();
        aimTurret(pose);
        autoUpdate();

        telemetry.addData("State", state);
        telemetry.addData("X", "%.1f", pose.getX());
        telemetry.addData("Y", "%.1f", pose.getY());
        telemetry.addData("Turret aim (deg)", "%.1f", turretAngleDeg(pose));
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeOff();
        feedOff();
        launcherOff();
        if (turret != null) turret.setPower(0);
    }
}
