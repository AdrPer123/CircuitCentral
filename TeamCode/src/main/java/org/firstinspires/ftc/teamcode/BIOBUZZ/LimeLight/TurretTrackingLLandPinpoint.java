package org.firstinspires.ftc.teamcode.BIOBUZZ.LimeLight;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.DECODE_Archive.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.BIOBUZZ.LimeLight.TurretMechanismAdvanced;

@TeleOp
public class TurretTrackingLLandPinpoint extends OpMode {

    private Limelight3A limelight;
    private Follower follower;

    private double hiveX, hiveY, robotX, robotY, globalTurretAngle, robotAngle, pinpointTx, LLTx;
    // UPDATE HIVE X AND Y

    private double ticksPerRevolution = 26.0; // filler value

    private double gearRatio = 4.0; //gear teeth of turret gear/motor gear

    private double ticksPerTurretRevolution = gearRatio*ticksPerRevolution;

    private static final double MAX_STALENESS_MS = 100;

    private boolean LLtracking;

    private final Pose startPose = new Pose(21.208053691275158, 124.35973154362415, Math.toRadians(144)); //NOT UPDATED

    private final TurretMechanismAdvanced turret = new TurretMechanismAdvanced();

    private double getGlobalTurretAngle (double robotAngle, double LocalturretAngle) {
        return robotAngle+LocalturretAngle;
    }
    private double getPinpointTx (double robotX, double robotY, double globalTurretAngle) {
        return wrapDegrees(globalTurretAngle-Math.toDegrees(Math.atan2((hiveY-robotY),(hiveX-robotX))));
    }

    private static double wrapDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    @Override
    public void init() {
        turret.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void start() {
        limelight.start();
        turret.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        boolean LLhasTarget = result != null
                && result.isValid()
                && result.getStaleness() < MAX_STALENESS_MS;
        double tx = LLhasTarget ? result.getTx() : 0;
        if (LLhasTarget) {
            turret.update(LLhasTarget, tx);
        }
        else  {
            robotX = follower.getPose().getX();
            robotY = follower.getPose().getY();
            double localTurretAngle = TurretMechanismAdvanced.turret.getCurrentPosition() * 360 / ticksPerTurretRevolution;
            robotAngle = Math.toDegrees(follower.getPose().getHeading());
            globalTurretAngle = getGlobalTurretAngle(robotAngle, localTurretAngle);
            pinpointTx = getPinpointTx(robotX, robotY, globalTurretAngle);
            turret.update(true, pinpointTx);
        }
        follower.update();
    }

}
